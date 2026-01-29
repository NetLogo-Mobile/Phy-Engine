#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/digital.h>
#include <phy_engine/verilog/digital/pe_synth.h>

#include <phy_engine/model/models/digital/logical/input.h>
#include <phy_engine/model/models/digital/logical/output.h>
#include <phy_engine/model/models/digital/verilog_module.h>

#include <phy_engine/netlist/operation.h>

#include <phy_engine/pe_nl_fileformat/pe_nl_fileformat.h>

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

namespace
{
using u8sv = ::fast_io::u8string_view;

struct include_ctx
{
    std::filesystem::path base_dir;
};

std::string read_file_text(std::filesystem::path const& path)
{
    std::ifstream ifs(path, std::ios::binary);
    if(!ifs.is_open()) { throw std::runtime_error("failed to open: " + path.string()); }
    std::string s;
    ifs.seekg(0, std::ios::end);
    auto const n = static_cast<std::size_t>(ifs.tellg());
    ifs.seekg(0, std::ios::beg);
    s.resize(n);
    if(n != 0) { ifs.read(s.data(), static_cast<std::streamsize>(n)); }
    return s;
}

bool include_resolver_fs(void* user, u8sv path, ::fast_io::u8string& out_text) noexcept
{
    try
    {
        auto* ctx = static_cast<include_ctx*>(user);
        std::string rel(reinterpret_cast<char const*>(path.data()), path.size());
        auto p = ctx->base_dir / rel;
        auto s = read_file_text(p);
        out_text.assign(u8sv{reinterpret_cast<char8_t const*>(s.data()), s.size()});
        return true;
    }
    catch(...)
    {
        return false;
    }
}

static void usage(char const* argv0)
{
    ::fast_io::io::perr(::fast_io::err(),
                        "usage: ",
                        ::fast_io::mnp::os_c_str(argv0),
                        " OUT.penl IN.v [--top TOP_MODULE]\n"
                        "  - Stores Verilog as a VERILOG_MODULE in PE netlist, with optional IO wrappers,\n"
                        "    then exports using PE-NL (LevelDB-backed) format.\n"
                        "options:\n"
                        "  --layout file|dir                         Output layout (default: file)\n"
                        "  --mode full|structure|checkpoint          Export mode (default: full)\n"
                        "  --no-io                                  Do not generate INPUT/OUTPUT models\n"
                        "  --synth                                  Synthesize to PE primitives (no VERILOG_MODULE)\n"
                        "  -O0|-O1|-O2|-O3                           PE synth optimization level (default: O0)\n"
                        "  --opt-level N                             PE synth optimization level (0..3)\n"
                        "  --assume-binary-inputs                    Treat X/Z as absent in synth (default: off)\n"
                        "  --no-assume-binary-inputs                 Preserve X-propagation logic\n"
                        "  --opt-wires|--no-opt-wires                Enable/disable YES buffer elimination (default: on)\n"
                        "  --opt-mul2|--no-opt-mul2                  Enable/disable MUL2 macro recognition (default: on)\n"
                        "  --opt-adders|--no-opt-adders              Enable/disable adder macro recognition (default: on)\n"
                        "  --allow-inout                             Allow inout ports (requires --no-io)\n"
                        "  --allow-multi-driver                      Allow multi-driver digital nets\n"
                        "  --overwrite                              Overwrite existing output\n");
}

static std::optional<std::string> arg_after(int argc, char** argv, std::string_view flag)
{
    for(int i = 1; i + 1 < argc; ++i)
    {
        if(std::string_view(argv[i]) == flag) { return std::string(argv[i + 1]); }
    }
    return std::nullopt;
}

static bool has_flag(int argc, char** argv, std::string_view flag)
{
    for(int i = 1; i < argc; ++i)
    {
        if(std::string_view(argv[i]) == flag) { return true; }
    }
    return false;
}

static std::optional<std::size_t> parse_size(std::string const& s)
{
    try
    {
        std::size_t idx{};
        auto v = std::stoull(s, &idx, 10);
        if(idx != s.size()) { return std::nullopt; }
        return static_cast<std::size_t>(v);
    }
    catch(...)
    {
        return std::nullopt;
    }
}

static std::optional<bool> parse_toggle(int argc, char** argv, std::string_view on_flag, std::string_view off_flag)
{
    std::optional<bool> v{};
    for(int i = 1; i < argc; ++i)
    {
        auto const a = std::string_view(argv[i]);
        if(a == on_flag) { v = true; }
        else if(a == off_flag) { v = false; }
    }
    return v;
}

static std::optional<std::uint8_t> parse_opt_level(int argc, char** argv)
{
    std::optional<std::uint8_t> lvl{};
    for(int i = 1; i < argc; ++i)
    {
        auto const a = std::string_view(argv[i]);
        if(a.size() == 3 && a[0] == '-' && a[1] == 'O')
        {
            char const d = a[2];
            if(d >= '0' && d <= '3') { lvl = static_cast<std::uint8_t>(d - '0'); }
        }
    }
    if(auto s = arg_after(argc, argv, "--opt-level"))
    {
        if(auto n = parse_size(*s); n && *n <= 3u) { lvl = static_cast<std::uint8_t>(*n); }
        else { return std::nullopt; }
    }
    return lvl ? lvl : std::optional<std::uint8_t>{static_cast<std::uint8_t>(0)};
}

static std::optional<::phy_engine::pe_nl_fileformat::storage_layout> parse_layout(std::optional<std::string> const& s)
{
    using ::phy_engine::pe_nl_fileformat::storage_layout;
    if(!s) { return storage_layout::single_file; }
    auto const& v = *s;
    if(v == "file" || v == "single" || v == "single_file" || v == "0") { return storage_layout::single_file; }
    if(v == "dir" || v == "directory" || v == "1") { return storage_layout::directory; }
    return std::nullopt;
}

static std::optional<::phy_engine::pe_nl_fileformat::export_mode> parse_mode(std::optional<std::string> const& s)
{
    using ::phy_engine::pe_nl_fileformat::export_mode;
    if(!s) { return export_mode::full; }
    auto const& v = *s;
    if(v == "full" || v == "0") { return export_mode::full; }
    if(v == "structure" || v == "structure_only" || v == "1") { return export_mode::structure_only; }
    if(v == "checkpoint" || v == "runtime" || v == "runtime_only" || v == "2") { return export_mode::runtime_only; }
    return std::nullopt;
}

static ::phy_engine::verilog::digital::compiled_module const*
find_top_module(::phy_engine::verilog::digital::compiled_design const& d, std::optional<std::string> const& top_override)
{
    using ::phy_engine::verilog::digital::find_module;

    if(top_override)
    {
        auto u8 = ::fast_io::u8string{u8sv{reinterpret_cast<char8_t const*>(top_override->data()), top_override->size()}};
        return find_module(d, u8);
    }

    // Heuristic: choose a module that is not instantiated by any other module.
    std::unordered_set<std::string> all{};
    std::unordered_set<std::string> used{};
    all.reserve(d.modules.size());
    used.reserve(d.modules.size());

    for(auto const& m : d.modules)
    {
        all.emplace(std::string(reinterpret_cast<char const*>(m.name.data()), m.name.size()));
        for(auto const& inst : m.instances)
        {
            used.emplace(std::string(reinterpret_cast<char const*>(inst.module_name.data()), inst.module_name.size()));
        }
    }

    std::vector<::phy_engine::verilog::digital::compiled_module const*> candidates{};
    for(auto const& m : d.modules)
    {
        std::string nm(reinterpret_cast<char const*>(m.name.data()), m.name.size());
        if(used.find(nm) == used.end()) { candidates.push_back(&m); }
    }

    if(candidates.size() == 1) { return candidates[0]; }
    if(candidates.empty())
    {
        if(d.modules.empty()) { return nullptr; }
        return &d.modules.back();
    }

    auto* best = candidates[0];
    for(auto* m : candidates)
    {
        if(m->ports.size() > best->ports.size()) { best = m; }
    }
    return best;
}

}  // namespace

int main(int argc, char** argv)
{
    if(argc < 3 || has_flag(argc, argv, "--help") || has_flag(argc, argv, "-h"))
    {
        usage(argv[0]);
        return (argc < 3) ? 2 : 0;
    }

    try
    {
        auto out_path = std::filesystem::path(argv[1]);
        auto in_path = std::filesystem::path(argv[2]);

        auto top_override = arg_after(argc, argv, "--top");
        auto layout = parse_layout(arg_after(argc, argv, "--layout"));
        if(!layout)
        {
            ::fast_io::io::perr(::fast_io::err(), "error: invalid --layout value\n");
            return 10;
        }

        auto mode = parse_mode(arg_after(argc, argv, "--mode"));
        if(!mode)
        {
            ::fast_io::io::perr(::fast_io::err(), "error: invalid --mode value\n");
            return 11;
        }

        bool const gen_io = !has_flag(argc, argv, "--no-io");
        bool const overwrite = has_flag(argc, argv, "--overwrite");
        bool const synth = has_flag(argc, argv, "--synth") || has_flag(argc, argv, "--synthesize");

        auto const opt_level = parse_opt_level(argc, argv);
        if(!opt_level)
        {
            ::fast_io::io::perr(::fast_io::err(), "error: invalid -O* / --opt-level\n");
            return 12;
        }

        bool assume_binary_inputs = false;
        bool opt_wires = true;
        bool opt_mul2 = true;
        bool opt_adders = true;
        if(auto v = parse_toggle(argc, argv, "--assume-binary-inputs", "--no-assume-binary-inputs")) { assume_binary_inputs = *v; }
        if(auto v = parse_toggle(argc, argv, "--opt-wires", "--no-opt-wires")) { opt_wires = *v; }
        if(auto v = parse_toggle(argc, argv, "--opt-mul2", "--no-opt-mul2")) { opt_mul2 = *v; }
        if(auto v = parse_toggle(argc, argv, "--opt-adders", "--no-opt-adders")) { opt_adders = *v; }
        bool const allow_inout = has_flag(argc, argv, "--allow-inout");
        bool const allow_multi_driver = has_flag(argc, argv, "--allow-multi-driver");
        if(allow_inout && gen_io)
        {
            ::fast_io::io::perr(::fast_io::err(), "error: --allow-inout requires --no-io\n");
            return 13;
        }

        using namespace ::phy_engine;
        using namespace ::phy_engine::verilog::digital;

        auto const src_s = read_file_text(in_path);
        auto const src = u8sv{reinterpret_cast<char8_t const*>(src_s.data()), src_s.size()};

        compile_options copt{};
        include_ctx ictx{.base_dir = in_path.parent_path()};
        copt.preprocess.user = __builtin_addressof(ictx);
        copt.preprocess.include_resolver = include_resolver_fs;

        auto const in_path_s = in_path.string();
        ::fast_io::io::perr(::fast_io::err(), "[verilog2penl] compile ", ::fast_io::mnp::os_c_str(in_path_s.c_str()), "\n");
        auto cr = ::phy_engine::verilog::digital::compile(src, copt);
        if(!cr.errors.empty())
        {
            diagnostic_options dop{};
            dop.filename = u8sv{reinterpret_cast<char8_t const*>(in_path_s.data()), in_path_s.size()};

            auto const diag = format_compile_errors(cr, src, dop);
            if(!diag.empty()) { ::fast_io::io::perr(::fast_io::u8err(), u8sv{diag.data(), diag.size()}); }
            if(cr.modules.empty())
            {
                ::fast_io::io::perr(::fast_io::err(),
                                    "note: no Verilog `module` was successfully parsed; check that the input is a Verilog source file\n");
            }
            return 1;
        }
        if(cr.modules.empty())
        {
            ::fast_io::io::perr(::fast_io::err(),
                                "error: no Verilog `module` found in input: ",
                                ::fast_io::mnp::os_c_str(in_path_s.c_str()),
                                "\n");
            return 1;
        }

        auto design_v = ::phy_engine::verilog::digital::build_design(::std::move(cr));
        auto design = ::std::make_shared<::phy_engine::verilog::digital::compiled_design>(::std::move(design_v));

        auto const* top_mod = find_top_module(*design, top_override);
        if(top_mod == nullptr) { return 3; }

        ::fast_io::io::perr(::fast_io::u8err(),
                            u8"[verilog2penl] top=",
                            u8sv{top_mod->name.data(), top_mod->name.size()},
                            u8" (ports=",
                            top_mod->ports.size(),
                            u8")\n");

        auto top_inst = ::phy_engine::verilog::digital::elaborate(*design, *top_mod);
        if(top_inst.mod == nullptr) { return 4; }

        ::phy_engine::circult c{};
        c.set_analyze_type(::phy_engine::analyze_type::TR);
        c.get_analyze_setting().tr.t_step = 1e-9;
        c.get_analyze_setting().tr.t_stop = 1e-9;
        auto& nl = c.get_netlist();

        // Create one node per port (in port-list order).
        std::vector<::phy_engine::model::node_t*> port_nodes{};
        port_nodes.reserve(top_inst.mod->ports.size());
        for(std::size_t i{}; i < top_inst.mod->ports.size(); ++i)
        {
            auto& n = ::phy_engine::netlist::create_node(nl);
            port_nodes.push_back(__builtin_addressof(n));
        }

        // Optional IO wrappers for interactive simulation.
        if(gen_io)
        {
            for(std::size_t pi{}; pi < top_inst.mod->ports.size(); ++pi)
            {
                auto const& p = top_inst.mod->ports.index_unchecked(pi);
                if(p.dir == port_dir::input)
                {
                    auto [m, pos] =
                        ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
                    (void)pos;
                    if(m == nullptr || m->ptr == nullptr) { return 5; }
                    m->name = p.name;
                    if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *port_nodes[pi])) { return 6; }
                }
                else if(p.dir == port_dir::output)
                {
                    auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
                    (void)pos;
                    if(m == nullptr || m->ptr == nullptr) { return 7; }
                    m->name = p.name;
                    if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *port_nodes[pi])) { return 8; }
                }
                else
                {
                    ::fast_io::io::perr(::fast_io::err(), "error: unsupported port dir (inout)\n");
                    return 9;
                }
            }
        }

        if(synth)
        {
            ::phy_engine::verilog::digital::pe_synth_error err{};
            ::phy_engine::verilog::digital::pe_synth_options opt{
                .allow_inout = allow_inout,
                .allow_multi_driver = allow_multi_driver,
                .assume_binary_inputs = assume_binary_inputs,
                .opt_level = *opt_level,
                .optimize_wires = opt_wires,
                .optimize_mul2 = opt_mul2,
                .optimize_adders = opt_adders,
            };

            ::fast_io::io::perr(::fast_io::err(), "[verilog2penl] synthesize_to_pe_netlist\n");
            if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, port_nodes, &err, opt))
            {
                ::fast_io::io::perr(::fast_io::u8err(),
                                    u8"error: synthesize_to_pe_netlist failed: ",
                                    u8sv{err.message.data(), err.message.size()},
                                    u8"\n");
                return 14;
            }
        }
        else
        {
            // Add the VERILOG_MODULE model (preserves source + supports checkpointing).
            ::phy_engine::model::VERILOG_MODULE vm{};
            vm.source = ::fast_io::u8string{src};
            vm.top = top_mod->name;
            vm.design = std::move(design);
            vm.top_instance = std::move(top_inst);

            vm.pin_name_storage.clear();
            vm.pins.clear();
            vm.pin_name_storage.reserve(vm.top_instance.mod->ports.size());
            vm.pins.reserve(vm.top_instance.mod->ports.size());
            for(auto const& p : vm.top_instance.mod->ports)
            {
                vm.pin_name_storage.push_back(p.name);
                auto const& name = vm.pin_name_storage.back_unchecked();
                vm.pins.push_back({::fast_io::u8string_view{name.data(), name.size()}, nullptr, nullptr});
            }

            auto added = ::phy_engine::netlist::add_model(nl, ::std::move(vm));
            if(added.mod == nullptr || added.mod->ptr == nullptr) { return 15; }
            added.mod->name = top_mod->name;
            {
                auto const p = in_path.filename().string();
                added.mod->describe = ::fast_io::u8string{u8sv{reinterpret_cast<char8_t const*>(p.data()), p.size()}};
            }

            for(std::size_t pi{}; pi < port_nodes.size(); ++pi)
            {
                if(!::phy_engine::netlist::add_to_node(nl, added.mod_pos, pi, *port_nodes[pi])) { return 16; }
            }
        }

        ::phy_engine::pe_nl_fileformat::save_options sopt{};
        sopt.overwrite = overwrite;
        sopt.mode = *mode;
        sopt.layout = *layout;

        ::fast_io::io::perr(::fast_io::err(), "[verilog2penl] save\n");
        auto st = ::phy_engine::pe_nl_fileformat::save(out_path, c, sopt);
        if(!st)
        {
            ::fast_io::io::perr(::fast_io::err(),
                                "error: save failed: ",
                                ::fast_io::mnp::os_c_str(st.message.c_str()),
                                "\n");
            return 20;
        }

        return 0;
    }
    catch(std::exception const& e)
    {
        ::fast_io::io::perr(::fast_io::err(), "fatal: ", ::fast_io::mnp::os_c_str(e.what()), "\n");
        return 100;
    }
}
