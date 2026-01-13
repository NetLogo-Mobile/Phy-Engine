#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/digital.h>
#include <phy_engine/verilog/digital/pe_synth.h>

#include <phy_engine/model/models/digital/logical/input.h>
#include <phy_engine/model/models/digital/logical/output.h>
#include <phy_engine/netlist/operation.h>

#include <phy_engine/phy_lab_wrapper/pe_to_pl.h>

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace
{
using u8sv = ::fast_io::u8string_view;

::phy_engine::model::variant dv(::phy_engine::model::digital_node_statement_t v) noexcept
{
    ::phy_engine::model::variant vi{};
    vi.digital = v;
    vi.type = ::phy_engine::model::variant_type::digital;
    return vi;
}

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

struct include_ctx
{
    std::filesystem::path base_dir;
};

bool include_resolver_fs(void* user, ::fast_io::u8string_view path, ::fast_io::u8string& out_text) noexcept
{
    try
    {
        auto* ctx = static_cast<include_ctx*>(user);
        std::string rel(reinterpret_cast<char const*>(path.data()), path.size());
        auto p = ctx->base_dir / rel;
        auto s = read_file_text(p);
        out_text.assign(::fast_io::u8string_view{reinterpret_cast<char8_t const*>(s.data()), s.size()});
        return true;
    }
    catch(...)
    {
        return false;
    }
}

std::optional<std::size_t> parse_bit_index(std::string_view s, std::string_view base)
{
    if(!s.starts_with(base)) { return std::nullopt; }
    if(s.size() < base.size() + 3) { return std::nullopt; }  // "a[0]"
    if(s[base.size()] != '[') { return std::nullopt; }
    if(s.back() != ']') { return std::nullopt; }
    auto inner = s.substr(base.size() + 1, s.size() - (base.size() + 2));
    if(inner.empty()) { return std::nullopt; }
    std::size_t v{};
    for(char ch : inner)
    {
        if(ch < '0' || ch > '9') { return std::nullopt; }
        v = v * 10 + static_cast<std::size_t>(ch - '0');
    }
    return v;
}
}  // namespace

int main()
{
    using namespace phy_engine;
    using namespace phy_engine::verilog::digital;
    using namespace phy_engine::phy_lab_wrapper;

    static constexpr std::size_t kA = 16;
    static constexpr std::size_t kB = 16;
    static constexpr std::size_t kOP = 2;
    static constexpr std::size_t kY = 16;

    auto const src_path = std::filesystem::path(__FILE__).parent_path() / "fp16_fpu.v";
    auto const src_s = read_file_text(src_path);
    auto const src = ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(src_s.data()), src_s.size()};

    ::phy_engine::verilog::digital::compile_options copt{};
    include_ctx ictx{.base_dir = src_path.parent_path()};
    copt.preprocess.user = __builtin_addressof(ictx);
    copt.preprocess.include_resolver = include_resolver_fs;
    auto cr = ::phy_engine::verilog::digital::compile(src, copt);
    if(!cr.errors.empty() || cr.modules.empty()) { return 1; }

    auto design = ::phy_engine::verilog::digital::build_design(::std::move(cr));
    auto const* top_mod = ::phy_engine::verilog::digital::find_module(design, u8"fp16_fpu_top");
    if(top_mod == nullptr) { return 2; }

    // Elaborate for PE synthesis.
    auto top_inst = ::phy_engine::verilog::digital::elaborate(design, *top_mod);
    if(top_inst.mod == nullptr) { return 3; }

    // Build a PE netlist with explicit IO models per port bit.
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    auto& setting = c.get_analyze_setting();
    setting.tr.t_step = 1e-9;
    setting.tr.t_stop = 1e-9;
    auto& nl = c.get_netlist();

    std::vector<::phy_engine::model::node_t*> ports{};
    ports.reserve(top_inst.mod->ports.size());
    for(std::size_t i{}; i < top_inst.mod->ports.size(); ++i)
    {
        auto& n = ::phy_engine::netlist::create_node(nl);
        ports.push_back(__builtin_addressof(n));
    }

    std::array<::phy_engine::model::model_base*, kA> in_a{};
    std::array<::phy_engine::model::model_base*, kB> in_b{};
    std::array<::phy_engine::model::model_base*, kOP> in_op{};
    std::array<std::size_t, kY> out_y{};
    in_a.fill(nullptr);
    in_b.fill(nullptr);
    in_op.fill(nullptr);
    out_y.fill(static_cast<std::size_t>(-1));

    for(std::size_t pi{}; pi < top_inst.mod->ports.size(); ++pi)
    {
        auto const& p = top_inst.mod->ports.index_unchecked(pi);
        std::string port_name(reinterpret_cast<char const*>(p.name.data()), p.name.size());

        if(p.dir == port_dir::input)
        {
            auto [m, pos] =
                ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
            (void)pos;
            if(m == nullptr || m->ptr == nullptr) { return 4; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *ports[pi])) { return 5; }

            if(auto idx = parse_bit_index(port_name, "a"); idx && *idx < kA) { in_a[*idx] = m; }
            else if(auto idx = parse_bit_index(port_name, "b"); idx && *idx < kB) { in_b[*idx] = m; }
            else if(auto idx = parse_bit_index(port_name, "op"); idx && *idx < kOP) { in_op[*idx] = m; }
        }
        else if(p.dir == port_dir::output)
        {
            auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
            (void)pos;
            if(m == nullptr || m->ptr == nullptr) { return 6; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *ports[pi])) { return 7; }
            if(auto idx = parse_bit_index(port_name, "y"); idx && *idx < kY) { out_y[*idx] = pi; }
        }
        else
        {
            return 8;
        }
    }

    for(auto* m : in_a) { if(m == nullptr) { return 9; } }
    for(auto* m : in_b) { if(m == nullptr) { return 10; } }
    for(auto* m : in_op) { if(m == nullptr) { return 11; } }
    for(auto pi : out_y) { if(pi == static_cast<std::size_t>(-1)) { return 12; } }

    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt{
        .allow_inout = false,
        .allow_multi_driver = false,
        .assume_binary_inputs = true,
        .optimize_wires = true,
        .optimize_adders = true,
    };
    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, ports, &err, opt)) { return 13; }

    auto count_models = [&]() -> std::size_t {
        return ::phy_engine::netlist::get_num_of_model(nl);
    };
    auto count_nodes = [&]() -> std::size_t {
        std::size_t n{};
        for(auto const& blk : nl.nodes) { n += static_cast<std::size_t>(blk.curr - blk.begin); }
        return n;
    };
    auto count_pins = [&]() -> std::size_t {
        std::size_t total{};
        for(auto const& blk : nl.nodes)
        {
            for(auto const* n = blk.begin; n != blk.curr; ++n) { total += n->pins.size(); }
        }
        total += nl.ground_node.pins.size();
        return total;
    };

    auto count_models_by_name = [&]() {
        struct key_hash
        {
            std::size_t operator()(::fast_io::u8string const& s) const noexcept
            {
                return std::hash<std::string_view>{}(std::string_view(reinterpret_cast<char const*>(s.data()), s.size()));
            }
        };
        struct key_eq
        {
            bool operator()(::fast_io::u8string const& a, ::fast_io::u8string const& b) const noexcept { return a == b; }
        };

        std::unordered_map<::fast_io::u8string, std::size_t, key_hash, key_eq> counts{};
        counts.reserve(64);
        for(auto const& blk : nl.models)
        {
            for(auto const* m = blk.begin; m != blk.curr; ++m)
            {
                if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }
                counts[::fast_io::u8string{m->ptr->get_model_name()}] += 1;
            }
        }
        return counts;
    };

    // Analyze once so `pe_to_pl` sees a linked netlist.
    if(!c.analyze()) { return 14; }

    // Export PE->PL (.sav) with deterministic IO placement:
    // row1: a[0..15], row2: b[0..15], row3: op[0..1], row4: y[0..15]
    {
        ::phy_engine::phy_lab_wrapper::pe_to_pl::options popt{};
        popt.fixed_pos = {0.0, 0.0, 0.0};
        popt.generate_wires = true;
        popt.keep_pl_macros = true;

        auto const row_y_a = 1.0;
        auto const row_y_b = 0.5;
        auto const row_y_op = 0.0;
        auto const row_y_y = -0.5;

        auto const x_for_bit = [](std::size_t idx, std::size_t n) -> double {
            if(n <= 1) { return 0.0; }
            double const t = static_cast<double>(idx) / static_cast<double>(n - 1);
            return -1.0 + 2.0 * t;
        };

        popt.element_placer = [&](::phy_engine::phy_lab_wrapper::pe_to_pl::options::placement_context const& ctx) -> std::optional<position> {
            if(ctx.pl_model_id == pl_model_id::logic_input)
            {
                if(auto idx = parse_bit_index(ctx.pe_instance_name, "a"); idx && *idx < kA)
                {
                    return position{x_for_bit(*idx, kA), row_y_a, 0.0};
                }
                if(auto idx = parse_bit_index(ctx.pe_instance_name, "b"); idx && *idx < kB)
                {
                    return position{x_for_bit(*idx, kB), row_y_b, 0.0};
                }
                if(auto idx = parse_bit_index(ctx.pe_instance_name, "op"); idx && *idx < kOP)
                {
                    return position{x_for_bit(*idx, kOP), row_y_op, 0.0};
                }
            }

            if(ctx.pl_model_id == pl_model_id::logic_output)
            {
                if(auto idx = parse_bit_index(ctx.pe_instance_name, "y"); idx && *idx < kY)
                {
                    return position{x_for_bit(*idx, kY), row_y_y, 0.0};
                }
            }

            return std::nullopt;
        };

        auto r = ::phy_engine::phy_lab_wrapper::pe_to_pl::convert(nl, popt);
        assert(!r.ex.wires().empty());

        auto const out_path = std::filesystem::path("fp16_fpu_pe_to_pl.sav");
        r.ex.save(out_path, 2);
        if(!std::filesystem::exists(out_path)) { return 15; }
        if(std::filesystem::file_size(out_path) < 128) { return 16; }

        std::printf("PE netlist scale (fp16_fpu_top):\n");
        std::printf("- models: %zu\n", count_models());
        std::printf("- nodes: %zu (+ ground)\n", count_nodes());
        std::printf("- node pins (sum): %zu\n", count_pins());

        auto const counts = count_models_by_name();
        auto get = [&](char const* name) -> std::size_t {
            auto const u8 = ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(name), std::strlen(name)};
            if(auto it = counts.find(::fast_io::u8string{u8}); it != counts.end()) { return it->second; }
            return 0;
        };
        std::printf("- gates: YES=%zu NOT=%zu AND=%zu OR=%zu XOR=%zu XNOR=%zu\n",
                    get("YES"),
                    get("NOT"),
                    get("AND"),
                    get("OR"),
                    get("XOR"),
                    get("XNOR"));
        std::printf("- macros: HALF_ADDER=%zu FULL_ADDER=%zu\n", get("HALF_ADDER"), get("FULL_ADDER"));
        std::printf("PLSAV scale (fp16_fpu_pe_to_pl.sav):\n");
        std::printf("- elements: %zu\n", static_cast<std::size_t>(r.ex.elements().size()));
        std::printf("- wires: %zu\n", static_cast<std::size_t>(r.ex.wires().size()));
        std::printf("- file_size: %zu bytes\n", static_cast<std::size_t>(std::filesystem::file_size(out_path)));
    }

    return 0;
}
