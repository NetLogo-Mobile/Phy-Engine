#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/digital.h>
#include <phy_engine/verilog/digital/pe_synth.h>

#include <phy_engine/model/models/digital/logical/input.h>
#include <phy_engine/model/models/digital/logical/output.h>
#include <phy_engine/netlist/operation.h>

#include <phy_engine/phy_lab_wrapper/pe_to_pl.h>
#include <phy_engine/phy_lab_wrapper/auto_layout/auto_layout.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace
{
using u8sv = ::fast_io::u8string_view;
using ::phy_engine::phy_lab_wrapper::experiment;
using ::phy_engine::phy_lab_wrapper::element;
using ::phy_engine::phy_lab_wrapper::position;

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

struct port_group
{
    std::string base{};
    ::phy_engine::verilog::digital::port_dir dir{};
    std::size_t width{};  // max_bit+1 for vectors; 1 for scalar
};

struct group_layout
{
    std::unordered_map<std::string, std::size_t> row_by_base{};
    std::unordered_map<std::string, std::size_t> width_by_base{};
    std::vector<std::string> order{};
};

static std::string base_name(std::string_view port_name)
{
    auto pos = port_name.find('[');
    if(pos == std::string_view::npos) { return std::string(port_name); }
    return std::string(port_name.substr(0, pos));
}

static group_layout collect_groups(::phy_engine::verilog::digital::compiled_module const& m, ::phy_engine::verilog::digital::port_dir dir)
{
    group_layout gl{};
    for(auto const& p : m.ports)
    {
        if(p.dir != dir) { continue; }
        std::string pn(reinterpret_cast<char const*>(p.name.data()), p.name.size());
        auto base = base_name(pn);
        if(gl.width_by_base.find(base) == gl.width_by_base.end())
        {
            gl.order.push_back(base);
            gl.width_by_base.emplace(base, 1);
        }
        if(auto idx = parse_bit_index(pn, base))
        {
            auto& w = gl.width_by_base[base];
            w = std::max(w, *idx + 1);
        }
    }
    gl.row_by_base.reserve(gl.order.size());
    for(std::size_t i{}; i < gl.order.size(); ++i) { gl.row_by_base.emplace(gl.order[i], i); }
    return gl;
}

static double row_center_y(std::size_t row, std::size_t nrows, double y_min, double y_max)
{
    if(nrows <= 1) { return 0.5 * (y_min + y_max); }
    double const t = static_cast<double>(row) / static_cast<double>(nrows - 1);
    return y_max - (y_max - y_min) * t;
}

static double x_for_bit_lsb_right(std::size_t bit, std::size_t width, double x_min, double x_max)
{
    if(width <= 1) { return 0.5 * (x_min + x_max); }
    auto const ridx = (width - 1) - bit;
    double const t = static_cast<double>(ridx) / static_cast<double>(width - 1);
    return x_min + (x_max - x_min) * t;
}

static bool is_io_model_id(std::string_view mid) noexcept
{
    return mid == ::phy_engine::phy_lab_wrapper::pl_model_id::logic_input ||
           mid == ::phy_engine::phy_lab_wrapper::pl_model_id::logic_output ||
           mid == ::phy_engine::phy_lab_wrapper::pl_model_id::eight_bit_input ||
           mid == ::phy_engine::phy_lab_wrapper::pl_model_id::eight_bit_display;
}

static bool is_port_io_element(element const& e, group_layout const& inputs, group_layout const& outputs)
{
    auto const mid = e.data().value("ModelID", "");
    if(!is_io_model_id(mid)) { return false; }

    auto it_label = e.data().find("Label");
    if(it_label == e.data().end() || !it_label->is_string()) { return false; }

    auto const pn = it_label->get<std::string>();
    auto const base = base_name(pn);

    if(mid == ::phy_engine::phy_lab_wrapper::pl_model_id::logic_input ||
       mid == ::phy_engine::phy_lab_wrapper::pl_model_id::eight_bit_input)
    {
        return inputs.row_by_base.find(base) != inputs.row_by_base.end();
    }
    if(mid == ::phy_engine::phy_lab_wrapper::pl_model_id::logic_output ||
       mid == ::phy_engine::phy_lab_wrapper::pl_model_id::eight_bit_display)
    {
        return outputs.row_by_base.find(base) != outputs.row_by_base.end();
    }
    return false;
}

static std::size_t estimate_required_cells(experiment const& ex, ::phy_engine::phy_lab_wrapper::auto_layout::options const& aopt)
{
    std::size_t demand{};
    for(auto const& e : ex.elements())
    {
        if(!e.participate_in_layout()) { continue; }
        auto const fp = e.is_big_element() ? aopt.big_element : aopt.small_element;
        demand += fp.w * fp.h;
    }
    return demand;
}

static void refine_steps_to_fit_table(std::size_t required_cells,
                                      bool generate_wires,
                                      ::phy_engine::phy_lab_wrapper::auto_layout::options& aopt)
{
    if(required_cells == 0) { return; }
    if(!std::isfinite(aopt.step_x) || !std::isfinite(aopt.step_y) || aopt.step_x <= 0.0 || aopt.step_y <= 0.0) { return; }

    // The "desk" bounds are fixed to x/y in [-1, 1]. If wires are enabled, the middle third is the layout region.
    constexpr double table_extent = 1.0;
    constexpr double third = 1.0 / 3.0;
    constexpr double fill = 1.25;  // packing overhead
    constexpr std::size_t kMaxGridCells = 2'000'000;

    double const span_x = 2.0 * table_extent;
    double const span_y = generate_wires ? (2.0 * table_extent * third) : (2.0 * table_extent);

    auto capacity = [&](double step_x, double step_y) -> double {
        if(!(step_x > 0.0) || !(step_y > 0.0)) { return 0.0; }
        auto const w = std::floor(span_x / step_x + 1e-12) + 1.0;
        auto const h = std::floor(span_y / step_y + 1e-12) + 1.0;
        return w * h;
    };

    double const target = static_cast<double>(required_cells) * fill;
    // Reduce step sizes until there is enough grid capacity (or until we hit a reasonable minimum).
    for(std::size_t iter{}; iter < 200 && capacity(aopt.step_x, aopt.step_y) < target; ++iter)
    {
        double next_x = aopt.step_x * 0.95;
        double next_y = aopt.step_y * 0.95;

        // Keep a low but safe minimum; finer grid -> more subcells to avoid pile-up.
        next_x = std::max(next_x, 0.001);
        next_y = std::max(next_y, 0.001);

        auto const w = static_cast<std::size_t>(std::floor(span_x / next_x + 1e-12)) + 1;
        auto const h = static_cast<std::size_t>(std::floor(span_y / next_y + 1e-12)) + 1;
        if(w > 0 && h > 0 && w > (kMaxGridCells / h))
        {
            // Cannot refine further without making the grid too large.
            break;
        }

        aopt.step_x = next_x;
        aopt.step_y = next_y;
    }
}

static void reposition_io_elements(experiment& ex,
                                   group_layout const& inputs,
                                   group_layout const& outputs,
                                   double extent,
                                   bool generate_wires)
{
    constexpr double third = 1.0 / 3.0;
    double const io_gap = generate_wires ? ::phy_engine::phy_lab_wrapper::element_xyz::y_unit : 0.0;

    double const in_y_min = generate_wires ? ((extent * third) + io_gap) : (extent * (1.0 / 16.0));
    double const in_y_max = extent;
    double const out_y_min = -extent;
    double const out_y_max = generate_wires ? ((-extent * third) - io_gap) : (-extent * (1.0 / 16.0));

    for(auto const& e : ex.elements())
    {
        auto const mid = e.data().value("ModelID", "");
        if(mid != ::phy_engine::phy_lab_wrapper::pl_model_id::logic_input && mid != ::phy_engine::phy_lab_wrapper::pl_model_id::logic_output)
        {
            continue;
        }

        auto it_label = e.data().find("Label");
        if(it_label == e.data().end() || !it_label->is_string()) { continue; }
        auto const pn = it_label->get<std::string>();
        auto const base = base_name(pn);

        if(mid == ::phy_engine::phy_lab_wrapper::pl_model_id::logic_input)
        {
            auto it_row = inputs.row_by_base.find(base);
            if(it_row == inputs.row_by_base.end()) { continue; }

            auto const row = it_row->second;
            auto const nrows = std::max<std::size_t>(1, inputs.order.size());
            auto const y = row_center_y(row, nrows, in_y_min, in_y_max);

            auto it_w = inputs.width_by_base.find(base);
            auto const width = (it_w == inputs.width_by_base.end()) ? 1 : it_w->second;
            std::size_t bit{};
            if(auto idx = parse_bit_index(pn, base)) { bit = *idx; }
            auto const x = x_for_bit_lsb_right(bit, width, -extent, extent);

            ex.get_element(e.identifier()).set_element_position(position{x, y, 0.0}, false);
            continue;
        }

        auto it_row = outputs.row_by_base.find(base);
        if(it_row == outputs.row_by_base.end()) { continue; }

        auto const row = it_row->second;
        auto const nrows = std::max<std::size_t>(1, outputs.order.size());
        auto const y = row_center_y(row, nrows, out_y_min, out_y_max);

        auto it_w = outputs.width_by_base.find(base);
        auto const width = (it_w == outputs.width_by_base.end()) ? 1 : it_w->second;
        std::size_t bit{};
        if(auto idx = parse_bit_index(pn, base)) { bit = *idx; }
        auto const x = x_for_bit_lsb_right(bit, width, -extent, extent);

        ex.get_element(e.identifier()).set_element_position(position{x, y, 0.0}, false);
    }
}

static void usage(char const* argv0)
{
    std::fprintf(stderr,
                 "usage: %s OUT.sav IN.v [--top TOP_MODULE]\n"
                 "  - Compiles Verilog (subset), synthesizes to PE netlist with optimizations,\n"
                 "    then exports PhysicsLab .sav with IO auto-placement and auto-layout.\n"
                 "options:\n"
                 "  --layout fast|cluster|spectral|hier|force   Layout algorithm (default: fast)\n"
                 "  --no-wires                                 Disable auto wire generation\n",
                 argv0);
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

static std::optional<phy_engine::phy_lab_wrapper::auto_layout::mode>
parse_layout_mode(std::optional<std::string> const& s)
{
    using phy_engine::phy_lab_wrapper::auto_layout::mode;
    if(!s) { return mode::fast; }
    auto const& v = *s;
    if(v == "fast" || v == "0") { return mode::fast; }
    if(v == "cluster" || v == "1") { return mode::cluster; }
    if(v == "spectral" || v == "2") { return mode::spectral; }
    if(v == "hier" || v == "hierarchical" || v == "3") { return mode::hierarchical; }
    if(v == "force" || v == "4") { return mode::force; }
    return std::nullopt;
}

static ::phy_engine::verilog::digital::compiled_module const*
find_top_module(::phy_engine::verilog::digital::compiled_design const& d, std::optional<std::string> const& top_override)
{
    if(top_override)
    {
        auto u8 = ::fast_io::u8string{u8sv{reinterpret_cast<char8_t const*>(top_override->data()), top_override->size()}};
        return ::phy_engine::verilog::digital::find_module(d, u8);
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
        // Fallback: last module.
        if(d.modules.empty()) { return nullptr; }
        return &d.modules.back();
    }

    // If multiple, prefer the one with the most ports (common "top" characteristic).
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

    auto out_path = std::filesystem::path(argv[1]);
    auto in_path = std::filesystem::path(argv[2]);
    auto top_override = arg_after(argc, argv, "--top");
    auto layout_mode_arg = arg_after(argc, argv, "--layout");
    auto layout_mode = parse_layout_mode(layout_mode_arg);
    if(!layout_mode)
    {
        std::fprintf(stderr, "error: invalid --layout value\n");
        return 12;
    }
    bool const generate_wires = !has_flag(argc, argv, "--no-wires");

    using namespace phy_engine;
    using namespace phy_engine::verilog::digital;
    using namespace phy_engine::phy_lab_wrapper;

    auto const src_s = read_file_text(in_path);
    auto const src = u8sv{reinterpret_cast<char8_t const*>(src_s.data()), src_s.size()};

    compile_options copt{};
    include_ctx ictx{.base_dir = in_path.parent_path()};
    copt.preprocess.user = __builtin_addressof(ictx);
    copt.preprocess.include_resolver = include_resolver_fs;

    std::fprintf(stderr, "[verilog2plsav] compile %s\n", in_path.string().c_str());
    auto cr = ::phy_engine::verilog::digital::compile(src, copt);
    if(!cr.errors.empty() || cr.modules.empty())
    {
        for(auto const& e : cr.errors)
        {
            std::fprintf(stderr,
                         "error: %.*s\n",
                         static_cast<int>(e.message.size()),
                         reinterpret_cast<char const*>(e.message.data()));
        }
        return 1;
    }

    auto design = ::phy_engine::verilog::digital::build_design(::std::move(cr));
    auto const* top_mod = find_top_module(design, top_override);
    if(top_mod == nullptr) { return 3; }

    std::fprintf(stderr,
                 "[verilog2plsav] top=%.*s (ports=%zu)\n",
                 static_cast<int>(top_mod->name.size()),
                 reinterpret_cast<char const*>(top_mod->name.data()),
                 top_mod->ports.size());

    auto top_inst = ::phy_engine::verilog::digital::elaborate(design, *top_mod);
    if(top_inst.mod == nullptr) { return 4; }

    // Build a PE netlist with explicit IO models per port bit.
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    c.get_analyze_setting().tr.t_step = 1e-9;
    c.get_analyze_setting().tr.t_stop = 1e-9;
    auto& nl = c.get_netlist();

    std::vector<::phy_engine::model::node_t*> ports{};
    ports.reserve(top_inst.mod->ports.size());
    for(std::size_t i{}; i < top_inst.mod->ports.size(); ++i)
    {
        auto& n = ::phy_engine::netlist::create_node(nl);
        ports.push_back(__builtin_addressof(n));
    }

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
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *ports[pi])) { return 6; }
        }
        else if(p.dir == port_dir::output)
        {
            auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
            (void)pos;
            if(m == nullptr || m->ptr == nullptr) { return 7; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *ports[pi])) { return 8; }
        }
        else
        {
            std::fprintf(stderr, "error: unsupported port dir (inout)\n");
            return 9;
        }
    }

    // Synthesize to PE primitive netlist with optimizations.
    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt{
        .allow_inout = false,
        .allow_multi_driver = false,
        .assume_binary_inputs = true,
        .optimize_wires = true,
        .optimize_mul2 = true,
        .optimize_adders = true,
    };

    std::fprintf(stderr, "[verilog2plsav] synthesize_to_pe_netlist\n");
    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, ports, &err, opt))
    {
        std::fprintf(stderr,
                     "error: synthesize_to_pe_netlist failed: %.*s\n",
                     static_cast<int>(err.message.size()),
                     reinterpret_cast<char const*>(err.message.data()));
        return 10;
    }

    auto const inputs = collect_groups(*top_inst.mod, port_dir::input);
    auto const outputs = collect_groups(*top_inst.mod, port_dir::output);

    // Export PE->PL (.sav): keep core elements at (0,0,0), place IO around it.
    ::phy_engine::phy_lab_wrapper::pe_to_pl::options popt{};
    popt.fixed_pos = {0.0, 0.0, 0.0};  // core stays here
    popt.generate_wires = generate_wires;
    popt.keep_pl_macros = true;

    // The export coordinate system uses a symmetric "extent" for convenience.
    // When wires are enabled, the top/bottom thirds are reserved for IO and the middle third is used for layout.
    constexpr double third = 1.0 / 3.0;
    constexpr double extent = 1.0;
    double const io_gap = generate_wires ? ::phy_engine::phy_lab_wrapper::element_xyz::y_unit : 0.0;
    double in_y_min = generate_wires ? ((extent * third) + io_gap) : (extent * (1.0 / 16.0));
    double in_y_max = extent;
    double out_y_min = -extent;
    double out_y_max = generate_wires ? ((-extent * third) - io_gap) : (-extent * (1.0 / 16.0));

    popt.element_placer = [&](::phy_engine::phy_lab_wrapper::pe_to_pl::options::placement_context const& ctx)
        -> std::optional<phy_engine::phy_lab_wrapper::position> {
        // Inputs: top rectangle (y in [1/16, 1]).
        if(ctx.pl_model_id == pl_model_id::logic_input)
        {
            std::string_view const pn = ctx.pe_instance_name;
            auto base = base_name(pn);
            auto it_row = inputs.row_by_base.find(base);
            if(it_row == inputs.row_by_base.end()) { return std::nullopt; }

            auto const row = it_row->second;
            auto const nrows = std::max<std::size_t>(1, inputs.order.size());
            auto const y = row_center_y(row, nrows, in_y_min, in_y_max);

            auto it_w = inputs.width_by_base.find(base);
            auto const width = (it_w == inputs.width_by_base.end()) ? 1 : it_w->second;
            std::size_t bit{};
            if(auto idx = parse_bit_index(pn, base)) { bit = *idx; }

            // LSB on the right: bit0 at +x, MSB at -x.
            auto const x = x_for_bit_lsb_right(bit, width, -extent, extent);
            return position{x, y, 0.0};
        }

        // Outputs: bottom rectangle (y in [-1, -1/16]).
        if(ctx.pl_model_id == pl_model_id::logic_output)
        {
            std::string_view const pn = ctx.pe_instance_name;
            auto base = base_name(pn);
            auto it_row = outputs.row_by_base.find(base);
            if(it_row == outputs.row_by_base.end()) { return std::nullopt; }

            auto const row = it_row->second;
            auto const nrows = std::max<std::size_t>(1, outputs.order.size());
            auto const y = row_center_y(row, nrows, out_y_min, out_y_max);

            auto it_w = outputs.width_by_base.find(base);
            auto const width = (it_w == outputs.width_by_base.end()) ? 1 : it_w->second;
            std::size_t bit{};
            if(auto idx = parse_bit_index(pn, base)) { bit = *idx; }

            auto const x = x_for_bit_lsb_right(bit, width, -extent, extent);
            return position{x, y, 0.0};
        }

        return std::nullopt;
    };

    std::fprintf(stderr, "[verilog2plsav] pe_to_pl convert\n");
    auto r = ::phy_engine::phy_lab_wrapper::pe_to_pl::convert(nl, popt);

    // Keep IO elements fixed for layout.
    for(auto const& e : r.ex.elements())
    {
        if(is_port_io_element(e, inputs, outputs))
        {
            r.ex.get_element(e.identifier()).set_participate_in_layout(false);
        }
    }

    // Auto-layout internal elements into the requested region.
    {
        ::phy_engine::phy_lab_wrapper::auto_layout::options aopt{};
        aopt.mode = *layout_mode;
        aopt.respect_fixed_elements = true;
        aopt.small_element = {1, 1};
        aopt.big_element = {2, 2};
        // Exclude boundary-placed IO elements from being treated as obstacles inside the layout grid.
        aopt.margin_x = 1e-6;
        aopt.margin_y = 1e-6;
        // Finer grid => more available subcells (prevents pile-up without enlarging the desk).
        aopt.step_x = 0.01;
        aopt.step_y = 0.01;
        if(aopt.mode == ::phy_engine::phy_lab_wrapper::auto_layout::mode::cluster)
        {
            // More sub-blocks / macro tiles for a chip-like feel.
            aopt.cluster_max_nodes = 24;
            aopt.cluster_channel_spacing = 2;
        }

        // Defensive: if some converter path ever marks internal elements as non-participating, undo that here.
        for(auto const& e : r.ex.elements())
        {
            auto const mid = e.data().value("ModelID", "");
            if(is_port_io_element(e, inputs, outputs)) { continue; }
            if(!e.participate_in_layout()) { r.ex.get_element(e.identifier()).set_participate_in_layout(true); }
        }

        // Auto-scale extent so the middle-third layout region has enough capacity. Otherwise, skipped elements
        // stay at `fixed_pos` (0,0,0) and pile up visually.
        auto const demand = estimate_required_cells(r.ex, aopt);

        ::phy_engine::phy_lab_wrapper::auto_layout::stats st{};
        for(std::size_t attempt{}; attempt < 10; ++attempt)
        {
            // Keep everything within the fixed desk bounds: (-1,-1,0)~(1,1,0).
            // If the layout region is too small for the number of elements, refine the discretization step.
            refine_steps_to_fit_table(demand, generate_wires, aopt);

            // Re-place IO (fixed bounds, but this keeps the partition consistent).
            reposition_io_elements(r.ex, inputs, outputs, extent, generate_wires);

            double const layout_y_min = generate_wires ? (-extent * third) : (-extent);
            double const layout_y_max = generate_wires ? (extent * third) : (extent);

            st = ::phy_engine::phy_lab_wrapper::auto_layout::layout(r.ex,
                                                                    position{-extent, layout_y_min, 0.0},
                                                                    position{extent, layout_y_max, 0.0},
                                                                    0.0,
                                                                    aopt);

            // If this mode fell back (e.g., cluster->fast) or there are still skipped elements, try again with smaller steps.
            if(st.skipped == 0 && st.mode == aopt.mode) { break; }
            aopt.step_x *= 0.92;
            aopt.step_y *= 0.92;
        }

        std::fprintf(stderr,
                     "[verilog2plsav] layout=%d extent=%.3f step=(%.4f,%.4f) grid=%zux%zu placed=%zu skipped=%zu fixed=%zu\n",
                     static_cast<int>(st.mode),
                     extent,
                     aopt.step_x,
                     aopt.step_y,
                     st.grid_w,
                     st.grid_h,
                     st.placed,
                     st.skipped,
                     st.fixed_obstacles);
    }

    r.ex.save(out_path, 2);

    if(!std::filesystem::exists(out_path))
    {
        std::fprintf(stderr, "error: failed to write %s\n", out_path.string().c_str());
        return 11;
    }
    std::fprintf(stderr, "[verilog2plsav] wrote %s\n", out_path.string().c_str());
    return 0;
}
