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
#include <cstdlib>
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
                                      double min_step,
                                      double fill,
                                      ::phy_engine::phy_lab_wrapper::auto_layout::options& aopt)
{
    if(required_cells == 0) { return; }
    if(!std::isfinite(aopt.step_x) || !std::isfinite(aopt.step_y) || aopt.step_x <= 0.0 || aopt.step_y <= 0.0) { return; }

    // The "desk" bounds are fixed to x/y in [-1, 1]. If wires are enabled, the middle third is the layout region.
    constexpr double table_extent = 1.0;
    constexpr double third = 1.0 / 3.0;
    constexpr std::size_t kMaxGridCells = 2'000'000;
    if(!std::isfinite(fill) || fill < 1.0) { fill = 1.0; }
    if(!std::isfinite(min_step) || min_step <= 0.0) { min_step = 0.001; }

    double const span_x = 2.0 * table_extent;
    double const span_y = generate_wires ? (2.0 * table_extent * third) : (2.0 * table_extent);

    auto capacity = [&](double step_x, double step_y) -> double {
        if(!(step_x > 0.0) || !(step_y > 0.0)) { return 0.0; }
        auto const w = std::floor(span_x / step_x + 1e-12) + 1.0;
        auto const h = std::floor(span_y / step_y + 1e-12) + 1.0;
        return w * h;
    };

    auto grid_ok = [&](double step_x, double step_y) -> bool {
        if(!(step_x > 0.0) || !(step_y > 0.0)) { return false; }
        auto const w = static_cast<std::size_t>(std::floor(span_x / step_x + 1e-12)) + 1;
        auto const h = static_cast<std::size_t>(std::floor(span_y / step_y + 1e-12)) + 1;
        if(w == 0 || h == 0) { return false; }
        if(w > (kMaxGridCells / h)) { return false; }
        return true;
    };

    // Effective minimum step must also respect the max grid size.
    double effective_min_step = std::max(min_step, 1e-9);
    for(std::size_t guard{}; guard < 256 && !grid_ok(effective_min_step, effective_min_step); ++guard)
    {
        // Increase slightly until it fits (covers floor(+1) effects).
        effective_min_step *= 1.02;
    }

    if(!grid_ok(aopt.step_x, aopt.step_y))
    {
        auto const safe = std::max(effective_min_step, 1e-6);
        ::fast_io::io::perr(::fast_io::err(),
                            "[verilog2plsav] warning: layout step too small; clamping step to ",
                            ::fast_io::mnp::fixed(safe),
                            " to fit max grid\n");
        aopt.step_x = safe;
        aopt.step_y = safe;
    }

    double const target = static_cast<double>(required_cells) * fill;
    // Reduce step sizes until there is enough grid capacity (or until we hit a reasonable minimum).
    for(std::size_t iter{}; iter < 200 && capacity(aopt.step_x, aopt.step_y) < target; ++iter)
    {
        double next_x = aopt.step_x * 0.95;
        double next_y = aopt.step_y * 0.95;

        // Keep a low but safe minimum; finer grid -> more subcells to avoid pile-up.
        next_x = std::max(next_x, effective_min_step);
        next_y = std::max(next_y, effective_min_step);

        if(!grid_ok(next_x, next_y))
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
    ::fast_io::io::perr(::fast_io::err(),
                        "usage: ",
                        ::fast_io::mnp::os_c_str(argv0),
                        " OUT.sav IN.v [--top TOP_MODULE]\n"
                        "  - Compiles Verilog (subset), synthesizes to PE netlist with optimizations,\n"
                        "    then exports PhysicsLab .sav with IO auto-placement and auto-layout.\n"
                        "options:\n"
                        "  -O0|-O1|-O2|-O3|-O4|-Omax                  PE synth optimization level (default: O0)\n"
                        "  --opt-level N                             PE synth optimization level (0..4)\n"
                        "  --opt-timeout-ms MS                        Omax: wall-clock budget (0 disables; default: 0)\n"
                        "  --opt-max-iter N                           Omax: max restarts/tries (default: 32)\n"
                        "  --opt-randomize                            Omax: enable randomized search variants (default: off)\n"
                        "  --opt-rand-seed SEED                       Omax: RNG seed (default: 1)\n"
                        "  --opt-allow-regress                        Omax: allow non-improving tries to be kept (default: off)\n"
                        "  --opt-verify                               Omax: verify candidate netlists (comb-only; default: off)\n"
                        "  --opt-verify-exact-max-inputs N            Omax: exhaustive verify threshold (default: 12)\n"
                        "  --opt-verify-rand-vectors N                Omax: random vectors when not exhaustive (default: 256)\n"
                        "  --opt-verify-seed SEED                     Omax: verify RNG seed (default: 1)\n"
                        "  --opt-cost gate|weighted                   Omax: objective cost model (default: gate)\n"
                        "  --opt-weight-NOT N                         Omax: weighted cost (default: 1)\n"
                        "  --opt-weight-AND N                         Omax: weighted cost (default: 1)\n"
                        "  --opt-weight-OR N                          Omax: weighted cost (default: 1)\n"
                        "  --opt-weight-XOR N                         Omax: weighted cost (default: 1)\n"
                        "  --opt-weight-XNOR N                        Omax: weighted cost (default: 1)\n"
                        "  --opt-weight-NAND N                        Omax: weighted cost (default: 1)\n"
                        "  --opt-weight-NOR N                         Omax: weighted cost (default: 1)\n"
                        "  --opt-weight-IMP N                         Omax: weighted cost (default: 1)\n"
                        "  --opt-weight-NIMP N                        Omax: weighted cost (default: 1)\n"
                        "  --opt-weight-YES N                         Omax: weighted cost (default: 1)\n"
                        "  --qm-max-vars N                            Two-level minimize budget (0 disables; default: 10)\n"
                        "  --qm-max-gates N                           Two-level minimize budget (default: 64)\n"
                        "  --qm-max-primes N                          Two-level minimize budget (default: 4096)\n"
                        "  --qm-max-minterms N                        Two-level minimize budget (0 disables; default: 0)\n"
                        "  --resub-max-vars N                         Resub budget (0 disables; default: 6)\n"
                        "  --resub-max-gates N                        Resub budget (default: 64)\n"
                        "  --sweep-max-vars N                         Sweep budget (0 disables; default: 6)\n"
                        "  --sweep-max-gates N                        Sweep budget (default: 64)\n"
                        "  --rewrite-max-candidates N                 Rewrite budget (0 disables; default: 0)\n"
                        "  --max-total-nodes N                        Global growth guard (0 disables)\n"
                        "  --max-total-models N                       Global growth guard (0 disables)\n"
                        "  --max-total-logic-gates N                  Global growth guard (0 disables)\n"
                        "  --assume-binary-inputs                    Treat X/Z as absent in synth (default: on)\n"
                        "  --no-assume-binary-inputs                 Preserve X-propagation logic\n"
                        "  --opt-wires|--no-opt-wires                Enable/disable YES buffer elimination (default: on)\n"
                        "  --opt-mul2|--no-opt-mul2                  Enable/disable MUL2 macro recognition (default: on)\n"
                        "  --opt-adders|--no-opt-adders              Enable/disable adder macro recognition (default: on)\n"
                        "  --layout fast|cluster|spectral|hier|force   Layout algorithm (default: fast)\n"
                        "  --layout3d xy|hier|force|pack               Use 3D layout variant (z step is fixed at 0.02)\n"
                        "  --layout3d-z-base Z                         3D layout base Z (default: 0; b/c start at 0.02)\n"
                        "  --no-wires                                 Disable auto wire generation\n"
                        "  --layout-step STEP                         Layout grid step (default: 0.01)\n"
                        "  --layout-min-step STEP                     Minimum step for auto-refine (default: 0.001)\n"
                        "  --layout-fill FACTOR                       Capacity safety factor (>=1, default: 1.25)\n"
                        "  --no-layout-refine                         Disable step auto-refinement\n"
                        "  --cluster-max-nodes N                      Cluster macro max nodes (default: 24)\n"
                        "  --cluster-channel-spacing N                Cluster macro spacing in cells (default: 2)\n");
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

static std::optional<std::size_t> parse_size(std::string const& s);

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
    // -O0 .. -O4 / -Omax
    for(int i = 1; i < argc; ++i)
    {
        auto const a = std::string_view(argv[i]);
        if(a == "-Omax" || a == "--Omax") { lvl = 4; }
        if(a.size() == 3 && a[0] == '-' && a[1] == 'O')
        {
            char const d = a[2];
            if(d >= '0' && d <= '4') { lvl = static_cast<std::uint8_t>(d - '0'); }
        }
    }
    // --opt-level N (overrides if present later)
    if(auto s = arg_after(argc, argv, "--opt-level"))
    {
        if(auto n = parse_size(*s); n && *n <= 4u) { lvl = static_cast<std::uint8_t>(*n); }
        else { return std::nullopt; }
    }
    return lvl ? lvl : std::optional<std::uint8_t>{static_cast<std::uint8_t>(0)};
}

static std::optional<double> parse_double(std::string const& s)
{
    try
    {
        std::size_t idx{};
        double v = std::stod(s, &idx);
        if(idx != s.size()) { return std::nullopt; }
        if(!std::isfinite(v)) { return std::nullopt; }
        return v;
    }
    catch(...)
    {
        return std::nullopt;
    }
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

enum class layout3d_kind : int
{
    xy = 0,     // 2D layout + z step per call
    hier = 1,   // hierarchical layering in z
    force = 2,  // 3D force-directed
    pack = 3,   // packing in xy + z step per call
};

static std::optional<layout3d_kind> parse_layout3d_kind(std::optional<std::string> const& s)
{
    if(!s) { return std::nullopt; }
    auto const& v = *s;
    if(v == "xy" || v == "XY" || v == "lift2d" || v == "0") { return layout3d_kind::xy; }
    if(v == "hier" || v == "hierarchical" || v == "layer" || v == "layers" || v == "1") { return layout3d_kind::hier; }
    if(v == "force" || v == "force3d" || v == "2") { return layout3d_kind::force; }
    if(v == "pack" || v == "packing" || v == "3") { return layout3d_kind::pack; }

    // Legacy aliases (kept for compatibility).
    if(v == "a" || v == "A" || v == "a3d" || v == "A3D") { return layout3d_kind::xy; }
    if(v == "b" || v == "B" || v == "b3d" || v == "B3D") { return layout3d_kind::hier; }
    if(v == "c" || v == "C" || v == "c3d" || v == "C3D") { return layout3d_kind::force; }
    if(v == "d" || v == "D" || v == "d3d" || v == "D3D") { return layout3d_kind::pack; }
    return std::nullopt;
}

static char const* layout3d_name(layout3d_kind k) noexcept
{
    switch(k)
    {
        case layout3d_kind::xy: return "xy";
        case layout3d_kind::hier: return "hier";
        case layout3d_kind::force: return "force";
        case layout3d_kind::pack: return "pack";
        default: return "unknown";
    }
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

    try
    {
    auto out_path = std::filesystem::path(argv[1]);
    auto in_path = std::filesystem::path(argv[2]);
    auto top_override = arg_after(argc, argv, "--top");

    auto const opt_level = parse_opt_level(argc, argv);
    if(!opt_level)
    {
        ::fast_io::io::perr(::fast_io::err(), "error: invalid -O* / --opt-level\n");
        return 10;
    }
    bool assume_binary_inputs = true;
    bool opt_wires = true;
    bool opt_mul2 = true;
    bool opt_adders = true;
    if(auto v = parse_toggle(argc, argv, "--assume-binary-inputs", "--no-assume-binary-inputs")) { assume_binary_inputs = *v; }
    if(auto v = parse_toggle(argc, argv, "--opt-wires", "--no-opt-wires")) { opt_wires = *v; }
    if(auto v = parse_toggle(argc, argv, "--opt-mul2", "--no-opt-mul2")) { opt_mul2 = *v; }
    if(auto v = parse_toggle(argc, argv, "--opt-adders", "--no-opt-adders")) { opt_adders = *v; }

    // Omax / budget knobs (also affect O3 since they are per-pass/global budgets).
    std::size_t omax_timeout_ms = 0;
    if(auto s = arg_after(argc, argv, "--opt-timeout-ms"))
    {
        auto v = parse_size(*s);
        if(!v) { ::fast_io::io::perr(::fast_io::err(), "error: invalid --opt-timeout-ms\n"); return 11; }
        omax_timeout_ms = *v;
    }

    std::size_t omax_max_iter = 32;
    if(auto s = arg_after(argc, argv, "--opt-max-iter"))
    {
        auto v = parse_size(*s);
        if(!v) { ::fast_io::io::perr(::fast_io::err(), "error: invalid --opt-max-iter\n"); return 11; }
        omax_max_iter = *v;
    }

    bool const omax_randomize = has_flag(argc, argv, "--opt-randomize");
    std::uint64_t omax_rand_seed = 1;
    if(auto s = arg_after(argc, argv, "--opt-rand-seed"))
    {
        auto v = parse_size(*s);
        if(!v) { ::fast_io::io::perr(::fast_io::err(), "error: invalid --opt-rand-seed\n"); return 11; }
        omax_rand_seed = static_cast<std::uint64_t>(*v);
    }
    bool const omax_allow_regress = has_flag(argc, argv, "--opt-allow-regress");

    bool const omax_verify = has_flag(argc, argv, "--opt-verify");
    std::size_t omax_verify_exact_max_inputs = 12;
    if(auto s = arg_after(argc, argv, "--opt-verify-exact-max-inputs"))
    {
        auto v = parse_size(*s);
        if(!v) { ::fast_io::io::perr(::fast_io::err(), "error: invalid --opt-verify-exact-max-inputs\n"); return 11; }
        omax_verify_exact_max_inputs = *v;
    }
    std::size_t omax_verify_random_vectors = 256;
    if(auto s = arg_after(argc, argv, "--opt-verify-rand-vectors"))
    {
        auto v = parse_size(*s);
        if(!v) { ::fast_io::io::perr(::fast_io::err(), "error: invalid --opt-verify-rand-vectors\n"); return 11; }
        omax_verify_random_vectors = *v;
    }
    std::uint64_t omax_verify_seed = 1;
    if(auto s = arg_after(argc, argv, "--opt-verify-seed"))
    {
        auto v = parse_size(*s);
        if(!v) { ::fast_io::io::perr(::fast_io::err(), "error: invalid --opt-verify-seed\n"); return 11; }
        omax_verify_seed = static_cast<std::uint64_t>(*v);
    }

    auto omax_cost = ::phy_engine::verilog::digital::pe_synth_options::omax_cost_model::gate_count;
    if(auto s = arg_after(argc, argv, "--opt-cost"))
    {
        if(*s == "gate" || *s == "gate_count" || *s == "0") { omax_cost = ::phy_engine::verilog::digital::pe_synth_options::omax_cost_model::gate_count; }
        else if(*s == "weighted" || *s == "weighted_gate_count" || *s == "1")
        {
            omax_cost = ::phy_engine::verilog::digital::pe_synth_options::omax_cost_model::weighted_gate_count;
        }
        else
        {
            ::fast_io::io::perr(::fast_io::err(), "error: invalid --opt-cost (use gate|weighted)\n");
            return 11;
        }
    }

    auto w_not = std::uint16_t{1};
    auto w_and = std::uint16_t{1};
    auto w_or = std::uint16_t{1};
    auto w_xor = std::uint16_t{1};
    auto w_xnor = std::uint16_t{1};
    auto w_nand = std::uint16_t{1};
    auto w_nor = std::uint16_t{1};
    auto w_imp = std::uint16_t{1};
    auto w_nimp = std::uint16_t{1};
    auto w_yes = std::uint16_t{1};
    auto parse_w16 = [&](char const* flag, std::uint16_t& out) -> bool
    {
        if(auto s = arg_after(argc, argv, flag))
        {
            auto v = parse_size(*s);
            if(!v || *v > 65535u) { return false; }
            out = static_cast<std::uint16_t>(*v);
        }
        return true;
    };
    if(!parse_w16("--opt-weight-NOT", w_not) || !parse_w16("--opt-weight-AND", w_and) || !parse_w16("--opt-weight-OR", w_or) ||
       !parse_w16("--opt-weight-XOR", w_xor) || !parse_w16("--opt-weight-XNOR", w_xnor) || !parse_w16("--opt-weight-NAND", w_nand) ||
       !parse_w16("--opt-weight-NOR", w_nor) || !parse_w16("--opt-weight-IMP", w_imp) || !parse_w16("--opt-weight-NIMP", w_nimp) ||
       !parse_w16("--opt-weight-YES", w_yes))
    {
        ::fast_io::io::perr(::fast_io::err(), "error: invalid --opt-weight-*\n");
        return 11;
    }

    // Per-pass/global budgets.
    auto parse_budget = [&](char const* flag, std::size_t& out) -> bool
    {
        if(auto s = arg_after(argc, argv, flag))
        {
            auto v = parse_size(*s);
            if(!v) { return false; }
            out = *v;
        }
        return true;
    };
    std::size_t qm_max_vars = 10, qm_max_gates = 64, qm_max_primes = 4096, qm_max_minterms = 0;
    std::size_t resub_max_vars = 6, resub_max_gates = 64;
    std::size_t sweep_max_vars = 6, sweep_max_gates = 64;
    std::size_t rewrite_max_candidates = 0;
    std::size_t max_total_nodes = 0, max_total_models = 0, max_total_logic_gates = 0;
    if(!parse_budget("--qm-max-vars", qm_max_vars) || !parse_budget("--qm-max-gates", qm_max_gates) || !parse_budget("--qm-max-primes", qm_max_primes) ||
       !parse_budget("--qm-max-minterms", qm_max_minterms) || !parse_budget("--resub-max-vars", resub_max_vars) || !parse_budget("--resub-max-gates", resub_max_gates) ||
       !parse_budget("--sweep-max-vars", sweep_max_vars) || !parse_budget("--sweep-max-gates", sweep_max_gates) ||
       !parse_budget("--rewrite-max-candidates", rewrite_max_candidates) || !parse_budget("--max-total-nodes", max_total_nodes) ||
       !parse_budget("--max-total-models", max_total_models) || !parse_budget("--max-total-logic-gates", max_total_logic_gates))
    {
        ::fast_io::io::perr(::fast_io::err(), "error: invalid budget option (expects an integer)\n");
        return 11;
    }
    auto layout_mode_arg = arg_after(argc, argv, "--layout");
    auto layout_mode = parse_layout_mode(layout_mode_arg);
    if(!layout_mode)
    {
        ::fast_io::io::perr(::fast_io::err(), "error: invalid --layout value\n");
        return 12;
    }

    auto layout3d_arg = arg_after(argc, argv, "--layout3d");
    auto layout3d = parse_layout3d_kind(layout3d_arg);
    if(layout3d_arg && !layout3d)
    {
        ::fast_io::io::perr(::fast_io::err(), "error: invalid --layout3d value\n");
        return 13;
    }
    auto layout3d_z_base_arg = arg_after(argc, argv, "--layout3d-z-base");
    double layout3d_z_base = 0.0;
    if(layout3d_z_base_arg)
    {
        auto v = parse_double(*layout3d_z_base_arg);
        if(!v)
        {
            ::fast_io::io::perr(::fast_io::err(), "error: invalid --layout3d-z-base value\n");
            return 14;
        }
        layout3d_z_base = *v;
    }

    bool const generate_wires = !has_flag(argc, argv, "--no-wires");
    bool const layout_refine = !has_flag(argc, argv, "--no-layout-refine");

    double layout_step = 0.01;
    if(auto s = arg_after(argc, argv, "--layout-step"))
    {
        auto v = parse_double(*s);
        if(!v || !(*v > 0.0))
        {
            ::fast_io::io::perr(::fast_io::err(), "error: invalid --layout-step\n");
            return 13;
        }
        layout_step = *v;
    }

    double layout_min_step = 0.001;
    if(auto s = arg_after(argc, argv, "--layout-min-step"))
    {
        auto v = parse_double(*s);
        if(!v || !(*v > 0.0))
        {
            ::fast_io::io::perr(::fast_io::err(), "error: invalid --layout-min-step\n");
            return 14;
        }
        layout_min_step = *v;
    }

    double layout_fill = 1.25;
    if(auto s = arg_after(argc, argv, "--layout-fill"))
    {
        auto v = parse_double(*s);
        if(!v || !(*v >= 1.0))
        {
            ::fast_io::io::perr(::fast_io::err(), "error: invalid --layout-fill (must be >= 1.0)\n");
            return 15;
        }
        layout_fill = *v;
    }

    std::size_t cluster_max_nodes = 24;
    if(auto s = arg_after(argc, argv, "--cluster-max-nodes"))
    {
        auto v = parse_size(*s);
        if(!v || *v == 0)
        {
            ::fast_io::io::perr(::fast_io::err(), "error: invalid --cluster-max-nodes\n");
            return 16;
        }
        cluster_max_nodes = *v;
    }

    std::size_t cluster_channel_spacing = 2;
    if(auto s = arg_after(argc, argv, "--cluster-channel-spacing"))
    {
        auto v = parse_size(*s);
        if(!v)
        {
            ::fast_io::io::perr(::fast_io::err(), "error: invalid --cluster-channel-spacing\n");
            return 17;
        }
        cluster_channel_spacing = *v;
    }

    using namespace phy_engine;
    using namespace phy_engine::verilog::digital;
    using namespace phy_engine::phy_lab_wrapper;

    auto const src_s = read_file_text(in_path);
    auto const src = u8sv{reinterpret_cast<char8_t const*>(src_s.data()), src_s.size()};

    compile_options copt{};
    include_ctx ictx{.base_dir = in_path.parent_path()};
    copt.preprocess.user = __builtin_addressof(ictx);
    copt.preprocess.include_resolver = include_resolver_fs;

    auto const in_path_s = in_path.string();
    ::fast_io::io::perr(::fast_io::err(), "[verilog2plsav] compile ", ::fast_io::mnp::os_c_str(in_path_s.c_str()), "\n");
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

    auto design = ::phy_engine::verilog::digital::build_design(::std::move(cr));
    auto const* top_mod = find_top_module(design, top_override);
    if(top_mod == nullptr) { return 3; }

    ::fast_io::io::perr(::fast_io::u8err(),
                        u8"[verilog2plsav] top=",
                        u8sv{top_mod->name.data(), top_mod->name.size()},
                        u8" (ports=",
                        top_mod->ports.size(),
                        u8")\n");

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
            ::fast_io::io::perr(::fast_io::err(), "error: unsupported port dir (inout)\n");
            return 9;
        }
    }

    // Synthesize to PE primitive netlist with optimizations.
    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt{};
    opt.allow_inout = false;
    opt.allow_multi_driver = false;
    opt.assume_binary_inputs = assume_binary_inputs;
    opt.opt_level = *opt_level;
    opt.optimize_wires = opt_wires;
    opt.optimize_mul2 = opt_mul2;
    opt.optimize_adders = opt_adders;

    opt.omax_timeout_ms = omax_timeout_ms;
    opt.omax_max_iter = omax_max_iter;
    opt.omax_randomize = omax_randomize;
    opt.omax_rand_seed = omax_rand_seed;
    opt.omax_allow_regress = omax_allow_regress;
    opt.omax_verify = omax_verify;
    opt.omax_verify_exact_max_inputs = omax_verify_exact_max_inputs;
    opt.omax_verify_random_vectors = omax_verify_random_vectors;
    opt.omax_verify_seed = omax_verify_seed;
    opt.omax_cost = omax_cost;
    opt.omax_gate_weights.not_w = w_not;
    opt.omax_gate_weights.and_w = w_and;
    opt.omax_gate_weights.or_w = w_or;
    opt.omax_gate_weights.xor_w = w_xor;
    opt.omax_gate_weights.xnor_w = w_xnor;
    opt.omax_gate_weights.nand_w = w_nand;
    opt.omax_gate_weights.nor_w = w_nor;
    opt.omax_gate_weights.imp_w = w_imp;
    opt.omax_gate_weights.nimp_w = w_nimp;
    opt.omax_gate_weights.yes_w = w_yes;

    opt.qm_max_vars = qm_max_vars;
    opt.qm_max_gates = qm_max_gates;
    opt.qm_max_primes = qm_max_primes;
    opt.qm_max_minterms = qm_max_minterms;
    opt.resub_max_vars = resub_max_vars;
    opt.resub_max_gates = resub_max_gates;
    opt.sweep_max_vars = sweep_max_vars;
    opt.sweep_max_gates = sweep_max_gates;
    opt.rewrite_max_candidates = rewrite_max_candidates;
    opt.max_total_nodes = max_total_nodes;
    opt.max_total_models = max_total_models;
    opt.max_total_logic_gates = max_total_logic_gates;

    ::fast_io::io::perr(::fast_io::err(), "[verilog2plsav] synthesize_to_pe_netlist\n");
    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, ports, &err, opt))
    {
        ::fast_io::io::perr(::fast_io::u8err(),
                            u8"error: synthesize_to_pe_netlist failed: ",
                            u8sv{err.message.data(), err.message.size()},
                            u8"\n");
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

    ::fast_io::io::perr(::fast_io::err(), "[verilog2plsav] pe_to_pl convert\n");
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
        aopt.layout_mode = *layout_mode;
        aopt.respect_fixed_elements = true;
        aopt.small_element = {1, 1};
        aopt.big_element = {2, 2};
        // Exclude boundary-placed IO elements from being treated as obstacles inside the layout grid.
        aopt.margin_x = 1e-6;
        aopt.margin_y = 1e-6;
        // Finer grid => more available subcells (prevents pile-up without enlarging the desk).
        aopt.step_x = layout_step;
        aopt.step_y = layout_step;
        if(aopt.layout_mode == ::phy_engine::phy_lab_wrapper::auto_layout::mode::cluster)
        {
            // More sub-blocks / macro tiles for a chip-like feel.
            aopt.cluster_max_nodes = cluster_max_nodes;
            aopt.cluster_channel_spacing = cluster_channel_spacing;
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

        if(layout3d)
        {
            ::fast_io::io::perr(::fast_io::err(),
                                "[verilog2plsav] note: using 3D layout (*_3d): ",
                                ::fast_io::mnp::os_c_str(layout3d_name(*layout3d)),
                                ", z_step=",
                                ::fast_io::mnp::fixed(::phy_engine::phy_lab_wrapper::auto_layout::z_step_3d),
                                "\n");
        }

        ::phy_engine::phy_lab_wrapper::auto_layout::stats st{};
        for(std::size_t attempt{}; attempt < 10; ++attempt)
        {
            // Keep everything within the fixed desk bounds: (-1,-1,0)~(1,1,0).
            // If the layout region is too small for the number of elements, refine the discretization step.
            if(layout_refine)
            {
                refine_steps_to_fit_table(demand, generate_wires, layout_min_step, layout_fill, aopt);
            }

            // Re-place IO (fixed bounds, but this keeps the partition consistent).
            reposition_io_elements(r.ex, inputs, outputs, extent, generate_wires);

            double const layout_y_min = generate_wires ? (-extent * third) : (-extent);
            double const layout_y_max = generate_wires ? (extent * third) : (extent);

            auto const corner0 = position{-extent, layout_y_min, 0.0};
            auto const corner1 = position{extent, layout_y_max, 0.0};
            if(layout3d)
            {
                // Keep z stable across retry attempts: retries are internal, not user-visible layout steps.
                double z_io = layout3d_z_base;
                double const z_base_bc = layout3d_z_base + ::phy_engine::phy_lab_wrapper::auto_layout::z_step_3d;

                switch(*layout3d)
                {
                    case layout3d_kind::xy:
                        st = ::phy_engine::phy_lab_wrapper::auto_layout::layout_a_3d(r.ex, corner0, corner1, z_io, aopt);
                        break;
                    case layout3d_kind::hier:
                        st = ::phy_engine::phy_lab_wrapper::auto_layout::layout_b_3d(r.ex, corner0, corner1, z_base_bc, aopt);
                        break;
                    case layout3d_kind::force:
                        st = ::phy_engine::phy_lab_wrapper::auto_layout::layout_c_3d(r.ex, corner0, corner1, z_base_bc, aopt);
                        break;
                    case layout3d_kind::pack:
                        st = ::phy_engine::phy_lab_wrapper::auto_layout::layout_d_3d(r.ex, corner0, corner1, z_io, aopt);
                        break;
                    default:
                        st = ::phy_engine::phy_lab_wrapper::auto_layout::layout(r.ex, corner0, corner1, 0.0, aopt);
                        break;
                }
            }
            else
            {
                st = ::phy_engine::phy_lab_wrapper::auto_layout::layout(r.ex, corner0, corner1, 0.0, aopt);
            }

            // If this mode fell back (e.g., cluster->fast) or there are still skipped elements, try again with smaller steps.
            if(layout3d)
            {
                if(st.skipped == 0) { break; }
            }
            else
            {
                if(st.skipped == 0 && st.layout_mode == aopt.layout_mode) { break; }
            }
            if(layout_refine)
            {
                aopt.step_x = std::max(aopt.step_x * 0.92, layout_min_step);
                aopt.step_y = std::max(aopt.step_y * 0.92, layout_min_step);
            }
        }

        ::fast_io::io::perr(::fast_io::err(),
                            "[verilog2plsav] layout=",
                            static_cast<int>(st.layout_mode),
                            " extent=",
                            ::fast_io::mnp::fixed(extent),
                            " step=(",
                            ::fast_io::mnp::fixed(aopt.step_x),
                            ",",
                            ::fast_io::mnp::fixed(aopt.step_y),
                            ") grid=",
                            st.grid_w,
                            "x",
                            st.grid_h,
                            " placed=",
                            st.placed,
                            " skipped=",
                            st.skipped,
                            " fixed=",
                            st.fixed_obstacles,
                            "\n");
    }

    r.ex.save(out_path, 2);

    if(!std::filesystem::exists(out_path))
    {
        auto const out_path_s = out_path.string();
        ::fast_io::io::perr(::fast_io::err(), "error: failed to write ", ::fast_io::mnp::os_c_str(out_path_s.c_str()), "\n");
        return 11;
    }
    {
        auto const out_path_s = out_path.string();
        ::fast_io::io::perr(::fast_io::err(), "[verilog2plsav] wrote ", ::fast_io::mnp::os_c_str(out_path_s.c_str()), "\n");
    }
    return 0;
    }
    catch(std::exception const& e)
    {
        ::fast_io::io::perr(::fast_io::err(), "error: ", ::fast_io::mnp::os_c_str(e.what()), "\n");
        return 99;
    }
}
