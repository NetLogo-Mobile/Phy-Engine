#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/digital.h>
#include <phy_engine/verilog/digital/pe_synth.h>

#include <phy_engine/model/models/digital/logical/input.h>
#include <phy_engine/model/models/digital/logical/output.h>
#include <phy_engine/netlist/operation.h>

#include <phy_engine/phy_lab_wrapper/pe_to_pl.h>
#include <phy_engine/phy_lab_wrapper/auto_layout/auto_layout.h>

#include <fast_io/fast_io.h>

#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <stdexcept>
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

static std::string read_file_text(std::filesystem::path const& path)
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

static bool include_resolver_fs(void* user, u8sv path, ::fast_io::u8string& out_text) noexcept
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

static std::optional<std::size_t> parse_bit_index(std::string_view s, std::string_view base)
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

    double effective_min_step = std::max(min_step, 1e-9);
    for(std::size_t guard{}; guard < 256 && !grid_ok(effective_min_step, effective_min_step); ++guard) { effective_min_step *= 1.02; }

    if(!grid_ok(aopt.step_x, aopt.step_y))
    {
        auto const safe = std::max(effective_min_step, 1e-6);
        ::fast_io::io::perr(::fast_io::err(),
                            "[0026.modules] warning: layout step too small; clamping step to ",
                            ::fast_io::mnp::fixed(safe),
                            " to fit max grid\n");
        aopt.step_x = safe;
        aopt.step_y = safe;
    }

    double const target = static_cast<double>(required_cells) * fill;
    for(std::size_t iter{}; iter < 200 && capacity(aopt.step_x, aopt.step_y) < target; ++iter)
    {
        double next_x = aopt.step_x * 0.95;
        double next_y = aopt.step_y * 0.95;
        next_x = std::max(next_x, effective_min_step);
        next_y = std::max(next_y, effective_min_step);
        if(!grid_ok(next_x, next_y)) { break; }
        aopt.step_x = next_x;
        aopt.step_y = next_y;
    }
}

static void reposition_io_elements(experiment& ex, group_layout const& inputs, group_layout const& outputs, double extent, bool generate_wires)
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

static std::optional<double> env_f64(char const* name) noexcept
{
    auto const* s = std::getenv(name);
    if(s == nullptr || *s == '\0') { return std::nullopt; }
    try
    {
        std::size_t idx{};
        double v = std::stod(std::string(s), &idx);
        if(idx != std::string(s).size()) { return std::nullopt; }
        if(!std::isfinite(v)) { return std::nullopt; }
        return v;
    }
    catch(...)
    {
        return std::nullopt;
    }
}

static bool export_one_plsav(std::filesystem::path const& out_path,
                             std::filesystem::path const& include_base_dir,
                             std::string_view top_name,
                             std::string const& verilog_src,
                             bool generate_wires,
                             ::phy_engine::phy_lab_wrapper::auto_layout::mode layout_mode,
                             double layout_step,
                             double layout_min_step,
                             double layout_fill,
                             bool layout_refine)
{
    using namespace ::phy_engine;
    using namespace ::phy_engine::verilog::digital;

    compile_options copt{};
    include_ctx ictx{.base_dir = include_base_dir};
    copt.preprocess.user = __builtin_addressof(ictx);
    copt.preprocess.include_resolver = include_resolver_fs;

    auto const src = u8sv{reinterpret_cast<char8_t const*>(verilog_src.data()), verilog_src.size()};
    auto cr = ::phy_engine::verilog::digital::compile(src, copt);
    if(!cr.errors.empty() || cr.modules.empty())
    {
        diagnostic_options dop{};
        dop.filename = u8"<in-memory>";
        auto const diag = format_compile_errors(cr, src, dop);
        if(!diag.empty()) { ::fast_io::io::perr(::fast_io::u8err(), u8sv{diag.data(), diag.size()}); }
        return false;
    }

    auto design = ::phy_engine::verilog::digital::build_design(std::move(cr));
    auto u8top = ::fast_io::u8string{u8sv{reinterpret_cast<char8_t const*>(top_name.data()), top_name.size()}};
    auto const* top_mod = ::phy_engine::verilog::digital::find_module(design, u8top);
    if(top_mod == nullptr)
    {
        auto const tn = std::string(top_name);
        ::fast_io::io::perr(::fast_io::err(),
                            "[0026.modules] error: top module not found: ",
                            ::fast_io::mnp::os_c_str(tn.c_str()),
                            "\n");
        return false;
    }

    auto top_inst = ::phy_engine::verilog::digital::elaborate(design, *top_mod);
    if(top_inst.mod == nullptr)
    {
        auto const tn = std::string(top_name);
        ::fast_io::io::perr(::fast_io::err(),
                            "[0026.modules] error: elaborate failed: ",
                            ::fast_io::mnp::os_c_str(tn.c_str()),
                            "\n");
        return false;
    }

    // Build a PE netlist with explicit IO models per port bit (same as src/verilog2plsav.cpp).
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
            if(m == nullptr || m->ptr == nullptr) { return false; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *ports[pi])) { return false; }
        }
        else if(p.dir == port_dir::output)
        {
            auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
            (void)pos;
            if(m == nullptr || m->ptr == nullptr) { return false; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *ports[pi])) { return false; }
        }
        else
        {
            auto const tn = std::string(top_name);
            ::fast_io::io::perr(::fast_io::err(),
                                "[0026.modules] error: unsupported port dir (inout): ",
                                ::fast_io::mnp::os_c_str(tn.c_str()),
                                "\n");
            return false;
        }
    }

    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt{
        .allow_inout = false,
        .allow_multi_driver = false,
        .assume_binary_inputs = true,
        .optimize_wires = true,
        .optimize_mul2 = true,
        .optimize_adders = true,
    };

    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, ports, &err, opt))
    {
        ::fast_io::io::perr(::fast_io::u8err(),
                            u8"[0026.modules] error: synthesize_to_pe_netlist failed: ",
                            u8sv{err.message.data(), err.message.size()},
                            u8"\n");
        return false;
    }

    auto const inputs = collect_groups(*top_inst.mod, port_dir::input);
    auto const outputs = collect_groups(*top_inst.mod, port_dir::output);

    ::phy_engine::phy_lab_wrapper::pe_to_pl::options popt{};
    popt.fixed_pos = {0.0, 0.0, 0.0};
    popt.generate_wires = generate_wires;
    popt.keep_pl_macros = true;

    constexpr double third = 1.0 / 3.0;
    constexpr double extent = 1.0;
    double const io_gap = generate_wires ? ::phy_engine::phy_lab_wrapper::element_xyz::y_unit : 0.0;
    double const in_y_min = generate_wires ? ((extent * third) + io_gap) : (extent * (1.0 / 16.0));
    double const in_y_max = extent;
    double const out_y_min = -extent;
    double const out_y_max = generate_wires ? ((-extent * third) - io_gap) : (-extent * (1.0 / 16.0));

    popt.element_placer = [&](::phy_engine::phy_lab_wrapper::pe_to_pl::options::placement_context const& ctx)
        -> std::optional<phy_engine::phy_lab_wrapper::position> {
        if(ctx.pl_model_id == ::phy_engine::phy_lab_wrapper::pl_model_id::logic_input)
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
            auto const x = x_for_bit_lsb_right(bit, width, -extent, extent);
            return position{x, y, 0.0};
        }

        if(ctx.pl_model_id == ::phy_engine::phy_lab_wrapper::pl_model_id::logic_output)
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

    auto r = ::phy_engine::phy_lab_wrapper::pe_to_pl::convert(nl, popt);

    for(auto const& e : r.ex.elements())
    {
        if(is_port_io_element(e, inputs, outputs))
        {
            r.ex.get_element(e.identifier()).set_participate_in_layout(false);
        }
        else
        {
            // Ensure internal elements participate (defensive).
            r.ex.get_element(e.identifier()).set_participate_in_layout(true);
        }
    }

    ::phy_engine::phy_lab_wrapper::auto_layout::options aopt{};
    aopt.layout_mode = layout_mode;
    aopt.respect_fixed_elements = true;
    aopt.small_element = {1, 1};
    aopt.big_element = {2, 2};
    aopt.margin_x = 1e-6;
    aopt.margin_y = 1e-6;
    aopt.step_x = layout_step;
    aopt.step_y = layout_step;

    auto const demand = estimate_required_cells(r.ex, aopt);

    ::phy_engine::phy_lab_wrapper::auto_layout::stats st{};
    for(std::size_t attempt{}; attempt < 10; ++attempt)
    {
        if(layout_refine) { refine_steps_to_fit_table(demand, generate_wires, layout_min_step, layout_fill, aopt); }
        reposition_io_elements(r.ex, inputs, outputs, extent, generate_wires);

        double const layout_y_min = generate_wires ? (-extent * third) : (-extent);
        double const layout_y_max = generate_wires ? (extent * third) : (extent);
        auto const corner0 = position{-extent, layout_y_min, 0.0};
        auto const corner1 = position{extent, layout_y_max, 0.0};

        st = ::phy_engine::phy_lab_wrapper::auto_layout::layout(r.ex, corner0, corner1, 0.0, aopt);
        if(st.skipped == 0 && st.layout_mode == aopt.layout_mode) { break; }
        if(layout_refine)
        {
            aopt.step_x = std::max(aopt.step_x * 0.92, layout_min_step);
            aopt.step_y = std::max(aopt.step_y * 0.92, layout_min_step);
        }
    }

    r.ex.save(out_path, 2);
    return std::filesystem::exists(out_path);
}

static std::string concat_sources(std::vector<std::filesystem::path> const& files)
{
    std::string out{};
    std::size_t total{};
    for(auto const& p : files) { total += read_file_text(p).size() + 1; }
    out.reserve(total);
    for(auto const& p : files)
    {
        out.append(read_file_text(p));
        out.push_back('\n');
    }
    return out;
}

struct module_job
{
    std::string top{};
    std::string out_name{};
    std::string src{};
};
}  // namespace

int main()
{
    // To avoid generating lots of files during CI/ctest runs, this tool does nothing unless enabled.
    // Usage:
    //   PHY_ENGINE_TRACE_0026_EXPORT_MODULE_SAVS=1 ./build_test/0026.8086/x86_16_multi_module_export_plsav_modules
    if(std::getenv("PHY_ENGINE_TRACE_0026_EXPORT_MODULE_SAVS") == nullptr) { return 0; }

    auto const dir = std::filesystem::path(__FILE__).parent_path();

    bool const generate_wires = (std::getenv("PHY_ENGINE_TRACE_0026_MODULE_SAVS_NO_WIRES") == nullptr);
    auto layout_step = env_f64("PHY_ENGINE_TRACE_0026_MODULE_SAVS_LAYOUT_STEP").value_or(0.01);
    auto layout_min_step = env_f64("PHY_ENGINE_TRACE_0026_MODULE_SAVS_LAYOUT_MIN_STEP").value_or(0.001);
    auto layout_fill = env_f64("PHY_ENGINE_TRACE_0026_MODULE_SAVS_LAYOUT_FILL").value_or(1.25);
    bool const layout_refine = (std::getenv("PHY_ENGINE_TRACE_0026_MODULE_SAVS_NO_REFINE") == nullptr);
    auto const layout_mode = ::phy_engine::phy_lab_wrapper::auto_layout::mode::fast;

    // Per-module debug exports (one top per .sav).
    std::vector<module_job> jobs{};
    jobs.reserve(32);

    auto add_leaf = [&](char const* top, char const* vfile) {
        module_job j{};
        j.top = top;
        j.out_name = std::string(top) + ".sav";
        j.src = read_file_text(dir / vfile);
        jobs.push_back(std::move(j));
    };

    add_leaf("pc8", "pc8.v");
    add_leaf("rom256x16", "rom256x16.v");
    add_leaf("ir16", "ir16.v");
    add_leaf("decode16", "decode16.v");
    add_leaf("control16", "control16.v");
    add_leaf("imm_ext8_to_16", "imm_ext8_to_16.v");
    add_leaf("regfile4x16", "regfile4x16.v");
    add_leaf("mux16", "mux16.v");
    add_leaf("flag1", "flag1.v");

    // ALU leaf blocks.
    add_leaf("alu16_addsub", "alu16_addsub.v");
    add_leaf("alu16_sub_decode", "alu16_sub_decode.v");
    add_leaf("alu16_and", "alu16_and.v");
    add_leaf("alu16_or", "alu16_or.v");
    add_leaf("alu16_xor", "alu16_xor.v");
    add_leaf("alu16_mov", "alu16_mov.v");
    add_leaf("alu16_shl", "alu16_shl.v");
    add_leaf("alu16_shr", "alu16_shr.v");
    add_leaf("alu16_select", "alu16_select.v");

    // ALU top: needs all submodule definitions in one compilation unit.
    {
        module_job j{};
        j.top = "alu16";
        j.out_name = "alu16.sav";
        j.src = concat_sources({
            dir / "alu16_addsub.v",
            dir / "alu16_sub_decode.v",
            dir / "alu16_and.v",
            dir / "alu16_or.v",
            dir / "alu16_xor.v",
            dir / "alu16_mov.v",
            dir / "alu16_shl.v",
            dir / "alu16_shr.v",
            dir / "alu16_select.v",
            dir / "alu16.v",
        });
        jobs.push_back(std::move(j));
    }

    // Write into a dedicated folder to keep the workspace tidy.
    auto out_dir = std::filesystem::path("0026.8086.modules");
    std::filesystem::create_directories(out_dir);

    std::size_t ok{};
    for(auto const& j : jobs)
    {
        auto out_path = out_dir / j.out_name;
        bool const r = export_one_plsav(out_path,
                                        dir,
                                        j.top,
                                        j.src,
                                        generate_wires,
                                        layout_mode,
                                        layout_step,
                                        layout_min_step,
                                        layout_fill,
                                        layout_refine);
        if(r)
        {
            ::fast_io::io::perr(::fast_io::err(),
                                "[0026.modules] wrote ",
                                ::fast_io::mnp::os_c_str(out_path.string().c_str()),
                                " (top=",
                                ::fast_io::mnp::os_c_str(j.top.c_str()),
                                ")\n");
        }
        else
        {
            ::fast_io::io::perr(::fast_io::err(),
                                "[0026.modules] failed ",
                                ::fast_io::mnp::os_c_str(out_path.string().c_str()),
                                " (top=",
                                ::fast_io::mnp::os_c_str(j.top.c_str()),
                                ")\n");
        }
        ok += static_cast<std::size_t>(r);
    }

    if(ok != jobs.size())
    {
        ::fast_io::io::perr(::fast_io::err(), "[0026.modules] error: ", ok, "/", jobs.size(), " exports succeeded\n");
        return 2;
    }
    return 0;
}
