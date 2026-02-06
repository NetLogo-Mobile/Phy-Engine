#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/digital.h>
#include <phy_engine/verilog/digital/pe_synth.h>

#include <phy_engine/model/models/digital/logical/input.h>
#include <phy_engine/model/models/digital/logical/output.h>

#include <phy_engine/phy_lab_wrapper/auto_layout/auto_layout.h>
#include <phy_engine/phy_lab_wrapper/pe_to_pl.h>

#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace
{
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

std::optional<std::size_t> parse_bit_index(std::string_view s, std::string_view base)
{
    if(!s.starts_with(base)) { return std::nullopt; }
    if(s.size() < base.size() + 3) { return std::nullopt; } // "a[0]"
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
} // namespace

int main()
{
    using namespace phy_engine;
    using namespace phy_engine::phy_lab_wrapper;
    using namespace phy_engine::verilog::digital;

    static constexpr std::size_t kW = 9;
    static constexpr std::size_t kH = 9;

    auto const src_path = std::filesystem::path(__FILE__).parent_path() / "go9x9_lite.v";
    auto const src_s = read_file_text(src_path);
    auto const src = ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(src_s.data()), src_s.size()};

    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);
    auto& nl = c.get_netlist();

    auto cr = ::phy_engine::verilog::digital::compile(src);
    if(!cr.errors.empty() || cr.modules.empty())
    {
        if(!cr.errors.empty())
        {
            auto const& e = cr.errors.front_unchecked();
            throw std::runtime_error("verilog compile error at line " + std::to_string(e.line) + ": " +
                                     std::string(reinterpret_cast<char const*>(e.message.data()), e.message.size()));
        }
        return 1;
    }

    auto design = ::phy_engine::verilog::digital::build_design(::std::move(cr));
    auto const* mod = ::phy_engine::verilog::digital::find_module(design, u8"go9x9_lite");
    if(mod == nullptr) { return 2; }
    auto top_inst = ::phy_engine::verilog::digital::elaborate(design, *mod);
    if(top_inst.mod == nullptr) { return 3; }

    // Port nodes in module port order.
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
            if(m == nullptr || m->ptr == nullptr) { return 4; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *ports[pi])) { return 5; }
            continue;
        }

        if(p.dir == port_dir::output)
        {
            auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
            (void)pos;
            if(m == nullptr || m->ptr == nullptr) { return 6; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *ports[pi])) { return 7; }
            continue;
        }

        return 8;
    }

    // O4 synth.
    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt_o4{
        .allow_inout = false,
        .allow_multi_driver = false,
        .assume_binary_inputs = true,
        .opt_level = 4,
        .optimize_wires = true,
        .optimize_mul2 = true,
        .optimize_adders = true,
    };
    opt_o4.qm_max_vars = 0;
    opt_o4.resub_max_vars = 0;
    opt_o4.sweep_max_vars = 0;

    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, ports, &err, opt_o4))
    {
        throw std::runtime_error("pe_synth failed: " + std::string(reinterpret_cast<char const*>(err.message.data()), err.message.size()));
    }

    // Export PE->PL (.sav).
    {
        ::phy_engine::phy_lab_wrapper::pe_to_pl::options popt{};
        popt.fixed_pos = {0.0, 0.0, 0.0};
        popt.generate_wires = true;
        popt.keep_pl_macros = true;

        popt.element_placer = [&](::phy_engine::phy_lab_wrapper::pe_to_pl::options::placement_context const& ctx) -> std::optional<position> {
            // Table boundary is a square: x,z in [-1, 1]. Keep y=0.
            // - Logic outputs (row[0..8][0..8]) occupy the right half.
            // - Logic inputs (buttons) occupy the left half.

            if(ctx.pl_model_id == pl_model_id::logic_output)
            {
                if(ctx.pe_instance_name == "black") return position{0.7, 1.15, 0.0};
                if(ctx.pe_instance_name == "white") return position{0.85, 1.15, 0.0};

                for(std::size_t yy{}; yy < kH; ++yy)
                {
                    std::string base = "row" + std::to_string(yy);
                    if(auto xx = parse_bit_index(ctx.pe_instance_name, base); xx && *xx < kW)
                    {
                        auto const col = static_cast<double>(*xx);
                        auto const row = static_cast<double>(yy);
                        double const x = (kW <= 1) ? 1.0 : (col / static_cast<double>(kW - 1));
                        double const y = (kH <= 1) ? 0.0 : (1.0 - 2.0 * (row / static_cast<double>(kH - 1)));
                        return position{x, y, 0.0};
                    }
                }
            }

            if(ctx.pl_model_id == pl_model_id::logic_input)
            {
                if(ctx.pe_instance_name == "clk") return position{-1.0, 1.0, 0.0};
                if(ctx.pe_instance_name == "rst_n") return position{-1.0, 0.85, 0.0};

                if(ctx.pe_instance_name == "up") return position{-1.0, 0.55, 0.0};
                if(ctx.pe_instance_name == "down") return position{-1.0, 0.35, 0.0};
                if(ctx.pe_instance_name == "left") return position{-1.0, 0.15, 0.0};
                if(ctx.pe_instance_name == "right") return position{-1.0, -0.05, 0.0};
                if(ctx.pe_instance_name == "place") return position{-1.0, -0.35, 0.0};
                if(ctx.pe_instance_name == "pass") return position{-1.0, -0.55, 0.0};
            }

            return std::nullopt;
        };

        auto r = ::phy_engine::phy_lab_wrapper::pe_to_pl::convert(nl, popt);

        // Keep port I/O fixed; allow internal constants to be auto-placed.
        for(auto const& e : r.ex.elements())
        {
            auto const mid = e.data().value("ModelID", "");
            if(mid != pl_model_id::logic_input && mid != pl_model_id::logic_output) { continue; }

            auto it = e.data().find("Label");
            if(it == e.data().end() || !it->is_string()) { continue; }
            auto const label = it->get<std::string>();
            std::string_view const name{label};

            bool const is_port_io = (name == "clk" || name == "rst_n" || name == "up" || name == "down" || name == "left" || name == "right" ||
                                     name == "place" || name == "pass" || name == "black" || name == "white" || name.starts_with("row"));
            if(is_port_io)
            {
                r.ex.get_element(e.identifier()).set_participate_in_layout(false);
            }
        }

        // Place the core with hierarchical layout in a region above the output grid.
        {
            ::phy_engine::phy_lab_wrapper::auto_layout::options aopt{};
            aopt.layout_mode = ::phy_engine::phy_lab_wrapper::auto_layout::mode::hierarchical;
            aopt.respect_fixed_elements = true;
            aopt.small_element = {1, 1};
            aopt.big_element = {2, 2};
            aopt.margin_x = 1e-6;
            aopt.margin_y = 1e-6;

            auto const corner0 = position{-1.0, 1.25, 0.0};
            auto const corner1 = position{1.0, 2.0, 0.0};

            auto required_cells = [&] {
                std::size_t demand{};
                for(auto const& e : r.ex.elements())
                {
                    if(!e.participate_in_layout()) { continue; }
                    auto const fp = e.is_big_element() ? aopt.big_element : aopt.small_element;
                    demand += fp.w * fp.h;
                }
                return demand;
            }();

            auto grid_ok = [&](double step_x, double step_y) -> bool {
                if(!(step_x > 0.0) || !(step_y > 0.0)) { return false; }
                auto const min_x = std::min(corner0.x, corner1.x) + aopt.margin_x;
                auto const max_x = std::max(corner0.x, corner1.x) - aopt.margin_x;
                auto const min_y = std::min(corner0.y, corner1.y) + aopt.margin_y;
                auto const max_y = std::max(corner0.y, corner1.y) - aopt.margin_y;
                auto const span_x = max_x - min_x;
                auto const span_y = max_y - min_y;
                if(!(span_x >= 0.0) || !(span_y >= 0.0)) { return false; }
                auto const w = static_cast<std::size_t>(std::floor(span_x / step_x + 1e-12)) + 1;
                auto const h = static_cast<std::size_t>(std::floor(span_y / step_y + 1e-12)) + 1;
                constexpr std::size_t kMaxGridCells = 2'000'000;
                if(w == 0 || h == 0) { return false; }
                if(w > (kMaxGridCells / h)) { return false; }
                return true;
            };

            // Choose a step with headroom.
            if(required_cells != 0)
            {
                auto const span_x = std::abs(corner1.x - corner0.x);
                auto const span_y = std::abs(corner1.y - corner0.y);
                double const area = span_x * span_y;
                double const fill = 1.25;
                double const min_step = 0.001;
                double step = std::sqrt(std::max(1e-12, area / (static_cast<double>(required_cells) * fill)));
                if(!std::isfinite(step) || step <= 0.0) { step = 0.02; }
                step = std::max(step, min_step);
                for(std::size_t guard{}; guard < 256 && !grid_ok(step, step); ++guard) { step *= 1.02; }

                aopt.step_x = step;
                aopt.step_y = step;

                ::phy_engine::phy_lab_wrapper::auto_layout::stats st{};
                for(std::size_t attempt{}; attempt < 10; ++attempt)
                {
                    st = ::phy_engine::phy_lab_wrapper::auto_layout::layout(r.ex, corner0, corner1, 0.0, aopt);
                    if(st.skipped == 0 && st.layout_mode == aopt.layout_mode) { break; }
                    aopt.step_x = std::max(aopt.step_x * 0.92, min_step);
                    aopt.step_y = std::max(aopt.step_y * 0.92, min_step);
                    if(!grid_ok(aopt.step_x, aopt.step_y)) { break; }
                    aopt.max_candidates_per_element = std::max<std::size_t>(aopt.max_candidates_per_element, 16384);
                }
            }
            else
            {
                (void)::phy_engine::phy_lab_wrapper::auto_layout::layout(r.ex, corner0, corner1, 0.0, aopt);
            }

            for(auto const& e : r.ex.elements())
            {
                if(!e.participate_in_layout()) { continue; }
                auto const p = e.element_position();
                assert(p.x >= -1.000001 && p.x <= 1.000001);
                assert(p.y >= 1.249999 && p.y <= 2.000001);
            }
        }

        assert(!r.ex.wires().empty());

        auto const out_path = std::filesystem::path("go9x9_o4.sav");
        r.ex.save(out_path, 2);
        if(!std::filesystem::exists(out_path)) { return 9; }
        if(std::filesystem::file_size(out_path) < 128) { return 10; }
    }

    return 0;
}
