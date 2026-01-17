#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/digital.h>
#include <phy_engine/verilog/digital/pe_synth.h>

#include <phy_engine/model/models/digital/logical/input.h>
#include <phy_engine/model/models/digital/logical/output.h>
#include <phy_engine/netlist/operation.h>

#include <phy_engine/phy_lab_wrapper/auto_layout/auto_layout.h>
#include <phy_engine/phy_lab_wrapper/pe_to_pl.h>

#include <cassert>
#include <array>
#include <cstdio>
#include <cmath>
#include <cstdint>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace
{
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
    using namespace phy_engine::phy_lab_wrapper;

    static constexpr std::size_t kW = 8;
    static constexpr std::size_t kH = 8;
    static_assert(kW * kH == 64);

    auto const src_path = std::filesystem::path(__FILE__).parent_path() / "snake.v";
    auto const src_s = read_file_text(src_path);
    auto const src = ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(src_s.data()), src_s.size()};

    // PE circuit container.
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    auto& setting = c.get_analyze_setting();
    setting.tr.t_step = 1e-9;
    setting.tr.t_stop = 1e-9;
    auto& nl = c.get_netlist();

    // Compile + elaborate.
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
    auto const* mod = ::phy_engine::verilog::digital::find_module(design, u8"snake_top");
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

    std::unordered_map<std::string, ::phy_engine::model::model_base*> input_by_name{};
    input_by_name.reserve(top_inst.mod->ports.size());

    std::vector<std::size_t> pix_port_indices{};
    pix_port_indices.reserve(64);
    std::array<std::size_t, 64> pix_port_index_by_bit{};
    pix_port_index_by_bit.fill(static_cast<std::size_t>(-1));

    // External IO models.
    for(std::size_t pi{}; pi < top_inst.mod->ports.size(); ++pi)
    {
        auto const& p = top_inst.mod->ports.index_unchecked(pi);
        std::string port_name(reinterpret_cast<char const*>(p.name.data()), p.name.size());

        if(p.dir == ::phy_engine::verilog::digital::port_dir::input)
        {
            auto [m, pos] =
                ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
            (void)pos;
            if(m == nullptr || m->ptr == nullptr) { return 4; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *ports[pi])) { return 5; }
            if(port_name == "clk" || port_name == "rst_n")
            {
                auto [pm, ppos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
                (void)ppos;
                if(pm == nullptr || pm->ptr == nullptr) { return 6; }
                pm->name.clear();
                pm->name.append(u8"probe_");
                pm->name.append(p.name);
                if(!::phy_engine::netlist::add_to_node(nl, *pm, 0, *ports[pi])) { return 7; }
            }

            input_by_name.emplace(std::move(port_name), m);
            continue;
        }

        if(p.dir == ::phy_engine::verilog::digital::port_dir::output)
        {
            auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
            (void)pos;
            if(m == nullptr || m->ptr == nullptr) { return 6; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *ports[pi])) { return 7; }

            if(port_name.starts_with("pix["))
            {
                pix_port_indices.push_back(pi);
                auto bit = parse_bit_index(port_name, "pix");
                if(!bit || *bit >= 64) { return 9; }
                pix_port_index_by_bit[*bit] = pi;
            }
            continue;
        }

        // No inout at top-level for this test.
        return 8;
    }

    if(pix_port_indices.size() != 64) { return 9; }
    for(std::size_t b{}; b < 64; ++b)
    {
        if(pix_port_index_by_bit[b] == static_cast<std::size_t>(-1)) { return 9; }
    }
    if(!input_by_name.contains("clk") || !input_by_name.contains("rst_n")) { return 10; }

    // Synthesize to PE netlist (digital primitives).
    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt{
        .allow_inout = false,
        .allow_multi_driver = false,
    };
    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, ports, &err, opt))
    {
        throw std::runtime_error("pe_synth failed: " + std::string(reinterpret_cast<char const*>(err.message.data()), err.message.size()));
    }

    // Export PE->PL (.sav) with deterministic IO placement (no auto placement).
    {
        ::phy_engine::phy_lab_wrapper::pe_to_pl::options popt{};
        popt.fixed_pos = {0.0, 0.0, 0.0};
        popt.generate_wires = true;
        popt.keep_pl_macros = true;

        popt.element_placer = [&](::phy_engine::phy_lab_wrapper::pe_to_pl::options::placement_context const& ctx) -> std::optional<position> {
            // Table boundary is a square: x,z in [-1, 1]. Keep y=0.
            // - Logic outputs (pix[0..63]) occupy the right half.
            // - Logic inputs (buttons) occupy the left half.

            if(ctx.pl_model_id == pl_model_id::logic_output)
            {
                if(ctx.pe_instance_name == "probe_clk") return position{-0.7, 1.0, 0.0};
                if(ctx.pe_instance_name == "probe_rst_n") return position{-0.7, 0.85, 0.0};

                auto idx = parse_bit_index(ctx.pe_instance_name, "pix");
                if(!idx || *idx >= 64) { return std::nullopt; }

                auto const col = static_cast<double>(*idx % kW);
                auto const row = static_cast<double>(*idx / kW);

                double const x = (kW <= 1) ? 1.0 : (col / static_cast<double>(kW - 1));
                double const y = (kH <= 1) ? 0.0 : (1.0 - 2.0 * (row / static_cast<double>(kH - 1)));

                return position{x, y, 0.0};
            }

            if(ctx.pl_model_id == pl_model_id::logic_input)
            {
                // Clock/reset inputs (top-left)
                if(ctx.pe_instance_name == "clk") return position{-1.0, 1.0, 0.0};
                if(ctx.pe_instance_name == "rst_n") return position{-1.0, 0.85, 0.0};

                // Direction buttons on the left half (x=-1), top-to-bottom.
                if(ctx.pe_instance_name == "btn_up") return position{-1.0, 0.6, 0.0};
                if(ctx.pe_instance_name == "btn_down") return position{-1.0, 0.2, 0.0};
                if(ctx.pe_instance_name == "btn_left") return position{-1.0, -0.2, 0.0};
                if(ctx.pe_instance_name == "btn_right") return position{-1.0, -0.6, 0.0};
            }

            return std::nullopt;
        };

        auto r = ::phy_engine::phy_lab_wrapper::pe_to_pl::convert(nl, popt);

        // Keep *port* I/O fixed (do not let auto-layout move them).
        // Note: synthesis may also introduce internal constant drivers that map to "Logic Input" with empty label;
        // those should participate in layout (otherwise they'd pile up at fixed_pos=(0,0,0)).
        for(auto const& e : r.ex.elements())
        {
            auto const mid = e.data().value("ModelID", "");
            if(mid != pl_model_id::logic_input && mid != pl_model_id::logic_output) { continue; }

            auto it = e.data().find("Label");
            if(it == e.data().end() || !it->is_string()) { continue; }
            auto const label = it->get<std::string>();
            std::string_view const name{label};

            bool const is_port_io = (name == "clk" || name == "rst_n" || name == "btn_up" || name == "btn_down" || name == "btn_left" ||
                                     name == "btn_right" || name.starts_with("pix[") || name.starts_with("probe_"));
            if(is_port_io)
            {
                r.ex.get_element(e.identifier()).set_participate_in_layout(false);
            }
        }

        // Place the "core" (all movable, non-I/O elements) using hierarchical layout in the requested region:
        // start from y=1.25 to avoid overlapping the top output row (y=1.0).
        // corner0 = (-1, 1.25, 0), corner1 = (1, 2, 0).
        {
            ::phy_engine::phy_lab_wrapper::auto_layout::options aopt{};
            aopt.layout_mode = ::phy_engine::phy_lab_wrapper::auto_layout::mode::hierarchical;
            aopt.respect_fixed_elements = true;
            aopt.small_element = {1, 1};
            aopt.big_element = {2, 2};
            // Exclude boundary-placed IO elements from being treated as obstacles inside the layout grid.
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

            auto capacity = [&](double step_x, double step_y) -> double {
                if(!(step_x > 0.0) || !(step_y > 0.0)) { return 0.0; }
                auto const min_x = std::min(corner0.x, corner1.x) + aopt.margin_x;
                auto const max_x = std::max(corner0.x, corner1.x) - aopt.margin_x;
                auto const min_y = std::min(corner0.y, corner1.y) + aopt.margin_y;
                auto const max_y = std::max(corner0.y, corner1.y) - aopt.margin_y;
                auto const span_x = max_x - min_x;
                auto const span_y = max_y - min_y;
                if(!(span_x >= 0.0) || !(span_y >= 0.0)) { return 0.0; }
                auto const w = std::floor(span_x / step_x + 1e-12) + 1.0;
                auto const h = std::floor(span_y / step_y + 1e-12) + 1.0;
                return w * h;
            };

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
                // Keep memory bounded even if someone exports a huge design here.
                constexpr std::size_t kMaxGridCells = 2'000'000;
                if(w == 0 || h == 0) { return false; }
                if(w > (kMaxGridCells / h)) { return false; }
                return true;
            };

            // Choose a step small enough so the region has enough subcells to avoid pile-up.
            if(required_cells != 0)
            {
                auto const span_x = std::abs(corner1.x - corner0.x);
                auto const span_y = std::abs(corner1.y - corner0.y);
                double const area = span_x * span_y;
                double const fill = 1.25;     // extra headroom
                double const min_step = 0.001; // keep grid bounded by kMaxGridCells
                double step = std::sqrt(std::max(1e-12, area / (static_cast<double>(required_cells) * fill)));
                if(!std::isfinite(step) || step <= 0.0) { step = 0.02; }
                step = std::max(step, min_step);
                // Make sure the grid itself is allowed; if not, increase the step until it fits.
                for(std::size_t guard{}; guard < 256 && !grid_ok(step, step); ++guard) { step *= 1.02; }

                aopt.step_x = step;
                aopt.step_y = step;

                // Retry if the chosen step is still too coarse (skipped nodes stay at fixed_pos and overlap).
                ::phy_engine::phy_lab_wrapper::auto_layout::stats st{};
                for(std::size_t attempt{}; attempt < 10; ++attempt)
                {
                    st = ::phy_engine::phy_lab_wrapper::auto_layout::layout(r.ex, corner0, corner1, 0.0, aopt);
                    if(st.skipped == 0 && st.layout_mode == aopt.layout_mode) { break; }
                    aopt.step_x = std::max(aopt.step_x * 0.92, min_step);
                    aopt.step_y = std::max(aopt.step_y * 0.92, min_step);
                    if(!grid_ok(aopt.step_x, aopt.step_y)) { break; }
                    // If we already have enough capacity and still skip, allow a deeper candidate search.
                    if(capacity(aopt.step_x, aopt.step_y) >= static_cast<double>(required_cells) * fill)
                    {
                        aopt.max_candidates_per_element = std::max<std::size_t>(aopt.max_candidates_per_element, 16384);
                    }
                }
            }
            else
            {
                (void)::phy_engine::phy_lab_wrapper::auto_layout::layout(r.ex, corner0, corner1, 0.0, aopt);
            }

            // Validate the "core" ends up inside the requested region (I/O is excluded above).
            for(auto const& e : r.ex.elements())
            {
                if(!e.participate_in_layout()) { continue; }
                auto const p = e.element_position();
                assert(p.x >= -1.000001 && p.x <= 1.000001);
                assert(p.y >= 1.249999 && p.y <= 2.000001);
            }
        }

        // Must be a real netlist (nodes are linked), even if element placement is fixed.
        assert(!r.ex.wires().empty());

        auto const out_path = std::filesystem::path("snake_pe_to_pl.sav");
        r.ex.save(out_path, 2);
        if(!std::filesystem::exists(out_path)) { return 11; }
        if(std::filesystem::file_size(out_path) < 128) { return 12; }

        // Validate deterministic IO placement.
        struct key
        {
            std::string model_id;
            std::string name;
        };
        struct key_hash
        {
            std::size_t operator()(key const& k) const noexcept
            {
                return std::hash<std::string>{}(k.model_id) ^ (std::hash<std::string>{}(k.name) << 1);
            }
        };
        struct key_eq
        {
            bool operator()(key const& a, key const& b) const noexcept { return a.model_id == b.model_id && a.name == b.name; }
        };

        std::unordered_map<key, position, key_hash, key_eq> pos_by_kind_and_name{};
        pos_by_kind_and_name.reserve(r.ex.elements().size());
        for(auto const& e : r.ex.elements())
        {
            auto const mid = e.data().value("ModelID", "");
            std::string name;
            if(auto it = e.data().find("Label"); it != e.data().end() && it->is_string())
            {
                name = it->get<std::string>();
            }
            else
            {
                name.clear();
            }
            if(!mid.empty() && !name.empty())
            {
                pos_by_kind_and_name.emplace(key{mid, name}, e.element_position());
            }
        }

        for(std::size_t idx{}; idx < 64; ++idx)
        {
            auto const col = static_cast<double>(idx % kW);
            auto const row = static_cast<double>(idx / kW);
            double const expected_x = (kW <= 1) ? 1.0 : (col / static_cast<double>(kW - 1));
            double const expected_y = (kH <= 1) ? 0.0 : (1.0 - 2.0 * (row / static_cast<double>(kH - 1)));
            auto const expected = position{expected_x, expected_y, 0.0};
            auto const name = "pix[" + std::to_string(idx) + "]";

            auto it = pos_by_kind_and_name.find(key{std::string(pl_model_id::logic_output), name});
            if(it == pos_by_kind_and_name.end()) { return 13; }

            auto const p = it->second;
            assert(p.x == expected.x && p.y == expected.y && p.z == expected.z);
        }

        struct io_expect
        {
            char const* name;
            position pos;
        };
        constexpr io_expect ios[] = {
            {"btn_up", {-1.0, 0.6, 0.0}},
            {"btn_down", {-1.0, 0.2, 0.0}},
            {"btn_left", {-1.0, -0.2, 0.0}},
            {"btn_right", {-1.0, -0.6, 0.0}},
        };
        for(auto const& io : ios)
        {
            auto it = pos_by_kind_and_name.find(key{std::string(pl_model_id::logic_input), io.name});
            if(it == pos_by_kind_and_name.end()) { return 14; }
            auto const p = it->second;
            assert(p.x == io.pos.x && p.y == io.pos.y && p.z == io.pos.z);
        }
    }

    // Run PE simulation.
    if(!c.analyze()) { return 15; }

    auto* in_clk = input_by_name.at("clk");
    auto* in_rstn = input_by_name.at("rst_n");
    auto set_in = [&](::phy_engine::model::model_base* m, bool v) {
        (void)m->ptr->set_attribute(0, dv(v ? ::phy_engine::model::digital_node_statement_t::true_state
                                            : ::phy_engine::model::digital_node_statement_t::false_state));
    };

    auto* in_right = input_by_name.at("btn_right");
    auto* in_down = input_by_name.at("btn_down");
    auto* in_left = input_by_name.at("btn_left");
    auto* in_up = input_by_name.at("btn_up");

    auto tick = [&] {
        set_in(in_clk, true);
        c.digital_clk();
        set_in(in_clk, false);
        c.digital_clk();
    };

    auto reset = [&] {
        set_in(in_clk, false);
        c.digital_clk();

        // Make reset edge-visible: INPUT ports initialize to 0, so we explicitly drive 1->0.
        set_in(in_rstn, true);
        c.digital_clk();
        set_in(in_rstn, false);
        c.digital_clk();

        set_in(in_clk, true);
        c.digital_clk();
        set_in(in_clk, false);
        c.digital_clk();
        set_in(in_rstn, true);
        c.digital_clk();

        set_in(in_up, false);
        set_in(in_down, false);
        set_in(in_left, false);
        set_in(in_right, false);
        c.digital_clk();
    };

    auto pix_u64 = [&]() -> std::uint64_t {
        std::uint64_t v{};
        for(std::size_t bit{}; bit < 64; ++bit)
        {
            auto const pi = pix_port_index_by_bit[bit];
            auto const st = ports[pi]->node_information.dn.state;
            if(st == ::phy_engine::model::digital_node_statement_t::true_state)
            {
                v |= (std::uint64_t{1} << bit);
                continue;
            }
            assert(st == ::phy_engine::model::digital_node_statement_t::false_state);
        }
        return v;
    };

    auto bit_at = [](std::uint32_t x, std::uint32_t y) -> std::uint64_t { return (std::uint64_t{1} << (y * 8u + x)); };

    // Repro: if "down" is held before the first tick after reset, the snake should still render.
    // This specifically exercises indices >= 32 (row 4+) in pix.
    {
        reset();

        auto const initial = pix_u64();
        auto const expected_initial = bit_at(3, 3) | bit_at(2, 3) | bit_at(1, 3) | bit_at(0, 3) | bit_at(5, 5);
        if(initial != expected_initial)
        {
            std::fprintf(stderr, "initial pix mismatch: got=0x%016llx expected=0x%016llx\n",
                         static_cast<unsigned long long>(initial), static_cast<unsigned long long>(expected_initial));
        }
        assert(initial == expected_initial);

        set_in(in_down, true);
        c.digital_clk();
        tick();
        set_in(in_down, false);

        auto const after1 = pix_u64();
        auto const expected_after1 = bit_at(3, 4) | bit_at(3, 3) | bit_at(2, 3) | bit_at(1, 3) | bit_at(5, 5);
        if(after1 != expected_after1)
        {
            std::fprintf(stderr, "after1 pix mismatch: got=0x%016llx expected=0x%016llx\n",
                         static_cast<unsigned long long>(after1), static_cast<unsigned long long>(expected_after1));
        }
        assert(after1 == expected_after1);
    }

    // Food: ensure it's visible after reset, and that it moves after being eaten.
    {
        reset();

        // Move right twice (default direction) to x=5,y=3.
        tick();
        tick();

        // Hold "down" and move onto the food at (5,5).
        set_in(in_down, true);
        c.digital_clk();
        tick();  // y=4
        tick();  // y=5 (eat)
        set_in(in_down, false);

        auto const after_eat = pix_u64();
        assert((after_eat & bit_at(4, 6)) != 0);  // lfsr_next from 6'h3A is 6'h34 => (x=4,y=6)
    }

    reset();

    // Move right for a few cycles (default direction), then turn down, then left.
    set_in(in_right, true);
    c.digital_clk();
    for(int i = 0; i < 3; ++i)
    {
        tick();
    }
    set_in(in_right, false);

    set_in(in_down, true);
    c.digital_clk();
    for(int i = 0; i < 2; ++i)
    {
        tick();
    }
    set_in(in_down, false);

    set_in(in_left, true);
    c.digital_clk();
    for(int i = 0; i < 2; ++i)
    {
        tick();
    }
    set_in(in_left, false);

    // Pulse up once (direction change only; movement happens every cycle anyway).
    set_in(in_up, true);
    c.digital_clk();
    tick();
    set_in(in_up, false);

    // The snake renders 4 segments; at least 2 pixels should be asserted.
    std::size_t on{};
    for(auto pi : pix_port_indices)
    {
        if(ports[pi]->node_information.dn.state == ::phy_engine::model::digital_node_statement_t::true_state)
        {
            ++on;
        }
    }
    assert(on >= 2);

    return 0;
}
