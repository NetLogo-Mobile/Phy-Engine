#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/digital.h>
#include <phy_engine/verilog/digital/pe_synth.h>

#include <phy_engine/model/models/digital/combinational/counter4.h>
#include <phy_engine/model/models/digital/combinational/random_generator4.h>
#include <phy_engine/model/models/digital/logical/input.h>
#include <phy_engine/model/models/digital/logical/output.h>

#include <phy_engine/phy_lab_wrapper/auto_layout/auto_layout.h>
#include <phy_engine/phy_lab_wrapper/pe_to_pl.h>

#include <cassert>
#include <array>
#include <cstddef>
#include <cstdint>
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

struct synth_out
{
    ::phy_engine::verilog::digital::instance_state inst{};
    std::vector<::phy_engine::model::node_t*> ports{};
};

synth_out synth_one_module(::phy_engine::netlist::netlist& nl,
                           std::filesystem::path const& path,
                           ::fast_io::u8string_view top_name,
                           std::unordered_map<std::string, ::phy_engine::model::node_t*> const& bind)
{
    auto const src_s = read_file_text(path);
    auto const src = ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(src_s.data()), src_s.size()};

    auto cr = ::phy_engine::verilog::digital::compile(src);
    if(!cr.errors.empty() || cr.modules.empty())
    {
        if(!cr.errors.empty())
        {
            auto const& e = cr.errors.front_unchecked();
            throw std::runtime_error("verilog compile error at line " + std::to_string(e.line) + ": " +
                                     std::string(reinterpret_cast<char const*>(e.message.data()), e.message.size()));
        }
        throw std::runtime_error("verilog compile failed (no modules): " + path.string());
    }

    auto design = ::phy_engine::verilog::digital::build_design(::std::move(cr));
    auto const* mod = ::phy_engine::verilog::digital::find_module(design, top_name);
    if(mod == nullptr) { throw std::runtime_error("module not found: " + std::string(reinterpret_cast<char const*>(top_name.data()), top_name.size())); }

    auto top_inst = ::phy_engine::verilog::digital::elaborate(design, *mod);
    if(top_inst.mod == nullptr) { throw std::runtime_error("elaborate failed: " + path.string()); }

    synth_out out{};
    out.inst = std::move(top_inst);

    out.ports.reserve(out.inst.mod->ports.size());
    for(std::size_t pi{}; pi < out.inst.mod->ports.size(); ++pi)
    {
        auto const& p = out.inst.mod->ports.index_unchecked(pi);
        std::string port_name(reinterpret_cast<char const*>(p.name.data()), p.name.size());
        if(auto it = bind.find(port_name); it != bind.end() && it->second != nullptr)
        {
            out.ports.push_back(it->second);
        }
        else
        {
            auto& n = ::phy_engine::netlist::create_node(nl);
            out.ports.push_back(__builtin_addressof(n));
        }
    }

    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt{
        .allow_inout = false,
        .allow_multi_driver = false,
    };
    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, out.inst, out.ports, &err, opt))
    {
        throw std::runtime_error("pe_synth failed: " + std::string(reinterpret_cast<char const*>(err.message.data()), err.message.size()));
    }
    return out;
}

::phy_engine::model::node_t& node_ref(::phy_engine::netlist::netlist& nl) { return ::phy_engine::netlist::create_node(nl); }
}  // namespace

int main()
{
    using namespace phy_engine;
    using namespace phy_engine::phy_lab_wrapper;

    static constexpr std::size_t kW = 8;
    static constexpr std::size_t kH = 8;

    // PE circuit container.
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    auto& setting = c.get_analyze_setting();
    setting.tr.t_step = 1e-9;
    setting.tr.t_stop = 1e-9;
    auto& nl = c.get_netlist();

    auto const dir = std::filesystem::path(__FILE__).parent_path();

    // Global IO nodes.
    auto& n_clk = node_ref(nl);
    auto& n_rstn = node_ref(nl);
    auto& n_btn_up = node_ref(nl);
    auto& n_btn_down = node_ref(nl);
    auto& n_btn_left = node_ref(nl);
    auto& n_btn_right = node_ref(nl);

    // Global IO models.
    auto add_input = [&](std::string_view name, ::phy_engine::model::node_t& node) -> ::phy_engine::model::model_base* {
        auto [m, pos] =
            ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
        (void)pos;
        if(m == nullptr || m->ptr == nullptr) { return nullptr; }
        m->name.clear();
        m->name.append(::fast_io::u8string_view{reinterpret_cast<char8_t const*>(name.data()), name.size()});
        if(!::phy_engine::netlist::add_to_node(nl, *m, 0, node)) { return nullptr; }
        return m;
    };

    auto* in_clk = add_input("clk", n_clk);
    auto* in_rstn = add_input("rst_n", n_rstn);
    auto* in_up = add_input("btn_up", n_btn_up);
    auto* in_down = add_input("btn_down", n_btn_down);
    auto* in_left = add_input("btn_left", n_btn_left);
    auto* in_right = add_input("btn_right", n_btn_right);
    if(!in_clk || !in_rstn || !in_up || !in_down || !in_left || !in_right) { return 1; }

    // Clock divider (PE macro): use COUNTER4.q3 as a slow clock (~1/16 of base clock).
    auto& n_step_clk = node_ref(nl);
    {
        auto [ctr, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::COUNTER4{.value = 0});
        (void)pos;
        if(ctr == nullptr || ctr->ptr == nullptr) { return 2; }
        ctr->name.clear();
        ctr->name.append(u8"step_counter4");

        // q3 -> step clock
        if(!::phy_engine::netlist::add_to_node(nl, *ctr, 0, n_step_clk)) { return 3; }
        // clk
        if(!::phy_engine::netlist::add_to_node(nl, *ctr, 4, n_clk)) { return 3; }
        // en = rst_n (hold at 0 during reset)
        if(!::phy_engine::netlist::add_to_node(nl, *ctr, 5, n_rstn)) { return 3; }
    }

    // Random sources:
    // - PL-style: a tiny LFSR compiled from Verilog (rng4_pl.v)
    // - PE macro: RANDOM_GENERATOR4 (exports as "Random Generator" in PL when keep_pl_macros=true)
    std::array<::phy_engine::model::node_t*, 4> rnd_pl{};
    std::array<::phy_engine::model::node_t*, 4> rnd_pe{};
    for(auto& p : rnd_pl) { p = __builtin_addressof(node_ref(nl)); }
    for(auto& p : rnd_pe) { p = __builtin_addressof(node_ref(nl)); }

    // Synthesize rng4_pl.
    {
        std::unordered_map<std::string, ::phy_engine::model::node_t*> bind{};
        bind.emplace("clk", __builtin_addressof(n_step_clk));
        bind.emplace("rst_n", __builtin_addressof(n_rstn));
        for(std::size_t i{}; i < 4; ++i) { bind.emplace("rnd[" + std::to_string(i) + "]", rnd_pl[i]); }
        (void)synth_one_module(nl, dir / "rng4_pl.v", u8"rng4_pl", bind);
    }

    // Add PE random generator.
    {
        auto [rng, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::RANDOM_GENERATOR4{.state = 0x9});
        (void)pos;
        if(rng == nullptr || rng->ptr == nullptr) { return 4; }
        rng->name.clear();
        rng->name.append(u8"rng_pe4");
        // q3..q0
        for(std::size_t i{}; i < 4; ++i)
        {
            if(!::phy_engine::netlist::add_to_node(nl, *rng, i, *rnd_pe[i])) { return 5; }
        }
        // clk = slow clock
        if(!::phy_engine::netlist::add_to_node(nl, *rng, 4, n_step_clk)) { return 5; }
        // reset_n = rst_n
        if(!::phy_engine::netlist::add_to_node(nl, *rng, 5, n_rstn)) { return 5; }
    }

    // Synthesize snake_core and snake_render, then connect them by bus nodes.
    // Core outputs -> renderer inputs.
    std::array<::phy_engine::model::node_t*, 6> idx_head{};
    std::array<::phy_engine::model::node_t*, 6> idx0{};
    std::array<::phy_engine::model::node_t*, 6> idx1{};
    std::array<::phy_engine::model::node_t*, 6> idx2{};
    std::array<::phy_engine::model::node_t*, 6> idx_food{};
    for(auto& p : idx_head) { p = __builtin_addressof(node_ref(nl)); }
    for(auto& p : idx0) { p = __builtin_addressof(node_ref(nl)); }
    for(auto& p : idx1) { p = __builtin_addressof(node_ref(nl)); }
    for(auto& p : idx2) { p = __builtin_addressof(node_ref(nl)); }
    for(auto& p : idx_food) { p = __builtin_addressof(node_ref(nl)); }
    auto& n_game_over = node_ref(nl);

    // snake_core
    {
        std::unordered_map<std::string, ::phy_engine::model::node_t*> bind{};
        bind.emplace("clk", __builtin_addressof(n_step_clk));
        bind.emplace("rst_n", __builtin_addressof(n_rstn));
        bind.emplace("btn_up", __builtin_addressof(n_btn_up));
        bind.emplace("btn_down", __builtin_addressof(n_btn_down));
        bind.emplace("btn_left", __builtin_addressof(n_btn_left));
        bind.emplace("btn_right", __builtin_addressof(n_btn_right));

        for(std::size_t i{}; i < 4; ++i)
        {
            bind.emplace("rnd_pl[" + std::to_string(i) + "]", rnd_pl[i]);
            bind.emplace("rnd_pe[" + std::to_string(i) + "]", rnd_pe[i]);
        }

        for(std::size_t i{}; i < 6; ++i)
        {
            bind.emplace("idx_head[" + std::to_string(i) + "]", idx_head[i]);
            bind.emplace("idx0[" + std::to_string(i) + "]", idx0[i]);
            bind.emplace("idx1[" + std::to_string(i) + "]", idx1[i]);
            bind.emplace("idx2[" + std::to_string(i) + "]", idx2[i]);
            bind.emplace("idx_food[" + std::to_string(i) + "]", idx_food[i]);
        }
        bind.emplace("game_over", __builtin_addressof(n_game_over));

        (void)synth_one_module(nl, dir / "snake_core.v", u8"snake_core", bind);
    }

    // snake_render -> pix bus
    std::array<::phy_engine::model::node_t*, 64> pix{};
    for(auto& p : pix) { p = __builtin_addressof(node_ref(nl)); }

    {
        std::unordered_map<std::string, ::phy_engine::model::node_t*> bind{};
        for(std::size_t i{}; i < 6; ++i)
        {
            bind.emplace("idx_head[" + std::to_string(i) + "]", idx_head[i]);
            bind.emplace("idx0[" + std::to_string(i) + "]", idx0[i]);
            bind.emplace("idx1[" + std::to_string(i) + "]", idx1[i]);
            bind.emplace("idx2[" + std::to_string(i) + "]", idx2[i]);
            bind.emplace("idx_food[" + std::to_string(i) + "]", idx_food[i]);
        }
        bind.emplace("game_over", __builtin_addressof(n_game_over));
        for(std::size_t i{}; i < 64; ++i) { bind.emplace("pix[" + std::to_string(i) + "]", pix[i]); }

        (void)synth_one_module(nl, dir / "snake_render.v", u8"snake_render", bind);
    }

    // External observe pix as Logic Outputs.
    for(std::size_t i{}; i < 64; ++i)
    {
        auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
        (void)pos;
        if(m == nullptr || m->ptr == nullptr) { return 6; }
        m->name.clear();
        m->name.append(u8"pix[");
        auto const num = std::to_string(i);
        m->name.append(::fast_io::u8string_view{reinterpret_cast<char8_t const*>(num.data()), num.size()});
        m->name.append(u8"]");
        if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *pix[i])) { return 7; }
    }

    // Export PE->PL (.sav) with deterministic IO placement + hierarchical layout for core.
    {
        ::phy_engine::phy_lab_wrapper::pe_to_pl::options popt{};
        popt.fixed_pos = {0.0, 0.0, 0.0};
        popt.generate_wires = true;
        popt.keep_pl_macros = true;

        popt.element_placer = [&](::phy_engine::phy_lab_wrapper::pe_to_pl::options::placement_context const& ctx) -> std::optional<position> {
            if(ctx.pl_model_id == pl_model_id::logic_output)
            {
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
                if(ctx.pe_instance_name == "clk") return position{-1.0, 1.0, 0.0};
                if(ctx.pe_instance_name == "rst_n") return position{-1.0, 0.85, 0.0};
                if(ctx.pe_instance_name == "btn_up") return position{-1.0, 0.6, 0.0};
                if(ctx.pe_instance_name == "btn_down") return position{-1.0, 0.2, 0.0};
                if(ctx.pe_instance_name == "btn_left") return position{-1.0, -0.2, 0.0};
                if(ctx.pe_instance_name == "btn_right") return position{-1.0, -0.6, 0.0};
            }
            return std::nullopt;
        };

        auto r = ::phy_engine::phy_lab_wrapper::pe_to_pl::convert(nl, popt);

        // Keep only true port IO fixed; internal constant drivers should be laid out.
        for(auto const& e : r.ex.elements())
        {
            auto const mid = e.data().value("ModelID", "");
            if(mid != pl_model_id::logic_input && mid != pl_model_id::logic_output) { continue; }
            auto it = e.data().find("Label");
            if(it == e.data().end() || !it->is_string()) { continue; }
            auto const label = it->get<std::string>();
            std::string_view const name{label};
            bool const is_port_io = (name == "clk" || name == "rst_n" || name == "btn_up" || name == "btn_down" || name == "btn_left" ||
                                     name == "btn_right" || name.starts_with("pix["));
            if(is_port_io) { r.ex.get_element(e.identifier()).set_participate_in_layout(false); }
        }

        // Layout core area above the display: y in [1.25, 2.0], x in [-1, 1].
        {
            ::phy_engine::phy_lab_wrapper::auto_layout::options aopt{};
            aopt.mode = ::phy_engine::phy_lab_wrapper::auto_layout::mode::hierarchical;
            aopt.respect_fixed_elements = true;
            aopt.small_element = {1, 1};
            aopt.big_element = {2, 2};
            aopt.margin_x = 1e-6;
            aopt.margin_y = 1e-6;
            // Use a relatively fine grid (auto-scaled by verilog2plsav; here we pick a safe default).
            aopt.step_x = 0.01;
            aopt.step_y = 0.01;

            (void)::phy_engine::phy_lab_wrapper::auto_layout::layout(r.ex, position{-1.0, 1.25, 0.0}, position{1.0, 2.0, 0.0}, 0.0, aopt);

            for(auto const& e : r.ex.elements())
            {
                if(!e.participate_in_layout()) { continue; }
                auto const p = e.element_position();
                assert(p.x >= -1.000001 && p.x <= 1.000001);
                assert(p.y >= 1.249999 && p.y <= 2.000001);
            }
        }

        auto const out_path = std::filesystem::path("snake_combine_pe_to_pl.sav");
        r.ex.save(out_path, 2);
        if(!std::filesystem::exists(out_path)) { return 8; }
        if(std::filesystem::file_size(out_path) < 128) { return 9; }
    }

    // Run PE simulation.
    if(!c.analyze()) { return 10; }

    auto set_in = [&](::phy_engine::model::model_base* m, bool v) {
        (void)m->ptr->set_attribute(0, dv(v ? ::phy_engine::model::digital_node_statement_t::true_state
                                            : ::phy_engine::model::digital_node_statement_t::false_state));
    };

    auto tick = [&] {
        set_in(in_clk, false);
        c.digital_clk();
        set_in(in_clk, true);
        c.digital_clk();
    };

    // Reset.
    set_in(in_clk, false);
    set_in(in_up, false);
    set_in(in_down, false);
    set_in(in_left, false);
    set_in(in_right, false);
    set_in(in_rstn, false);
    for(int i = 0; i < 4; ++i) { tick(); }
    set_in(in_rstn, true);
    tick();

    auto is_high = [](::phy_engine::model::node_t const& n) -> bool {
        return n.node_information.dn.state == ::phy_engine::model::digital_node_statement_t::true_state;
    };

    // Basic sanity: for some ticks, ensure snake+food render without overlap (5 pixels total).
    auto popcount_pix = [&]() -> std::size_t {
        std::size_t on{};
        for(auto* n : pix)
        {
            if(n->node_information.dn.state == ::phy_engine::model::digital_node_statement_t::true_state) { ++on; }
        }
        return on;
    };

    // After reset, we expect exactly 5 lit pixels (4 segments + 1 food) until game over.
    for(int i = 0; i < 64; ++i)
    {
        tick();
        if(!is_high(n_game_over))
        {
            assert(popcount_pix() == 5);
        }
    }

    // Drive a simple movement pattern; design should remain alive for a while.
    set_in(in_right, true);
    for(int i = 0; i < 128; ++i) { tick(); }
    set_in(in_right, false);
    set_in(in_down, true);
    for(int i = 0; i < 128; ++i) { tick(); }
    set_in(in_down, false);

    assert(popcount_pix() >= 1);
    return 0;
}
