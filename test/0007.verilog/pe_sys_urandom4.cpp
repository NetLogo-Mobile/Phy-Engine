#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include <phy_engine/phy_engine.h>
#include <phy_engine/phy_lab_wrapper/pe_to_pl.h>
#include <phy_engine/verilog/digital/digital.h>
#include <phy_engine/verilog/digital/pe_synth.h>

namespace
{
inline ::phy_engine::model::variant dv(::phy_engine::model::digital_node_statement_t v) noexcept
{
    ::phy_engine::model::variant vi{};
    vi.digital = v;
    vi.type = ::phy_engine::model::variant_type::digital;
    return vi;
}
}  // namespace

int main()
{
    // `$urandom`/`$random` (subset) must synthesize to a compact PE RANDOM_GENERATOR4
    // and export as the PL "Random Generator" macro when keep_pl_macros=true.
    decltype(auto) src = u8R"(
module top(input clk, input rst_n, output [3:0] out);
  reg [3:0] r;
  always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) r <= 4'b0000;
    else       r <= $urandom;
  end
  assign out = r;
endmodule
)";

    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    auto& setting = c.get_analyze_setting();
    setting.tr.t_step = 1e-9;
    setting.tr.t_stop = 1e-9;

    auto& nl = c.get_netlist();

    auto cr = ::phy_engine::verilog::digital::compile(src);
    if(!cr.errors.empty() || cr.modules.empty()) { return 1; }
    auto design = ::phy_engine::verilog::digital::build_design(::std::move(cr));
    auto const* mod = ::phy_engine::verilog::digital::find_module(design, u8"top");
    if(mod == nullptr) { return 2; }
    auto top_inst = ::phy_engine::verilog::digital::elaborate(design, *mod);
    if(top_inst.mod == nullptr) { return 3; }

    ::std::vector<::phy_engine::model::node_t*> ports{};
    ports.reserve(top_inst.mod->ports.size());
    for(::std::size_t i{}; i < top_inst.mod->ports.size(); ++i)
    {
        auto& n = ::phy_engine::netlist::create_node(nl);
        ports.push_back(__builtin_addressof(n));
    }

    auto port_idx = [&](std::string_view name) noexcept -> ::std::size_t {
        for(::std::size_t i{}; i < top_inst.mod->ports.size(); ++i)
        {
            auto const& p = top_inst.mod->ports.index_unchecked(i);
            ::std::string_view pn(reinterpret_cast<char const*>(p.name.data()), p.name.size());
            if(pn == name) { return i; }
        }
        return SIZE_MAX;
    };

    auto [in_clk, in_clk_pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
    (void)in_clk_pos;
    if(in_clk == nullptr) { return 4; }
    auto [in_rstn, in_rstn_pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
    (void)in_rstn_pos;
    if(in_rstn == nullptr) { return 5; }

    std::array<::phy_engine::model::model_base*, 4> out_bits{};
    for(auto& p : out_bits)
    {
        auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
        (void)pos;
        if(m == nullptr) { return 6; }
        p = m;
    }

    auto const pi_clk = port_idx("clk");
    auto const pi_rstn = port_idx("rst_n");
    if(pi_clk == SIZE_MAX || pi_rstn == SIZE_MAX) { return 7; }
    if(!::phy_engine::netlist::add_to_node(nl, *in_clk, 0, *ports[pi_clk])) { return 8; }
    if(!::phy_engine::netlist::add_to_node(nl, *in_rstn, 0, *ports[pi_rstn])) { return 9; }

    std::array<std::size_t, 4> pi_out{};
    for(std::size_t i{}; i < 4; ++i)
    {
        pi_out[i] = port_idx("out[" + ::std::to_string(i) + "]");
        if(pi_out[i] == SIZE_MAX) { return 10; }
        if(!::phy_engine::netlist::add_to_node(nl, *out_bits[i], 0, *ports[pi_out[i]])) { return 11; }
    }

    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt{
        .allow_inout = false,
        .allow_multi_driver = false,
    };
    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, ports, &err, opt)) { return 12; }

    // Ensure the synthesized netlist actually contains the PE RNG model.
    bool has_rng{};
    for(auto& blk : nl.models)
    {
        for(auto* m = blk.begin; m != blk.curr; ++m)
        {
            if(m->type != ::phy_engine::model::model_type::normal) { continue; }
            if(m->ptr == nullptr) { continue; }
            if(m->ptr->get_model_name() == ::fast_io::u8string_view{u8"RANDOM_GENERATOR4"}) { has_rng = true; }
        }
    }
    if(!has_rng) { return 13; }

    // Ensure PE->PL export keeps it as the PL macro (not expanded gates).
    {
        auto r = ::phy_engine::phy_lab_wrapper::pe_to_pl::convert(nl);
        bool found{};
        for(auto const& e : r.ex.elements())
        {
            if(e.data().value("ModelID", "") == "Random Generator") { found = true; break; }
        }
        if(!found) { return 14; }
    }

    if(!c.analyze()) { return 15; }

    auto set_in = [&](::phy_engine::model::model_base* m, bool v) noexcept {
        (void)m->ptr->set_attribute(0, dv(v ? ::phy_engine::model::digital_node_statement_t::true_state
                                            : ::phy_engine::model::digital_node_statement_t::false_state));
    };

    auto tick = [&]() noexcept {
        set_in(in_clk, false);
        c.digital_clk();
        set_in(in_clk, true);
        c.digital_clk();
    };

    // Reset asserted: output must be 0.
    set_in(in_rstn, false);
    for(int i = 0; i < 2; ++i) { tick(); }
    for(std::size_t i{}; i < 4; ++i)
    {
        auto const s = ports[pi_out[i]]->node_information.dn.state;
        if(s != ::phy_engine::model::digital_node_statement_t::false_state) { return 16; }
    }

    // Release reset and expect output to change at least once.
    set_in(in_rstn, true);
    tick();
    std::uint8_t first{};
    for(std::size_t i{}; i < 4; ++i)
    {
        auto const s = ports[pi_out[i]]->node_information.dn.state;
        if(s == ::phy_engine::model::digital_node_statement_t::true_state) { first |= static_cast<std::uint8_t>(1u << i); }
        else if(s != ::phy_engine::model::digital_node_statement_t::false_state) { return 17; }
    }

    bool changed{};
    for(int i = 0; i < 32; ++i)
    {
        tick();
        std::uint8_t now{};
        for(std::size_t b{}; b < 4; ++b)
        {
            auto const s = ports[pi_out[b]]->node_information.dn.state;
            if(s == ::phy_engine::model::digital_node_statement_t::true_state) { now |= static_cast<std::uint8_t>(1u << b); }
            else if(s != ::phy_engine::model::digital_node_statement_t::false_state) { return 18; }
        }
        if(now != first)
        {
            changed = true;
            break;
        }
    }
    if(!changed) { return 19; }

    return 0;
}

