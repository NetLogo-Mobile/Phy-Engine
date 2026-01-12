#include <cstddef>

#include <phy_engine/phy_engine.h>
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
    decltype(auto) src = u8R"(
module top(input [1:0] s, input a, input b, output reg y);
  always @* begin
    case(s)
      2'b00: y = a;
      2'b01: y = b;
      default: y = 1'b0;
    endcase
  end
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

    auto [in_s1, p0] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
    auto [in_s0, p1] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
    auto [in_a, p2] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
    auto [in_b, p3] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
    auto [out_y, p4] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
    (void)p0;
    (void)p1;
    (void)p2;
    (void)p3;
    (void)p4;
    if(in_s1 == nullptr || in_s0 == nullptr || in_a == nullptr || in_b == nullptr || out_y == nullptr) { return 4; }

    auto port_idx = [&](::fast_io::u8string_view name) noexcept -> ::std::size_t
    {
        for(::std::size_t i{}; i < top_inst.mod->ports.size(); ++i)
        {
            auto const& p = top_inst.mod->ports.index_unchecked(i);
            if(p.name == name) { return i; }
        }
        return SIZE_MAX;
    };

    auto const i_s1 = port_idx(u8"s[1]");
    auto const i_s0 = port_idx(u8"s[0]");
    auto const i_a = port_idx(u8"a");
    auto const i_b = port_idx(u8"b");
    auto const i_y = port_idx(u8"y");
    if(i_s1 == SIZE_MAX || i_s0 == SIZE_MAX || i_a == SIZE_MAX || i_b == SIZE_MAX || i_y == SIZE_MAX) { return 5; }

    if(!::phy_engine::netlist::add_to_node(nl, *in_s1, 0, *ports[i_s1])) { return 6; }
    if(!::phy_engine::netlist::add_to_node(nl, *in_s0, 0, *ports[i_s0])) { return 7; }
    if(!::phy_engine::netlist::add_to_node(nl, *in_a, 0, *ports[i_a])) { return 8; }
    if(!::phy_engine::netlist::add_to_node(nl, *in_b, 0, *ports[i_b])) { return 9; }
    if(!::phy_engine::netlist::add_to_node(nl, *out_y, 0, *ports[i_y])) { return 10; }

    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt{
        .allow_inout = true,
        .allow_multi_driver = true,
    };
    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, ports, &err, opt)) { return 11; }

    if(!c.analyze()) { return 12; }
    auto tick = [&]() noexcept { c.digital_clk(); };
    auto settle = [&]() noexcept
    {
        tick();
        tick();
    };

    auto set_in = [&](::phy_engine::model::model_base* in, ::phy_engine::model::digital_node_statement_t v) noexcept {
        (void)in->ptr->set_attribute(0, dv(v));
    };

    // s=00 => y=a
    set_in(in_s1, ::phy_engine::model::digital_node_statement_t::false_state);
    set_in(in_s0, ::phy_engine::model::digital_node_statement_t::false_state);
    set_in(in_a, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(in_b, ::phy_engine::model::digital_node_statement_t::false_state);
    settle();
    if(ports[i_y]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::true_state) { return 13; }

    // s=01 => y=b
    set_in(in_s1, ::phy_engine::model::digital_node_statement_t::false_state);
    set_in(in_s0, ::phy_engine::model::digital_node_statement_t::true_state);
    settle();
    if(ports[i_y]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::false_state) { return 14; }

    // s=10 => default => 0
    set_in(in_s1, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(in_s0, ::phy_engine::model::digital_node_statement_t::false_state);
    set_in(in_b, ::phy_engine::model::digital_node_statement_t::true_state);
    settle();
    if(ports[i_y]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::false_state) { return 15; }

    return 0;
}
