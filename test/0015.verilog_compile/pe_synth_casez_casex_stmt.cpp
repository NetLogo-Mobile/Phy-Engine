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
module top(input [1:0] s, output reg y0, output reg y1);
  always @* begin
    casez(s)
      2'b0z: y0 = 1'b1;
      2'b10: y0 = 1'b0;
      default: y0 = 1'b0;
    endcase
  end

  always @* begin
    casex(s)
      2'b10: y1 = 1'b1;
      default: y1 = 1'b0;
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
    auto [out_y0, p2] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
    auto [out_y1, p3] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
    (void)p0;
    (void)p1;
    (void)p2;
    (void)p3;
    if(in_s1 == nullptr || in_s0 == nullptr || out_y0 == nullptr || out_y1 == nullptr) { return 4; }

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
    auto const i_y0 = port_idx(u8"y0");
    auto const i_y1 = port_idx(u8"y1");
    if(i_s1 == SIZE_MAX || i_s0 == SIZE_MAX || i_y0 == SIZE_MAX || i_y1 == SIZE_MAX) { return 5; }

    if(!::phy_engine::netlist::add_to_node(nl, *in_s1, 0, *ports[i_s1])) { return 6; }
    if(!::phy_engine::netlist::add_to_node(nl, *in_s0, 0, *ports[i_s0])) { return 7; }
    if(!::phy_engine::netlist::add_to_node(nl, *out_y0, 0, *ports[i_y0])) { return 8; }
    if(!::phy_engine::netlist::add_to_node(nl, *out_y1, 0, *ports[i_y1])) { return 9; }

    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt{
        .allow_inout = true,
        .allow_multi_driver = true,
    };
    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, ports, &err, opt)) { return 10; }

    if(!c.analyze()) { return 11; }
    auto tick = [&]() noexcept { c.digital_clk(); };
    auto settle = [&]() noexcept
    {
        tick();
        tick();
    };

    auto set_in = [&](::phy_engine::model::model_base* in, ::phy_engine::model::digital_node_statement_t v) noexcept {
        (void)in->ptr->set_attribute(0, dv(v));
    };

    // casez: s=01 matches 0z => y0=1
    set_in(in_s1, ::phy_engine::model::digital_node_statement_t::false_state);
    set_in(in_s0, ::phy_engine::model::digital_node_statement_t::true_state);
    settle();
    if(ports[i_y0]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::true_state) { return 12; }

    // casex: s=1x matches 10 (because x is don't-care in casex) => y1=1
    set_in(in_s1, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(in_s0, ::phy_engine::model::digital_node_statement_t::indeterminate_state);
    settle();
    if(ports[i_y1]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::true_state) { return 13; }

    return 0;
}
