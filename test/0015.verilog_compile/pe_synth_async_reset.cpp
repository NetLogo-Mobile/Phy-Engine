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
module top(input clk, input rst_n, input d, output reg q);
  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) q <= 0;
    else q <= d;
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

    auto [in_clk, p0] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
    auto [in_rst, p1] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
    auto [in_d, p2] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
    auto [out_q, p3] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
    (void)p0;
    (void)p1;
    (void)p2;
    (void)p3;
    if(in_clk == nullptr || in_rst == nullptr || in_d == nullptr || out_q == nullptr) { return 4; }

    if(!::phy_engine::netlist::add_to_node(nl, *in_clk, 0, *ports[0])) { return 5; }
    if(!::phy_engine::netlist::add_to_node(nl, *in_rst, 0, *ports[1])) { return 6; }
    if(!::phy_engine::netlist::add_to_node(nl, *in_d, 0, *ports[2])) { return 7; }
    if(!::phy_engine::netlist::add_to_node(nl, *out_q, 0, *ports[3])) { return 8; }

    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt{
        .allow_inout = true,
        .allow_multi_driver = true,
    };
    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, ports, &err, opt)) { return 9; }

    if(!c.analyze()) { return 10; }

    auto tick = [&]() noexcept { c.digital_clk(); };

    // Reset asserted => q=0 without clock edge.
    (void)in_rst->ptr->set_attribute(0, dv(::phy_engine::model::digital_node_statement_t::false_state));
    (void)in_d->ptr->set_attribute(0, dv(::phy_engine::model::digital_node_statement_t::true_state));
    tick();
    if(ports[3]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::false_state) { return 11; }

    // Deassert reset then posedge clk captures d=1.
    (void)in_rst->ptr->set_attribute(0, dv(::phy_engine::model::digital_node_statement_t::true_state));
    tick();
    (void)in_clk->ptr->set_attribute(0, dv(::phy_engine::model::digital_node_statement_t::true_state));
    tick();
    if(ports[3]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::true_state) { return 12; }

    // Async reset again => q=0.
    (void)in_rst->ptr->set_attribute(0, dv(::phy_engine::model::digital_node_statement_t::false_state));
    tick();
    if(ports[3]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::false_state) { return 13; }

    return 0;
}

