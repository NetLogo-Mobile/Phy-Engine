#include <cstddef>

#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/digital.h>
#include <phy_engine/verilog/digital/pe_synth.h>

int main()
{
    decltype(auto) src = u8R"(
module top(input a, output reg y);
  always @* begin
    #1 y = a;
  end
endmodule
)";

    ::phy_engine::circult cct{};
    cct.set_analyze_type(::phy_engine::analyze_type::TR);
    cct.get_analyze_setting().tr.t_step = 1e-9;
    cct.get_analyze_setting().tr.t_stop = 1e-9;

    auto& nl = cct.get_netlist();

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

    auto [in_a, p0] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{});
    auto [out_y, p1] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
    (void)p0;
    (void)p1;
    if(in_a == nullptr || out_y == nullptr) { return 4; }

    if(!::phy_engine::netlist::add_to_node(nl, *in_a, 0, *ports[0])) { return 5; }
    if(!::phy_engine::netlist::add_to_node(nl, *out_y, 0, *ports[1])) { return 6; }

    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt{
        .allow_inout = true,
        .allow_multi_driver = true,
    };

    // Delays are a simulation feature; synthesis must reject them.
    if(::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, ports, &err, opt)) { return 7; }
    if(err.message.empty()) { return 8; }
    return 0;
}

