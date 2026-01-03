#include <array>

#include <phy_engine/phy_engine.h>

int main()
{
    decltype(auto) src = u8R"(
module child(input [3:0] a, output [3:0] y);
  assign y = a;
endmodule

module top(output [3:0] y);
  child u0(.a(4'hA), .y(y));
endmodule
)";
    constexpr char8_t top_name[] = u8"top";

    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    c.get_analyze_setting().tr.t_step = 1e-9;
    c.get_analyze_setting().tr.t_stop = 1e-9;

    auto& nl{c.get_netlist()};

    auto [vmod, vmod_pos]{add_model(nl, ::phy_engine::model::make_verilog_module(
                                      ::fast_io::u8string_view{src, sizeof(src) - 1},
                                      ::fast_io::u8string_view{top_name, sizeof(top_name) - 1}))};

    // top ports: y[3:0] => y[3],y[2],y[1],y[0]
    ::std::array<::phy_engine::model::node_t*, 4> y{};
    for(auto& n: y) { n = __builtin_addressof(create_node(nl)); }

    add_to_node(nl, *vmod, 0, *y[0]);
    add_to_node(nl, *vmod, 1, *y[1]);
    add_to_node(nl, *vmod, 2, *y[2]);
    add_to_node(nl, *vmod, 3, *y[3]);

    if(!c.analyze()) { return 1; }
    c.digital_clk();

    // 4'hA = 1010
    if(y[0]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::true_state) { return 1; }
    if(y[1]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::false_state) { return 1; }
    if(y[2]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::true_state) { return 1; }
    if(y[3]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::false_state) { return 1; }

    return 0;
}

