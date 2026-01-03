#include <phy_engine/phy_engine.h>

int main()
{
    decltype(auto) src = u8R"(
module child(input [1:0] a, output [1:0] y);
  assign y = a;
endmodule

module top(input b0, input b1, output o0, output o1);
  child u0(.a({b1, b0}), .y({o1, o0}));
endmodule
)";
    constexpr char8_t top_name[] = u8"top";

    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    c.get_analyze_setting().tr.t_step = 1e-9;
    c.get_analyze_setting().tr.t_stop = 1e-9;

    auto& nl{c.get_netlist()};

    auto [inb0, inb0_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [inb1, inb1_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [vmod, vmod_pos]{add_model(nl, ::phy_engine::model::make_verilog_module(
                                      ::fast_io::u8string_view{src, sizeof(src) - 1},
                                      ::fast_io::u8string_view{top_name, sizeof(top_name) - 1}))};

    auto& nb0{create_node(nl)};
    auto& nb1{create_node(nl)};
    auto& no0{create_node(nl)};
    auto& no1{create_node(nl)};

    add_to_node(nl, *inb0, 0, nb0);
    add_to_node(nl, *inb1, 0, nb1);

    // top(b0,b1,o0,o1) in port order
    add_to_node(nl, *vmod, 0, nb0);
    add_to_node(nl, *vmod, 1, nb1);
    add_to_node(nl, *vmod, 2, no0);
    add_to_node(nl, *vmod, 3, no1);

    if(!c.analyze()) { return 1; }

    auto set_in = [&](::phy_engine::model::model_base* in, ::phy_engine::model::digital_node_statement_t v) {
        in->ptr->set_attribute(0, {.digital{v}, .type{::phy_engine::model::variant_type::digital}});
    };

    // {b1,b0} => {o1,o0}
    set_in(inb1, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(inb0, ::phy_engine::model::digital_node_statement_t::false_state);
    c.digital_clk();
    if(no1.node_information.dn.state != ::phy_engine::model::digital_node_statement_t::true_state) { return 1; }
    if(no0.node_information.dn.state != ::phy_engine::model::digital_node_statement_t::false_state) { return 1; }

    set_in(inb1, ::phy_engine::model::digital_node_statement_t::false_state);
    set_in(inb0, ::phy_engine::model::digital_node_statement_t::true_state);
    c.digital_clk();
    if(no1.node_information.dn.state != ::phy_engine::model::digital_node_statement_t::false_state) { return 1; }
    if(no0.node_information.dn.state != ::phy_engine::model::digital_node_statement_t::true_state) { return 1; }

    return 0;
}

