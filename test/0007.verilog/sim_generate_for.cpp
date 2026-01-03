#include <phy_engine/phy_engine.h>

int main()
{
    decltype(auto) src = u8R"(
module and2(input a, input b, output y);
  assign y = a & b;
endmodule

module top(input [1:0] a, input [1:0] b, output [1:0] y);
  genvar i;
  generate
    for(i=0; i<2; i=i+1) begin : g
      and2 u(.a(a[i]), .b(b[i]), .y(y[i]));
    end
  endgenerate
endmodule
)";
    constexpr char8_t top_name[] = u8"top";

    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    c.get_analyze_setting().tr.t_step = 1e-9;
    c.get_analyze_setting().tr.t_stop = 1e-9;

    auto& nl{c.get_netlist()};

    auto [a1, a1_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [a0, a0_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [b1, b1_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [b0, b0_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [vmod, vmod_pos]{add_model(nl, ::phy_engine::model::make_verilog_module(
                                      ::fast_io::u8string_view{src, sizeof(src) - 1},
                                      ::fast_io::u8string_view{top_name, sizeof(top_name) - 1}))};

    auto& na1{create_node(nl)};
    auto& na0{create_node(nl)};
    auto& nb1{create_node(nl)};
    auto& nb0{create_node(nl)};
    auto& ny1{create_node(nl)};
    auto& ny0{create_node(nl)};

    add_to_node(nl, *a1, 0, na1);
    add_to_node(nl, *a0, 0, na0);
    add_to_node(nl, *b1, 0, nb1);
    add_to_node(nl, *b0, 0, nb0);

    // Port expansion order: a[1],a[0], b[1],b[0], y[1],y[0]
    add_to_node(nl, *vmod, 0, na1);
    add_to_node(nl, *vmod, 1, na0);
    add_to_node(nl, *vmod, 2, nb1);
    add_to_node(nl, *vmod, 3, nb0);
    add_to_node(nl, *vmod, 4, ny1);
    add_to_node(nl, *vmod, 5, ny0);

    if(!c.analyze()) { return 1; }

    auto set_in = [&](::phy_engine::model::model_base* in, ::phy_engine::model::digital_node_statement_t v) {
        in->ptr->set_attribute(0, {.digital{v}, .type{::phy_engine::model::variant_type::digital}});
    };

    set_in(a1, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(a0, ::phy_engine::model::digital_node_statement_t::false_state);
    set_in(b1, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(b0, ::phy_engine::model::digital_node_statement_t::true_state);
    c.digital_clk();
    if(ny1.node_information.dn.state != ::phy_engine::model::digital_node_statement_t::true_state) { return 1; }
    if(ny0.node_information.dn.state != ::phy_engine::model::digital_node_statement_t::false_state) { return 1; }

    set_in(a1, ::phy_engine::model::digital_node_statement_t::false_state);
    set_in(a0, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(b1, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(b0, ::phy_engine::model::digital_node_statement_t::false_state);
    c.digital_clk();
    if(ny1.node_information.dn.state != ::phy_engine::model::digital_node_statement_t::false_state) { return 1; }
    if(ny0.node_information.dn.state != ::phy_engine::model::digital_node_statement_t::false_state) { return 1; }

    return 0;
}

