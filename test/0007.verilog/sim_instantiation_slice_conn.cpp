#include <array>

#include <phy_engine/phy_engine.h>

int main()
{
    decltype(auto) src = u8R"(
module child(input [1:0] a, output [1:0] y);
  assign y = a;
endmodule

module top(input [3:0] b, output [1:0] y);
  child u0(.a(b[3:2]), .y(y));
endmodule
)";
    constexpr char8_t top_name[] = u8"top";

    ::phy_engine::circult cct{};
    cct.set_analyze_type(::phy_engine::analyze_type::TR);
    cct.get_analyze_setting().tr.t_step = 1e-9;
    cct.get_analyze_setting().tr.t_stop = 1e-9;

    auto& nl{cct.get_netlist()};

    auto [b3, b3_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [b2, b2_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [b1, b1_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [b0, b0_pos]{add_model(nl, ::phy_engine::model::INPUT{})};

    auto [vmod, vmod_pos]{add_model(nl, ::phy_engine::model::make_verilog_module(
                                      ::fast_io::u8string_view{src, sizeof(src) - 1},
                                      ::fast_io::u8string_view{top_name, sizeof(top_name) - 1}))};

    // Port order: b[3:0], y[1:0]
    ::std::array<::phy_engine::model::node_t*, 6> ns{};
    for(auto& n: ns) { n = __builtin_addressof(create_node(nl)); }

    // b[3:0]
    add_to_node(nl, *b3, 0, *ns[0]);
    add_to_node(nl, *vmod, 0, *ns[0]);
    add_to_node(nl, *b2, 0, *ns[1]);
    add_to_node(nl, *vmod, 1, *ns[1]);
    add_to_node(nl, *b1, 0, *ns[2]);
    add_to_node(nl, *vmod, 2, *ns[2]);
    add_to_node(nl, *b0, 0, *ns[3]);
    add_to_node(nl, *vmod, 3, *ns[3]);

    // y[1:0]
    add_to_node(nl, *vmod, 4, *ns[4]);
    add_to_node(nl, *vmod, 5, *ns[5]);

    if(!cct.analyze()) { return 1; }

    auto set_in = [&](::phy_engine::model::model_base* in, ::phy_engine::model::digital_node_statement_t v) {
        in->ptr->set_attribute(0, {.digital{v}, .type{::phy_engine::model::variant_type::digital}});
    };

    // b = 1010 => b[3:2] = 10 => y = 10
    set_in(b3, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(b2, ::phy_engine::model::digital_node_statement_t::false_state);
    set_in(b1, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(b0, ::phy_engine::model::digital_node_statement_t::false_state);
    cct.digital_clk();

    if(ns[4]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::true_state) { return 1; }
    if(ns[5]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::false_state) { return 1; }

    return 0;
}

