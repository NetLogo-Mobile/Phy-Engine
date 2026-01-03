#include <array>

#include <phy_engine/phy_engine.h>

int main()
{
    decltype(auto) src = u8R"(
module v_and(input [3:0] a, input [3:0] b, output [3:0] y);
  assign y = a & b;
endmodule
)";
    constexpr char8_t top_name[] = u8"v_and";

    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    c.get_analyze_setting().tr.t_step = 1e-9;
    c.get_analyze_setting().tr.t_stop = 1e-9;

    auto& nl{c.get_netlist()};

    auto [a3, a3_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [a2, a2_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [a1, a1_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [a0, a0_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [b3, b3_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [b2, b2_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [b1, b1_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [b0, b0_pos]{add_model(nl, ::phy_engine::model::INPUT{})};

    auto [vmod, vmod_pos]{add_model(nl, ::phy_engine::model::make_verilog_module(
                                      ::fast_io::u8string_view{src, sizeof(src) - 1},
                                      ::fast_io::u8string_view{top_name, sizeof(top_name) - 1}))};

    // Port expansion order for [3:0]: [3],[2],[1],[0]
    // v_and ports: a[3:0], b[3:0], y[3:0]
    ::std::array<::phy_engine::model::node_t*, 12> ns{};
    for(auto& n: ns) { n = __builtin_addressof(create_node(nl)); }

    // a[3:0]
    add_to_node(nl, *a3, 0, *ns[0]);
    add_to_node(nl, *vmod, 0, *ns[0]);
    add_to_node(nl, *a2, 0, *ns[1]);
    add_to_node(nl, *vmod, 1, *ns[1]);
    add_to_node(nl, *a1, 0, *ns[2]);
    add_to_node(nl, *vmod, 2, *ns[2]);
    add_to_node(nl, *a0, 0, *ns[3]);
    add_to_node(nl, *vmod, 3, *ns[3]);

    // b[3:0]
    add_to_node(nl, *b3, 0, *ns[4]);
    add_to_node(nl, *vmod, 4, *ns[4]);
    add_to_node(nl, *b2, 0, *ns[5]);
    add_to_node(nl, *vmod, 5, *ns[5]);
    add_to_node(nl, *b1, 0, *ns[6]);
    add_to_node(nl, *vmod, 6, *ns[6]);
    add_to_node(nl, *b0, 0, *ns[7]);
    add_to_node(nl, *vmod, 7, *ns[7]);

    // y[3:0]
    add_to_node(nl, *vmod, 8, *ns[8]);
    add_to_node(nl, *vmod, 9, *ns[9]);
    add_to_node(nl, *vmod, 10, *ns[10]);
    add_to_node(nl, *vmod, 11, *ns[11]);

    if(!c.analyze()) { return 1; }

    auto set_in = [&](::phy_engine::model::model_base* in, ::phy_engine::model::digital_node_statement_t v) {
        in->ptr->set_attribute(0, {.digital{v}, .type{::phy_engine::model::variant_type::digital}});
    };

    // a = 1010, b = 1100 => y = 1000 (bitwise and)
    set_in(a3, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(a2, ::phy_engine::model::digital_node_statement_t::false_state);
    set_in(a1, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(a0, ::phy_engine::model::digital_node_statement_t::false_state);

    set_in(b3, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(b2, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(b1, ::phy_engine::model::digital_node_statement_t::false_state);
    set_in(b0, ::phy_engine::model::digital_node_statement_t::false_state);

    c.digital_clk();

    if(ns[8]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::true_state) { return 1; }
    if(ns[9]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::false_state) { return 1; }
    if(ns[10]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::false_state) { return 1; }
    if(ns[11]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::false_state) { return 1; }

    return 0;
}

