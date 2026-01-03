#include <array>

#include <phy_engine/phy_engine.h>

int main()
{
    decltype(auto) src = u8R"(
module concat4(input a, input b, input c, input d, output [3:0] y);
  assign y = {a, b, c, d};
endmodule
)";
    constexpr char8_t top_name[] = u8"concat4";

    ::phy_engine::circult cct{};
    cct.set_analyze_type(::phy_engine::analyze_type::TR);
    cct.get_analyze_setting().tr.t_step = 1e-9;
    cct.get_analyze_setting().tr.t_stop = 1e-9;

    auto& nl{cct.get_netlist()};

    auto [a, a_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [b, b_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [c, c_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [d, d_pos]{add_model(nl, ::phy_engine::model::INPUT{})};

    auto [vmod, vmod_pos]{add_model(nl, ::phy_engine::model::make_verilog_module(
                                      ::fast_io::u8string_view{src, sizeof(src) - 1},
                                      ::fast_io::u8string_view{top_name, sizeof(top_name) - 1}))};

    // Port order: a, b, c, d, y[3], y[2], y[1], y[0]
    ::std::array<::phy_engine::model::node_t*, 8> ns{};
    for(auto& n: ns) { n = __builtin_addressof(create_node(nl)); }

    add_to_node(nl, *a, 0, *ns[0]);
    add_to_node(nl, *vmod, 0, *ns[0]);
    add_to_node(nl, *b, 0, *ns[1]);
    add_to_node(nl, *vmod, 1, *ns[1]);
    add_to_node(nl, *c, 0, *ns[2]);
    add_to_node(nl, *vmod, 2, *ns[2]);
    add_to_node(nl, *d, 0, *ns[3]);
    add_to_node(nl, *vmod, 3, *ns[3]);

    for(::std::size_t i{}; i < 4; ++i) { add_to_node(nl, *vmod, 4 + i, *ns[4 + i]); }

    if(!cct.analyze()) { return 1; }

    auto set_in = [&](::phy_engine::model::model_base* in, ::phy_engine::model::digital_node_statement_t v) {
        in->ptr->set_attribute(0, {.digital{v}, .type{::phy_engine::model::variant_type::digital}});
    };

    // {a,b,c,d} = 1010
    set_in(a, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(b, ::phy_engine::model::digital_node_statement_t::false_state);
    set_in(c, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(d, ::phy_engine::model::digital_node_statement_t::false_state);

    cct.digital_clk();

    if(ns[4]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::true_state) { return 1; }
    if(ns[5]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::false_state) { return 1; }
    if(ns[6]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::true_state) { return 1; }
    if(ns[7]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::false_state) { return 1; }

    return 0;
}

