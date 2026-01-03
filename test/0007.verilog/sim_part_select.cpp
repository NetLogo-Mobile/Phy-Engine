#include <array>

#include <phy_engine/phy_engine.h>

int main()
{
    decltype(auto) src = u8R"(
module ps(input [3:0] a, output [1:0] y1, output [1:0] y2, output [3:0] yswap);
  assign y1 = a[2:1];
  assign y2 = a[1:2];
  assign yswap[3:2] = a[1:0];
  assign yswap[1:0] = a[3:2];
endmodule
)";
    constexpr char8_t top_name[] = u8"ps";

    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    c.get_analyze_setting().tr.t_step = 1e-9;
    c.get_analyze_setting().tr.t_stop = 1e-9;

    auto& nl{c.get_netlist()};

    auto [a3, a3_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [a2, a2_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [a1, a1_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [a0, a0_pos]{add_model(nl, ::phy_engine::model::INPUT{})};

    auto [vmod, vmod_pos]{add_model(nl, ::phy_engine::model::make_verilog_module(
                                      ::fast_io::u8string_view{src, sizeof(src) - 1},
                                      ::fast_io::u8string_view{top_name, sizeof(top_name) - 1}))};

    // Port expansion order:
    // a[3:0] => a[3],a[2],a[1],a[0]
    // y1[1:0] => y1[1],y1[0]
    // y2[1:0] => y2[1],y2[0]
    // yswap[3:0] => yswap[3],yswap[2],yswap[1],yswap[0]
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

    // y1[1:0], y2[1:0], yswap[3:0]
    for(::std::size_t i{}; i < 8; ++i) { add_to_node(nl, *vmod, 4 + i, *ns[4 + i]); }

    if(!c.analyze()) { return 1; }

    auto set_in = [&](::phy_engine::model::model_base* in, ::phy_engine::model::digital_node_statement_t v) {
        in->ptr->set_attribute(0, {.digital{v}, .type{::phy_engine::model::variant_type::digital}});
    };

    // a = 1101
    set_in(a3, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(a2, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(a1, ::phy_engine::model::digital_node_statement_t::false_state);
    set_in(a0, ::phy_engine::model::digital_node_statement_t::true_state);

    c.digital_clk();

    // y1 = a[2:1] = 10
    if(ns[4]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::true_state) { return 1; }
    if(ns[5]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::false_state) { return 1; }

    // y2 = a[1:2] = 01
    if(ns[6]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::false_state) { return 1; }
    if(ns[7]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::true_state) { return 1; }

    // yswap[3:2] = a[1:0] = 01; yswap[1:0] = a[3:2] = 11 => 0111
    if(ns[8]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::false_state) { return 1; }
    if(ns[9]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::true_state) { return 1; }
    if(ns[10]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::true_state) { return 1; }
    if(ns[11]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::true_state) { return 1; }

    return 0;
}

