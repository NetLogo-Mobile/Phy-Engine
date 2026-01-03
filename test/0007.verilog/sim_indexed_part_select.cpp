#include <array>

#include <phy_engine/phy_engine.h>

int main()
{
    decltype(auto) src = u8R"(
module ips(input [7:0] a, input [2:0] idx, output [3:0] y);
  assign y = a[idx +: 4];
endmodule
)";
    constexpr char8_t top_name[] = u8"ips";

    ::phy_engine::circult cct{};
    cct.set_analyze_type(::phy_engine::analyze_type::TR);
    cct.get_analyze_setting().tr.t_step = 1e-9;
    cct.get_analyze_setting().tr.t_stop = 1e-9;

    auto& nl{cct.get_netlist()};

    auto [a7, a7_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [a6, a6_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [a5, a5_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [a4, a4_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [a3, a3_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [a2, a2_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [a1, a1_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [a0, a0_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [idx2, idx2_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [idx1, idx1_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [idx0, idx0_pos]{add_model(nl, ::phy_engine::model::INPUT{})};

    auto [vmod, vmod_pos]{add_model(nl, ::phy_engine::model::make_verilog_module(
                                      ::fast_io::u8string_view{src, sizeof(src) - 1},
                                      ::fast_io::u8string_view{top_name, sizeof(top_name) - 1}))};

    // Port order: a[7:0], idx[2:0], y[3:0]
    ::std::array<::phy_engine::model::node_t*, 15> ns{};
    for(auto& n: ns) { n = __builtin_addressof(create_node(nl)); }

    // a[7:0] => [7],[6],[5],[4],[3],[2],[1],[0]
    add_to_node(nl, *a7, 0, *ns[0]);
    add_to_node(nl, *vmod, 0, *ns[0]);
    add_to_node(nl, *a6, 0, *ns[1]);
    add_to_node(nl, *vmod, 1, *ns[1]);
    add_to_node(nl, *a5, 0, *ns[2]);
    add_to_node(nl, *vmod, 2, *ns[2]);
    add_to_node(nl, *a4, 0, *ns[3]);
    add_to_node(nl, *vmod, 3, *ns[3]);
    add_to_node(nl, *a3, 0, *ns[4]);
    add_to_node(nl, *vmod, 4, *ns[4]);
    add_to_node(nl, *a2, 0, *ns[5]);
    add_to_node(nl, *vmod, 5, *ns[5]);
    add_to_node(nl, *a1, 0, *ns[6]);
    add_to_node(nl, *vmod, 6, *ns[6]);
    add_to_node(nl, *a0, 0, *ns[7]);
    add_to_node(nl, *vmod, 7, *ns[7]);

    // idx[2:0]
    add_to_node(nl, *idx2, 0, *ns[8]);
    add_to_node(nl, *vmod, 8, *ns[8]);
    add_to_node(nl, *idx1, 0, *ns[9]);
    add_to_node(nl, *vmod, 9, *ns[9]);
    add_to_node(nl, *idx0, 0, *ns[10]);
    add_to_node(nl, *vmod, 10, *ns[10]);

    // y[3:0]
    for(::std::size_t i{}; i < 4; ++i) { add_to_node(nl, *vmod, 11 + i, *ns[11 + i]); }

    if(!cct.analyze()) { return 1; }

    auto set_in = [&](::phy_engine::model::model_base* in, ::phy_engine::model::digital_node_statement_t v) {
        in->ptr->set_attribute(0, {.digital{v}, .type{::phy_engine::model::variant_type::digital}});
    };

    // a = 1010_1100
    set_in(a7, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(a6, ::phy_engine::model::digital_node_statement_t::false_state);
    set_in(a5, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(a4, ::phy_engine::model::digital_node_statement_t::false_state);
    set_in(a3, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(a2, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(a1, ::phy_engine::model::digital_node_statement_t::false_state);
    set_in(a0, ::phy_engine::model::digital_node_statement_t::false_state);

    auto check_y = [&]([[maybe_unused]] ::std::array<::phy_engine::model::digital_node_statement_t, 4> exp) -> bool
    {
        return ns[11]->node_information.dn.state == exp[0] && ns[12]->node_information.dn.state == exp[1] && ns[13]->node_information.dn.state == exp[2] &&
               ns[14]->node_information.dn.state == exp[3];
    };

    // idx=0 => y=a[3:0]=1100
    set_in(idx2, ::phy_engine::model::digital_node_statement_t::false_state);
    set_in(idx1, ::phy_engine::model::digital_node_statement_t::false_state);
    set_in(idx0, ::phy_engine::model::digital_node_statement_t::false_state);
    cct.digital_clk();
    if(!check_y({::phy_engine::model::digital_node_statement_t::true_state,
                ::phy_engine::model::digital_node_statement_t::true_state,
                ::phy_engine::model::digital_node_statement_t::false_state,
                ::phy_engine::model::digital_node_statement_t::false_state}))
    {
        return 1;
    }

    // idx=2 => y=a[5:2]=1011
    set_in(idx2, ::phy_engine::model::digital_node_statement_t::false_state);
    set_in(idx1, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(idx0, ::phy_engine::model::digital_node_statement_t::false_state);
    cct.digital_clk();
    if(!check_y({::phy_engine::model::digital_node_statement_t::true_state,
                ::phy_engine::model::digital_node_statement_t::false_state,
                ::phy_engine::model::digital_node_statement_t::true_state,
                ::phy_engine::model::digital_node_statement_t::true_state}))
    {
        return 1;
    }

    // idx=4 => y=a[7:4]=1010
    set_in(idx2, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(idx1, ::phy_engine::model::digital_node_statement_t::false_state);
    set_in(idx0, ::phy_engine::model::digital_node_statement_t::false_state);
    cct.digital_clk();
    if(!check_y({::phy_engine::model::digital_node_statement_t::true_state,
                ::phy_engine::model::digital_node_statement_t::false_state,
                ::phy_engine::model::digital_node_statement_t::true_state,
                ::phy_engine::model::digital_node_statement_t::false_state}))
    {
        return 1;
    }

    return 0;
}

