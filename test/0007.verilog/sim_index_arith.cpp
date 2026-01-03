#include <array>

#include <phy_engine/phy_engine.h>

int main()
{
    decltype(auto) src = u8R"(
module idx_const(input [3:0] a, output y);
  assign y = a[3-1];
endmodule
)";
    constexpr char8_t top_name[] = u8"idx_const";

    ::phy_engine::circult cct{};
    cct.set_analyze_type(::phy_engine::analyze_type::TR);
    cct.get_analyze_setting().tr.t_step = 1e-9;
    cct.get_analyze_setting().tr.t_stop = 1e-9;

    auto& nl{cct.get_netlist()};

    auto [a3, a3_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [a2, a2_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [a1, a1_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [a0, a0_pos]{add_model(nl, ::phy_engine::model::INPUT{})};

    auto [vmod, vmod_pos]{add_model(nl, ::phy_engine::model::make_verilog_module(
                                      ::fast_io::u8string_view{src, sizeof(src) - 1},
                                      ::fast_io::u8string_view{top_name, sizeof(top_name) - 1}))};

    // Port expansion order: a[3],a[2],a[1],a[0], y
    ::std::array<::phy_engine::model::node_t*, 5> ns{};
    for(auto& n: ns) { n = __builtin_addressof(create_node(nl)); }

    add_to_node(nl, *a3, 0, *ns[0]);
    add_to_node(nl, *vmod, 0, *ns[0]);
    add_to_node(nl, *a2, 0, *ns[1]);
    add_to_node(nl, *vmod, 1, *ns[1]);
    add_to_node(nl, *a1, 0, *ns[2]);
    add_to_node(nl, *vmod, 2, *ns[2]);
    add_to_node(nl, *a0, 0, *ns[3]);
    add_to_node(nl, *vmod, 3, *ns[3]);

    add_to_node(nl, *vmod, 4, *ns[4]);

    if(!cct.analyze()) { return 1; }

    auto set_in = [&](::phy_engine::model::model_base* in, ::phy_engine::model::digital_node_statement_t v) {
        in->ptr->set_attribute(0, {.digital{v}, .type{::phy_engine::model::variant_type::digital}});
    };

    // a = 1011 => a[2] = 0, so y should be 0
    set_in(a3, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(a2, ::phy_engine::model::digital_node_statement_t::false_state);
    set_in(a1, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(a0, ::phy_engine::model::digital_node_statement_t::true_state);

    cct.digital_clk();

    if(ns[4]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::false_state) { return 1; }
    return 0;
}
