#include <array>

#include <phy_engine/phy_engine.h>

int main()
{
    decltype(auto) src = u8R"(
module lhs_idx(input [3:0] a, input [1:0] idx, output reg [3:0] y);
  always @* begin
    y = 4'b0000;
    y[idx] = a[idx];
  end
endmodule
)";
    constexpr char8_t top_name[] = u8"lhs_idx";

    ::phy_engine::circult cct{};
    cct.set_analyze_type(::phy_engine::analyze_type::TR);
    cct.get_analyze_setting().tr.t_step = 1e-9;
    cct.get_analyze_setting().tr.t_stop = 1e-9;

    auto& nl{cct.get_netlist()};

    auto [a3, a3_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [a2, a2_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [a1, a1_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [a0, a0_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [idx1, idx1_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [idx0, idx0_pos]{add_model(nl, ::phy_engine::model::INPUT{})};

    auto [vmod, vmod_pos]{add_model(nl, ::phy_engine::model::make_verilog_module(
                                      ::fast_io::u8string_view{src, sizeof(src) - 1},
                                      ::fast_io::u8string_view{top_name, sizeof(top_name) - 1}))};

    // Port order: a[3:0], idx[1:0], y[3:0]
    ::std::array<::phy_engine::model::node_t*, 10> ns{};
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

    // idx[1:0]
    add_to_node(nl, *idx1, 0, *ns[4]);
    add_to_node(nl, *vmod, 4, *ns[4]);
    add_to_node(nl, *idx0, 0, *ns[5]);
    add_to_node(nl, *vmod, 5, *ns[5]);

    // y[3:0]
    for(::std::size_t i{}; i < 4; ++i) { add_to_node(nl, *vmod, 6 + i, *ns[6 + i]); }

    if(!cct.analyze()) { return 1; }

    auto set_in = [&](::phy_engine::model::model_base* in, ::phy_engine::model::digital_node_statement_t v) {
        in->ptr->set_attribute(0, {.digital{v}, .type{::phy_engine::model::variant_type::digital}});
    };

    // a = 1011
    set_in(a3, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(a2, ::phy_engine::model::digital_node_statement_t::false_state);
    set_in(a1, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(a0, ::phy_engine::model::digital_node_statement_t::true_state);

    auto check_y = [&]([[maybe_unused]] ::std::array<::phy_engine::model::digital_node_statement_t, 4> exp) -> bool
    {
        return ns[6]->node_information.dn.state == exp[0] && ns[7]->node_information.dn.state == exp[1] && ns[8]->node_information.dn.state == exp[2] &&
               ns[9]->node_information.dn.state == exp[3];
    };

    // idx = 0 => y = 0001
    set_in(idx1, ::phy_engine::model::digital_node_statement_t::false_state);
    set_in(idx0, ::phy_engine::model::digital_node_statement_t::false_state);
    cct.digital_clk();
    if(!check_y({::phy_engine::model::digital_node_statement_t::false_state,
                ::phy_engine::model::digital_node_statement_t::false_state,
                ::phy_engine::model::digital_node_statement_t::false_state,
                ::phy_engine::model::digital_node_statement_t::true_state}))
    {
        return 1;
    }

    // idx = 1 => y = 0010
    set_in(idx1, ::phy_engine::model::digital_node_statement_t::false_state);
    set_in(idx0, ::phy_engine::model::digital_node_statement_t::true_state);
    cct.digital_clk();
    if(!check_y({::phy_engine::model::digital_node_statement_t::false_state,
                ::phy_engine::model::digital_node_statement_t::false_state,
                ::phy_engine::model::digital_node_statement_t::true_state,
                ::phy_engine::model::digital_node_statement_t::false_state}))
    {
        return 1;
    }

    // idx = 2 => y = 0000 (a[2]=0)
    set_in(idx1, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(idx0, ::phy_engine::model::digital_node_statement_t::false_state);
    cct.digital_clk();
    if(!check_y({::phy_engine::model::digital_node_statement_t::false_state,
                ::phy_engine::model::digital_node_statement_t::false_state,
                ::phy_engine::model::digital_node_statement_t::false_state,
                ::phy_engine::model::digital_node_statement_t::false_state}))
    {
        return 1;
    }

    // idx = 3 => y = 1000
    set_in(idx1, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(idx0, ::phy_engine::model::digital_node_statement_t::true_state);
    cct.digital_clk();
    if(!check_y({::phy_engine::model::digital_node_statement_t::true_state,
                ::phy_engine::model::digital_node_statement_t::false_state,
                ::phy_engine::model::digital_node_statement_t::false_state,
                ::phy_engine::model::digital_node_statement_t::false_state}))
    {
        return 1;
    }

    return 0;
}

