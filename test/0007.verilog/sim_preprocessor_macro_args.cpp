#include <phy_engine/phy_engine.h>

int main()
{
    decltype(auto) src = u8R"(
`define AND2(a,b) ((a) & (b))
`define MUX(s,a,b) ((s) ? (a) : (b))
`define ZERO() 1'b0

module top(input a, input b, input s, output y);
  assign y = `MUX(s, `AND2(a,b), a | `ZERO());
endmodule
)";
    constexpr char8_t top_name[] = u8"top";

    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    c.get_analyze_setting().tr.t_step = 1e-9;
    c.get_analyze_setting().tr.t_stop = 1e-9;

    auto& nl{c.get_netlist()};

    auto [ina, ina_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [inb, inb_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [ins, ins_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [vmod, vmod_pos]{add_model(nl, ::phy_engine::model::make_verilog_module(
                                      ::fast_io::u8string_view{src, sizeof(src) - 1},
                                      ::fast_io::u8string_view{top_name, sizeof(top_name) - 1}))};

    auto& na{create_node(nl)};
    auto& nb{create_node(nl)};
    auto& ns{create_node(nl)};
    auto& ny{create_node(nl)};

    add_to_node(nl, *ina, 0, na);
    add_to_node(nl, *inb, 0, nb);
    add_to_node(nl, *ins, 0, ns);

    add_to_node(nl, *vmod, 0, na);
    add_to_node(nl, *vmod, 1, nb);
    add_to_node(nl, *vmod, 2, ns);
    add_to_node(nl, *vmod, 3, ny);

    if(!c.analyze()) { return 1; }

    auto set_in = [&](::phy_engine::model::model_base* in, ::phy_engine::model::digital_node_statement_t v) {
        in->ptr->set_attribute(0, {.digital{v}, .type{::phy_engine::model::variant_type::digital}});
    };

    set_in(ina, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(inb, ::phy_engine::model::digital_node_statement_t::false_state);
    set_in(ins, ::phy_engine::model::digital_node_statement_t::false_state);
    c.digital_clk();
    if(ny.node_information.dn.state != ::phy_engine::model::digital_node_statement_t::true_state) { return 1; }

    set_in(ins, ::phy_engine::model::digital_node_statement_t::true_state);
    c.digital_clk();
    if(ny.node_information.dn.state != ::phy_engine::model::digital_node_statement_t::false_state) { return 1; }

    set_in(inb, ::phy_engine::model::digital_node_statement_t::true_state);
    c.digital_clk();
    if(ny.node_information.dn.state != ::phy_engine::model::digital_node_statement_t::true_state) { return 1; }

    return 0;
}

