#include <phy_engine/phy_engine.h>

int main()
{
    decltype(auto) src = u8R"(
module selmod #(parameter SEL = 0, localparam L = 0) (input a, input b, output y0, output y1);
  assign y0 = SEL ? a : b;
  assign y1 = L ? a : b;
endmodule

module top(input a, input b, output y_sel1, output y_sel0, output y_local);
  selmod #(.SEL(1), .L(1)) u1(.a(a), .b(b), .y0(y_sel1), .y1(y_local));
  selmod #(0) u0(.a(a), .b(b), .y0(y_sel0), .y1());
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
    auto [vmod, vmod_pos]{add_model(nl, ::phy_engine::model::make_verilog_module(
                                      ::fast_io::u8string_view{src, sizeof(src) - 1},
                                      ::fast_io::u8string_view{top_name, sizeof(top_name) - 1}))};

    auto& na{create_node(nl)};
    auto& nb{create_node(nl)};
    auto& ny_sel1{create_node(nl)};
    auto& ny_sel0{create_node(nl)};
    auto& ny_local{create_node(nl)};

    add_to_node(nl, *ina, 0, na);
    add_to_node(nl, *inb, 0, nb);

    add_to_node(nl, *vmod, 0, na);
    add_to_node(nl, *vmod, 1, nb);
    add_to_node(nl, *vmod, 2, ny_sel1);
    add_to_node(nl, *vmod, 3, ny_sel0);
    add_to_node(nl, *vmod, 4, ny_local);

    if(!c.analyze())
    {
        ::fast_io::io::perr("sim_param_localparam_instantiation: analyze failed\n");
        return 1;
    }

    auto set_in = [&](::phy_engine::model::model_base* in, ::phy_engine::model::digital_node_statement_t v) {
        in->ptr->set_attribute(0, {.digital{v}, .type{::phy_engine::model::variant_type::digital}});
    };

    set_in(ina, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(inb, ::phy_engine::model::digital_node_statement_t::false_state);
    c.digital_clk();
    if(ny_sel1.node_information.dn.state != ::phy_engine::model::digital_node_statement_t::true_state)
    {
        ::fast_io::io::perr("sim_param_localparam_instantiation: y_sel1 mismatch\n");
        return 1;
    }
    if(ny_sel0.node_information.dn.state != ::phy_engine::model::digital_node_statement_t::false_state)
    {
        ::fast_io::io::perr("sim_param_localparam_instantiation: y_sel0 mismatch\n");
        return 1;
    }
    // localparam override must be ignored: L=0 => selects b (0)
    if(ny_local.node_information.dn.state != ::phy_engine::model::digital_node_statement_t::false_state)
    {
        ::fast_io::io::perr("sim_param_localparam_instantiation: y_local mismatch (localparam override applied?)\n");
        return 1;
    }

    set_in(ina, ::phy_engine::model::digital_node_statement_t::false_state);
    set_in(inb, ::phy_engine::model::digital_node_statement_t::true_state);
    c.digital_clk();
    if(ny_sel1.node_information.dn.state != ::phy_engine::model::digital_node_statement_t::false_state)
    {
        ::fast_io::io::perr("sim_param_localparam_instantiation: y_sel1 mismatch (case 2)\n");
        return 1;
    }
    if(ny_sel0.node_information.dn.state != ::phy_engine::model::digital_node_statement_t::true_state)
    {
        ::fast_io::io::perr("sim_param_localparam_instantiation: y_sel0 mismatch (case 2)\n");
        return 1;
    }
    if(ny_local.node_information.dn.state != ::phy_engine::model::digital_node_statement_t::true_state)
    {
        ::fast_io::io::perr("sim_param_localparam_instantiation: y_local mismatch (case 2)\n");
        return 1;
    }

    return 0;
}
