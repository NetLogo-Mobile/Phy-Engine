#include <phy_engine/phy_engine.h>

int main()
{
    decltype(auto) src = u8R"(
module dff_delay(input clk, input d, output reg q);
  always @(posedge clk) begin
    #2 q <= d;
  end
endmodule
)";
    constexpr char8_t top_name[] = u8"dff_delay";

    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    c.get_analyze_setting().tr.t_step = 1e-9;
    c.get_analyze_setting().tr.t_stop = 1e-9;

    auto& nl{c.get_netlist()};

    auto [inclk, inclk_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [ind, ind_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [vmod, vmod_pos]{add_model(nl, ::phy_engine::model::make_verilog_module(
                                      ::fast_io::u8string_view{src, sizeof(src) - 1},
                                      ::fast_io::u8string_view{top_name, sizeof(top_name) - 1}))};

    auto& nclk{create_node(nl)};
    auto& nd{create_node(nl)};
    auto& nq{create_node(nl)};

    add_to_node(nl, *inclk, 0, nclk);
    add_to_node(nl, *ind, 0, nd);

    add_to_node(nl, *vmod, 0, nclk);
    add_to_node(nl, *vmod, 1, nd);
    add_to_node(nl, *vmod, 2, nq);

    if(!c.analyze()) { return 1; }

    auto set_in = [&](::phy_engine::model::model_base* in, ::phy_engine::model::digital_node_statement_t v) {
        in->ptr->set_attribute(0, {.digital{v}, .type{::phy_engine::model::variant_type::digital}});
    };

    // init
    set_in(inclk, ::phy_engine::model::digital_node_statement_t::false_state);
    set_in(ind, ::phy_engine::model::digital_node_statement_t::false_state);
    c.digital_clk();

    // schedule q update (posedge clk), but delayed by 2 ticks
    set_in(ind, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(inclk, ::phy_engine::model::digital_node_statement_t::true_state);
    c.digital_clk();

    if(nq.node_information.dn.state == ::phy_engine::model::digital_node_statement_t::true_state) { return 1; }

    // tick +1
    c.digital_clk();
    if(nq.node_information.dn.state == ::phy_engine::model::digital_node_statement_t::true_state) { return 1; }

    // tick +2
    c.digital_clk();
    if(nq.node_information.dn.state != ::phy_engine::model::digital_node_statement_t::true_state) { return 1; }

    return 0;
}

