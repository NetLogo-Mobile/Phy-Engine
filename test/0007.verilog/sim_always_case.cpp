#include <phy_engine/phy_engine.h>

int main()
{
    decltype(auto) src = u8R"(
module mux2_if(input sel, input a, input b, output reg y);
  always @* begin
    if(sel) y = b;
    else y = a;
  end
endmodule

module mux2_case(input sel, input a, input b, output reg y);
  always @* begin
    case(sel)
      1'b0: y = a;
      1'b1: y = b;
      default: y = 1'bx;
    endcase
  end
endmodule
)";

    constexpr char8_t top_name_if[] = u8"mux2_if";
    constexpr char8_t top_name_case[] = u8"mux2_case";

    auto run_one = [&](::fast_io::u8string_view top_name,
                       ::phy_engine::model::digital_node_statement_t sel,
                       ::phy_engine::model::digital_node_statement_t a,
                       ::phy_engine::model::digital_node_statement_t b,
                       ::phy_engine::model::digital_node_statement_t expected) -> bool {
        ::phy_engine::circult c{};
        c.set_analyze_type(::phy_engine::analyze_type::TR);
        c.get_analyze_setting().tr.t_step = 1e-9;
        c.get_analyze_setting().tr.t_stop = 1e-9;

        auto& nl{c.get_netlist()};

        auto [insel, insel_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
        auto [ina, ina_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
        auto [inb, inb_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
        auto [vmod, vmod_pos]{add_model(nl, ::phy_engine::model::make_verilog_module(
                                          ::fast_io::u8string_view{src, sizeof(src) - 1},
                                          top_name))};

        auto& nsel{create_node(nl)};
        auto& na{create_node(nl)};
        auto& nb{create_node(nl)};
        auto& ny{create_node(nl)};

        add_to_node(nl, *insel, 0, nsel);
        add_to_node(nl, *ina, 0, na);
        add_to_node(nl, *inb, 0, nb);

        add_to_node(nl, *vmod, 0, nsel);
        add_to_node(nl, *vmod, 1, na);
        add_to_node(nl, *vmod, 2, nb);
        add_to_node(nl, *vmod, 3, ny);

        if(!c.analyze()) { return false; }

        auto set_in = [&](::phy_engine::model::model_base* in, ::phy_engine::model::digital_node_statement_t v) {
            in->ptr->set_attribute(0, {.digital{v}, .type{::phy_engine::model::variant_type::digital}});
        };

        set_in(insel, sel);
        set_in(ina, a);
        set_in(inb, b);
        c.digital_clk();

        return ny.node_information.dn.state == expected;
    };

    if(!run_one(::fast_io::u8string_view{top_name_if, sizeof(top_name_if) - 1},
                ::phy_engine::model::digital_node_statement_t::false_state,
                ::phy_engine::model::digital_node_statement_t::false_state,
                ::phy_engine::model::digital_node_statement_t::true_state,
                ::phy_engine::model::digital_node_statement_t::false_state))
    {
        return 1;
    }
    if(!run_one(::fast_io::u8string_view{top_name_if, sizeof(top_name_if) - 1},
                ::phy_engine::model::digital_node_statement_t::true_state,
                ::phy_engine::model::digital_node_statement_t::false_state,
                ::phy_engine::model::digital_node_statement_t::true_state,
                ::phy_engine::model::digital_node_statement_t::true_state))
    {
        return 1;
    }

    if(!run_one(::fast_io::u8string_view{top_name_case, sizeof(top_name_case) - 1},
                ::phy_engine::model::digital_node_statement_t::false_state,
                ::phy_engine::model::digital_node_statement_t::true_state,
                ::phy_engine::model::digital_node_statement_t::false_state,
                ::phy_engine::model::digital_node_statement_t::true_state))
    {
        return 1;
    }
    if(!run_one(::fast_io::u8string_view{top_name_case, sizeof(top_name_case) - 1},
                ::phy_engine::model::digital_node_statement_t::true_state,
                ::phy_engine::model::digital_node_statement_t::true_state,
                ::phy_engine::model::digital_node_statement_t::false_state,
                ::phy_engine::model::digital_node_statement_t::false_state))
    {
        return 1;
    }

    return 0;
}

