#include <phy_engine/phy_engine.h>

namespace
{
    using u8sv = ::fast_io::u8string_view;

    bool include_resolve(void*, u8sv path, ::fast_io::u8string& out_text) noexcept
    {
        if(path == u8sv{u8"defs.vh"})
        {
            out_text.assign(u8R"(
`define ONE 1'b1
`include "and2.vh"
)");
            return true;
        }

        if(path == u8sv{u8"and2.vh"})
        {
            out_text.assign(u8R"(
`define AND2(a,b) ((a) & (b))
)");
            return true;
        }

        return false;
    }
}  // namespace

int main()
{
    decltype(auto) src = u8R"(
`ifdef SKIP_MISSING
  `include "missing.vh"
`endif

`include "defs.vh"

module top(input a, input b, output y);
  assign y = `AND2(a,b) & `ONE;
endmodule
)";
    constexpr char8_t top_name[] = u8"top";

    ::phy_engine::verilog::digital::compile_options copts{};
    copts.preprocess.include_resolver = &include_resolve;

    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    c.get_analyze_setting().tr.t_step = 1e-9;
    c.get_analyze_setting().tr.t_stop = 1e-9;

    auto& nl{c.get_netlist()};

    auto [ina, ina_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [inb, inb_pos]{add_model(nl, ::phy_engine::model::INPUT{})};
    auto [vmod, vmod_pos]{add_model(nl, ::phy_engine::model::make_verilog_module(
                                      u8sv{src, sizeof(src) - 1},
                                      u8sv{top_name, sizeof(top_name) - 1},
                                      copts))};

    auto& na{create_node(nl)};
    auto& nb{create_node(nl)};
    auto& ny{create_node(nl)};

    add_to_node(nl, *ina, 0, na);
    add_to_node(nl, *inb, 0, nb);
    add_to_node(nl, *vmod, 0, na);
    add_to_node(nl, *vmod, 1, nb);
    add_to_node(nl, *vmod, 2, ny);

    if(!c.analyze()) { return 1; }

    auto set_in = [&](::phy_engine::model::model_base* in, ::phy_engine::model::digital_node_statement_t v) {
        in->ptr->set_attribute(0, {.digital{v}, .type{::phy_engine::model::variant_type::digital}});
    };

    set_in(ina, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(inb, ::phy_engine::model::digital_node_statement_t::false_state);
    c.digital_clk();
    if(ny.node_information.dn.state != ::phy_engine::model::digital_node_statement_t::false_state) { return 1; }

    set_in(inb, ::phy_engine::model::digital_node_statement_t::true_state);
    c.digital_clk();
    if(ny.node_information.dn.state != ::phy_engine::model::digital_node_statement_t::true_state) { return 1; }

    return 0;
}

