#include <phy_engine/phy_engine.h>

int main()
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);

    auto& nl{c.get_netlist()};

    auto [i1, i1_pos]{add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::H})};

    auto& ground_node{get_ground_node(nl)};
}
