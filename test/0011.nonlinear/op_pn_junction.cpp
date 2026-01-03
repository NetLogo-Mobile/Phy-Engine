#include <phy_engine/phy_engine.h>

int main()
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::OP);

    auto& nl{c.get_netlist()};

    auto [v1, v1_pos]{add_model(nl, ::phy_engine::model::VDC{.V = 1.0})};
    auto [r1, r1_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 1000.0})};
    auto [d1, d1_pos]{add_model(nl, ::phy_engine::model::PN_junction{})};

    auto& n_vdd{create_node(nl)};
    auto& n_d{create_node(nl)};
    auto& gnd{get_ground_node(nl)};

    add_to_node(nl, *v1, 0, n_vdd);
    add_to_node(nl, *v1, 1, gnd);

    add_to_node(nl, *r1, 0, n_vdd);
    add_to_node(nl, *r1, 1, n_d);

    add_to_node(nl, *d1, 0, n_d);
    add_to_node(nl, *d1, 1, gnd);

    if(!c.analyze()) { return 1; }

    double const v_d{n_d.node_information.an.voltage.real()};
    if(!(v_d > 0.5 && v_d < 0.9)) { return 2; }

    return 0;
}

