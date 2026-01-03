#include <phy_engine/phy_engine.h>

int main()
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::AC);
    c.get_analyze_setting().ac.omega = 1000.0;  // rad/s (1 / (R*C))

    auto& nl{c.get_netlist()};

    auto [vac, vac_pos]{add_model(nl, ::phy_engine::model::VAC{.m_Vp{1.0}, .m_omega{1000.0}})};
    auto [r1, r1_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 1000.0})};
    auto [c1, c1_pos]{add_model(nl, ::phy_engine::model::capacitor{.m_kZimag{1e-6}})};

    auto& n_in{create_node(nl)};
    auto& n_out{create_node(nl)};
    auto& gnd{get_ground_node(nl)};

    add_to_node(nl, *vac, 0, n_in);
    add_to_node(nl, *vac, 1, gnd);

    add_to_node(nl, *r1, 0, n_in);
    add_to_node(nl, *r1, 1, n_out);

    add_to_node(nl, *c1, 0, n_out);
    add_to_node(nl, *c1, 1, gnd);

    if(!c.analyze()) { return 1; }

    auto const v_out{n_out.node_information.an.voltage};
    double const mag{::std::abs(v_out)};
    if(!(mag > 0.6 && mag < 0.8)) { return 2; }

    return 0;
}

