#include <cmath>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/linear/resistance.h>
#include <phy_engine/model/models/non-linear/PN_junction.h>
#include <phy_engine/netlist/impl.h>

static double run_case(double temp_c) noexcept
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);
    c.get_environment().temperature = temp_c;

    auto& nl = c.get_netlist();

    // 1V supply.
    auto [v1, v1_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 1.0});
    auto& n_v1 = create_node(nl);
    add_to_node(nl, *v1, 0, n_v1);
    add_to_node(nl, *v1, 1, nl.ground_node);

    // Series resistor to a diode to ground.
    auto [r, r_pos] = add_model(nl, ::phy_engine::model::resistance{.r = 1'000.0});
    auto& n_x = create_node(nl);
    add_to_node(nl, *r, 0, n_v1);
    add_to_node(nl, *r, 1, n_x);

    ::phy_engine::model::PN_junction d{};
    d.Is = 1e-14;
    d.N = 1.0;
    auto [dj, dj_pos] = add_model(nl, d);
    add_to_node(nl, *dj, 0, n_x);
    add_to_node(nl, *dj, 1, nl.ground_node);

    if(!c.analyze()) { return std::numeric_limits<double>::quiet_NaN(); }
    return n_x.node_information.an.voltage.real();
}

int main()
{
    double const v27 = run_case(27.0);
    double const v127 = run_case(127.0);

    if(!std::isfinite(v27) || !std::isfinite(v127)) { return 1; }

    // With Is fixed, higher temperature increases Ut and reduces diode current at a given V,
    // so the node voltage should rise (less drop across the resistor).
    if(!(v127 > v27 + 1e-6)) { return 2; }

    return 0;
}

