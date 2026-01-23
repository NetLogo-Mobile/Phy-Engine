#include <cmath>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/linear/resistance.h>
#include <phy_engine/model/models/non-linear/bsim3v32.h>
#include <phy_engine/netlist/impl.h>

int main()
{
    // Regression: ensure the MOSFET conducts with "reverse" Vds (drain below source) by swapping the channel mode.
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);

    auto& nl = c.get_netlist();

    // Supplies: 5V rail and a 10V gate drive.
    auto [v5_src, v5_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 5.0});
    auto& n_v5 = create_node(nl);
    add_to_node(nl, *v5_src, 0, n_v5);
    add_to_node(nl, *v5_src, 1, nl.ground_node);

    auto [v10_src, v10_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 10.0});
    auto& n_v10 = create_node(nl);
    add_to_node(nl, *v10_src, 0, n_v10);
    add_to_node(nl, *v10_src, 1, nl.ground_node);

    // Bias: source pulled up to 5V via 1k, drain pulled down to GND via 1k.
    auto [rs, rs_pos] = add_model(nl, ::phy_engine::model::resistance{.r = 1'000.0});
    auto& n_source = create_node(nl);
    add_to_node(nl, *rs, 0, n_v5);
    add_to_node(nl, *rs, 1, n_source);

    auto [rd, rd_pos] = add_model(nl, ::phy_engine::model::resistance{.r = 1'000.0});
    auto& n_drain = create_node(nl);
    add_to_node(nl, *rd, 0, n_drain);
    add_to_node(nl, *rd, 1, nl.ground_node);

    // NMOS: D=drain, G=10V, S=source (higher than drain), B=GND.
    auto [m1, m1_pos] = add_model(nl, ::phy_engine::model::bsim3v32_nmos{});
    add_to_node(nl, *m1, 0, n_drain);
    add_to_node(nl, *m1, 1, n_v10);
    add_to_node(nl, *m1, 2, n_source);
    add_to_node(nl, *m1, 3, nl.ground_node);

    if(!c.analyze()) { return 10; }

    double const vd = n_drain.node_information.an.voltage.real();
    double const vs = n_source.node_information.an.voltage.real();

    if(!std::isfinite(vd) || !std::isfinite(vs)) { return 20; }
    // If the reverse-mode channel handling is missing, vd~0 and vs~5.
    if(!(vd > 0.1)) { return 1; }
    if(!(vs < 4.9)) { return 2; }

    return 0;
}

