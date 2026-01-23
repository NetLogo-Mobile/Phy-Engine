#include <cmath>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/linear/VAC.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/linear/resistance.h>
#include <phy_engine/model/models/non-linear/bsim3v32.h>
#include <phy_engine/netlist/impl.h>

int main()
{
    // Smoke: analyze_type::AC should auto-run OP first when non-linear devices exist.
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::AC);
    double const omega{1e6};
    c.get_analyze_setting().ac.omega = omega;

    auto& nl = c.get_netlist();

    // VDD = 3V (DC). In AC, VDC is a 0-V source (AC ground).
    auto [vdd_src, vdd_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 3.0});
    auto& n_vdd = create_node(nl);
    add_to_node(nl, *vdd_src, 0, n_vdd);
    add_to_node(nl, *vdd_src, 1, nl.ground_node);

    // Gate drive is a DC + AC series source:
    // - In OP, VAC is a 0-V source => V(G)=Vbias.
    // - In AC, VDC is a 0-V source => the bias node is AC ground and VAC drives the gate with 1V.
    auto [vbias_src, vbias_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 2.0});
    auto& n_bias = create_node(nl);
    add_to_node(nl, *vbias_src, 0, n_bias);
    add_to_node(nl, *vbias_src, 1, nl.ground_node);

    auto [vac_src, vac_pos] = add_model(nl, ::phy_engine::model::VAC{.m_Vp{1.0}, .m_omega{omega}});
    auto& n_gate = create_node(nl);
    add_to_node(nl, *vac_src, 0, n_gate);
    add_to_node(nl, *vac_src, 1, n_bias);

    // Drain load resistor from VDD to drain.
    auto [rd, rd_pos] = add_model(nl, ::phy_engine::model::resistance{.r = 10'000.0});
    auto& n_drain = create_node(nl);
    add_to_node(nl, *rd, 0, n_vdd);
    add_to_node(nl, *rd, 1, n_drain);

    // NMOS: D=drain, G=gate, S=GND, B=GND.
    auto [m1, m1_pos] = add_model(nl, ::phy_engine::model::bsim3v32_nmos{});
    add_to_node(nl, *m1, 0, n_drain);
    add_to_node(nl, *m1, 1, n_gate);
    add_to_node(nl, *m1, 2, nl.ground_node);
    add_to_node(nl, *m1, 3, nl.ground_node);

    if(!c.analyze()) { return 1; }

    auto const vg = n_gate.node_information.an.voltage;
    if(!std::isfinite(vg.real()) || !std::isfinite(vg.imag())) { return 2; }

    double const mag_g = std::abs(vg);
    if(!(mag_g > 0.9 && mag_g < 1.1)) { return 3; }

    return 0;
}

