#include <cmath>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/linear/resistance.h>
#include <phy_engine/model/models/non-linear/bsim3v32.h>
#include <phy_engine/netlist/impl.h>

int main()
{
    // Smoke: transient solve with BSIM3v3.2 intrinsic C-matrix companion enabled (capMod!=0 by default).
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    auto& setting{c.get_analyze_setting()};
    setting.tr.t_step = 1e-9;
    setting.tr.t_stop = 1e-7;

    auto& nl = c.get_netlist();

    // VDD = 3V, VG = 2V.
    auto [vdd_src, vdd_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 3.0});
    auto& n_vdd = create_node(nl);
    add_to_node(nl, *vdd_src, 0, n_vdd);
    add_to_node(nl, *vdd_src, 1, nl.ground_node);

    auto [vg_src, vg_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 2.0});
    auto& n_gate = create_node(nl);
    add_to_node(nl, *vg_src, 0, n_gate);
    add_to_node(nl, *vg_src, 1, nl.ground_node);

    // Drain load resistor from VDD to drain.
    auto [rd, rd_pos] = add_model(nl, ::phy_engine::model::resistance{.r = 10'000.0});
    auto& n_drain = create_node(nl);
    add_to_node(nl, *rd, 0, n_vdd);
    add_to_node(nl, *rd, 1, n_drain);

    // NMOS: D=drain, G=2V, S=GND, B=GND.
    auto [m1, m1_pos] = add_model(nl, ::phy_engine::model::bsim3v32_nmos{});
    add_to_node(nl, *m1, 0, n_drain);
    add_to_node(nl, *m1, 1, n_gate);
    add_to_node(nl, *m1, 2, nl.ground_node);
    add_to_node(nl, *m1, 3, nl.ground_node);

    if(!c.analyze()) { return 1; }

    double const vd = n_drain.node_information.an.voltage.real();
    double const vg = n_gate.node_information.an.voltage.real();

    if(!std::isfinite(vd) || !std::isfinite(vg)) { return 2; }
    if(!(vg > 1.9 && vg < 2.1)) { return 3; }
    if(!(vd > -0.5 && vd < 3.5)) { return 4; }

    return 0;
}

