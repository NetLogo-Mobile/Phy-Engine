#include <cmath>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/linear/resistance.h>
#include <phy_engine/model/models/non-linear/bsim3v32.h>
#include <phy_engine/netlist/impl.h>

static double run_case(double vg, double vth0) noexcept
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);

    auto& nl = c.get_netlist();

    // VDD = 3V
    auto [vdd_src, vdd_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 3.0});
    auto& n_vdd = create_node(nl);
    add_to_node(nl, *vdd_src, 0, n_vdd);
    add_to_node(nl, *vdd_src, 1, nl.ground_node);

    // Gate source: VG referenced to ground
    auto [vg_src, vg_pos] = add_model(nl, ::phy_engine::model::VDC{.V = vg});
    auto& n_gate = create_node(nl);
    add_to_node(nl, *vg_src, 0, n_gate);
    add_to_node(nl, *vg_src, 1, nl.ground_node);

    // Load resistor from drain to ground
    auto [rload, rload_pos] = add_model(nl, ::phy_engine::model::resistance{.r = 10'000.0});
    auto& n_drain = create_node(nl);
    add_to_node(nl, *rload, 0, n_drain);
    add_to_node(nl, *rload, 1, nl.ground_node);

    // PMOS: S=VDD, D=drain, G=gate, B=VDD.
    ::phy_engine::model::bsim3v32_pmos mp{};
    mp.Vth0 = vth0;
    mp.Kp = 5e-3;
    auto [m1, m1_pos] = add_model(nl, mp);
    add_to_node(nl, *m1, 0, n_drain); // D
    add_to_node(nl, *m1, 1, n_gate);  // G
    add_to_node(nl, *m1, 2, n_vdd);   // S
    add_to_node(nl, *m1, 3, n_vdd);   // B

    (void)c.analyze();
    return n_drain.node_information.an.voltage.real();
}

int main()
{
    // Gate low: PMOS on -> drain pulled up.
    double const v_on_pos = run_case(0.0, 0.7);
    double const v_on_neg = run_case(0.0, -0.7);
    if(!(v_on_pos > 1.0)) { return 1; }
    if(!(v_on_neg > 1.0)) { return 2; }

    // Gate high: PMOS off -> drain pulled down.
    double const v_off_pos = run_case(3.0, 0.7);
    double const v_off_neg = run_case(3.0, -0.7);
    if(!(v_off_pos < 1.0)) { return 3; }
    if(!(v_off_neg < 1.0)) { return 4; }

    return 0;
}
