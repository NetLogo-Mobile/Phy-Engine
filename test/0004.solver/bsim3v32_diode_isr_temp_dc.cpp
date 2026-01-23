#include <cmath>
#include <limits>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/linear/resistance.h>
#include <phy_engine/model/models/non-linear/bsim3v32.h>
#include <phy_engine/netlist/impl.h>

static double run_case(double temp_c) noexcept
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);
    c.get_environment().temperature = temp_c;

    auto& nl = c.get_netlist();

    auto [vdd_src, vdd_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 3.0});
    auto& n_vdd = create_node(nl);
    add_to_node(nl, *vdd_src, 0, n_vdd);
    add_to_node(nl, *vdd_src, 1, nl.ground_node);

    auto [rload, rload_pos] = add_model(nl, ::phy_engine::model::resistance{.r = 1'000'000.0});
    auto& n_drain = create_node(nl);
    add_to_node(nl, *rload, 0, n_vdd);
    add_to_node(nl, *rload, 1, n_drain);

    auto [vg_src, vg_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 0.0});
    auto& n_gate = create_node(nl);
    add_to_node(nl, *vg_src, 0, n_gate);
    add_to_node(nl, *vg_src, 1, nl.ground_node);

    ::phy_engine::model::bsim3v32_nmos mos{};
    // Disable channel; isolate diode leakage path.
    mos.Kp = 0.0;

    // Use recombination current as the dominant reverse leakage term.
    mos.diode_Is = 1e-30;
    mos.diode_N = 1.0;
    mos.diode_Isr = 1e-12;
    mos.diode_Nr = 2.0;

    mos.tnom = 27.0;
    mos.xti = 3.0;
    mos.eg = 1.11;

    auto [m1, m1_pos] = add_model(nl, mos);
    add_to_node(nl, *m1, 0, n_drain);
    add_to_node(nl, *m1, 1, n_gate);
    add_to_node(nl, *m1, 2, nl.ground_node); // source
    add_to_node(nl, *m1, 3, nl.ground_node); // bulk

    if(!c.analyze()) { return std::numeric_limits<double>::quiet_NaN(); }
    return n_drain.node_information.an.voltage.real();
}

int main()
{
    double const vd_27 = run_case(27.0);
    double const vd_127 = run_case(127.0);

    if(!std::isfinite(vd_27) || !std::isfinite(vd_127)) { return 1; }

    // With higher temperature, Isr should scale up (like Is) -> larger reverse leakage -> drain node droops more.
    if(!(vd_127 < vd_27 - 1e-4)) { return 2; }

    return 0;
}

