#include <cmath>
#include <limits>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/linear/resistance.h>
#include <phy_engine/model/models/non-linear/bsim3v32.h>
#include <phy_engine/netlist/impl.h>

static double run_case(double tnom_c) noexcept
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);
    c.get_environment().temperature = 27.0;
    c.get_environment().norm_temperature = tnom_c;

    auto& nl = c.get_netlist();

    // VDD = 3V
    auto [vdd_src, vdd_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 3.0});
    auto& n_vdd = create_node(nl);
    add_to_node(nl, *vdd_src, 0, n_vdd);
    add_to_node(nl, *vdd_src, 1, nl.ground_node);

    // VG = 3V
    auto [vg_src, vg_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 3.0});
    auto& n_gate = create_node(nl);
    add_to_node(nl, *vg_src, 0, n_gate);
    add_to_node(nl, *vg_src, 1, nl.ground_node);

    // Load resistor from VDD to drain
    auto [rload, rload_pos] = add_model(nl, ::phy_engine::model::resistance{.r = 1'000.0});
    auto& n_drain = create_node(nl);
    add_to_node(nl, *rload, 0, n_vdd);
    add_to_node(nl, *rload, 1, n_drain);

    // NMOS: keep device Temp fixed, but make Vth temperature dependence visible via kt1.
    // The point of this regression is to ensure env.norm_temperature (TNOM) is propagated
    // into models that expose a "tnom" attribute.
    ::phy_engine::model::bsim3v32_nmos mos{};
    mos.Kp = 1e-3;
    mos.kt1 = 1e-3;

    auto [m1, m1_pos] = add_model(nl, mos);
    add_to_node(nl, *m1, 0, n_drain);
    add_to_node(nl, *m1, 1, n_gate);
    add_to_node(nl, *m1, 2, nl.ground_node);
    add_to_node(nl, *m1, 3, nl.ground_node);

    if(!c.analyze()) { return std::numeric_limits<double>::quiet_NaN(); }
    return n_drain.node_information.an.voltage.real();
}

int main()
{
    double const vd_tnom27 = run_case(27.0);
    double const vd_tnom127 = run_case(127.0);

    if(!std::isfinite(vd_tnom27) || !std::isfinite(vd_tnom127)) { return 1; }

    // With Temp fixed at 27C and positive kt1, increasing TNOM reduces (Temp-Tnom),
    // lowering |Vth0(T)| and increasing conduction => drain voltage should drop.
    if(!(vd_tnom127 < vd_tnom27 - 1e-6)) { return 2; }

    return 0;
}

