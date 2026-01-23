#include <cmath>

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

    // NMOS with explicit temp-scaling knobs enabled.
    ::phy_engine::model::bsim3v32_nmos mos{};
    mos.Kp = 1e-3;
    mos.ute = 1.0;
    mos.tnom = 27.0;

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
    double const vd_27 = run_case(27.0);
    double const vd_127 = run_case(127.0);

    if(!std::isfinite(vd_27) || !std::isfinite(vd_127)) { return 1; }

    // With positive UTE (mobility decreases with temperature), the device should conduct less at higher temperature,
    // resulting in a higher drain voltage through the load resistor.
    if(!(vd_127 > vd_27 + 1e-6)) { return 2; }

    return 0;
}

