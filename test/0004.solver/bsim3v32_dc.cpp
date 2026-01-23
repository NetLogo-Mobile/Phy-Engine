#include <cmath>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/linear/resistance.h>
#include <phy_engine/model/models/non-linear/bsim3v32.h>
#include <phy_engine/netlist/impl.h>

static double run_case(double vg) noexcept
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);

    auto& nl = c.get_netlist();

    // Supply: VDD = 3V
    auto [vdd_src, vdd_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 3.0});
    auto& n_vdd = create_node(nl);
    add_to_node(nl, *vdd_src, 0, n_vdd);
    add_to_node(nl, *vdd_src, 1, nl.ground_node);

    // Gate source: VG
    auto [vg_src, vg_pos] = add_model(nl, ::phy_engine::model::VDC{.V = vg});
    auto& n_gate = create_node(nl);
    add_to_node(nl, *vg_src, 0, n_gate);
    add_to_node(nl, *vg_src, 1, nl.ground_node);

    // Load resistor from VDD to drain
    auto [rload, rload_pos] = add_model(nl, ::phy_engine::model::resistance{.r = 10'000.0});
    auto& n_drain = create_node(nl);
    add_to_node(nl, *rload, 0, n_vdd);
    add_to_node(nl, *rload, 1, n_drain);

    // NMOS: D=drain, G=gate, S=gnd, B=gnd
    auto [m1, m1_pos] = add_model(nl, ::phy_engine::model::bsim3v32_nmos{});
    add_to_node(nl, *m1, 0, n_drain);
    add_to_node(nl, *m1, 1, n_gate);
    add_to_node(nl, *m1, 2, nl.ground_node);
    add_to_node(nl, *m1, 3, nl.ground_node);

    (void)c.analyze();

    auto const pv = m1->ptr->generate_pin_view();
    return pv.pins[0].nodes->node_information.an.voltage.real();
}

int main()
{
    double const v_off = run_case(0.0);
    double const v_on = run_case(3.0);

    if(!(v_off > 2.99 && v_off < 3.01)) { return 1; }
    if(!(v_on < v_off - 1e-3)) { return 2; }
    if(!std::isfinite(v_on) || !std::isfinite(v_off)) { return 3; }

    return 0;
}

