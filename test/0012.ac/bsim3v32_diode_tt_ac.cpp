#include <cmath>
#include <complex>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/linear/VAC.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/non-linear/bsim3v32.h>
#include <phy_engine/netlist/impl.h>

static std::complex<double> run_case(double tt_s) noexcept
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::AC);
    double const omega{1e6};
    c.get_analyze_setting().ac.omega = omega;

    auto& nl = c.get_netlist();

    // Bias node at -0.7V DC (AC ground).
    auto [vbias_src, vbias_pos] = add_model(nl, ::phy_engine::model::VDC{.V = -0.7});
    auto& n_bias = create_node(nl);
    add_to_node(nl, *vbias_src, 0, n_bias);
    add_to_node(nl, *vbias_src, 1, nl.ground_node);

    // Drive drain with 1V AC around the DC bias.
    auto [vac_src, vac_pos] = add_model(nl, ::phy_engine::model::VAC{.m_Vp{1.0}, .m_omega{omega}});
    auto& n_drain = create_node(nl);
    add_to_node(nl, *vac_src, 0, n_drain);
    add_to_node(nl, *vac_src, 1, n_bias);

    // Gate held at 0V, so channel is off; only the drain-body diode conducts (forward-biased by -0.7V).
    auto [vg_src, vg_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 0.0});
    auto& n_gate = create_node(nl);
    add_to_node(nl, *vg_src, 0, n_gate);
    add_to_node(nl, *vg_src, 1, nl.ground_node);

    ::phy_engine::model::bsim3v32_nmos mos{};
    mos.tt = tt_s;
    mos.diode_Is = 1e-14;
    mos.diode_N = 1.0;

    auto [m1, m1_pos] = add_model(nl, mos);
    add_to_node(nl, *m1, 0, n_drain);
    add_to_node(nl, *m1, 1, n_gate);
    add_to_node(nl, *m1, 2, nl.ground_node);
    add_to_node(nl, *m1, 3, nl.ground_node);

    if(!c.analyze()) { return {NAN, NAN}; }

    auto const bv = vac_src->ptr->generate_branch_view();
    if(bv.size != 1 || bv.branches == nullptr) { return {NAN, NAN}; }
    return bv.branches[0].current;
}

int main()
{
    // With tt=0 the diode stamps only conductance in AC => imag current should be near zero.
    auto const i0 = run_case(0.0);
    auto const i1 = run_case(1e-6);

    if(!std::isfinite(i0.real()) || !std::isfinite(i0.imag())) { return 1; }
    if(!std::isfinite(i1.real()) || !std::isfinite(i1.imag())) { return 2; }

    if(!(std::abs(i1.imag()) > std::abs(i0.imag()) + 1e-9)) { return 3; }
    return 0;
}

