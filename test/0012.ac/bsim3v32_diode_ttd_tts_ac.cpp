#include <cmath>
#include <complex>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/linear/VAC.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/non-linear/bsim3v32.h>
#include <phy_engine/netlist/impl.h>

static std::complex<double> run_case(bool drive_drain, double ttd_s, double tts_s) noexcept
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

    // Drive the junction-under-test with 1V AC around the DC bias.
    auto [vac_src, vac_pos] = add_model(nl, ::phy_engine::model::VAC{.m_Vp{1.0}, .m_omega{omega}});
    auto& n_x = create_node(nl);
    add_to_node(nl, *vac_src, 0, n_x);
    add_to_node(nl, *vac_src, 1, n_bias);

    // Gate held at 0V, so channel is off; only the selected body diode conducts (forward-biased by -0.7V).
    auto [vg_src, vg_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 0.0});
    auto& n_gate = create_node(nl);
    add_to_node(nl, *vg_src, 0, n_gate);
    add_to_node(nl, *vg_src, 1, nl.ground_node);

    ::phy_engine::model::bsim3v32_nmos mos{};
    mos.tt = 0.0;  // ensure only per-junction overrides contribute
    mos.ttd = ttd_s;
    mos.tts = tts_s;
    mos.diode_Is = 1e-14;
    mos.diode_N = 1.0;

    auto [m1, m1_pos] = add_model(nl, mos);
    if(drive_drain)
    {
        // Drain diode under test.
        add_to_node(nl, *m1, 0, n_x);
        add_to_node(nl, *m1, 1, n_gate);
        add_to_node(nl, *m1, 2, nl.ground_node);
        add_to_node(nl, *m1, 3, nl.ground_node);
    }
    else
    {
        // Source diode under test.
        add_to_node(nl, *m1, 0, nl.ground_node);
        add_to_node(nl, *m1, 1, n_gate);
        add_to_node(nl, *m1, 2, n_x);
        add_to_node(nl, *m1, 3, nl.ground_node);
    }

    if(!c.analyze()) { return {NAN, NAN}; }

    auto const bv = vac_src->ptr->generate_branch_view();
    if(bv.size != 1 || bv.branches == nullptr) { return {NAN, NAN}; }
    return bv.branches[0].current;
}

int main()
{
    // Drain: only ttd should increase imag current.
    auto const d0 = run_case(true, 0.0, 0.0);
    auto const d_ttd = run_case(true, 1e-6, 0.0);
    auto const d_tts = run_case(true, 0.0, 1e-6);

    if(!std::isfinite(d0.real()) || !std::isfinite(d0.imag())) { return 1; }
    if(!std::isfinite(d_ttd.real()) || !std::isfinite(d_ttd.imag())) { return 2; }
    if(!std::isfinite(d_tts.real()) || !std::isfinite(d_tts.imag())) { return 3; }

    if(!(std::abs(d_ttd.imag()) > std::abs(d0.imag()) + 1e-9)) { return 4; }
    if(!(std::abs(d_tts.imag()) <= std::abs(d0.imag()) + 1e-9)) { return 5; }

    // Source: only tts should increase imag current.
    auto const s0 = run_case(false, 0.0, 0.0);
    auto const s_ttd = run_case(false, 1e-6, 0.0);
    auto const s_tts = run_case(false, 0.0, 1e-6);

    if(!std::isfinite(s0.real()) || !std::isfinite(s0.imag())) { return 6; }
    if(!std::isfinite(s_ttd.real()) || !std::isfinite(s_ttd.imag())) { return 7; }
    if(!std::isfinite(s_tts.real()) || !std::isfinite(s_tts.imag())) { return 8; }

    if(!(std::abs(s_tts.imag()) > std::abs(s0.imag()) + 1e-9)) { return 9; }
    if(!(std::abs(s_ttd.imag()) <= std::abs(s0.imag()) + 1e-9)) { return 10; }

    return 0;
}

