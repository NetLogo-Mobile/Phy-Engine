#include <cmath>
#include <limits>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/linear/VAC.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/non-linear/bsim3v32.h>
#include <phy_engine/netlist/impl.h>

int main()
{
    // Regression: capMod!=0 should produce a non-zero gate current in accumulation (negative Vgb for NMOS)
    // due to gate-bulk capacitance. This relies on charge-based C-matrix stamping.
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::AC);
    double const omega{1e9};
    c.get_analyze_setting().ac.sweep = ::phy_engine::analyzer::AC::sweep_type::single;
    c.get_analyze_setting().ac.omega = omega;

    auto& nl = c.get_netlist();

    // Bias node at -2V in OP (VDC becomes AC ground in AC).
    auto [vbias_src, vbias_pos] = add_model(nl, ::phy_engine::model::VDC{.V = -2.0});
    auto& n_bias = create_node(nl);
    add_to_node(nl, *vbias_src, 0, n_bias);
    add_to_node(nl, *vbias_src, 1, nl.ground_node);

    // Small-signal gate drive (VAC becomes 0V in OP).
    auto [vac_src, vac_pos] = add_model(nl, ::phy_engine::model::VAC{.m_Vp{1.0}, .m_omega{omega}});
    auto& n_gate = create_node(nl);
    add_to_node(nl, *vac_src, 0, n_gate);
    add_to_node(nl, *vac_src, 1, n_bias);

    ::phy_engine::model::bsim3v32_nmos mos{};
    mos.capMod = 3.0;
    mos.W = 10e-6;
    mos.L = 10e-6;
    mos.tox = 1e-8;
    mos.toxm = 1e-8;
    mos.Cgs = mos.Cgd = mos.Cgb = 0.0;
    mos.cgso = mos.cgdo = mos.cgbo = 0.0;

    auto [m1, m1_pos] = add_model(nl, mos);
    add_to_node(nl, *m1, 0, nl.ground_node); // D
    add_to_node(nl, *m1, 1, n_gate);         // G
    add_to_node(nl, *m1, 2, nl.ground_node); // S
    add_to_node(nl, *m1, 3, nl.ground_node); // B

    if(!c.analyze()) { return 1; }

    auto const bv = vac_src->ptr->generate_branch_view();
    if(bv.size == 0) { return 2; }
    double const i_mag = std::abs(bv.branches[0].current);
    if(!std::isfinite(i_mag)) { return 3; }

    // Expect a noticeable capacitive current in accumulation. With the chosen geometry/tox and omega,
    // this should be far above numerical noise.
    if(!(i_mag > 1e-8)) { return 4; }

    return 0;
}

