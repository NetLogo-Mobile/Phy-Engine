#include <cmath>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/linear/VAC.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/non-linear/bsim3v32.h>
#include <phy_engine/netlist/impl.h>

int main()
{
    // Regression: at DC Vds=0 (drain tied to source), the small-signal channel conductance (gds) must still be finite.
    // We validate by driving the drain with a 1V AC source and checking the VAC branch current magnitude at a low omega,
    // where capacitive current is negligible.
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::AC);
    c.get_analyze_setting().ac.omega = 1.0; // rad/s (keep C current tiny)

    auto& nl = c.get_netlist();

    // Gate bias: 2V DC (AC-grounded in AC analysis).
    auto [vg_src, vg_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 2.0});
    auto& n_gate = create_node(nl);
    add_to_node(nl, *vg_src, 0, n_gate);
    add_to_node(nl, *vg_src, 1, nl.ground_node);

    // Drain drive: VAC in series with a 0V VDC to force drain DC=0 and AC=1V.
    auto [v0_src, v0_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 0.0});
    auto& n_bias = create_node(nl);
    add_to_node(nl, *v0_src, 0, n_bias);
    add_to_node(nl, *v0_src, 1, nl.ground_node);

    auto [vac_src, vac_pos] = add_model(nl, ::phy_engine::model::VAC{.m_Vp{1.0}, .m_omega{1.0}});
    auto& n_drain = create_node(nl);
    add_to_node(nl, *vac_src, 0, n_drain);
    add_to_node(nl, *vac_src, 1, n_bias);

    // NMOS: D=drain, G=gate, S=GND, B=GND => DC Vds=0.
    auto [m1, m1_pos] = add_model(nl, ::phy_engine::model::bsim3v32_nmos{});
    add_to_node(nl, *m1, 0, n_drain);
    add_to_node(nl, *m1, 1, n_gate);
    add_to_node(nl, *m1, 2, nl.ground_node);
    add_to_node(nl, *m1, 3, nl.ground_node);

    if(!c.analyze()) { return 1; }

    auto const bv = vac_src->ptr->generate_branch_view();
    if(bv.size != 1 || bv.branches == nullptr) { return 2; }

    auto const iac = bv.branches[0].current;
    if(!std::isfinite(iac.real()) || !std::isfinite(iac.imag())) { return 3; }

    // Expect a noticeable resistive current draw (gds*Vac) at Vgs=2V.
    if(!(std::abs(iac) > 1e-9)) { return 4; }

    return 0;
}

