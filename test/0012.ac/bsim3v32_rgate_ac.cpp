#include <cmath>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/linear/VAC.h>
#include <phy_engine/model/models/non-linear/bsim3v32.h>
#include <phy_engine/netlist/impl.h>

static double run_case(double rg) noexcept
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::AC);
    double const omega{1e9};
    c.get_analyze_setting().ac.sweep = ::phy_engine::analyzer::AC::sweep_type::single;
    c.get_analyze_setting().ac.omega = omega;

    auto& nl = c.get_netlist();

    auto [vac_src, vac_pos] = add_model(nl, ::phy_engine::model::VAC{.m_Vp{1.0}, .m_omega{omega}});
    auto& n_gate = create_node(nl);
    add_to_node(nl, *vac_src, 0, n_gate);
    add_to_node(nl, *vac_src, 1, nl.ground_node);

    ::phy_engine::model::bsim3v32_nmos mos{};
    mos.capMod = 0.0;
    mos.rg = rg;
    // Make the external Cgs dominate so the test is robust.
    mos.Cgs = 1e-9;
    mos.Cgd = 0.0;
    mos.Cgb = 0.0;

    auto [m1, m1_pos] = add_model(nl, mos);
    add_to_node(nl, *m1, 0, nl.ground_node); // D
    add_to_node(nl, *m1, 1, n_gate);         // G (external)
    add_to_node(nl, *m1, 2, nl.ground_node); // S
    add_to_node(nl, *m1, 3, nl.ground_node); // B

    if(!c.analyze()) { return std::numeric_limits<double>::quiet_NaN(); }

    auto const bv = vac_src->ptr->generate_branch_view();
    if(bv.size == 0) { return std::numeric_limits<double>::quiet_NaN(); }
    return std::abs(bv.branches[0].current);
}

int main()
{
    double const i0 = run_case(0.0);
    double const i1 = run_case(1e6);

    if(!std::isfinite(i0) || !std::isfinite(i1)) { return 1; }
    if(!(i0 > 1e-2)) { return 2; }
    if(!(i1 < i0 * 1e-3)) { return 3; }
    return 0;
}

