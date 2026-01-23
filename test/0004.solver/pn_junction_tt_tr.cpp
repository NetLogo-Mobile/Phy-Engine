#include <cmath>
#include <limits>
#include <numbers>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/linear/VAC.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/non-linear/PN_junction.h>
#include <phy_engine/netlist/impl.h>

static double run_case(double tt_s) noexcept
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    auto& setting = c.get_analyze_setting();
    double const dt = 1e-8;
    setting.tr.t_step = dt;
    setting.tr.t_stop = 2.0 * dt;

    auto& nl = c.get_netlist();

    // Choose omega so that successive sample points land on different sine values:
    // omega*dt = pi/2 => sin(pi/2)=1 at step1, sin(pi)=0 at step2.
    double const omega = std::numbers::pi / (2.0 * dt);

    auto [vbias_src, vbias_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 0.7});
    auto& n_bias = create_node(nl);
    add_to_node(nl, *vbias_src, 0, n_bias);
    add_to_node(nl, *vbias_src, 1, nl.ground_node);

    auto [vac_src, vac_pos] = add_model(nl, ::phy_engine::model::VAC{.m_Vp{0.1}, .m_omega{omega}});
    auto& n_anode = create_node(nl);
    add_to_node(nl, *vac_src, 0, n_anode);
    add_to_node(nl, *vac_src, 1, n_bias);

    ::phy_engine::model::PN_junction d{};
    d.Is = 1e-14;
    d.N = 1.0;
    d.tt = tt_s;

    auto [d1, d1_pos] = add_model(nl, d);
    add_to_node(nl, *d1, 0, n_anode);
    add_to_node(nl, *d1, 1, nl.ground_node);

    if(!c.analyze()) { return std::numeric_limits<double>::quiet_NaN(); }

    auto const bv = vac_src->ptr->generate_branch_view();
    if(bv.size != 1 || bv.branches == nullptr) { return std::numeric_limits<double>::quiet_NaN(); }
    return bv.branches[0].current.real();
}

int main()
{
    auto const i0 = run_case(0.0);
    auto const i1 = run_case(1e-9);
    if(!std::isfinite(i0) || !std::isfinite(i1)) { return 1; }
    if(!(std::abs(i1) > std::abs(i0) + 1e-4)) { return 2; }
    return 0;
}
