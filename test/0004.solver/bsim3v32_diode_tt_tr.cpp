#include <cmath>
#include <limits>
#include <numbers>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/linear/VAC.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/non-linear/bsim3v32.h>
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

    // Drain drive: a small sine around a -0.7V DC bias (keeps the drain-body diode forward biased).
    // Choose omega so that omega*dt = pi/2 => big per-step voltage change to excite diffusion capacitance.
    double const omega = std::numbers::pi / (2.0 * dt);

    auto [vneg_src, vneg_pos] = add_model(nl, ::phy_engine::model::VDC{.V = -0.7});
    auto& n_bias = create_node(nl);
    add_to_node(nl, *vneg_src, 0, n_bias);
    add_to_node(nl, *vneg_src, 1, nl.ground_node);

    auto [vac_src, vac_pos] = add_model(nl, ::phy_engine::model::VAC{.m_Vp{0.1}, .m_omega{omega}});
    auto& n_drain = create_node(nl);
    add_to_node(nl, *vac_src, 0, n_drain);
    add_to_node(nl, *vac_src, 1, n_bias);

    // Gate held at 0V (channel off), source/bulk at GND.
    auto [vg_src, vg_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 0.0});
    auto& n_gate = create_node(nl);
    add_to_node(nl, *vg_src, 0, n_gate);
    add_to_node(nl, *vg_src, 1, nl.ground_node);

    ::phy_engine::model::bsim3v32_nmos mos{};
    mos.tt = tt_s;
    mos.diode_Is = 1e-14;
    mos.diode_N = 1.0;
    // Reduce MOS intrinsic capacitances so the transient branch current mostly reflects the diode diffusion cap.
    mos.tox = 1.0;
    mos.toxm = 1.0;

    auto [m1, m1_pos] = add_model(nl, mos);
    add_to_node(nl, *m1, 0, n_drain);
    add_to_node(nl, *m1, 1, n_gate);
    add_to_node(nl, *m1, 2, nl.ground_node);
    add_to_node(nl, *m1, 3, nl.ground_node);

    // Sanity: ensure the instance tt parameter is actually set.
    {
        auto const v = m1->ptr->get_attribute(83);
        if(v.type != ::phy_engine::model::variant_type::d || !(std::abs(v.d - tt_s) < 1e-18))
        {
            return std::numeric_limits<double>::quiet_NaN();
        }
    }

    if(!c.analyze()) { return std::numeric_limits<double>::quiet_NaN(); }

    auto const bv = vac_src->ptr->generate_branch_view();
    if(bv.size != 1 || bv.branches == nullptr) { return std::numeric_limits<double>::quiet_NaN(); }
    return bv.branches[0].current.real();
}

int main()
{
    auto const i0 = run_case(0.0);
    auto const i1 = run_case(2e-9);

    if(!std::isfinite(i0) || !std::isfinite(i1)) { return 1; }

    // With tt enabled, the diode stamps a diffusion-capacitance companion in TR, increasing transient current magnitude.
    if(!(std::abs(i1) > std::abs(i0) + 1e-4))
    {
        return 2;
    }
    return 0;
}
