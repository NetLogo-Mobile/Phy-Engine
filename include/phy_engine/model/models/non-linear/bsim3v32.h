#pragma once

// Clean-room (no third-party source/terms) BSIM3v3.2 module placeholder.
//
// NOTE:
// BSIM3v3.2 is a very large compact model (hundreds of parameters, charge-based
// C/V, temperature + geometry scaling, optional NQS, etc.). The full Berkeley
// reference implementation cannot be imported here per project constraints.
//
// This header keeps the existing public integration points (type names, pins,
// DC/AC/TR hooks) so the engine builds, while providing a solid foundation to
// incrementally implement the full BSIM3v3.2 equations.

#include <algorithm>
#include <cmath>
#include <complex>

#include <fast_io/fast_io_dsal/string_view.h>

#include "../../model_refs/base.h"
#include "PN_junction.h"

namespace phy_engine::model
{
    namespace details
    {
        // Physical constants (SI)
        inline constexpr double k_q{1.602176634e-19};     // C
        inline constexpr double k_kb{1.380649e-23};       // J/K
        inline constexpr double k_t0{273.15};             // K offset from Celsius
        inline constexpr double k_eps0{8.854187817e-12};  // F/m
        inline constexpr double k_eps_ox{3.9 * k_eps0};   // F/m (SiO2)
        inline constexpr double k_eps_si{11.7 * k_eps0};  // F/m (Si)

        struct bsim3v32_cap_state
        {
            double tr_hist_current{};  // Norton history current
            double tr_prev_g{};        // last-step companion conductance (2*C/dt)
        };

        inline constexpr void
            stamp_cap_ac(::phy_engine::MNA::MNA& mna, ::phy_engine::model::node_t* a, ::phy_engine::model::node_t* b, double c, double omega) noexcept
        {
            if(a == nullptr || b == nullptr || c == 0.0 || omega == 0.0) [[unlikely]] { return; }
            ::std::complex<double> const y{0.0, c * omega};
            mna.G_ref(a->node_index, a->node_index) += y;
            mna.G_ref(a->node_index, b->node_index) -= y;
            mna.G_ref(b->node_index, a->node_index) -= y;
            mna.G_ref(b->node_index, b->node_index) += y;
        }

        inline constexpr void step_cap_tr(bsim3v32_cap_state& st, ::phy_engine::model::node_t* a, ::phy_engine::model::node_t* b, double c, double dt) noexcept
        {
            if(a == nullptr || b == nullptr || c == 0.0 || dt <= 0.0) [[unlikely]]
            {
                st.tr_hist_current = 0.0;
                st.tr_prev_g = 0.0;
                return;
            }

            double const v_prev{a->node_information.an.voltage.real() - b->node_information.an.voltage.real()};
            double const g_new{2.0 * c / dt};

            // Trapezoidal companion source update (order=2):
            // i_hist[n] = - i_hist[n-1] - (g_new + g_old) * v_prev
            st.tr_hist_current = -(g_new + st.tr_prev_g) * v_prev - st.tr_hist_current;
            st.tr_prev_g = g_new;
        }

        inline constexpr void
            stamp_cap_tr(::phy_engine::MNA::MNA& mna, ::phy_engine::model::node_t* a, ::phy_engine::model::node_t* b, bsim3v32_cap_state const& st) noexcept
        {
            if(a == nullptr || b == nullptr || st.tr_prev_g == 0.0) [[unlikely]] { return; }
            double const geq{st.tr_prev_g};
            double const Ieq{st.tr_hist_current};

            mna.G_ref(a->node_index, a->node_index) += geq;
            mna.G_ref(a->node_index, b->node_index) -= geq;
            mna.G_ref(b->node_index, a->node_index) -= geq;
            mna.G_ref(b->node_index, b->node_index) += geq;

            mna.I_ref(a->node_index) -= Ieq;
            mna.I_ref(b->node_index) += Ieq;
        }

        template <bool is_pmos>
        inline constexpr double body_vsb_eff(double vb, double vs) noexcept
        {
            if constexpr(is_pmos)
            {
                // PMOS in n-well: reverse bias increases with (Vb - Vs)
                return ::std::max(vb - vs, 0.0);
            }
            else
            {
                // NMOS in p-sub: reverse bias increases with (Vs - Vb)
                return ::std::max(vs - vb, 0.0);
            }
        }

        template <bool is_pmos>
        inline constexpr void attach_body_diodes([[maybe_unused]] ::phy_engine::model::node_t* nd,
                                                 [[maybe_unused]] ::phy_engine::model::node_t* ns,
                                                 [[maybe_unused]] ::phy_engine::model::node_t* nb,
                                                 [[maybe_unused]] PN_junction& db,
                                                 [[maybe_unused]] PN_junction& sb) noexcept
        {
            if constexpr(is_pmos)
            {
                // p+ diffusion to n-well: anode at D/S, cathode at B
                db.pins[0].nodes = nd;
                db.pins[1].nodes = nb;
                sb.pins[0].nodes = ns;
                sb.pins[1].nodes = nb;
            }
            else
            {
                // p-sub to n+ diffusion: anode at B, cathode at D/S
                db.pins[0].nodes = nb;
                db.pins[1].nodes = nd;
                sb.pins[0].nodes = nb;
                sb.pins[1].nodes = ns;
            }
        }

        inline constexpr void stamp_gm_gds_gmb(::phy_engine::MNA::MNA& mna,
                                               ::phy_engine::model::node_t* nd,
                                               ::phy_engine::model::node_t* ng,
                                               ::phy_engine::model::node_t* ns,
                                               ::phy_engine::model::node_t* nb,
                                               double gm,
                                               double gds,
                                               double gmb,
                                               double Ieq) noexcept
        {
            if(nd == nullptr || ng == nullptr || ns == nullptr || nb == nullptr) [[unlikely]] { return; }

            // gds between D and S
            mna.G_ref(nd->node_index, nd->node_index) += gds;
            mna.G_ref(nd->node_index, ns->node_index) -= gds;
            mna.G_ref(ns->node_index, nd->node_index) -= gds;
            mna.G_ref(ns->node_index, ns->node_index) += gds;

            // gm: VCCS D->S controlled by Vg-Vs
            mna.G_ref(nd->node_index, ng->node_index) += gm;
            mna.G_ref(nd->node_index, ns->node_index) -= gm;
            mna.G_ref(ns->node_index, ng->node_index) -= gm;
            mna.G_ref(ns->node_index, ns->node_index) += gm;

            // gmb: VCCS D->S controlled by Vb-Vs
            mna.G_ref(nd->node_index, nb->node_index) += gmb;
            mna.G_ref(nd->node_index, ns->node_index) -= gmb;
            mna.G_ref(ns->node_index, nb->node_index) -= gmb;
            mna.G_ref(ns->node_index, ns->node_index) += gmb;

            // current source equivalent Ids from D->S
            mna.I_ref(nd->node_index) -= Ieq;
            mna.I_ref(ns->node_index) += Ieq;
        }

        inline constexpr void stamp_resistor(::phy_engine::MNA::MNA& mna, ::phy_engine::model::node_t* a, ::phy_engine::model::node_t* b, double r) noexcept
        {
            if(a == nullptr || b == nullptr) [[unlikely]] { return; }
            if(r <= 0.0) [[unlikely]] { return; }
            double const g{1.0 / r};

            mna.G_ref(a->node_index, a->node_index) += g;
            mna.G_ref(a->node_index, b->node_index) -= g;
            mna.G_ref(b->node_index, a->node_index) -= g;
            mna.G_ref(b->node_index, b->node_index) += g;
        }

        inline double bsim3v32_junction_cap(double cj0, double vj, double pb, double m, double fc) noexcept
        {
            if(cj0 == 0.0) { return 0.0; }
            pb = (pb > 1e-12) ? pb : 1e-12;
            fc = ::std::clamp(fc, 0.0, 0.99);

            double const vfc{fc * pb};
            if(vj < vfc)
            {
                double const arg{1.0 - vj / pb};
                // For reverse bias (vj <= 0) arg > 1 and capacitance decreases.
                return cj0 / ::std::pow(arg, m);
            }

            // Forward-bias linear continuation beyond fc*pb (standard SPICE junction C model).
            double const denom{::std::pow(1.0 - fc, m)};
            double const slope{m / (pb * (1.0 - fc))};
            return (cj0 / denom) * (1.0 + slope * (vj - vfc));
        }

        inline constexpr double bsim3v32_temp_linear_scale(double base, double tc, double temp_c, double tnom_c) noexcept
        {
            // Simple SPICE-style linear temperature scaling anchored at TNOM:
            // X(T) = X(Tnom) * (1 + tc*(T - Tnom)).
            return base * (1.0 + tc * (temp_c - tnom_c));
        }

        inline constexpr double bsim3v32_temp_linear_scale_clamped(double base, double tc, double temp_c, double tnom_c, double min_value) noexcept
        {
            double const v{bsim3v32_temp_linear_scale(base, tc, temp_c, tnom_c)};
            return v > min_value ? v : min_value;
        }

        inline constexpr double bsim3v32_lwref_default(double x) noexcept
        {
            // BSIM-style reference geometry; use 1um when unspecified.
            return (x > 0.0) ? x : 1e-6;
        }

        inline constexpr double
            bsim3v32_lw_scale(double base, double l_coeff, double w_coeff, double p_coeff, double leff, double weff, double lref, double wref) noexcept
        {
            // Clean-room BSIM-style linear L/W scaling:
            // p_eff = p0 + l*(Leff - Lref) + w*(Weff - Wref) + p*(Leff - Lref)*(Weff - Wref)
            double const lref_eff{bsim3v32_lwref_default(lref)};
            double const wref_eff{bsim3v32_lwref_default(wref)};
            double const dl{leff - lref_eff};
            double const dw{weff - wref_eff};
            return base + l_coeff * dl + w_coeff * dw + p_coeff * dl * dw;
        }

        inline double thermal_voltage(double temp_c) noexcept
        {
            double const t_k{temp_c + k_t0};
            if(t_k <= 1.0) { return 0.0; }
            return k_kb * t_k / k_q;
        }

        inline double silicon_bandgap_ev(double t_k) noexcept
        {
            // Empirical Si bandgap vs temperature (eV). Valid over typical circuit temps.
            // Eg(T) = 1.16 - 7.02e-4 * T^2 / (T + 1108)
            if(!(t_k > 1.0)) { return 1.11; }
            double const t2{t_k * t_k};
            return 1.16 - (7.02e-4 * t2) / (t_k + 1108.0);
        }

        inline double silicon_ni_m3(double t_k) noexcept
        {
            // Intrinsic carrier concentration for silicon (1/m^3), normalized to ni(300K)=1.45e16 1/m^3.
            // ni(T) = ni300 * (T/300)^(3/2) * exp(-(Eg(T)/(2kT)) + (Eg(300)/(2k*300))).
            constexpr double ni300_m3{1.45e16};
            constexpr double t300{300.0};
            constexpr double k_kb_ev{8.617333262e-5};  // eV/K
            if(!(t_k > 1.0)) { return ni300_m3; }
            double const eg_t{silicon_bandgap_ev(t_k)};
            double const eg_300{silicon_bandgap_ev(t300)};
            double const ratio{t_k / t300};
            double const pow_term{::std::pow(ratio, 1.5)};
            double const exp_term{-(eg_t / (2.0 * k_kb_ev * t_k)) + (eg_300 / (2.0 * k_kb_ev * t300))};
            return ni300_m3 * pow_term * ::std::exp(exp_term);
        }

        inline double bsim3v32_phi_temp(double phi0, double nch_m3, double temp_c, double tnom_c) noexcept
        {
            // Temperature-scaled surface potential, anchored such that phi(Tnom) == phi0.
            // Uses a MOS-physics-consistent form: phi(T) ~ 2*Vt(T)*ln(Nch/ni(T)).
            double const phi0_eff{phi0 > 1e-12 ? phi0 : 1e-12};
            if(!(nch_m3 > 0.0)) { return phi0_eff; }

            double const t_k{temp_c + k_t0};
            double const tnom_k{tnom_c + k_t0};
            if(!(t_k > 1.0) || !(tnom_k > 1.0)) { return phi0_eff; }

            double const ni_t{silicon_ni_m3(t_k)};
            double const ni_nom{silicon_ni_m3(tnom_k)};
            if(!(ni_t > 0.0) || !(ni_nom > 0.0)) { return phi0_eff; }

            double const vt_t{k_kb * t_k / k_q};
            double const vt_nom{k_kb * tnom_k / k_q};

            double const ratio_t{::std::max(nch_m3 / ni_t, 1.0 + 1e-30)};
            double const ratio_nom{::std::max(nch_m3 / ni_nom, 1.0 + 1e-30)};

            double const phi_form{2.0 * vt_t * ::std::log(ratio_t)};
            double const phi_form_nom{2.0 * vt_nom * ::std::log(ratio_nom)};
            if(!(phi_form_nom > 1e-12)) { return phi0_eff; }

            double const phi_t{phi0_eff * (phi_form / phi_form_nom)};
            return (phi_t > 1e-12) ? phi_t : 1e-12;
        }

        inline double limexp(double x) noexcept
        {
            // Smooth overflow protection similar to SPICE "limexp".
            if(x > 50.0) { return ::std::exp(50.0) * (1.0 + (x - 50.0)); }
            if(x < -50.0) { return ::std::exp(-50.0); }
            return ::std::exp(x);
        }

        inline double bsim3v32_is_temp_scale(double temp_c, double tnom_c, double xti, double eg) noexcept
        {
            // Simple SPICE-style saturation current temperature scaling:
            // Is(T) = Is(Tnom) * (T/Tnom)^xti * exp(-Eg/k * (1/T - 1/Tnom))
            constexpr double k_kb_ev{8.617333262e-5};  // eV/K
            double const t{temp_c + k_t0};
            double const tnom{tnom_c + k_t0};
            if(t <= 1.0 || tnom <= 1.0) { return 1.0; }
            double const ratio{t / tnom};
            double const xti_eff{xti != 0.0 ? xti : 3.0};
            double const eg_eff{eg > 0.0 ? eg : 1.11};
            double const exp_term{-eg_eff / k_kb_ev * (1.0 / t - 1.0 / tnom)};
            return ::std::pow(ratio, xti_eff) * ::std::exp(exp_term);
        }

        inline double bsim3v32_vth0_temp_mag(double vth0, double temp_c, double tnom_c, double kt1, double kt2) noexcept
        {
            // Treat Vth0 as a magnitude internally to allow PMOS modelcards that specify negative Vth0.
            double const dt{temp_c - tnom_c};
            double const v{vth0 + kt1 * dt + kt2 * dt * dt};
            return ::std::abs(v);
        }

        inline double bsim3v32_fetlim(double vnew, double vold, double vto) noexcept
        {
            // Voltage limiting for FET controlling voltages (Vgs/Vgd/Vbs-like) to improve Newton convergence.
            // Clean-room implementation inspired by common SPICE limiting behavior.
            if(!::std::isfinite(vnew) || !::std::isfinite(vold) || !::std::isfinite(vto)) { return vnew; }

            double const dv{vnew - vold};
            double const vtox{vto + 3.5};

            double const vtsthi{::std::abs(2.0 * (vold - vto)) + 2.0};
            double const vtstlo{0.5 * vtsthi + 2.0};

            if(vold > vto)
            {
                if(dv > 0.0)
                {
                    double const max_step{vnew > vtox ? vtsthi : vtstlo};
                    if(dv > max_step) { vnew = vold + max_step; }
                }
                else if(dv < 0.0)
                {
                    double const vmin{vto - 0.5};
                    if(vnew < vmin) { vnew = vmin; }
                }
            }
            else
            {
                if(dv > 0.0)
                {
                    double const vmax{vto + 0.5};
                    if(vnew > vmax) { vnew = vmax; }
                }
                else if(dv < 0.0)
                {
                    if(vnew < vto - 0.5)
                    {
                        if(-dv > vtsthi) { vnew = vold - vtsthi; }
                    }
                    else
                    {
                        if(-dv > vtstlo) { vnew = vold - vtstlo; }
                    }
                }
            }

            return vnew;
        }

        inline double bsim3v32_limvds(double vnew, double vold) noexcept
        {
            // Voltage limiting for Vds to avoid large jumps that can destabilize the MOS equations.
            // Clean-room implementation following common SPICE limiting behavior.
            if(!::std::isfinite(vnew) || !::std::isfinite(vold)) { return vnew; }

            if(vold >= 0.0)
            {
                if(vnew >= 0.0)
                {
                    double const vmax{3.0 * vold + 2.0};
                    double const vmin{vold - 2.0};
                    vnew = ::std::min(vnew, vmax);
                    vnew = ::std::max(vnew, vmin);
                }
                else
                {
                    vnew = -2.0;
                }
            }
            else
            {
                if(vnew <= 0.0)
                {
                    double const vmin{3.0 * vold - 2.0};
                    double const vmax{vold + 2.0};
                    vnew = ::std::max(vnew, vmin);
                    vnew = ::std::min(vnew, vmax);
                }
                else
                {
                    vnew = 2.0;
                }
            }

            return vnew;
        }

        struct bsim3v32_dual3
        {
            double val{};
            double dvgs{};
            double dvds{};
            double dvbs{};

            constexpr bsim3v32_dual3() noexcept = default;

            constexpr bsim3v32_dual3(double v) noexcept : val(v) {}

            constexpr bsim3v32_dual3(double v, double dvgs_in, double dvds_in, double dvbs_in) noexcept : val(v), dvgs(dvgs_in), dvds(dvds_in), dvbs(dvbs_in) {}
        };

        inline constexpr bsim3v32_dual3 bsim3v32_dual3_vgs(double v) noexcept { return {v, 1.0, 0.0, 0.0}; }

        inline constexpr bsim3v32_dual3 bsim3v32_dual3_vds(double v) noexcept { return {v, 0.0, 1.0, 0.0}; }

        inline constexpr bsim3v32_dual3 bsim3v32_dual3_vbs(double v) noexcept { return {v, 0.0, 0.0, 1.0}; }

        inline constexpr double bsim3v32_value(double x) noexcept { return x; }

        inline constexpr double bsim3v32_value(bsim3v32_dual3 x) noexcept { return x.val; }

        inline constexpr bsim3v32_dual3 operator+ (bsim3v32_dual3 a, bsim3v32_dual3 b) noexcept
        { return {a.val + b.val, a.dvgs + b.dvgs, a.dvds + b.dvds, a.dvbs + b.dvbs}; }

        inline constexpr bsim3v32_dual3 operator- (bsim3v32_dual3 a, bsim3v32_dual3 b) noexcept
        { return {a.val - b.val, a.dvgs - b.dvgs, a.dvds - b.dvds, a.dvbs - b.dvbs}; }

        inline constexpr bsim3v32_dual3 operator- (bsim3v32_dual3 a) noexcept { return {-a.val, -a.dvgs, -a.dvds, -a.dvbs}; }

        inline constexpr bsim3v32_dual3 operator* (bsim3v32_dual3 a, bsim3v32_dual3 b) noexcept
        {
            return {
                a.val * b.val,
                a.dvgs * b.val + b.dvgs * a.val,
                a.dvds * b.val + b.dvds * a.val,
                a.dvbs * b.val + b.dvbs * a.val,
            };
        }

        inline constexpr bsim3v32_dual3 operator/ (bsim3v32_dual3 a, bsim3v32_dual3 b) noexcept
        {
            double const inv{1.0 / b.val};
            double const inv2{inv * inv};
            return {
                a.val * inv,
                (a.dvgs * b.val - a.val * b.dvgs) * inv2,
                (a.dvds * b.val - a.val * b.dvds) * inv2,
                (a.dvbs * b.val - a.val * b.dvbs) * inv2,
            };
        }

        inline constexpr bsim3v32_dual3& operator+= (bsim3v32_dual3& a, bsim3v32_dual3 b) noexcept
        {
            a = a + b;
            return a;
        }

        inline constexpr bsim3v32_dual3& operator-= (bsim3v32_dual3& a, bsim3v32_dual3 b) noexcept
        {
            a = a - b;
            return a;
        }

        inline constexpr bsim3v32_dual3& operator*= (bsim3v32_dual3& a, bsim3v32_dual3 b) noexcept
        {
            a = a * b;
            return a;
        }

        inline constexpr bsim3v32_dual3& operator/= (bsim3v32_dual3& a, bsim3v32_dual3 b) noexcept
        {
            a = a / b;
            return a;
        }

        inline double bsim3v32_exp(double x) noexcept { return ::std::exp(x); }

        inline bsim3v32_dual3 bsim3v32_exp(bsim3v32_dual3 x) noexcept
        {
            double const e{::std::exp(x.val)};
            return {e, e * x.dvgs, e * x.dvds, e * x.dvbs};
        }

        inline double bsim3v32_log(double x) noexcept { return ::std::log(x); }

        inline bsim3v32_dual3 bsim3v32_log(bsim3v32_dual3 x) noexcept
        {
            double const inv{1.0 / x.val};
            return {::std::log(x.val), inv * x.dvgs, inv * x.dvds, inv * x.dvbs};
        }

        inline double bsim3v32_log1p(double x) noexcept { return ::std::log1p(x); }

        inline bsim3v32_dual3 bsim3v32_log1p(bsim3v32_dual3 x) noexcept
        {
            double const inv{1.0 / (1.0 + x.val)};
            return {::std::log1p(x.val), inv * x.dvgs, inv * x.dvds, inv * x.dvbs};
        }

        inline double bsim3v32_sqrt(double x) noexcept { return ::std::sqrt(x); }

        inline bsim3v32_dual3 bsim3v32_sqrt(bsim3v32_dual3 x) noexcept
        {
            if(x.val <= 0.0) { return {}; }
            double const s{::std::sqrt(x.val)};
            double const inv2s{0.5 / s};
            return {s, inv2s * x.dvgs, inv2s * x.dvds, inv2s * x.dvbs};
        }

        inline double bsim3v32_abs_smooth(double x) noexcept
        {
            // Smooth |x| ~= sqrt(x^2 + eps) to keep derivatives well-defined.
            constexpr double eps{1e-30};
            return ::std::sqrt(x * x + eps);
        }

        inline bsim3v32_dual3 bsim3v32_abs_smooth(bsim3v32_dual3 x) noexcept
        {
            constexpr double eps{1e-30};
            bsim3v32_dual3 const s{bsim3v32_sqrt(x * x + eps)};
            if(s.val == 0.0) { return {}; }
            double const invs{1.0 / s.val};
            // d/dx sqrt(x^2+eps) = x / sqrt(x^2+eps)
            return {s.val, x.val * x.dvgs * invs, x.val * x.dvds * invs, x.val * x.dvbs * invs};
        }

        inline double bsim3v32_max(double x, double lo) noexcept { return x > lo ? x : lo; }

        inline bsim3v32_dual3 bsim3v32_max(bsim3v32_dual3 x, double lo) noexcept { return x.val > lo ? x : bsim3v32_dual3{lo}; }

        template <typename Real>
        inline Real bsim3v32_vbseff(Real vbs, double vbc, double delta1) noexcept
        {
            // Eq. (2.1.1) / Appendix B: effective Vbs (clamps to vbc with smooth transition).
            // NOTE: vbc is typically negative (max reverse body bias), so the sqrt uses "-4*delta1*vbc".
            Real const t0{vbs - vbc - delta1};
            Real const arg{t0 * t0 - 4.0 * delta1 * vbc};
            Real const s{bsim3v32_value(arg) > 0.0 ? bsim3v32_sqrt(arg) : Real{}};
            return vbc + 0.5 * (t0 + s);
        }

        template <typename Real>
        inline Real bsim3v32_vgsteff(Real vgs_minus_vth, Real n, double vt) noexcept
        {
            // Eq. (3.1.3): smooth effective (Vgs - Vth) to cover subthreshold.
            Real const denom{2.0 * n * vt};
            if(bsim3v32_value(denom) <= 0.0) { return bsim3v32_max(vgs_minus_vth, 0.0); }
            Real const x{vgs_minus_vth / denom};
            // Use log1p(exp(x)) with overflow protection.
            if(bsim3v32_value(x) > 40.0) { return vgs_minus_vth; }
            return 2.0 * n * vt * bsim3v32_log1p(bsim3v32_exp(x));
        }

        template <typename Real>
        inline Real bsim3v32_ueff_mobmod3(double u0, double ua, double ub, double uc, Real vgst_eff, Real vbs_eff, double tox, double vt) noexcept
        {
            // Eq. (3.2.3) (mobMod=3)
            if(u0 <= 0.0 || tox <= 0.0) { return Real{}; }
            Real const t0{(vgst_eff) + 2.0 * vt};
            Real const eeff{t0 / tox};
            Real denom{1.0 + (ua * eeff + ub * eeff * eeff) * (1.0 + uc * vbs_eff)};
            if(bsim3v32_value(denom) <= 1e-18) { denom = 1e-18; }
            return u0 / denom;
        }

        struct bsim3v32_dc_eval
        {
            double Id{};  // current from D->S (external sign convention)
        };

        template <typename Real>
        struct bsim3v32_core_cache_t
        {
            Real weff{};
            Real leff{};
            Real cox{};
            Real vth{};
            Real vgsteff{};
            Real vdsat{};
            Real vdseff{};
        };

        using bsim3v32_core_cache = bsim3v32_core_cache_t<double>;

        template <typename Mos, typename Real>
        inline Real bsim3v32_ids_core(Mos const& m, Real vgs, Real vds, Real vbs, double vt, bsim3v32_core_cache_t<Real>* cache) noexcept
        {
            // Minimal BSIM3v3.x DC core based on the equation list (threshold + Vgsteff + velocity sat + CLM hooks).
            // This is intentionally written as a clean-room implementation (no third-party code).

            if(cache) { *cache = {}; }

            // This core is written for Vds >= 0 in the internal (n-type) coordinate.
            // The outer stamping selects the channel orientation so this is normally true, but keep it robust.
            if(bsim3v32_value(vds) < 0.0) { vds = Real{}; }

            // Effective geometry
            double const weff{::std::max(m.W - 2.0 * ::std::max(m.dwc, 0.0), 0.0)};
            double const leff{::std::max(m.L - 2.0 * ::std::max(m.dlc, 0.0), 1e-18)};
            if(weff == 0.0) { return 0.0; }

            double const tox{m.tox > 0.0 ? m.tox : 1e-8};
            double const toxm{m.toxm > 0.0 ? m.toxm : tox};
            double const tox_ratio{tox / toxm};
            double const cox{tox > 0.0 ? (k_eps_ox / tox) : 0.0};
            if(cox <= 0.0) { return 0.0; }
            if(cache)
            {
                cache->weff = weff;
                cache->leff = leff;
                cache->cox = cox;
            }

            // Mobility base: allow legacy Kp (= u0*Cox) fallback.
            double const vth0_eff_geom{details::bsim3v32_lw_scale(m.Vth0, m.lvth0, m.wvth0, m.pvth0, leff, weff, m.lref, m.wref)};
            double const kp_eff_geom{details::bsim3v32_lw_scale(m.Kp, m.lkp, m.wkp, m.pkp, leff, weff, m.lref, m.wref)};
            double u0{details::bsim3v32_lw_scale(m.u0, m.lu0, m.wu0, m.pu0, leff, weff, m.lref, m.wref)};
            double const ua_eff{details::bsim3v32_lw_scale(m.ua, m.lua, m.wua, m.pua, leff, weff, m.lref, m.wref)};
            double const ub_eff{details::bsim3v32_lw_scale(m.ub, m.lub, m.wub, m.pub, leff, weff, m.lref, m.wref)};
            double const uc_eff{details::bsim3v32_lw_scale(m.uc, m.luc, m.wuc, m.puc, leff, weff, m.lref, m.wref)};
            double const vsat_eff_geom{details::bsim3v32_lw_scale(m.vsat, m.lvsat, m.wvsat, m.pvsat, leff, weff, m.lref, m.wref)};
            if(u0 <= 0.0)
            {
                if(kp_eff_geom > 0.0) { u0 = kp_eff_geom / cox; }
                else
                {
                    u0 = 0.0;
                }
            }

            // Temperature scaling (subset).
            double const t_k{m.Temp + k_t0};
            double const tnom_k{m.tnom + k_t0};
            if(u0 > 0.0 && m.ute != 0.0 && t_k > 1.0 && tnom_k > 1.0)
            {
                double const ratio{t_k / tnom_k};
                // mobility ~ (T/Tnom)^(-ute)
                u0 *= ::std::pow(ratio, -m.ute);
            }

            double const nch_eff_raw{details::bsim3v32_lw_scale(m.nch, m.lnch, m.wnch, m.pnch, leff, weff, m.lref, m.wref)};
            double const nch_eff{nch_eff_raw > 1.0 ? nch_eff_raw : (m.nch > 1.0 ? m.nch : 1e23)};
            double const phi0_eff_geom{details::bsim3v32_lw_scale(m.phi, m.lphi, m.wphi, m.pphi, leff, weff, m.lref, m.wref)};
            double const phi_s{bsim3v32_phi_temp(phi0_eff_geom, nch_eff, m.Temp, m.tnom)};
            double const vbm{(m.vbm < 0.0) ? m.vbm : -3.0};
            double const delta1{(m.delta1 > 0.0) ? m.delta1 : 1e-3};
            double const vbc{vbm};
            Real const vbseff{bsim3v32_vbseff(vbs, vbc, delta1)};

            double const gamma_eff_raw{details::bsim3v32_lw_scale(m.gamma, m.lgamma, m.wgamma, m.pgamma, leff, weff, m.lref, m.wref)};
            double const gamma_eff{gamma_eff_raw > 0.0 ? gamma_eff_raw : 0.0};
            double const k1_base{(m.k1 != 0.0) ? m.k1 : gamma_eff};
            double const k1_eff{details::bsim3v32_lw_scale(k1_base, m.lk1, m.wk1, m.pk1, leff, weff, m.lref, m.wref)};
            double const k2_eff{details::bsim3v32_lw_scale(m.k2, m.lk2, m.wk2, m.pk2, leff, weff, m.lref, m.wref)};
            double const k1ox{k1_eff * tox_ratio};
            double const k2ox{k2_eff * tox_ratio};

            double const sqrt_phi{::std::sqrt(phi_s)};
            double const dt_c{m.Temp - m.tnom};
            double const vth0_t{bsim3v32_vth0_temp_mag(vth0_eff_geom, m.Temp, m.tnom, m.kt1, m.kt2)};
            double const vth0ox{vth0_t - k1_eff * sqrt_phi};

            Real const sqrt_phi_vbs{bsim3v32_sqrt(bsim3v32_max(phi_s - vbseff, 1e-12))};

            // Xdep / lt / lt0 (simplified)
            double const nch{nch_eff > 1.0 ? nch_eff : 1e23};  // 1/m^3
            Real const xdep{bsim3v32_sqrt(2.0 * k_eps_si * bsim3v32_max(phi_s - vbseff, 1e-12) / (k_q * nch))};
            double const xdep0{::std::sqrt(2.0 * k_eps_si * phi_s / (k_q * nch))};
            double const lt0{::std::sqrt((k_eps_si / k_eps_ox) * tox * xdep0)};
            Real lt{bsim3v32_sqrt((k_eps_si / k_eps_ox) * tox * xdep)};
            double const dvt2_eff{details::bsim3v32_lw_scale(m.dvt2, m.ldvt2, m.wdvt2, m.pdvt2, leff, weff, m.lref, m.wref)};
            lt *= (1.0 + dvt2_eff * vbseff);
            if(bsim3v32_value(lt) <= 1e-18) { lt = 1e-18; }

            // Short-channel Vth roll-off
            double const dvt0_eff{details::bsim3v32_lw_scale(m.dvt0, m.ldvt0, m.wdvt0, m.pdvt0, leff, weff, m.lref, m.wref)};
            double const dvt1_eff{details::bsim3v32_lw_scale(m.dvt1, m.ldvt1, m.wdvt1, m.pdvt1, leff, weff, m.lref, m.wref)};
            Real const theta_th{dvt0_eff * (bsim3v32_exp(-dvt1_eff * leff / (2.0 * lt)) + 2.0 * bsim3v32_exp(-dvt1_eff * leff / lt))};
            double const vbi{m.vbi > 0.0 ? m.vbi : (phi_s + 0.5)};
            Real const dvth_sc{theta_th * (vbi - phi_s)};

            // DIBL
            double const dsub_eff{details::bsim3v32_lw_scale(m.dsub, m.ldsub, m.wdsub, m.pdsub, leff, weff, m.lref, m.wref)};
            double const eta0_eff{details::bsim3v32_lw_scale(m.eta0, m.leta0, m.weta0, m.peta0, leff, weff, m.lref, m.wref)};
            double const etab_eff{details::bsim3v32_lw_scale(m.etab, m.letab, m.wetab, m.petab, leff, weff, m.lref, m.wref)};
            double const theta_dibl{::std::exp(-dsub_eff * leff / (2.0 * lt0)) + 2.0 * ::std::exp(-dsub_eff * leff / lt0)};
            Real const dvth_dibl{theta_dibl * (eta0_eff + etab_eff * vbseff) * vds};

            // Narrow width / doping non-uniformity (simplified)
            double const nlx_eff{details::bsim3v32_lw_scale(m.nlx, m.lnlx, m.wnlx, m.pnlx, leff, weff, m.lref, m.wref)};
            double const k3_eff{details::bsim3v32_lw_scale(m.k3, m.lk3, m.wk3, m.pk3, leff, weff, m.lref, m.wref)};
            double const k3b_eff{details::bsim3v32_lw_scale(m.k3b, m.lk3b, m.wk3b, m.pk3b, leff, weff, m.lref, m.wref)};
            double const w0_eff_raw{details::bsim3v32_lw_scale(m.w0, m.lw0, m.ww0, m.pw0, leff, weff, m.lref, m.wref)};
            double const w0_eff{w0_eff_raw > 0.0 ? w0_eff_raw : 0.0};

            double const dvth_nlx{k1ox * ((nlx_eff > 0.0 ? nlx_eff : 0.0) / leff) * sqrt_phi};
            Real const dvth_nw{(k3_eff + k3b_eff * vbseff) * tox_ratio * phi_s / (::std::max(weff + w0_eff, 1e-18))};

            Real const vth{vth0ox + k1ox * sqrt_phi_vbs - k2ox * vbseff + dvth_nlx + dvth_nw - dvth_sc - dvth_dibl};
            if(cache) { cache->vth = vth; }

            // n factor (simplified)
            Real const cd{k_eps_si / bsim3v32_max(xdep, 1e-18)};
            double const nfactor_eff_raw{details::bsim3v32_lw_scale(m.nfactor, m.lnfactor, m.wnfactor, m.pnfactor, leff, weff, m.lref, m.wref)};
            double const cit_eff{details::bsim3v32_lw_scale(m.cit, m.lcit, m.wcit, m.pcit, leff, weff, m.lref, m.wref)};
            Real n{1.0 + (nfactor_eff_raw > 0.0 ? nfactor_eff_raw : 0.0)};
            n += (cd + cit_eff) / cox;
            if(bsim3v32_value(n) < 1.0) { n = 1.0; }

            // Effective (Vgs - Vth - Voff)
            double const voff_eff{details::bsim3v32_lw_scale(m.voff, m.lvoff, m.wvoff, m.pvoff, leff, weff, m.lref, m.wref)};
            Real const vgst{vgs - vth - voff_eff};
            Real const vgsteff{bsim3v32_vgsteff(vgst, n, vt)};
            if(cache) { cache->vgsteff = vgsteff; }

            // Subthreshold blending (clean-room, simplified):
            // - Use an exponential Ids below threshold with slope ~ 1/(n*Vt).
            // - Blend smoothly with the strong-inversion core using a sign-based smooth step on Vgs-Vth.
            Real f_sub{};
            Real ids_sub{};
            if(vt > 0.0 && u0 > 0.0)
            {
                // f_sub ~= 1 for Vgs<Vth, ~=0 for Vgs>Vth.
                Real const abs_vgst{bsim3v32_abs_smooth(vgst)};
                f_sub = 0.5 * (1.0 - vgst / bsim3v32_max(abs_vgst, 1e-24));

                // Clamp exponent input to <=0 to avoid overflow when Vgs>Vth (the blend should suppress it anyway).
                Real const vgst_neg{0.5 * (vgst - abs_vgst)};
                Real const inv_nvt{1.0 / bsim3v32_max(n * vt, 1e-24)};
                Real const exp_vgs{bsim3v32_exp(vgst_neg * inv_nvt)};

                Real const vds_over_vt{vds / vt};
                Real vds_term{};
                if(bsim3v32_value(vds_over_vt) < 1e-5)
                {
                    // 1 - exp(-x) ~ x - x^2/2 for small x
                    Real const x{vds_over_vt};
                    vds_term = x - 0.5 * x * x;
                }
                else
                {
                    vds_term = 1.0 - bsim3v32_exp(-vds_over_vt);
                }
                if(bsim3v32_value(vds_term) < 0.0) { vds_term = Real{}; }

                Real const id0{(weff * u0 * cox * vt * vt) / bsim3v32_max(leff, 1e-24)};
                ids_sub = id0 * exp_vgs * vds_term;
            }

            // Mobility (mobMod=3 default)
            Real ueff{bsim3v32_ueff_mobmod3(u0, ua_eff, ub_eff, uc_eff, vgsteff, vbseff, tox, vt)};
            if(bsim3v32_value(ueff) <= 0.0) { ueff = u0; }

            // Abulk (start with 1, keep hooks for keta)
            Real abulk{1.0};
            double const keta_eff{details::bsim3v32_lw_scale(m.keta, m.lketa, m.wketa, m.pketa, leff, weff, m.lref, m.wref)};
            abulk *= (1.0 + keta_eff * vbseff);

            // Velocity saturation
            double vsat{vsat_eff_geom > 0.0 ? vsat_eff_geom : 8e4};
            if(m.at != 0.0)
            {
                vsat *= (1.0 + m.at * dt_c);
                if(vsat <= 1.0) { vsat = 1.0; }
            }
            Real const esat{2.0 * vsat / bsim3v32_max(ueff, 1e-18)};

            Real const vdsat{vgsteff / (abulk + vgsteff / bsim3v32_max(esat * leff, 1e-18))};
            if(cache) { cache->vdsat = vdsat; }

            // Vdseff smoothing (Eq. 3.5.1 / Appendix B)
            double const delta{(m.delta > 0.0) ? m.delta : 1e-2};
            Real const t1{vdsat - vds - delta};
            Real const vdseff{vdsat - 0.5 * (t1 + bsim3v32_sqrt(t1 * t1 + 4.0 * delta * vdsat))};
            if(cache) { cache->vdseff = vdseff; }

            // Idso (Eq. 3.5.2 / Appendix B)
            Real const vgst2{vgsteff + 2.0 * vt};
            Real const t2{1.0 - abulk * vdseff / (2.0 * bsim3v32_max(vgst2, 1e-18))};
            Real const denom{leff * (1.0 + vdseff / bsim3v32_max(esat * leff, 1e-18))};
            Real idso{(weff * ueff * cox * vgsteff * t2 * vdseff) / bsim3v32_max(denom, 1e-24)};

            double const pclm_eff_raw{details::bsim3v32_lw_scale(m.pclm, m.lpclm, m.wpclm, m.ppclm, leff, weff, m.lref, m.wref)};
            double const pdiblc1_eff{details::bsim3v32_lw_scale(m.pdiblc1, m.lpdiblc1, m.wpdiblc1, m.ppdiblc1, leff, weff, m.lref, m.wref)};
            double const pdiblc2_eff{details::bsim3v32_lw_scale(m.pdiblc2, m.lpdiblc2, m.wpdiblc2, m.ppdiblc2, leff, weff, m.lref, m.wref)};
            double const pdiblcb_eff{details::bsim3v32_lw_scale(m.pdiblcb, m.lpdiblcb, m.wpdiblcb, m.ppdiblcb, leff, weff, m.lref, m.wref)};
            double const drout_eff_raw{details::bsim3v32_lw_scale(m.drout, m.ldrout, m.wdrout, m.pdrout, leff, weff, m.lref, m.wref)};
            double const pvag_eff{details::bsim3v32_lw_scale(m.pvag, m.lpvag, m.wpvag, m.ppvag, leff, weff, m.lref, m.wref)};
            double const pscbe1_eff_raw{details::bsim3v32_lw_scale(m.pscbe1, m.lpscbe1, m.wpscbe1, m.ppscbe1, leff, weff, m.lref, m.wref)};
            double const pscbe2_eff{details::bsim3v32_lw_scale(m.pscbe2, m.lpscbe2, m.wpscbe2, m.ppscbe2, leff, weff, m.lref, m.wref)};

            double const pclm_eff{pclm_eff_raw > 0.0 ? pclm_eff_raw : 0.0};
            double const drout_eff{drout_eff_raw > 0.0 ? drout_eff_raw : 0.0};
            double const pscbe1_eff{pscbe1_eff_raw > 0.0 ? pscbe1_eff_raw : 0.0};

            // Simple legacy CLM hook (lambda) if advanced CLM params are unset.
            if(pclm_eff == 0.0 && pdiblc1_eff == 0.0 && m.lambda != 0.0) { idso *= (1.0 + m.lambda * vds); }

            // Channel length modulation + DIBL + SCBE (optional)
            Real const vdsx{bsim3v32_max(vds - vdseff, 0.0)};

            Real vaclm{};
            if(pclm_eff > 0.0 && bsim3v32_value(vdsx) > 0.0)
            {
                vaclm = (abulk * esat * leff + vgsteff) * vdsx / (pclm_eff * abulk * esat * bsim3v32_max(lt, 1e-18));
            }

            double theta_rout{};
            if(pdiblc1_eff != 0.0 || pdiblc2_eff != 0.0)
            {
                theta_rout = pdiblc1_eff * (::std::exp(-drout_eff * leff / (2.0 * lt0)) + 2.0 * ::std::exp(-drout_eff * leff / lt0)) + pdiblc2_eff;
            }

            Real vadiblc{};
            if(theta_rout != 0.0)
            {
                Real const t3{1.0 - abulk * vdsat / bsim3v32_max(abulk * vdsat + vgst2, 1e-18)};
                Real const pvag_factor{1.0 + pvag_eff * ueff * vgsteff / ::std::max(2.0 * vsat * leff, 1e-18)};
                vadiblc = (vgst2 / bsim3v32_max(theta_rout * (1.0 + pdiblcb_eff * vbseff) * t3, 1e-18)) * pvag_factor;
            }

            Real inv_va{};
            if(bsim3v32_value(vaclm) > 0.0) { inv_va += 1.0 / vaclm; }
            if(bsim3v32_value(vadiblc) > 0.0) { inv_va += 1.0 / vadiblc; }
            Real const va{bsim3v32_value(inv_va) > 0.0 ? (1.0 / inv_va) : Real{1e30}};

            Real inv_vascbe{};
            if(pscbe1_eff > 0.0 && pscbe2_eff != 0.0 && bsim3v32_value(vdsx) > 1e-12)
            {
                inv_vascbe = (pscbe2_eff * bsim3v32_exp(-pscbe1_eff * lt / vdsx)) / leff;
            }
            Real const vascbe{bsim3v32_value(inv_vascbe) > 0.0 ? (1.0 / inv_vascbe) : Real{1e30}};

            Real rds_eff{::std::max(m.rds, 0.0)};
            double const rdsw_eff_geom{details::bsim3v32_lw_scale(m.rdsw, m.lrdsw, m.wrdsw, m.prdsw, leff, weff, m.lref, m.wref)};
            if(rdsw_eff_geom > 0.0)
            {
                // rdsw is Ohm*m; convert to Ohm by dividing by effective width.
                double const weff_d{weff > 1e-18 ? weff : 1e-18};
                double const rdsw_term{rdsw_eff_geom / weff_d};
                double const prwg_eff{details::bsim3v32_lw_scale(m.prwg, m.lprwg, m.wprwg, m.pprwg, leff, weff, m.lref, m.wref)};
                double const prwb_eff{details::bsim3v32_lw_scale(m.prwb, m.lprwb, m.wprwb, m.pprwb, leff, weff, m.lref, m.wref)};
                Real mod{1.0 + prwg_eff * vgsteff + prwb_eff * vbseff};
                if(bsim3v32_value(mod) < 0.0) { mod = Real{}; }
                rds_eff += rdsw_term * mod;
            }
            Real const clm_factor{1.0 + vdsx / va};
            Real const scbe_factor{1.0 + vdsx / vascbe};
            Real const rds_factor{1.0 / (1.0 + rds_eff * bsim3v32_abs_smooth(idso))};

            Real const ids_inv{idso * clm_factor * scbe_factor * rds_factor};
            Real const ids{(1.0 - f_sub) * ids_inv + f_sub * ids_sub};
            return ids;
        }

        template <typename Mos>
        inline void
            bsim3v32_meyer_intrinsic_caps(Mos const& m, double vgs_s, double vds_s, double vbs_s, double vt, double& cgs, double& cgd, double& cgb) noexcept
        {
            bsim3v32_core_cache c{};
            (void)bsim3v32_ids_core(m, vgs_s, vds_s, vbs_s, vt, __builtin_addressof(c));

            double const cgg{c.cox * c.weff * c.leff};
            cgs = 0.0;
            cgd = 0.0;
            cgb = 0.0;
            if(cgg <= 0.0) { return; }

            if(c.vgsteff <= 0.0)
            {
                // Cutoff: channel absent -> gate couples mostly to bulk.
                cgb = cgg;
                return;
            }

            // Use Vdseff vs Vgsteff as a simple region indicator.
            if(c.vdseff < c.vgsteff)
            {
                // Linear
                cgs = (2.0 / 3.0) * cgg;
                cgd = (1.0 / 3.0) * cgg;
            }
            else
            {
                // Saturation
                cgs = (2.0 / 3.0) * cgg;
                cgd = 0.0;
            }
        }

        template <typename Real>
        struct bsim3v32_charge_vec4_t
        {
            Real q[4]{};  // [D,G,S,B] (Coulomb)
        };

        using bsim3v32_charge_vec4 = bsim3v32_charge_vec4_t<double>;

        template <bool is_pmos, typename Mos, typename Real>
        inline bsim3v32_charge_vec4_t<Real> bsim3v32_intrinsic_charges_capmod0_simple_s(Mos const& m, Real vgs_s, Real vds_s, Real vbs_s, double vt) noexcept
        {
            // Clean-room simplified charge-based intrinsic model:
            // - Uses Vgsteff/Vdseff from the DC core cache.
            // - Uses xpart selection (0/100, 50/50, 40/60) for Qinv partition.
            // - Approximates Qb using the k1 body-effect term.
            //
            // NOTE: This is a stepping stone towards the full BSIM3v3.2 C/V implementation.

            double const pol{is_pmos ? -1.0 : 1.0};

            details::bsim3v32_core_cache_t<Real> c{};
            (void)bsim3v32_ids_core(m, vgs_s, vds_s, vbs_s, vt, __builtin_addressof(c));

            Real const coxwl{c.cox * c.weff * c.leff};
            if(bsim3v32_value(coxwl) <= 0.0) { return {}; }

            // Vbseff and k1ox for depletion charge approximation.
            double const nch_eff_raw{details::bsim3v32_lw_scale(
                m.nch, m.lnch, m.wnch, m.pnch, bsim3v32_value(c.leff), bsim3v32_value(c.weff), m.lref, m.wref)};
            double const nch_eff{nch_eff_raw > 1.0 ? nch_eff_raw : (m.nch > 1.0 ? m.nch : 1e23)};
            double const phi0_eff_geom{details::bsim3v32_lw_scale(
                m.phi, m.lphi, m.wphi, m.pphi, bsim3v32_value(c.leff), bsim3v32_value(c.weff), m.lref, m.wref)};
            double const phi_s{bsim3v32_phi_temp(phi0_eff_geom, nch_eff, m.Temp, m.tnom)};
            double const tox{m.tox > 0.0 ? m.tox : 1e-8};
            double const toxm{m.toxm > 0.0 ? m.toxm : tox};
            double const tox_ratio{tox / toxm};

            double const gamma_eff_raw{details::bsim3v32_lw_scale(
                m.gamma, m.lgamma, m.wgamma, m.pgamma, bsim3v32_value(c.leff), bsim3v32_value(c.weff), m.lref, m.wref)};
            double const gamma_eff{gamma_eff_raw > 0.0 ? gamma_eff_raw : 0.0};
            double const k1_base{(m.k1 != 0.0) ? m.k1 : gamma_eff};
            double const k1_eff{details::bsim3v32_lw_scale(
                k1_base, m.lk1, m.wk1, m.pk1, bsim3v32_value(c.leff), bsim3v32_value(c.weff), m.lref, m.wref)};
            double const k1ox{k1_eff * tox_ratio};

            double const vbm{(m.vbm < 0.0) ? m.vbm : -3.0};
            double const delta1{(m.delta1 > 0.0) ? m.delta1 : 1e-3};
            Real const vbseff{bsim3v32_vbseff(vbs_s, vbm, delta1)};
            double const sqrt_phi{::std::sqrt(phi_s)};
            Real const sqrt_phi_vbs{bsim3v32_sqrt(bsim3v32_max(phi_s - vbseff, 1e-12))};

            Real const qb_n{coxwl * k1ox * (sqrt_phi_vbs - sqrt_phi)};  // bulk (depletion) charge (approx)

            // Inversion charge (clean-room, long-channel baseline).
            // - Linear:  Qinv ≈ -CoxWL*(Vgsteff - Abulk*Vdseff/2)
            // - Saturation: Qinv ≈ -(2/3)*CoxWL*Vgsteff
            double const keta_eff{details::bsim3v32_lw_scale(
                m.keta, m.lketa, m.wketa, m.pketa, bsim3v32_value(c.leff), bsim3v32_value(c.weff), m.lref, m.wref)};
            Real const abulk{1.0 + keta_eff * vbseff};
            bool const is_linear{bsim3v32_value(c.vdseff) < bsim3v32_value(c.vdsat)};
            Real qinv_n{};
            if(is_linear) { qinv_n = -coxwl * (c.vgsteff - abulk * c.vdseff / 2.0); }
            else
            {
                qinv_n = -(2.0 / 3.0) * coxwl * c.vgsteff;
            }

            // Channel inversion charge partition (clean-room, charge-conserving):
            // - Linear region: Ward–Dutton partition (bias-dependent).
            // - Saturation: xpart selects 0/100, 50/50, or 40/60 partition.
            Real qd_n{};
            Real qs_n{};
            if(is_linear)
            {
                // Ward–Dutton with a linear channel potential approximation.
                qd_n = -coxwl * (0.5 * c.vgsteff - (abulk * c.vdseff) / 3.0);
                qs_n = qinv_n - qd_n;
            }
            else
            {
                // xpart selection: 0.0 -> 0/100; 0.5 -> 50/50; 1.0 -> 40/60.
                double frac_d{};
                if(m.xpart < 0.25) { frac_d = 0.0; }
                else if(m.xpart < 0.75) { frac_d = 0.5; }
                else
                {
                    frac_d = 0.4;
                }

                qd_n = frac_d * qinv_n;
                qs_n = (1.0 - frac_d) * qinv_n;
            }

            Real const qg_n{-(qinv_n + qb_n)};
            Real qb_adj_n{qb_n};

            // Depletion/accumulation gate-bulk charge (clean-room simplified):
            // - Add an extra G-B term in depletion to avoid near-zero Cgb in cutoff when capMod!=0.
            // - Keep it smoothly disabled in strong inversion based on Vgs-Vth (signed coordinate).
            //
            // Approximate flatband as Vfb ≈ Vth0(T) - phi (BSIM3 uses phi as surface potential).
            double const vth0_t{bsim3v32_vth0_temp_mag(m.Vth0, m.Temp, m.tnom, m.kt1, m.kt2)};
            double const vfb{vth0_t - phi_s};
            Real const vgb{vgs_s - vbs_s};
            Real const x{vgb - vfb};
            Real const abs_x{bsim3v32_abs_smooth(x)};
            // min(x,0) and max(x,0) with smooth abs to keep derivatives continuous.
            Real const minx{0.5 * (x - abs_x)};
            Real const maxx{0.5 * (x + abs_x)};

            // Smooth cutoff factor based on Vgs-Vth (signed coordinate from DC cache).
            Real const vgst{vgs_s - c.vth};
            Real const abs_vgst{bsim3v32_abs_smooth(vgst)};
            Real const f_cut{0.5 * (1.0 - vgst / bsim3v32_max(abs_vgst, 1e-24))};

            Real const qacc_g{coxwl * minx};         // <= 0 in accumulation (NMOS)
            Real const qdep_g{coxwl * maxx * f_cut}; // >= 0 in depletion/cutoff; fades out in inversion

            qb_adj_n += -(qacc_g + qdep_g);
            Real const qg_adj{qg_n + qacc_g + qdep_g};

            bsim3v32_charge_vec4_t<Real> q{};
            q.q[0] = pol * qd_n;
            q.q[1] = pol * qg_adj;
            q.q[2] = pol * qs_n;
            q.q[3] = pol * qb_adj_n;
            return q;
        }

        template <bool is_pmos, typename Mos>
        inline bsim3v32_charge_vec4 bsim3v32_intrinsic_charges_capmod0_simple(Mos const& m, double vd, double vg, double vs, double vb) noexcept
        {
            double const sgn{is_pmos ? -1.0 : 1.0};
            double const vt{thermal_voltage(m.Temp)};
            double const vgs_s{sgn * (vg - vs)};
            double const vds_s{sgn * (vd - vs)};
            double const vbs_s{sgn * (vb - vs)};
            return bsim3v32_intrinsic_charges_capmod0_simple_s<is_pmos>(m, vgs_s, vds_s, vbs_s, vt);
        }

        template <bool is_pmos, typename Mos>
        inline void bsim3v32_cmatrix_capmod0_simple(Mos const& m, double vd, double vg, double vs, double vb, double cmat[4][4]) noexcept
        {
            // Dual-number Jacobian of terminal charges: C[i][j] = ∂Qi/∂Vj (F).
            // Order is [D,G,S,B].
            for(int i{}; i < 4; ++i)
            {
                for(int j{}; j < 4; ++j) { cmat[i][j] = 0.0; }
            }

            double const sgn{is_pmos ? -1.0 : 1.0};
            double const vt{thermal_voltage(m.Temp)};

            double const vgs_s{sgn * (vg - vs)};
            double const vds_s{sgn * (vd - vs)};
            double const vbs_s{sgn * (vb - vs)};

            auto const q =
                bsim3v32_intrinsic_charges_capmod0_simple_s<is_pmos>(m, bsim3v32_dual3_vgs(vgs_s), bsim3v32_dual3_vds(vds_s), bsim3v32_dual3_vbs(vbs_s), vt);

            for(int i{}; i < 4; ++i)
            {
                double const dqi_dvgs_s{q.q[i].dvgs};
                double const dqi_dvds_s{q.q[i].dvds};
                double const dqi_dvbs_s{q.q[i].dvbs};

                // Map signed small-signal variables (v*_s = sgn*(V* - Vs)) back to terminal voltages.
                cmat[i][0] = sgn * dqi_dvds_s;                         // dQi/dVd
                cmat[i][1] = sgn * dqi_dvgs_s;                         // dQi/dVg
                cmat[i][3] = sgn * dqi_dvbs_s;                         // dQi/dVb
                cmat[i][2] = -(cmat[i][0] + cmat[i][1] + cmat[i][3]);  // dQi/dVs (common-mode invariance)
            }
        }

        inline constexpr void
            stamp_cap_matrix_ac(::phy_engine::MNA::MNA& mna, ::phy_engine::model::node_t* const nodes[4], double const cmat[4][4], double omega) noexcept
        {
            if(nodes[0] == nullptr || nodes[1] == nullptr || nodes[2] == nullptr || nodes[3] == nullptr || omega == 0.0) [[unlikely]] { return; }
            for(int i{}; i < 4; ++i)
            {
                auto const ri = nodes[i]->node_index;
                if(ri == SIZE_MAX) [[unlikely]] { continue; }
                for(int j{}; j < 4; ++j)
                {
                    double const c = cmat[i][j];
                    if(c == 0.0) { continue; }
                    auto const cj = nodes[j]->node_index;
                    if(cj == SIZE_MAX) [[unlikely]] { continue; }
                    mna.G_ref(ri, cj) += ::std::complex<double>{0.0, omega * c};
                }
            }
        }

        inline constexpr void
            step_cap_matrix_tr(double hist[4], double prev_g[4][4], ::phy_engine::model::node_t* const nodes[4], double const cmat[4][4], double dt) noexcept
        {
            if(nodes[0] == nullptr || nodes[1] == nullptr || nodes[2] == nullptr || nodes[3] == nullptr || dt <= 0.0) [[unlikely]]
            {
                for(int i{}; i < 4; ++i)
                {
                    hist[i] = 0.0;
                    for(int j{}; j < 4; ++j) { prev_g[i][j] = 0.0; }
                }
                return;
            }

            double g_new[4][4]{};
            for(int i{}; i < 4; ++i)
            {
                for(int j{}; j < 4; ++j) { g_new[i][j] = 2.0 * cmat[i][j] / dt; }
            }

            double const v_prev[4]{
                nodes[0]->node_information.an.voltage.real(),
                nodes[1]->node_information.an.voltage.real(),
                nodes[2]->node_information.an.voltage.real(),
                nodes[3]->node_information.an.voltage.real(),
            };

            double hist_new[4]{};
            for(int i{}; i < 4; ++i)
            {
                double acc{};
                for(int j{}; j < 4; ++j) { acc += (g_new[i][j] + prev_g[i][j]) * v_prev[j]; }
                hist_new[i] = -acc - hist[i];
            }

            for(int i{}; i < 4; ++i)
            {
                hist[i] = hist_new[i];
                for(int j{}; j < 4; ++j) { prev_g[i][j] = g_new[i][j]; }
            }
        }

        inline constexpr void stamp_cap_matrix_tr(::phy_engine::MNA::MNA& mna,
                                                  ::phy_engine::model::node_t* const nodes[4],
                                                  double const prev_g[4][4],
                                                  double const hist[4]) noexcept
        {
            if(nodes[0] == nullptr || nodes[1] == nullptr || nodes[2] == nullptr || nodes[3] == nullptr) [[unlikely]] { return; }
            for(int i{}; i < 4; ++i)
            {
                auto const ri = nodes[i]->node_index;
                if(ri == SIZE_MAX) [[unlikely]] { continue; }
                for(int j{}; j < 4; ++j)
                {
                    double const g = prev_g[i][j];
                    if(g == 0.0) { continue; }
                    auto const cj = nodes[j]->node_index;
                    if(cj == SIZE_MAX) [[unlikely]] { continue; }
                    mna.G_ref(ri, cj) += g;
                }
            }
            for(int i{}; i < 4; ++i)
            {
                auto const ri = nodes[i]->node_index;
                if(ri == SIZE_MAX) [[unlikely]] { continue; }
                double const i_hist = hist[i];
                if(i_hist == 0.0) { continue; }
                mna.I_ref(ri) -= i_hist;
            }
        }
    }  // namespace details

    template <bool is_pmos>
    struct bsim3v32_mos
    {
        inline static constexpr ::fast_io::u8string_view model_name{is_pmos ? u8"BSIM3v3.2 PMOS" : u8"BSIM3v3.2 NMOS"};

        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::non_linear};
        inline static constexpr ::fast_io::u8string_view identification_name{is_pmos ? u8"BSIM3P" : u8"BSIM3N"};

        // Pins: D, G, S, B
        ::phy_engine::model::pin pins[4]{{{u8"D"}}, {{u8"G"}}, {{u8"S"}}, {{u8"B"}}};

        // Geometry (instance parameters)
        double W{1e-6};  // m
        double L{1e-6};  // m
        // Effective geometry corrections (BSIM3-style). Leff/Weff will clamp at >= 0.
        double dwc{0.0};     // m
        double dlc{0.0};     // m
        double m_mult{1.0};  // parallel multiplier (BSIM "m")
        double nf{1.0};      // number of fingers (clean-room: treated as an additional parallel multiplier)

        // Optional terminal series resistances (Ohm). When >0, use internal D'/S' nodes.
        double Rd{0.0};
        double Rs{0.0};
        // BSIM3 sheet resistance model (clean-room subset):
        // - rsh: sheet resistance (Ohm/square)
        // - nrd/nrs: drain/source diffusion squares
        // Effective series resistances are:
        //   Rd_total = Rd + rsh*nrd
        //   Rs_total = Rs + rsh*nrs
        double rsh{0.0};  // Ohm/square
        double nrd{0.0};  // squares
        double nrs{0.0};  // squares
        // Optional gate resistance (Ohm). When >0, use internal G' node for intrinsic gm/caps.
        double rg{0.0};

        // Charge/capacitance model controls (BSIM3 names).
        // Note: BSIM3/NGSPICE default capMod is typically 3. For now:
        // - capMod == 0: use a Meyer-style intrinsic C model
        // - capMod != 0: use a charge-based intrinsic C-matrix (clean-room simplified, WIP)
        double capMod{3.0};
        double xpart{0.0};  // 0.0=0/100, 0.5=50/50, 1.0=40/60

        // Core parameters (temporary approximation; will be replaced with full BSIM3v3.2 set)
        double Kp{50e-6};    // A/V^2 (interpreted as μ*Cox)
        double lambda{0.0};  // 1/V
        double Vth0{0.7};    // V (|Vth| for PMOS)
        double gamma{0.0};   // V^0.5 (body effect)
        double phi{0.7};     // V (surface potential, must be >0)

        // Reference geometry for L/W scaling (meters).
        double lref{1e-6};
        double wref{1e-6};
        // L/W scaling coefficients (clean-room subset).
        double lvth0{0.0};
        double wvth0{0.0};
        double pvth0{0.0};
        double lkp{0.0};
        double wkp{0.0};
        double pkp{0.0};
        double lu0{0.0};
        double wu0{0.0};
        double pu0{0.0};
        double lrdsw{0.0};
        double wrdsw{0.0};
        double prdsw{0.0};
        double lua{0.0};
        double wua{0.0};
        double pua{0.0};
        double lub{0.0};
        double wub{0.0};
        double pub{0.0};
        double luc{0.0};
        double wuc{0.0};
        double puc{0.0};
        double lvsat{0.0};
        double wvsat{0.0};
        double pvsat{0.0};
        double ldsub{0.0};
        double wdsub{0.0};
        double pdsub{0.0};
        double leta0{0.0};
        double weta0{0.0};
        double peta0{0.0};
        double letab{0.0};
        double wetab{0.0};
        double petab{0.0};
        double lpclm{0.0};
        double wpclm{0.0};
        double ppclm{0.0};
        double lpdiblc1{0.0};
        double wpdiblc1{0.0};
        double ppdiblc1{0.0};
        double lpdiblc2{0.0};
        double wpdiblc2{0.0};
        double ppdiblc2{0.0};
        double lpdiblcb{0.0};
        double wpdiblcb{0.0};
        double ppdiblcb{0.0};
        double ldrout{0.0};
        double wdrout{0.0};
        double pdrout{0.0};
        double lpvag{0.0};
        double wpvag{0.0};
        double ppvag{0.0};
        double lpscbe1{0.0};
        double wpscbe1{0.0};
        double ppscbe1{0.0};
        double lpscbe2{0.0};
        double wpscbe2{0.0};
        double ppscbe2{0.0};
        double ldvt0{0.0};
        double wdvt0{0.0};
        double pdvt0{0.0};
        double ldvt1{0.0};
        double wdvt1{0.0};
        double pdvt1{0.0};
        double ldvt2{0.0};
        double wdvt2{0.0};
        double pdvt2{0.0};
        double lnfactor{0.0};
        double wnfactor{0.0};
        double pnfactor{0.0};
        double lcit{0.0};
        double wcit{0.0};
        double pcit{0.0};
        double lketa{0.0};
        double wketa{0.0};
        double pketa{0.0};
        double lprwg{0.0};
        double wprwg{0.0};
        double pprwg{0.0};
        double lprwb{0.0};
        double wprwb{0.0};
        double pprwb{0.0};
        double lk1{0.0};
        double wk1{0.0};
        double pk1{0.0};
        double lk2{0.0};
        double wk2{0.0};
        double pk2{0.0};
        double lk3{0.0};
        double wk3{0.0};
        double pk3{0.0};
        double lk3b{0.0};
        double wk3b{0.0};
        double pk3b{0.0};
        double lw0{0.0};
        double ww0{0.0};
        double pw0{0.0};
        double lnlx{0.0};
        double wnlx{0.0};
        double pnlx{0.0};
        double lvoff{0.0};
        double wvoff{0.0};
        double pvoff{0.0};
        double lnch{0.0};
        double wnch{0.0};
        double pnch{0.0};
        double lgamma{0.0};
        double wgamma{0.0};
        double pgamma{0.0};
        double lphi{0.0};
        double wphi{0.0};
        double pphi{0.0};

        // BSIM3v3.x key parameters (subset; additional params can be added as needed).
        // Units follow the BSIM3 manuals (SI for geometry, V, and A; mobility in m^2/Vs).
        double tox{1e-8};    // m
        double toxm{1e-8};   // m (extraction oxide thickness)
        double nch{1.7e23};  // 1/m^3 (channel doping)
        double u0{0.0};      // m^2/Vs (if 0, derived from Kp/Cox)
        double ua{0.0};
        double ub{0.0};
        double uc{0.0};
        double vsat{8e4};  // m/s

        double k1{0.0};
        double k2{0.0};
        double k3{0.0};
        double k3b{0.0};
        double w0{0.0};
        double nlx{0.0};
        double vbm{-3.0};
        double delta1{1e-3};  // smoothing for Vbseff
        double vbi{0.0};      // V (if 0, derived from phi)

        double dvt0{0.0};
        double dvt1{0.0};
        double dvt2{0.0};
        double dsub{0.0};
        double eta0{0.0};
        double etab{0.0};

        double nfactor{0.0};
        double cit{0.0};
        double voff{0.0};

        // CLM / DIBL / SCBE
        double pclm{0.0};
        double pdiblc1{0.0};
        double pdiblc2{0.0};
        double pdiblcb{0.0};
        double drout{0.0};
        double pvag{0.0};
        double pscbe1{0.0};
        double pscbe2{0.0};
        double delta{1e-2};  // smoothing for Vdseff
        double rds{0.0};     // internal Rds (Ohm), used in Ids expression
        // BSIM3 Rds model (clean-room subset):
        // - rdsw: source/drain resistance per effective width (Ohm*m). Effective Rds adds rdsw/Weff.
        // - prwg/prwb: bias dependence vs Vgsteff and Vbseff.
        double rdsw{0.0};  // Ohm*m
        double prwg{0.0};  // 1/V
        double prwb{0.0};  // 1/V

        double keta{0.0};

        // Optional fixed capacitances (F) - placeholder for full charge-based C/V model
        double Cgs{0.0};
        double Cgd{0.0};
        double Cgb{0.0};

        // Gate overlap capacitances (BSIM-style).
        // - cgso/cgdo: F/m scaled by effective width (Weff)
        // - cgbo:      F/m scaled by effective length (Leff)
        double cgso{0.0};
        double cgdo{0.0};
        double cgbo{0.0};

        // Body diode current (placeholder; full BSIM3 junction model TBD)
        double diode_Is{1e-14};
        double diode_N{1.0};
        // Junction transit time (seconds): enables diffusion capacitance in the internal body diodes (tt * dId/dV).
        double tt{0.0};
        double Temp{27.0};  // Celsius

        // Temperature scaling (clean-room subset; aligns with common BSIM3 usage).
        // - tnom: nominal model temperature (Celsius)
        // - ute: mobility temperature exponent
        // - kt1/kt2: Vth temperature coefficients
        // - at: vsat temperature coefficient
        double tnom{27.0};  // Celsius
        double ute{0.0};
        double kt1{0.0};
        double kt2{0.0};
        double at{0.0};

        // Optional diffusion junction current density parameters (clean-room subset):
        // Is_d = m*(js*Area_d + jsw*Perim_d), Is_s = m*(js*Area_s + jsw*Perim_s).
        // When js==0 and jsw==0, fall back to diode_Is/diode_N with Area=m.
        double js{0.0};    // A/m^2
        double jsw{0.0};   // A/m
        double jswg{0.0};  // A/m (gate-edge sidewall current density, scaled by Weff)

        // Optional S/D-B depletion capacitance (simple SPICE-style junction C model).
        // Units:
        // - Cj*Area [F] where Cj is F/m^2 and Area is m^2
        // - Cjsw*Perimeter [F] where Cjsw is F/m and Perimeter is m
        double drainArea{0.0};        // m^2
        double sourceArea{0.0};       // m^2
        double drainPerimeter{0.0};   // m
        double sourcePerimeter{0.0};  // m

        double cj{0.0};     // F/m^2 (bottom junction cap density)
        double cjsw{0.0};   // F/m   (sidewall junction cap density)
        double cjswg{0.0};  // F/m  (gate-edge sidewall junction cap density)
        double pb{1.0};     // V (junction potential)
        // Sidewall junction potential (V). If <=0, fall back to pb.
        double pbsw{0.0};
        // Gate-edge sidewall junction potential (V). If <=0, fall back to pbsw/pb.
        double pbswg{0.0};
        // Junction depletion temperature coefficients (clean-room SPICE-style subset).
        // These scale around TNOM:
        //   cj(T)   = cj(Tnom)   * (1 + tcj*(T-Tnom))
        //   cjsw(T) = cjsw(Tnom) * (1 + tcjsw*(T-Tnom))
        //   pb(T)   = pb(Tnom)   * (1 + tpb*(T-Tnom))
        //   pbsw(T) = pbsw(Tnom) * (1 + tpbsw*(T-Tnom))
        double tcj{0.0};
        double tcjsw{0.0};
        double tpb{0.0};
        double tpbsw{0.0};
        double tcjswg{0.0};
        double tpbswg{0.0};
        double mj{0.5};  // grading coefficient
        double mjsw{0.33};
        double mjswg{0.33};
        double fc{0.5};  // forward-bias coefficient

        // Temperature scaling for junction saturation currents (SPICE-style).
        double xti{3.0};  // temperature exponent
        double eg{1.11};  // eV

        // Linearization state (Ids defined positive from D->S)
        double gm{};
        double gds{};
        double gmb{};
        double Ieq{};
        bool mode_swapped{};  // true if channel "source/drain" are swapped in the last DC linearization
        double vgs_s_last{};
        double vds_s_last{};
        double vbs_s_last{};
        bool dc_bias_valid{};
        bool dc_bias_swapped{};  // last bias values correspond to swapped mode

        // Internal diode models (D-B and S-B)
        PN_junction diode_db{};
        PN_junction diode_sb{};

        // TR capacitor state
        details::bsim3v32_cap_state cgs_state{};
        details::bsim3v32_cap_state cgd_state{};
        details::bsim3v32_cap_state cgb_state{};
        details::bsim3v32_cap_state cgs_int_state{};
        details::bsim3v32_cap_state cgd_int_state{};
        details::bsim3v32_cap_state cgb_int_state{};
        bool cap_mode_swapped{};  // frozen channel orientation for TR companion caps
        details::bsim3v32_cap_state capbd_state{};
        details::bsim3v32_cap_state capbs_state{};

        // AC small-signal capacitances captured at the (real) operating point (ACOP / DC / OP).
        double cgs_intr_ac{};
        double cgd_intr_ac{};
        double cgb_intr_ac{};
        double cbd_ac{};
        double cbs_ac{};

        // Charge-based intrinsic C-matrix for AC (captured at OP) and TR (trapezoidal companion).
        // Order is [D,G,S,B] where D/S follow the current channel mode selection.
        double cmat_ac[4][4]{};
        double cmat_tr_prev_g[4][4]{};
        double cmat_tr_hist[4]{};

        // Internal nodes (only used when Rd/Rs > 0).
        ::phy_engine::model::node_t internal_nodes[3]{};  // packed prefix of used nodes
        ::phy_engine::model::node_t* nd_int{};            // D'
        ::phy_engine::model::node_t* ns_int{};            // S'
        ::phy_engine::model::node_t* ng_int{};            // G'
    };

    using bsim3v32_nmos = bsim3v32_mos<false>;
    using bsim3v32_pmos = bsim3v32_mos<true>;

    static_assert(::phy_engine::model::model<bsim3v32_nmos>);
    static_assert(::phy_engine::model::model<bsim3v32_pmos>);

    template <bool is_pmos>
    inline constexpr bool set_attribute_define(::phy_engine::model::model_reserve_type_t<bsim3v32_mos<is_pmos>>,
                                               bsim3v32_mos<is_pmos>& m,
                                               ::std::size_t idx,
                                               ::phy_engine::model::variant vi) noexcept
    {
        if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
        switch(idx)
        {
            case 0: m.W = vi.d; return true;
            case 1: m.L = vi.d; return true;
            case 13: m.m_mult = vi.d; return true;
            case 100: m.nf = vi.d; return true;
            case 102: m.lref = vi.d; return true;
            case 103: m.wref = vi.d; return true;
            case 104: m.lvth0 = vi.d; return true;
            case 105: m.wvth0 = vi.d; return true;
            case 106: m.pvth0 = vi.d; return true;
            case 107: m.lkp = vi.d; return true;
            case 108: m.wkp = vi.d; return true;
            case 109: m.pkp = vi.d; return true;
            case 110: m.lu0 = vi.d; return true;
            case 111: m.wu0 = vi.d; return true;
            case 112: m.pu0 = vi.d; return true;
            case 113: m.lrdsw = vi.d; return true;
            case 114: m.wrdsw = vi.d; return true;
            case 115: m.prdsw = vi.d; return true;
            case 116: m.lua = vi.d; return true;
            case 117: m.wua = vi.d; return true;
            case 118: m.pua = vi.d; return true;
            case 119: m.lub = vi.d; return true;
            case 120: m.wub = vi.d; return true;
            case 121: m.pub = vi.d; return true;
            case 122: m.luc = vi.d; return true;
            case 123: m.wuc = vi.d; return true;
            case 124: m.puc = vi.d; return true;
            case 125: m.lvsat = vi.d; return true;
            case 126: m.wvsat = vi.d; return true;
            case 127: m.pvsat = vi.d; return true;
            case 128: m.ldsub = vi.d; return true;
            case 129: m.wdsub = vi.d; return true;
            case 130: m.pdsub = vi.d; return true;
            case 131: m.leta0 = vi.d; return true;
            case 132: m.weta0 = vi.d; return true;
            case 133: m.peta0 = vi.d; return true;
            case 134: m.letab = vi.d; return true;
            case 135: m.wetab = vi.d; return true;
            case 136: m.petab = vi.d; return true;
            case 137: m.lpclm = vi.d; return true;
            case 138: m.wpclm = vi.d; return true;
            case 139: m.ppclm = vi.d; return true;
            case 140: m.lpdiblc1 = vi.d; return true;
            case 141: m.wpdiblc1 = vi.d; return true;
            case 142: m.ppdiblc1 = vi.d; return true;
            case 143: m.lpdiblc2 = vi.d; return true;
            case 144: m.wpdiblc2 = vi.d; return true;
            case 145: m.ppdiblc2 = vi.d; return true;
            case 146: m.lpdiblcb = vi.d; return true;
            case 147: m.wpdiblcb = vi.d; return true;
            case 148: m.ppdiblcb = vi.d; return true;
            case 149: m.ldrout = vi.d; return true;
            case 150: m.wdrout = vi.d; return true;
            case 151: m.pdrout = vi.d; return true;
            case 152: m.lpvag = vi.d; return true;
            case 153: m.wpvag = vi.d; return true;
            case 154: m.ppvag = vi.d; return true;
            case 155: m.lpscbe1 = vi.d; return true;
            case 156: m.wpscbe1 = vi.d; return true;
            case 157: m.ppscbe1 = vi.d; return true;
            case 158: m.lpscbe2 = vi.d; return true;
            case 159: m.wpscbe2 = vi.d; return true;
            case 160: m.ppscbe2 = vi.d; return true;
            case 161: m.ldvt0 = vi.d; return true;
            case 162: m.wdvt0 = vi.d; return true;
            case 163: m.pdvt0 = vi.d; return true;
            case 164: m.ldvt1 = vi.d; return true;
            case 165: m.wdvt1 = vi.d; return true;
            case 166: m.pdvt1 = vi.d; return true;
            case 167: m.ldvt2 = vi.d; return true;
            case 168: m.wdvt2 = vi.d; return true;
            case 169: m.pdvt2 = vi.d; return true;
            case 170: m.lnfactor = vi.d; return true;
            case 171: m.wnfactor = vi.d; return true;
            case 172: m.pnfactor = vi.d; return true;
            case 173: m.lcit = vi.d; return true;
            case 174: m.wcit = vi.d; return true;
            case 175: m.pcit = vi.d; return true;
            case 176: m.lketa = vi.d; return true;
            case 177: m.wketa = vi.d; return true;
            case 178: m.pketa = vi.d; return true;
            case 185: m.lprwg = vi.d; return true;
            case 186: m.wprwg = vi.d; return true;
            case 187: m.pprwg = vi.d; return true;
            case 188: m.lprwb = vi.d; return true;
            case 189: m.wprwb = vi.d; return true;
            case 190: m.pprwb = vi.d; return true;
            case 191: m.lk1 = vi.d; return true;
            case 192: m.wk1 = vi.d; return true;
            case 193: m.pk1 = vi.d; return true;
            case 194: m.lk2 = vi.d; return true;
            case 195: m.wk2 = vi.d; return true;
            case 196: m.pk2 = vi.d; return true;
            case 197: m.lk3 = vi.d; return true;
            case 198: m.wk3 = vi.d; return true;
            case 199: m.pk3 = vi.d; return true;
            case 200: m.lk3b = vi.d; return true;
            case 201: m.wk3b = vi.d; return true;
            case 202: m.pk3b = vi.d; return true;
            case 203: m.lw0 = vi.d; return true;
            case 204: m.ww0 = vi.d; return true;
            case 205: m.pw0 = vi.d; return true;
            case 206: m.lnlx = vi.d; return true;
            case 207: m.wnlx = vi.d; return true;
            case 208: m.pnlx = vi.d; return true;
            case 209: m.voff = vi.d; return true;
            case 210: m.lvoff = vi.d; return true;
            case 211: m.wvoff = vi.d; return true;
            case 212: m.pvoff = vi.d; return true;
            case 213: m.lnch = vi.d; return true;
            case 214: m.wnch = vi.d; return true;
            case 215: m.pnch = vi.d; return true;
            case 216: m.lgamma = vi.d; return true;
            case 217: m.wgamma = vi.d; return true;
            case 218: m.pgamma = vi.d; return true;
            case 219: m.lphi = vi.d; return true;
            case 220: m.wphi = vi.d; return true;
            case 221: m.pphi = vi.d; return true;
            case 14: m.Rd = vi.d; return true;
            case 15: m.Rs = vi.d; return true;
            case 97: m.rsh = vi.d; return true;
            case 98: m.nrd = vi.d; return true;
            case 99: m.nrs = vi.d; return true;
            case 78: m.rg = vi.d; return true;
            case 2: m.Kp = vi.d; return true;
            case 3: m.lambda = vi.d; return true;
            case 4: m.Vth0 = vi.d; return true;
            case 5: m.gamma = vi.d; return true;
            case 6: m.phi = vi.d; return true;
            case 7: m.Cgs = vi.d; return true;
            case 8: m.Cgd = vi.d; return true;
            case 9: m.Cgb = vi.d; return true;
            case 66: m.cgso = vi.d; return true;
            case 67: m.cgdo = vi.d; return true;
            case 68: m.cgbo = vi.d; return true;
            case 10: m.diode_Is = vi.d; return true;
            case 11: m.diode_N = vi.d; return true;
            case 12: m.Temp = vi.d; return true;
            case 83: m.tt = vi.d; return true;
            case 71: m.tnom = vi.d; return true;
            case 72: m.ute = vi.d; return true;
            case 73: m.kt1 = vi.d; return true;
            case 74: m.kt2 = vi.d; return true;
            case 75: m.at = vi.d; return true;
            case 69: m.js = vi.d; return true;
            case 70: m.jsw = vi.d; return true;
            case 101: m.jswg = vi.d; return true;
            case 76: m.xti = vi.d; return true;
            case 77: m.eg = vi.d; return true;
            case 16: m.drainArea = vi.d; return true;
            case 17: m.sourceArea = vi.d; return true;
            case 18: m.drainPerimeter = vi.d; return true;
            case 19: m.sourcePerimeter = vi.d; return true;
            // SPICE-style instance parameter aliases.
            case 79: m.drainArea = vi.d; return true;        // ad
            case 80: m.sourceArea = vi.d; return true;       // as
            case 81: m.drainPerimeter = vi.d; return true;   // pd
            case 82: m.sourcePerimeter = vi.d; return true;  // ps
            case 20: m.cj = vi.d; return true;
            case 21: m.cjsw = vi.d; return true;
            case 89: m.cjswg = vi.d; return true;
            case 22: m.pb = vi.d; return true;
            case 84: m.pbsw = vi.d; return true;
            case 90: m.pbswg = vi.d; return true;
            case 85: m.tcj = vi.d; return true;
            case 86: m.tcjsw = vi.d; return true;
            case 87: m.tpb = vi.d; return true;
            case 88: m.tpbsw = vi.d; return true;
            case 91: m.tcjswg = vi.d; return true;
            case 92: m.tpbswg = vi.d; return true;
            case 23: m.mj = vi.d; return true;
            case 24: m.mjsw = vi.d; return true;
            case 93: m.mjswg = vi.d; return true;
            case 25: m.fc = vi.d; return true;
            case 26: m.tox = vi.d; return true;
            case 27: m.toxm = vi.d; return true;
            case 28: m.nch = vi.d; return true;
            case 29: m.u0 = vi.d; return true;
            case 30: m.ua = vi.d; return true;
            case 31: m.ub = vi.d; return true;
            case 32: m.uc = vi.d; return true;
            case 33: m.vsat = vi.d; return true;
            case 34: m.k1 = vi.d; return true;
            case 35: m.k2 = vi.d; return true;
            case 36: m.k3 = vi.d; return true;
            case 37: m.k3b = vi.d; return true;
            case 38: m.w0 = vi.d; return true;
            case 39: m.nlx = vi.d; return true;
            case 40: m.vbm = vi.d; return true;
            case 41: m.delta1 = vi.d; return true;
            case 42: m.vbi = vi.d; return true;
            case 43: m.dvt0 = vi.d; return true;
            case 44: m.dvt1 = vi.d; return true;
            case 45: m.dvt2 = vi.d; return true;
            case 46: m.dsub = vi.d; return true;
            case 47: m.eta0 = vi.d; return true;
            case 48: m.etab = vi.d; return true;
            case 49: m.nfactor = vi.d; return true;
            case 50: m.cit = vi.d; return true;
            case 51: m.pclm = vi.d; return true;
            case 52: m.pdiblc1 = vi.d; return true;
            case 53: m.pdiblc2 = vi.d; return true;
            case 54: m.pdiblcb = vi.d; return true;
            case 55: m.drout = vi.d; return true;
            case 56: m.pvag = vi.d; return true;
            case 57: m.pscbe1 = vi.d; return true;
            case 58: m.pscbe2 = vi.d; return true;
            case 59: m.delta = vi.d; return true;
            case 60: m.rds = vi.d; return true;
            case 94: m.rdsw = vi.d; return true;
            case 95: m.prwg = vi.d; return true;
            case 96: m.prwb = vi.d; return true;
            case 61: m.keta = vi.d; return true;
            case 62: m.capMod = vi.d; return true;
            case 63: m.xpart = vi.d; return true;
            case 64: m.dwc = vi.d; return true;
            case 65: m.dlc = vi.d; return true;
            default: return false;
        }
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<bsim3v32_nmos>);
    static_assert(::phy_engine::model::defines::has_set_attribute<bsim3v32_pmos>);

    template <bool is_pmos>
    inline constexpr ::phy_engine::model::variant
        get_attribute_define(::phy_engine::model::model_reserve_type_t<bsim3v32_mos<is_pmos>>, bsim3v32_mos<is_pmos> const& m, ::std::size_t idx) noexcept
    {
        switch(idx)
        {
            case 0: return {.d{m.W}, .type{::phy_engine::model::variant_type::d}};
            case 1: return {.d{m.L}, .type{::phy_engine::model::variant_type::d}};
            case 13: return {.d{m.m_mult}, .type{::phy_engine::model::variant_type::d}};
            case 100: return {.d{m.nf}, .type{::phy_engine::model::variant_type::d}};
            case 102: return {.d{m.lref}, .type{::phy_engine::model::variant_type::d}};
            case 103: return {.d{m.wref}, .type{::phy_engine::model::variant_type::d}};
            case 104: return {.d{m.lvth0}, .type{::phy_engine::model::variant_type::d}};
            case 105: return {.d{m.wvth0}, .type{::phy_engine::model::variant_type::d}};
            case 106: return {.d{m.pvth0}, .type{::phy_engine::model::variant_type::d}};
            case 107: return {.d{m.lkp}, .type{::phy_engine::model::variant_type::d}};
            case 108: return {.d{m.wkp}, .type{::phy_engine::model::variant_type::d}};
            case 109: return {.d{m.pkp}, .type{::phy_engine::model::variant_type::d}};
            case 110: return {.d{m.lu0}, .type{::phy_engine::model::variant_type::d}};
            case 111: return {.d{m.wu0}, .type{::phy_engine::model::variant_type::d}};
            case 112: return {.d{m.pu0}, .type{::phy_engine::model::variant_type::d}};
            case 113: return {.d{m.lrdsw}, .type{::phy_engine::model::variant_type::d}};
            case 114: return {.d{m.wrdsw}, .type{::phy_engine::model::variant_type::d}};
            case 115: return {.d{m.prdsw}, .type{::phy_engine::model::variant_type::d}};
            case 116: return {.d{m.lua}, .type{::phy_engine::model::variant_type::d}};
            case 117: return {.d{m.wua}, .type{::phy_engine::model::variant_type::d}};
            case 118: return {.d{m.pua}, .type{::phy_engine::model::variant_type::d}};
            case 119: return {.d{m.lub}, .type{::phy_engine::model::variant_type::d}};
            case 120: return {.d{m.wub}, .type{::phy_engine::model::variant_type::d}};
            case 121: return {.d{m.pub}, .type{::phy_engine::model::variant_type::d}};
            case 122: return {.d{m.luc}, .type{::phy_engine::model::variant_type::d}};
            case 123: return {.d{m.wuc}, .type{::phy_engine::model::variant_type::d}};
            case 124: return {.d{m.puc}, .type{::phy_engine::model::variant_type::d}};
            case 125: return {.d{m.lvsat}, .type{::phy_engine::model::variant_type::d}};
            case 126: return {.d{m.wvsat}, .type{::phy_engine::model::variant_type::d}};
            case 127: return {.d{m.pvsat}, .type{::phy_engine::model::variant_type::d}};
            case 128: return {.d{m.ldsub}, .type{::phy_engine::model::variant_type::d}};
            case 129: return {.d{m.wdsub}, .type{::phy_engine::model::variant_type::d}};
            case 130: return {.d{m.pdsub}, .type{::phy_engine::model::variant_type::d}};
            case 131: return {.d{m.leta0}, .type{::phy_engine::model::variant_type::d}};
            case 132: return {.d{m.weta0}, .type{::phy_engine::model::variant_type::d}};
            case 133: return {.d{m.peta0}, .type{::phy_engine::model::variant_type::d}};
            case 134: return {.d{m.letab}, .type{::phy_engine::model::variant_type::d}};
            case 135: return {.d{m.wetab}, .type{::phy_engine::model::variant_type::d}};
            case 136: return {.d{m.petab}, .type{::phy_engine::model::variant_type::d}};
            case 137: return {.d{m.lpclm}, .type{::phy_engine::model::variant_type::d}};
            case 138: return {.d{m.wpclm}, .type{::phy_engine::model::variant_type::d}};
            case 139: return {.d{m.ppclm}, .type{::phy_engine::model::variant_type::d}};
            case 140: return {.d{m.lpdiblc1}, .type{::phy_engine::model::variant_type::d}};
            case 141: return {.d{m.wpdiblc1}, .type{::phy_engine::model::variant_type::d}};
            case 142: return {.d{m.ppdiblc1}, .type{::phy_engine::model::variant_type::d}};
            case 143: return {.d{m.lpdiblc2}, .type{::phy_engine::model::variant_type::d}};
            case 144: return {.d{m.wpdiblc2}, .type{::phy_engine::model::variant_type::d}};
            case 145: return {.d{m.ppdiblc2}, .type{::phy_engine::model::variant_type::d}};
            case 146: return {.d{m.lpdiblcb}, .type{::phy_engine::model::variant_type::d}};
            case 147: return {.d{m.wpdiblcb}, .type{::phy_engine::model::variant_type::d}};
            case 148: return {.d{m.ppdiblcb}, .type{::phy_engine::model::variant_type::d}};
            case 149: return {.d{m.ldrout}, .type{::phy_engine::model::variant_type::d}};
            case 150: return {.d{m.wdrout}, .type{::phy_engine::model::variant_type::d}};
            case 151: return {.d{m.pdrout}, .type{::phy_engine::model::variant_type::d}};
            case 152: return {.d{m.lpvag}, .type{::phy_engine::model::variant_type::d}};
            case 153: return {.d{m.wpvag}, .type{::phy_engine::model::variant_type::d}};
            case 154: return {.d{m.ppvag}, .type{::phy_engine::model::variant_type::d}};
            case 155: return {.d{m.lpscbe1}, .type{::phy_engine::model::variant_type::d}};
            case 156: return {.d{m.wpscbe1}, .type{::phy_engine::model::variant_type::d}};
            case 157: return {.d{m.ppscbe1}, .type{::phy_engine::model::variant_type::d}};
            case 158: return {.d{m.lpscbe2}, .type{::phy_engine::model::variant_type::d}};
            case 159: return {.d{m.wpscbe2}, .type{::phy_engine::model::variant_type::d}};
            case 160: return {.d{m.ppscbe2}, .type{::phy_engine::model::variant_type::d}};
            case 161: return {.d{m.ldvt0}, .type{::phy_engine::model::variant_type::d}};
            case 162: return {.d{m.wdvt0}, .type{::phy_engine::model::variant_type::d}};
            case 163: return {.d{m.pdvt0}, .type{::phy_engine::model::variant_type::d}};
            case 164: return {.d{m.ldvt1}, .type{::phy_engine::model::variant_type::d}};
            case 165: return {.d{m.wdvt1}, .type{::phy_engine::model::variant_type::d}};
            case 166: return {.d{m.pdvt1}, .type{::phy_engine::model::variant_type::d}};
            case 167: return {.d{m.ldvt2}, .type{::phy_engine::model::variant_type::d}};
            case 168: return {.d{m.wdvt2}, .type{::phy_engine::model::variant_type::d}};
            case 169: return {.d{m.pdvt2}, .type{::phy_engine::model::variant_type::d}};
            case 170: return {.d{m.lnfactor}, .type{::phy_engine::model::variant_type::d}};
            case 171: return {.d{m.wnfactor}, .type{::phy_engine::model::variant_type::d}};
            case 172: return {.d{m.pnfactor}, .type{::phy_engine::model::variant_type::d}};
            case 173: return {.d{m.lcit}, .type{::phy_engine::model::variant_type::d}};
            case 174: return {.d{m.wcit}, .type{::phy_engine::model::variant_type::d}};
            case 175: return {.d{m.pcit}, .type{::phy_engine::model::variant_type::d}};
            case 176: return {.d{m.lketa}, .type{::phy_engine::model::variant_type::d}};
            case 177: return {.d{m.wketa}, .type{::phy_engine::model::variant_type::d}};
            case 178: return {.d{m.pketa}, .type{::phy_engine::model::variant_type::d}};
            case 185: return {.d{m.lprwg}, .type{::phy_engine::model::variant_type::d}};
            case 186: return {.d{m.wprwg}, .type{::phy_engine::model::variant_type::d}};
            case 187: return {.d{m.pprwg}, .type{::phy_engine::model::variant_type::d}};
            case 188: return {.d{m.lprwb}, .type{::phy_engine::model::variant_type::d}};
            case 189: return {.d{m.wprwb}, .type{::phy_engine::model::variant_type::d}};
            case 190: return {.d{m.pprwb}, .type{::phy_engine::model::variant_type::d}};
            case 191: return {.d{m.lk1}, .type{::phy_engine::model::variant_type::d}};
            case 192: return {.d{m.wk1}, .type{::phy_engine::model::variant_type::d}};
            case 193: return {.d{m.pk1}, .type{::phy_engine::model::variant_type::d}};
            case 194: return {.d{m.lk2}, .type{::phy_engine::model::variant_type::d}};
            case 195: return {.d{m.wk2}, .type{::phy_engine::model::variant_type::d}};
            case 196: return {.d{m.pk2}, .type{::phy_engine::model::variant_type::d}};
            case 197: return {.d{m.lk3}, .type{::phy_engine::model::variant_type::d}};
            case 198: return {.d{m.wk3}, .type{::phy_engine::model::variant_type::d}};
            case 199: return {.d{m.pk3}, .type{::phy_engine::model::variant_type::d}};
            case 200: return {.d{m.lk3b}, .type{::phy_engine::model::variant_type::d}};
            case 201: return {.d{m.wk3b}, .type{::phy_engine::model::variant_type::d}};
            case 202: return {.d{m.pk3b}, .type{::phy_engine::model::variant_type::d}};
            case 203: return {.d{m.lw0}, .type{::phy_engine::model::variant_type::d}};
            case 204: return {.d{m.ww0}, .type{::phy_engine::model::variant_type::d}};
            case 205: return {.d{m.pw0}, .type{::phy_engine::model::variant_type::d}};
            case 206: return {.d{m.lnlx}, .type{::phy_engine::model::variant_type::d}};
            case 207: return {.d{m.wnlx}, .type{::phy_engine::model::variant_type::d}};
            case 208: return {.d{m.pnlx}, .type{::phy_engine::model::variant_type::d}};
            case 209: return {.d{m.voff}, .type{::phy_engine::model::variant_type::d}};
            case 210: return {.d{m.lvoff}, .type{::phy_engine::model::variant_type::d}};
            case 211: return {.d{m.wvoff}, .type{::phy_engine::model::variant_type::d}};
            case 212: return {.d{m.pvoff}, .type{::phy_engine::model::variant_type::d}};
            case 213: return {.d{m.lnch}, .type{::phy_engine::model::variant_type::d}};
            case 214: return {.d{m.wnch}, .type{::phy_engine::model::variant_type::d}};
            case 215: return {.d{m.pnch}, .type{::phy_engine::model::variant_type::d}};
            case 216: return {.d{m.lgamma}, .type{::phy_engine::model::variant_type::d}};
            case 217: return {.d{m.wgamma}, .type{::phy_engine::model::variant_type::d}};
            case 218: return {.d{m.pgamma}, .type{::phy_engine::model::variant_type::d}};
            case 219: return {.d{m.lphi}, .type{::phy_engine::model::variant_type::d}};
            case 220: return {.d{m.wphi}, .type{::phy_engine::model::variant_type::d}};
            case 221: return {.d{m.pphi}, .type{::phy_engine::model::variant_type::d}};
            case 14: return {.d{m.Rd}, .type{::phy_engine::model::variant_type::d}};
            case 15: return {.d{m.Rs}, .type{::phy_engine::model::variant_type::d}};
            case 97: return {.d{m.rsh}, .type{::phy_engine::model::variant_type::d}};
            case 98: return {.d{m.nrd}, .type{::phy_engine::model::variant_type::d}};
            case 99: return {.d{m.nrs}, .type{::phy_engine::model::variant_type::d}};
            case 78: return {.d{m.rg}, .type{::phy_engine::model::variant_type::d}};
            case 2: return {.d{m.Kp}, .type{::phy_engine::model::variant_type::d}};
            case 3: return {.d{m.lambda}, .type{::phy_engine::model::variant_type::d}};
            case 4: return {.d{m.Vth0}, .type{::phy_engine::model::variant_type::d}};
            case 5: return {.d{m.gamma}, .type{::phy_engine::model::variant_type::d}};
            case 6: return {.d{m.phi}, .type{::phy_engine::model::variant_type::d}};
            case 7: return {.d{m.Cgs}, .type{::phy_engine::model::variant_type::d}};
            case 8: return {.d{m.Cgd}, .type{::phy_engine::model::variant_type::d}};
            case 9: return {.d{m.Cgb}, .type{::phy_engine::model::variant_type::d}};
            case 66: return {.d{m.cgso}, .type{::phy_engine::model::variant_type::d}};
            case 67: return {.d{m.cgdo}, .type{::phy_engine::model::variant_type::d}};
            case 68: return {.d{m.cgbo}, .type{::phy_engine::model::variant_type::d}};
            case 10: return {.d{m.diode_Is}, .type{::phy_engine::model::variant_type::d}};
            case 11: return {.d{m.diode_N}, .type{::phy_engine::model::variant_type::d}};
            case 12: return {.d{m.Temp}, .type{::phy_engine::model::variant_type::d}};
            case 83: return {.d{m.tt}, .type{::phy_engine::model::variant_type::d}};
            case 71: return {.d{m.tnom}, .type{::phy_engine::model::variant_type::d}};
            case 72: return {.d{m.ute}, .type{::phy_engine::model::variant_type::d}};
            case 73: return {.d{m.kt1}, .type{::phy_engine::model::variant_type::d}};
            case 74: return {.d{m.kt2}, .type{::phy_engine::model::variant_type::d}};
            case 75: return {.d{m.at}, .type{::phy_engine::model::variant_type::d}};
            case 69: return {.d{m.js}, .type{::phy_engine::model::variant_type::d}};
            case 70: return {.d{m.jsw}, .type{::phy_engine::model::variant_type::d}};
            case 101: return {.d{m.jswg}, .type{::phy_engine::model::variant_type::d}};
            case 76: return {.d{m.xti}, .type{::phy_engine::model::variant_type::d}};
            case 77: return {.d{m.eg}, .type{::phy_engine::model::variant_type::d}};
            case 16: return {.d{m.drainArea}, .type{::phy_engine::model::variant_type::d}};
            case 17: return {.d{m.sourceArea}, .type{::phy_engine::model::variant_type::d}};
            case 18: return {.d{m.drainPerimeter}, .type{::phy_engine::model::variant_type::d}};
            case 19: return {.d{m.sourcePerimeter}, .type{::phy_engine::model::variant_type::d}};
            case 79: return {.d{m.drainArea}, .type{::phy_engine::model::variant_type::d}};        // ad
            case 80: return {.d{m.sourceArea}, .type{::phy_engine::model::variant_type::d}};       // as
            case 81: return {.d{m.drainPerimeter}, .type{::phy_engine::model::variant_type::d}};   // pd
            case 82: return {.d{m.sourcePerimeter}, .type{::phy_engine::model::variant_type::d}};  // ps
            case 20: return {.d{m.cj}, .type{::phy_engine::model::variant_type::d}};
            case 21: return {.d{m.cjsw}, .type{::phy_engine::model::variant_type::d}};
            case 89: return {.d{m.cjswg}, .type{::phy_engine::model::variant_type::d}};
            case 22: return {.d{m.pb}, .type{::phy_engine::model::variant_type::d}};
            case 84: return {.d{m.pbsw}, .type{::phy_engine::model::variant_type::d}};
            case 90: return {.d{m.pbswg}, .type{::phy_engine::model::variant_type::d}};
            case 85: return {.d{m.tcj}, .type{::phy_engine::model::variant_type::d}};
            case 86: return {.d{m.tcjsw}, .type{::phy_engine::model::variant_type::d}};
            case 87: return {.d{m.tpb}, .type{::phy_engine::model::variant_type::d}};
            case 88: return {.d{m.tpbsw}, .type{::phy_engine::model::variant_type::d}};
            case 91: return {.d{m.tcjswg}, .type{::phy_engine::model::variant_type::d}};
            case 92: return {.d{m.tpbswg}, .type{::phy_engine::model::variant_type::d}};
            case 23: return {.d{m.mj}, .type{::phy_engine::model::variant_type::d}};
            case 24: return {.d{m.mjsw}, .type{::phy_engine::model::variant_type::d}};
            case 93: return {.d{m.mjswg}, .type{::phy_engine::model::variant_type::d}};
            case 25: return {.d{m.fc}, .type{::phy_engine::model::variant_type::d}};
            case 26: return {.d{m.tox}, .type{::phy_engine::model::variant_type::d}};
            case 27: return {.d{m.toxm}, .type{::phy_engine::model::variant_type::d}};
            case 28: return {.d{m.nch}, .type{::phy_engine::model::variant_type::d}};
            case 29: return {.d{m.u0}, .type{::phy_engine::model::variant_type::d}};
            case 30: return {.d{m.ua}, .type{::phy_engine::model::variant_type::d}};
            case 31: return {.d{m.ub}, .type{::phy_engine::model::variant_type::d}};
            case 32: return {.d{m.uc}, .type{::phy_engine::model::variant_type::d}};
            case 33: return {.d{m.vsat}, .type{::phy_engine::model::variant_type::d}};
            case 34: return {.d{m.k1}, .type{::phy_engine::model::variant_type::d}};
            case 35: return {.d{m.k2}, .type{::phy_engine::model::variant_type::d}};
            case 36: return {.d{m.k3}, .type{::phy_engine::model::variant_type::d}};
            case 37: return {.d{m.k3b}, .type{::phy_engine::model::variant_type::d}};
            case 38: return {.d{m.w0}, .type{::phy_engine::model::variant_type::d}};
            case 39: return {.d{m.nlx}, .type{::phy_engine::model::variant_type::d}};
            case 40: return {.d{m.vbm}, .type{::phy_engine::model::variant_type::d}};
            case 41: return {.d{m.delta1}, .type{::phy_engine::model::variant_type::d}};
            case 42: return {.d{m.vbi}, .type{::phy_engine::model::variant_type::d}};
            case 43: return {.d{m.dvt0}, .type{::phy_engine::model::variant_type::d}};
            case 44: return {.d{m.dvt1}, .type{::phy_engine::model::variant_type::d}};
            case 45: return {.d{m.dvt2}, .type{::phy_engine::model::variant_type::d}};
            case 46: return {.d{m.dsub}, .type{::phy_engine::model::variant_type::d}};
            case 47: return {.d{m.eta0}, .type{::phy_engine::model::variant_type::d}};
            case 48: return {.d{m.etab}, .type{::phy_engine::model::variant_type::d}};
            case 49: return {.d{m.nfactor}, .type{::phy_engine::model::variant_type::d}};
            case 50: return {.d{m.cit}, .type{::phy_engine::model::variant_type::d}};
            case 51: return {.d{m.pclm}, .type{::phy_engine::model::variant_type::d}};
            case 52: return {.d{m.pdiblc1}, .type{::phy_engine::model::variant_type::d}};
            case 53: return {.d{m.pdiblc2}, .type{::phy_engine::model::variant_type::d}};
            case 54: return {.d{m.pdiblcb}, .type{::phy_engine::model::variant_type::d}};
            case 55: return {.d{m.drout}, .type{::phy_engine::model::variant_type::d}};
            case 56: return {.d{m.pvag}, .type{::phy_engine::model::variant_type::d}};
            case 57: return {.d{m.pscbe1}, .type{::phy_engine::model::variant_type::d}};
            case 58: return {.d{m.pscbe2}, .type{::phy_engine::model::variant_type::d}};
            case 59: return {.d{m.delta}, .type{::phy_engine::model::variant_type::d}};
            case 60: return {.d{m.rds}, .type{::phy_engine::model::variant_type::d}};
            case 94: return {.d{m.rdsw}, .type{::phy_engine::model::variant_type::d}};
            case 95: return {.d{m.prwg}, .type{::phy_engine::model::variant_type::d}};
            case 96: return {.d{m.prwb}, .type{::phy_engine::model::variant_type::d}};
            case 61: return {.d{m.keta}, .type{::phy_engine::model::variant_type::d}};
            case 62: return {.d{m.capMod}, .type{::phy_engine::model::variant_type::d}};
            case 63: return {.d{m.xpart}, .type{::phy_engine::model::variant_type::d}};
            case 64: return {.d{m.dwc}, .type{::phy_engine::model::variant_type::d}};
            case 65: return {.d{m.dlc}, .type{::phy_engine::model::variant_type::d}};
            default: return {};
        }
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<bsim3v32_nmos>);
    static_assert(::phy_engine::model::defines::has_get_attribute<bsim3v32_pmos>);

    template <bool is_pmos>
    inline constexpr ::fast_io::u8string_view get_attribute_name_define(::phy_engine::model::model_reserve_type_t<bsim3v32_mos<is_pmos>>,
                                                                        ::std::size_t idx) noexcept
    {
        switch(idx)
        {
            case 0: return u8"W";
            case 1: return u8"L";
            case 13: return u8"m";
            case 100: return u8"nf";
            case 102: return u8"lref";
            case 103: return u8"wref";
            case 104: return u8"lvth0";
            case 105: return u8"wvth0";
            case 106: return u8"pvth0";
            case 107: return u8"lkp";
            case 108: return u8"wkp";
            case 109: return u8"pkp";
            case 110: return u8"lu0";
            case 111: return u8"wu0";
            case 112: return u8"pu0";
            case 113: return u8"lrdsw";
            case 114: return u8"wrdsw";
            case 115: return u8"prdsw";
            case 116: return u8"lua";
            case 117: return u8"wua";
            case 118: return u8"pua";
            case 119: return u8"lub";
            case 120: return u8"wub";
            case 121: return u8"pub";
            case 122: return u8"luc";
            case 123: return u8"wuc";
            case 124: return u8"puc";
            case 125: return u8"lvsat";
            case 126: return u8"wvsat";
            case 127: return u8"pvsat";
            case 128: return u8"ldsub";
            case 129: return u8"wdsub";
            case 130: return u8"pdsub";
            case 131: return u8"leta0";
            case 132: return u8"weta0";
            case 133: return u8"peta0";
            case 134: return u8"letab";
            case 135: return u8"wetab";
            case 136: return u8"petab";
            case 137: return u8"lpclm";
            case 138: return u8"wpclm";
            case 139: return u8"ppclm";
            case 140: return u8"lpdiblc1";
            case 141: return u8"wpdiblc1";
            case 142: return u8"ppdiblc1";
            case 143: return u8"lpdiblc2";
            case 144: return u8"wpdiblc2";
            case 145: return u8"ppdiblc2";
            case 146: return u8"lpdiblcb";
            case 147: return u8"wpdiblcb";
            case 148: return u8"ppdiblcb";
            case 149: return u8"ldrout";
            case 150: return u8"wdrout";
            case 151: return u8"pdrout";
            case 152: return u8"lpvag";
            case 153: return u8"wpvag";
            case 154: return u8"ppvag";
            case 155: return u8"lpscbe1";
            case 156: return u8"wpscbe1";
            case 157: return u8"ppscbe1";
            case 158: return u8"lpscbe2";
            case 159: return u8"wpscbe2";
            case 160: return u8"ppscbe2";
            case 161: return u8"ldvt0";
            case 162: return u8"wdvt0";
            case 163: return u8"pdvt0";
            case 164: return u8"ldvt1";
            case 165: return u8"wdvt1";
            case 166: return u8"pdvt1";
            case 167: return u8"ldvt2";
            case 168: return u8"wdvt2";
            case 169: return u8"pdvt2";
            case 170: return u8"lnfactor";
            case 171: return u8"wnfactor";
            case 172: return u8"pnfactor";
            case 173: return u8"lcit";
            case 174: return u8"wcit";
            case 175: return u8"pcit";
            case 176: return u8"lketa";
            case 177: return u8"wketa";
            case 178: return u8"pketa";
            case 185: return u8"lprwg";
            case 186: return u8"wprwg";
            case 187: return u8"pprwg";
            case 188: return u8"lprwb";
            case 189: return u8"wprwb";
            case 190: return u8"pprwb";
            case 191: return u8"lk1";
            case 192: return u8"wk1";
            case 193: return u8"pk1";
            case 194: return u8"lk2";
            case 195: return u8"wk2";
            case 196: return u8"pk2";
            case 197: return u8"lk3";
            case 198: return u8"wk3";
            case 199: return u8"pk3";
            case 200: return u8"lk3b";
            case 201: return u8"wk3b";
            case 202: return u8"pk3b";
            case 203: return u8"lw0";
            case 204: return u8"ww0";
            case 205: return u8"pw0";
            case 206: return u8"lnlx";
            case 207: return u8"wnlx";
            case 208: return u8"pnlx";
            case 209: return u8"voff";
            case 210: return u8"lvoff";
            case 211: return u8"wvoff";
            case 212: return u8"pvoff";
            case 213: return u8"lnch";
            case 214: return u8"wnch";
            case 215: return u8"pnch";
            case 216: return u8"lgamma";
            case 217: return u8"wgamma";
            case 218: return u8"pgamma";
            case 219: return u8"lphi";
            case 220: return u8"wphi";
            case 221: return u8"pphi";
            case 14: return u8"Rd";
            case 15: return u8"Rs";
            case 97: return u8"rsh";
            case 98: return u8"nrd";
            case 99: return u8"nrs";
            case 78: return u8"rg";
            case 2: return u8"Kp";
            case 3: return u8"lambda";
            case 4: return u8"Vth0";
            case 5: return u8"gamma";
            case 6: return u8"phi";
            case 7: return u8"Cgs";
            case 8: return u8"Cgd";
            case 9: return u8"Cgb";
            case 66: return u8"cgso";
            case 67: return u8"cgdo";
            case 68: return u8"cgbo";
            case 10: return u8"diode_Is";
            case 11: return u8"diode_N";
            case 12: return u8"Temp";
            case 83: return u8"tt";
            case 71: return u8"tnom";
            case 72: return u8"ute";
            case 73: return u8"kt1";
            case 74: return u8"kt2";
            case 75: return u8"at";
            case 69: return u8"js";
            case 70: return u8"jsw";
            case 101: return u8"jswg";
            case 76: return u8"xti";
            case 77: return u8"eg";
            case 16: return u8"drainArea";
            case 17: return u8"sourceArea";
            case 18: return u8"drainPerimeter";
            case 19: return u8"sourcePerimeter";
            case 79: return u8"ad";
            case 80: return u8"as";
            case 81: return u8"pd";
            case 82: return u8"ps";
            case 20: return u8"cj";
            case 21: return u8"cjsw";
            case 89: return u8"cjswg";
            case 22: return u8"pb";
            case 84: return u8"pbsw";
            case 90: return u8"pbswg";
            case 85: return u8"tcj";
            case 86: return u8"tcjsw";
            case 87: return u8"tpb";
            case 88: return u8"tpbsw";
            case 91: return u8"tcjswg";
            case 92: return u8"tpbswg";
            case 23: return u8"mj";
            case 24: return u8"mjsw";
            case 93: return u8"mjswg";
            case 25: return u8"fc";
            case 26: return u8"tox";
            case 27: return u8"toxm";
            case 28: return u8"nch";
            case 29: return u8"u0";
            case 30: return u8"ua";
            case 31: return u8"ub";
            case 32: return u8"uc";
            case 33: return u8"vsat";
            case 34: return u8"k1";
            case 35: return u8"k2";
            case 36: return u8"k3";
            case 37: return u8"k3b";
            case 38: return u8"w0";
            case 39: return u8"nlx";
            case 40: return u8"vbm";
            case 41: return u8"delta1";
            case 42: return u8"vbi";
            case 43: return u8"dvt0";
            case 44: return u8"dvt1";
            case 45: return u8"dvt2";
            case 46: return u8"dsub";
            case 47: return u8"eta0";
            case 48: return u8"etab";
            case 49: return u8"nfactor";
            case 50: return u8"cit";
            case 51: return u8"pclm";
            case 52: return u8"pdiblc1";
            case 53: return u8"pdiblc2";
            case 54: return u8"pdiblcb";
            case 55: return u8"drout";
            case 56: return u8"pvag";
            case 57: return u8"pscbe1";
            case 58: return u8"pscbe2";
            case 59: return u8"delta";
            case 60: return u8"rds";
            case 94: return u8"rdsw";
            case 95: return u8"prwg";
            case 96: return u8"prwb";
            case 61: return u8"keta";
            case 62: return u8"capMod";
            case 63: return u8"xpart";
            case 64: return u8"dwc";
            case 65: return u8"dlc";
            default: return {};
        }
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<bsim3v32_nmos>);
    static_assert(::phy_engine::model::defines::has_get_attribute_name<bsim3v32_pmos>);

    template <bool is_pmos>
    inline bool prepare_foundation_define(::phy_engine::model::model_reserve_type_t<bsim3v32_mos<is_pmos>>, bsim3v32_mos<is_pmos>& m) noexcept
    {
        auto const nd_ext{m.pins[0].nodes};
        auto const ns{m.pins[2].nodes};
        auto const nb_raw{m.pins[3].nodes};
        // allow 3-terminal usage (bulk tied to source diffusion)
        auto const nb{nb_raw ? nb_raw : (m.ns_int ? m.ns_int : ns)};
        if(nd_ext == nullptr || ns == nullptr || nb == nullptr) { return true; }

        bool const use_d_int{m.nd_int != nullptr};
        bool const use_s_int{m.ns_int != nullptr};
        auto const nd{use_d_int ? m.nd_int : nd_ext};
        auto const ns_int{use_s_int ? m.ns_int : ns};

        double const scale{(m.m_mult > 0.0 ? m.m_mult : 0.0) * (m.nf > 0.0 ? m.nf : 0.0)};
        // Keep diode internals numerically well-defined even if m<=0 (treat as "almost off").
        double const scale_diode{scale > 0.0 ? scale : 1e-30};
        double const temp_is_scale{details::bsim3v32_is_temp_scale(m.Temp, m.tnom, m.xti, m.eg)};
        double const diode_is{(m.diode_Is > 0.0 ? m.diode_Is : 1e-30) * temp_is_scale};

        // Propagate basic diode params and attach to external nodes.
        m.diode_db.N = m.diode_N;
        m.diode_db.Temp = m.Temp;
        m.diode_db.tt = m.tt;
        m.diode_sb.N = m.diode_N;
        m.diode_sb.Temp = m.Temp;
        m.diode_sb.tt = m.tt;

        if(m.js != 0.0 || m.jsw != 0.0 || m.jswg != 0.0)
        {
            // Use diffusion geometry scaling (area + sidewall perimeter).
            double const weff{::std::max(m.W - 2.0 * ::std::max(m.dwc, 0.0), 0.0)};
            double is_db{(m.js * m.drainArea + m.jsw * m.drainPerimeter + m.jswg * weff) * scale * temp_is_scale};
            double is_sb{(m.js * m.sourceArea + m.jsw * m.sourcePerimeter + m.jswg * weff) * scale * temp_is_scale};
            if(!(is_db > 0.0)) { is_db = diode_is * scale_diode; }
            if(!(is_sb > 0.0)) { is_sb = diode_is * scale_diode; }
            m.diode_db.Is = is_db;
            m.diode_db.Area = 1.0;
            m.diode_sb.Is = is_sb;
            m.diode_sb.Area = 1.0;
        }
        else
        {
            // Legacy: a single diode saturation current scaled only by m.
            m.diode_db.Is = diode_is;
            m.diode_db.Area = scale_diode;
            m.diode_sb.Is = diode_is;
            m.diode_sb.Area = scale_diode;
        }

        details::attach_body_diodes<is_pmos>(nd, ns_int, nb, m.diode_db, m.diode_sb);
        (void)prepare_foundation_define(::phy_engine::model::model_reserve_type<PN_junction>, m.diode_db);
        (void)prepare_foundation_define(::phy_engine::model::model_reserve_type<PN_junction>, m.diode_sb);
        return true;
    }

    static_assert(::phy_engine::model::defines::can_prepare_foundation<bsim3v32_nmos>);
    static_assert(::phy_engine::model::defines::can_prepare_foundation<bsim3v32_pmos>);

    template <bool is_pmos>
    inline bool load_temperature_define(::phy_engine::model::model_reserve_type_t<bsim3v32_mos<is_pmos>>, bsim3v32_mos<is_pmos>& m, double temp) noexcept
    {
        m.Temp = temp;
        // Recompute temperature-dependent internals (including diode Is scaling) at this temperature.
        // NOTE: The engine calls load_temperature() during prepare() for every analysis type, so this is
        // the right hook to update any temperature-dependent quantities.
        (void)prepare_foundation_define(::phy_engine::model::model_reserve_type<bsim3v32_mos<is_pmos>>, m);
        return true;
    }

    static_assert(::phy_engine::model::defines::can_load_temperature<bsim3v32_nmos>);
    static_assert(::phy_engine::model::defines::can_load_temperature<bsim3v32_pmos>);

    template <bool is_pmos>
    inline constexpr void bsim3v32_iterate_dc_core_no_diode(bsim3v32_mos<is_pmos>& m, ::phy_engine::MNA::MNA& mna) noexcept
    {
        auto const nd_ext{m.pins[0].nodes};
        auto const ng_ext{m.pins[1].nodes};
        auto const ns_ext{m.pins[2].nodes};
        auto const nb_raw{m.pins[3].nodes};
        auto const nd{m.nd_int ? m.nd_int : nd_ext};  // diffusion D'
        auto const ns{m.ns_int ? m.ns_int : ns_ext};  // diffusion S'
        auto const ng{m.ng_int ? m.ng_int : ng_ext};  // intrinsic gate
        auto const nb{nb_raw ? nb_raw : ns};          // allow 3-terminal usage (bulk tied to source diffusion)
        if(!(nd_ext && ng_ext && ns_ext && nb)) [[unlikely]] { return; }

        double const scale{(m.m_mult > 0.0 ? m.m_mult : 0.0) * (m.nf > 0.0 ? m.nf : 0.0)};

        // Terminal series resistances (linear stamps).
        double const rd_total{m.Rd + ::std::max(m.rsh, 0.0) * ::std::max(m.nrd, 0.0)};
        double const rs_total{m.Rs + ::std::max(m.rsh, 0.0) * ::std::max(m.nrs, 0.0)};
        double const rd_eff{scale > 0.0 ? (rd_total / scale) : rd_total};
        double const rs_eff{scale > 0.0 ? (rs_total / scale) : rs_total};
        if(m.nd_int) { details::stamp_resistor(mna, nd_ext, nd, rd_eff); }
        if(m.ns_int) { details::stamp_resistor(mna, ns_ext, ns, rs_eff); }

        // Gate resistance (linear stamp).
        double const rg_eff{scale > 0.0 ? (m.rg / scale) : m.rg};
        if(m.ng_int) { details::stamp_resistor(mna, ng_ext, ng, rg_eff); }

        // Channel mode selection (SPICE-style): swap D/S for the intrinsic channel if needed.
        double const sgn{is_pmos ? -1.0 : 1.0};

        auto* nd_ch = nd;
        auto* ns_ch = ns;
        double Vd_ch = nd_ch->node_information.an.voltage.real();
        double Vs_ch = ns_ch->node_information.an.voltage.real();
        if(sgn * Vd_ch < sgn * Vs_ch)
        {
            ::std::swap(nd_ch, ns_ch);
            ::std::swap(Vd_ch, Vs_ch);
            m.mode_swapped = true;
        }
        else
        {
            m.mode_swapped = false;
        }

        double const Vg{ng->node_information.an.voltage.real()};
        double const Vb{nb->node_information.an.voltage.real()};
        double const Vgs{Vg - Vs_ch};
        double const Vds{Vd_ch - Vs_ch};
        double const Vbs{Vb - Vs_ch};

        double const vt{details::thermal_voltage(m.Temp)};
        double const vgs_s{sgn * Vgs};
        double const vds_s{sgn * Vds};
        double const vbs_s{sgn * Vbs};

        // Apply SPICE-style voltage limiting to improve Newton convergence (clean-room).
        double vgs_s_eval{vgs_s};
        double vds_s_eval{vds_s};
        double vbs_s_eval{vbs_s};
        if(m.dc_bias_valid && m.dc_bias_swapped == m.mode_swapped)
        {
            double const vto_lim{details::bsim3v32_vth0_temp_mag(m.Vth0, m.Temp, m.tnom, m.kt1, m.kt2)};
            vgs_s_eval = details::bsim3v32_fetlim(vgs_s_eval, m.vgs_s_last, vto_lim);
            vds_s_eval = details::bsim3v32_limvds(vds_s_eval, m.vds_s_last);
            vbs_s_eval = details::bsim3v32_fetlim(vbs_s_eval, m.vbs_s_last, 0.0);
        }

        // Update bias history for the next Newton iteration.
        m.dc_bias_valid = true;
        m.dc_bias_swapped = m.mode_swapped;
        m.vgs_s_last = vgs_s_eval;
        m.vds_s_last = vds_s_eval;
        m.vbs_s_last = vbs_s_eval;

        double const Vgs_eval{sgn * vgs_s_eval};
        double const Vds_eval{sgn * vds_s_eval};
        double const Vbs_eval{sgn * vbs_s_eval};

        double Id{};
        m.gm = 0.0;
        m.gds = 0.0;
        m.gmb = 0.0;

        auto const ids_dual = details::bsim3v32_ids_core(m,
                                                         details::bsim3v32_dual3_vgs(vgs_s_eval),
                                                         details::bsim3v32_dual3_vds(vds_s_eval),
                                                         details::bsim3v32_dual3_vbs(vbs_s_eval),
                                                         vt,
                                                         static_cast<details::bsim3v32_core_cache_t<details::bsim3v32_dual3>*>(nullptr));
        Id = sgn * ids_dual.val;
        m.gm = ids_dual.dvgs;
        m.gds = ids_dual.dvds;
        m.gmb = ids_dual.dvbs;

        // Linearization: Id ≈ gm*Vgs + gds*Vds + gmb*Vbs + Ieq (Id is D->S between nd_ch and ns_ch)
        m.Ieq = Id - m.gm * Vgs_eval - m.gds * Vds_eval - m.gmb * Vbs_eval;
        details::stamp_gm_gds_gmb(mna, nd_ch, ng, ns_ch, nb, m.gm * scale, m.gds * scale, m.gmb * scale, m.Ieq * scale);
    }

    template <bool is_pmos>
    inline constexpr bool
        iterate_dc_define(::phy_engine::model::model_reserve_type_t<bsim3v32_mos<is_pmos>>, bsim3v32_mos<is_pmos>& m, ::phy_engine::MNA::MNA& mna) noexcept
    {
        bsim3v32_iterate_dc_core_no_diode<is_pmos>(m, mna);

        auto const nd_ext{m.pins[0].nodes};
        auto const ns_ext{m.pins[2].nodes};
        auto const nb_raw{m.pins[3].nodes};
        auto const nd{m.nd_int ? m.nd_int : nd_ext};
        auto const ns{m.ns_int ? m.ns_int : ns_ext};
        auto const nb{nb_raw ? nb_raw : ns};
        if(nd_ext && ns_ext && nb) [[likely]]
        {
            details::attach_body_diodes<is_pmos>(nd, ns, nb, m.diode_db, m.diode_sb);
            (void)iterate_dc_define(::phy_engine::model::model_reserve_type<PN_junction>, m.diode_db, mna);
            (void)iterate_dc_define(::phy_engine::model::model_reserve_type<PN_junction>, m.diode_sb, mna);
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_dc<bsim3v32_nmos>);
    static_assert(::phy_engine::model::defines::can_iterate_dc<bsim3v32_pmos>);

    template <bool is_pmos>
    inline constexpr bool iterate_ac_define(::phy_engine::model::model_reserve_type_t<bsim3v32_mos<is_pmos>>,
                                            bsim3v32_mos<is_pmos>& m,
                                            ::phy_engine::MNA::MNA& mna,
                                            double omega) noexcept
    {
        auto const nd_ext{m.pins[0].nodes};
        auto const ng_ext{m.pins[1].nodes};
        auto const ns_ext{m.pins[2].nodes};
        auto const nb_raw{m.pins[3].nodes};
        auto const nd{m.nd_int ? m.nd_int : nd_ext};
        auto const ns{m.ns_int ? m.ns_int : ns_ext};
        auto const ng{m.ng_int ? m.ng_int : ng_ext};
        auto const nb{nb_raw ? nb_raw : ns};  // allow 3-terminal usage (bulk tied to source diffusion)
        if(nd_ext && ng_ext && ns_ext && nb) [[likely]]
        {
            double const scale{(m.m_mult > 0.0 ? m.m_mult : 0.0) * (m.nf > 0.0 ? m.nf : 0.0)};

            double const rd_total{m.Rd + ::std::max(m.rsh, 0.0) * ::std::max(m.nrd, 0.0)};
            double const rs_total{m.Rs + ::std::max(m.rsh, 0.0) * ::std::max(m.nrs, 0.0)};
            double const rd_eff{scale > 0.0 ? (rd_total / scale) : rd_total};
            double const rs_eff{scale > 0.0 ? (rs_total / scale) : rs_total};
            if(m.nd_int) { details::stamp_resistor(mna, nd_ext, nd, rd_eff); }
            if(m.ns_int) { details::stamp_resistor(mna, ns_ext, ns, rs_eff); }

            double const rg_eff{scale > 0.0 ? (m.rg / scale) : m.rg};
            if(m.ng_int) { details::stamp_resistor(mna, ng_ext, ng, rg_eff); }

            // small-signal conductances
            auto* const nd_lin = m.mode_swapped ? ns : nd;
            auto* const ns_lin = m.mode_swapped ? nd : ns;
            details::stamp_gm_gds_gmb(mna, nd_lin, ng, ns_lin, nb, m.gm * scale, m.gds * scale, m.gmb * scale, 0.0);

            // fixed caps (including overlap)
            double const weff{::std::max(m.W - 2.0 * ::std::max(m.dwc, 0.0), 0.0)};
            double const leff{::std::max(m.L - 2.0 * ::std::max(m.dlc, 0.0), 1e-18)};
            double const cgs_fix{m.Cgs + m.cgso * weff};
            double const cgd_fix{m.Cgd + m.cgdo * weff};
            double const cgb_fix{m.Cgb + m.cgbo * leff};
            details::stamp_cap_ac(mna, ng, ns, cgs_fix * scale, omega);
            details::stamp_cap_ac(mna, ng, nd, cgd_fix * scale, omega);
            details::stamp_cap_ac(mna, ng, nb, cgb_fix * scale, omega);

            // Intrinsic (bias-dependent) caps captured at the real operating point (save_op).
            if(m.capMod != 0.0)
            {
                ::phy_engine::model::node_t* const nodes[4]{nd_lin, ng, ns_lin, nb};
                double cmat_scaled[4][4]{};
                for(int i{}; i < 4; ++i)
                {
                    for(int j{}; j < 4; ++j) { cmat_scaled[i][j] = m.cmat_ac[i][j] * scale; }
                }
                details::stamp_cap_matrix_ac(mna, nodes, cmat_scaled, omega);
            }
            else
            {
                details::stamp_cap_ac(mna, ng, ns_lin, m.cgs_intr_ac * scale, omega);
                details::stamp_cap_ac(mna, ng, nd_lin, m.cgd_intr_ac * scale, omega);
                details::stamp_cap_ac(mna, ng, nb, m.cgb_intr_ac * scale, omega);
            }

            // diode incremental conductances
            details::attach_body_diodes<is_pmos>(nd, ns, nb, m.diode_db, m.diode_sb);
            (void)iterate_ac_define(::phy_engine::model::model_reserve_type<PN_junction>, m.diode_db, mna, omega);
            (void)iterate_ac_define(::phy_engine::model::model_reserve_type<PN_junction>, m.diode_sb, mna, omega);

            // optional depletion caps across the same junctions
            if(m.cj != 0.0 || m.cjsw != 0.0 || m.cjswg != 0.0)
            {
                auto* db_a = m.diode_db.pins[0].nodes;
                auto* db_b = m.diode_db.pins[1].nodes;
                auto* sb_a = m.diode_sb.pins[0].nodes;
                auto* sb_b = m.diode_sb.pins[1].nodes;
                if(db_a && db_b && m.cbd_ac != 0.0) { details::stamp_cap_ac(mna, db_a, db_b, m.cbd_ac * scale, omega); }
                if(sb_a && sb_b && m.cbs_ac != 0.0) { details::stamp_cap_ac(mna, sb_a, sb_b, m.cbs_ac * scale, omega); }
            }
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_ac<bsim3v32_nmos>);
    static_assert(::phy_engine::model::defines::can_iterate_ac<bsim3v32_pmos>);

    template <bool is_pmos>
    inline constexpr bool step_changed_tr_define(::phy_engine::model::model_reserve_type_t<bsim3v32_mos<is_pmos>>,
                                                 bsim3v32_mos<is_pmos>& m,
                                                 [[maybe_unused]] double nlaststep,
                                                 double nstep) noexcept
    {
        auto const nd_ext{m.pins[0].nodes};
        auto const ng_ext{m.pins[1].nodes};
        auto const ns_ext{m.pins[2].nodes};
        auto const nb_raw{m.pins[3].nodes};
        auto const nd{m.nd_int ? m.nd_int : nd_ext};
        auto const ns{m.ns_int ? m.ns_int : ns_ext};
        auto const ng{m.ng_int ? m.ng_int : ng_ext};
        auto const nb{nb_raw ? nb_raw : ns};  // allow 3-terminal usage (bulk tied to source diffusion)

        double const scale{(m.m_mult > 0.0 ? m.m_mult : 0.0) * (m.nf > 0.0 ? m.nf : 0.0)};
        double const weff{::std::max(m.W - 2.0 * ::std::max(m.dwc, 0.0), 0.0)};
        double const leff{::std::max(m.L - 2.0 * ::std::max(m.dlc, 0.0), 1e-18)};
        double const cgs_fix{m.Cgs + m.cgso * weff};
        double const cgd_fix{m.Cgd + m.cgdo * weff};
        double const cgb_fix{m.Cgb + m.cgbo * leff};
        details::step_cap_tr(m.cgs_state, ng, ns, cgs_fix * scale, nstep);
        details::step_cap_tr(m.cgd_state, ng, nd, cgd_fix * scale, nstep);
        details::step_cap_tr(m.cgb_state, ng, nb, cgb_fix * scale, nstep);

        // Intrinsic charge/capacitance model (linearized at the previous step's bias).
        m.cap_mode_swapped = m.mode_swapped;
        if(m.capMod != 0.0)
        {
            // capMod!=0: charge-based 4×4 C-matrix companion (D,G,S,B).
            // Note: C-matrix already includes W/L geometry; apply m-mult scaling here.
            // Keep scalar intrinsic states inactive to avoid double-counting if the model toggles at runtime.
            m.cgs_int_state.tr_hist_current = 0.0;
            m.cgs_int_state.tr_prev_g = 0.0;
            m.cgd_int_state.tr_hist_current = 0.0;
            m.cgd_int_state.tr_prev_g = 0.0;
            m.cgb_int_state.tr_hist_current = 0.0;
            m.cgb_int_state.tr_prev_g = 0.0;

            if(nd && ng && ns && nb && scale > 0.0)
            {
                auto* const nd_lin = m.cap_mode_swapped ? ns : nd;
                auto* const ns_lin = m.cap_mode_swapped ? nd : ns;
                ::phy_engine::model::node_t* const nodes[4]{nd_lin, ng, ns_lin, nb};

                double const Vd{nd_lin->node_information.an.voltage.real()};
                double const Vs{ns_lin->node_information.an.voltage.real()};
                double const Vg{ng->node_information.an.voltage.real()};
                double const Vb{nb->node_information.an.voltage.real()};

                double cmat[4][4]{};
                details::bsim3v32_cmatrix_capmod0_simple<is_pmos>(m, Vd, Vg, Vs, Vb, cmat);
                for(int i{}; i < 4; ++i)
                {
                    for(int j{}; j < 4; ++j) { cmat[i][j] *= scale; }
                }
                details::step_cap_matrix_tr(m.cmat_tr_hist, m.cmat_tr_prev_g, nodes, cmat, nstep);
            }
            else
            {
                for(int i{}; i < 4; ++i)
                {
                    m.cmat_tr_hist[i] = 0.0;
                    for(int j{}; j < 4; ++j) { m.cmat_tr_prev_g[i][j] = 0.0; }
                }
            }
        }
        else
        {
            // Reset matrix state when using scalar intrinsic caps.
            for(int i{}; i < 4; ++i)
            {
                m.cmat_tr_hist[i] = 0.0;
                for(int j{}; j < 4; ++j) { m.cmat_tr_prev_g[i][j] = 0.0; }
            }

            // Meyer intrinsic caps (bias-dependent) linearized at the previous step's bias.
            if(nd && ng && ns && nb)
            {
                auto* const nd_lin = m.cap_mode_swapped ? ns : nd;
                auto* const ns_lin = m.cap_mode_swapped ? nd : ns;

                double const sgn{is_pmos ? -1.0 : 1.0};
                double const vt{details::thermal_voltage(m.Temp)};
                double const Vd{nd_lin->node_information.an.voltage.real()};
                double const Vs{ns_lin->node_information.an.voltage.real()};
                double const Vg{ng->node_information.an.voltage.real()};
                double const Vb{nb->node_information.an.voltage.real()};

                double const Vgs{Vg - Vs};
                double const Vds{Vd - Vs};
                double const Vbs{Vb - Vs};

                double cgs_i{}, cgd_i{}, cgb_i{};
                details::bsim3v32_meyer_intrinsic_caps(m, sgn * Vgs, sgn * Vds, sgn * Vbs, vt, cgs_i, cgd_i, cgb_i);
                details::step_cap_tr(m.cgs_int_state, ng, ns_lin, cgs_i * scale, nstep);
                details::step_cap_tr(m.cgd_int_state, ng, nd_lin, cgd_i * scale, nstep);
                details::step_cap_tr(m.cgb_int_state, ng, nb, cgb_i * scale, nstep);
            }
        }

        // Keep diode vlimit history aligned with node voltages.
        if(nd && ns && nb)
        {
            details::attach_body_diodes<is_pmos>(nd, ns, nb, m.diode_db, m.diode_sb);
            (void)step_changed_tr_define(::phy_engine::model::model_reserve_type<PN_junction>, m.diode_db, nlaststep, nstep);
            (void)step_changed_tr_define(::phy_engine::model::model_reserve_type<PN_junction>, m.diode_sb, nlaststep, nstep);

            // optional depletion caps across the same junctions (linearized per step at previous bias)
            if(m.cj != 0.0 || m.cjsw != 0.0 || m.cjswg != 0.0)
            {
                auto* db_a = m.diode_db.pins[0].nodes;
                auto* db_b = m.diode_db.pins[1].nodes;
                auto* sb_a = m.diode_sb.pins[0].nodes;
                auto* sb_b = m.diode_sb.pins[1].nodes;
                if(db_a && db_b && sb_a && sb_b)
                {
                    double const vbd = db_a->node_information.an.voltage.real() - db_b->node_information.an.voltage.real();
                    double const vbs = sb_a->node_information.an.voltage.real() - sb_b->node_information.an.voltage.real();

                    double const cj_eff = details::bsim3v32_temp_linear_scale_clamped(m.cj, m.tcj, m.Temp, m.tnom, 0.0);
                    double const cjsw_eff = details::bsim3v32_temp_linear_scale_clamped(m.cjsw, m.tcjsw, m.Temp, m.tnom, 0.0);
                    double const cjswg_eff = details::bsim3v32_temp_linear_scale_clamped(m.cjswg, m.tcjswg, m.Temp, m.tnom, 0.0);
                    double const pb_eff = details::bsim3v32_temp_linear_scale_clamped(m.pb, m.tpb, m.Temp, m.tnom, 1e-12);
                    double const pbsw0 = (m.pbsw > 0.0) ? m.pbsw : m.pb;
                    double const tpbsw0 = (m.pbsw > 0.0) ? m.tpbsw : m.tpb;
                    double const pbsw_eff = details::bsim3v32_temp_linear_scale_clamped(pbsw0, tpbsw0, m.Temp, m.tnom, 1e-12);
                    double const pbswg0 = (m.pbswg > 0.0) ? m.pbswg : pbsw0;
                    double const tpbswg0 = (m.pbswg > 0.0) ? m.tpbswg : tpbsw0;
                    double const pbswg_eff = details::bsim3v32_temp_linear_scale_clamped(pbswg0, tpbswg0, m.Temp, m.tnom, 1e-12);

                    double const cbd_bottom0 = cj_eff * m.drainArea;
                    double const cbs_bottom0 = cj_eff * m.sourceArea;
                    double const cbd_side0 = cjsw_eff * m.drainPerimeter;
                    double const cbs_side0 = cjsw_eff * m.sourcePerimeter;
                    double const cbd_gate0 = cjswg_eff * weff;
                    double const cbs_gate0 = cjswg_eff * weff;

                    double const cbd = details::bsim3v32_junction_cap(cbd_bottom0, vbd, pb_eff, m.mj, m.fc) +
                                       details::bsim3v32_junction_cap(cbd_side0, vbd, pbsw_eff, m.mjsw, m.fc) +
                                       details::bsim3v32_junction_cap(cbd_gate0, vbd, pbswg_eff, m.mjswg, m.fc);
                    double const cbs = details::bsim3v32_junction_cap(cbs_bottom0, vbs, pb_eff, m.mj, m.fc) +
                                       details::bsim3v32_junction_cap(cbs_side0, vbs, pbsw_eff, m.mjsw, m.fc) +
                                       details::bsim3v32_junction_cap(cbs_gate0, vbs, pbswg_eff, m.mjswg, m.fc);

                    details::step_cap_tr(m.capbd_state, db_a, db_b, cbd * scale, nstep);
                    details::step_cap_tr(m.capbs_state, sb_a, sb_b, cbs * scale, nstep);
                }
            }
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_step_changed_tr<bsim3v32_nmos>);
    static_assert(::phy_engine::model::defines::can_step_changed_tr<bsim3v32_pmos>);

    template <bool is_pmos>
    inline constexpr bool iterate_tr_define(::phy_engine::model::model_reserve_type_t<bsim3v32_mos<is_pmos>>,
                                            bsim3v32_mos<is_pmos>& m,
                                            ::phy_engine::MNA::MNA& mna,
                                            double /*t_time*/) noexcept
    {
        // Quasi-static: stamp channel DC conductances + companion capacitors.
        // Body diodes are stamped using their TR implementation so optional diffusion capacitance (tt) is included.
        bsim3v32_iterate_dc_core_no_diode<is_pmos>(m, mna);

        auto const nd_ext{m.pins[0].nodes};
        auto const ng_ext{m.pins[1].nodes};
        auto const ns_ext{m.pins[2].nodes};
        auto const nb_raw{m.pins[3].nodes};
        auto const nd{m.nd_int ? m.nd_int : nd_ext};
        auto const ns{m.ns_int ? m.ns_int : ns_ext};
        auto const ng{m.ng_int ? m.ng_int : ng_ext};
        auto const nb{nb_raw ? nb_raw : ns};  // allow 3-terminal usage (bulk tied to source diffusion)
        if(nd_ext && ng_ext && ns_ext && nb) [[likely]]
        {
            details::attach_body_diodes<is_pmos>(nd, ns, nb, m.diode_db, m.diode_sb);
            (void)iterate_tr_define(::phy_engine::model::model_reserve_type<PN_junction>, m.diode_db, mna, 0.0);
            (void)iterate_tr_define(::phy_engine::model::model_reserve_type<PN_junction>, m.diode_sb, mna, 0.0);

            details::stamp_cap_tr(mna, ng, ns, m.cgs_state);
            details::stamp_cap_tr(mna, ng, nd, m.cgd_state);
            details::stamp_cap_tr(mna, ng, nb, m.cgb_state);

            auto* const nd_lin = m.cap_mode_swapped ? ns : nd;
            auto* const ns_lin = m.cap_mode_swapped ? nd : ns;
            if(m.capMod != 0.0)
            {
                ::phy_engine::model::node_t* const nodes[4]{nd_lin, ng, ns_lin, nb};
                details::stamp_cap_matrix_tr(mna, nodes, m.cmat_tr_prev_g, m.cmat_tr_hist);
            }
            else
            {
                details::stamp_cap_tr(mna, ng, ns_lin, m.cgs_int_state);
                details::stamp_cap_tr(mna, ng, nd_lin, m.cgd_int_state);
                details::stamp_cap_tr(mna, ng, nb, m.cgb_int_state);
            }
        }

        // Optional depletion caps across body diodes (companion models prepared in step_changed_tr).
        auto const nd2{m.pins[0].nodes};
        auto const ns2{m.pins[2].nodes};
        auto const nb2_raw{m.pins[3].nodes};
        auto const nd_dio{m.nd_int ? m.nd_int : nd2};
        auto const ns_dio{m.ns_int ? m.ns_int : ns2};
        auto const nb2{nb2_raw ? nb2_raw : ns_dio};
        if(nd2 && ns2 && nb2)
        {
            details::attach_body_diodes<is_pmos>(nd_dio, ns_dio, nb2, m.diode_db, m.diode_sb);
            auto* db_a = m.diode_db.pins[0].nodes;
            auto* db_b = m.diode_db.pins[1].nodes;
            auto* sb_a = m.diode_sb.pins[0].nodes;
            auto* sb_b = m.diode_sb.pins[1].nodes;
            if(db_a && db_b) { details::stamp_cap_tr(mna, db_a, db_b, m.capbd_state); }
            if(sb_a && sb_b) { details::stamp_cap_tr(mna, sb_a, sb_b, m.capbs_state); }
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_tr<bsim3v32_nmos>);
    static_assert(::phy_engine::model::defines::can_iterate_tr<bsim3v32_pmos>);

    template <bool is_pmos>
    inline constexpr bool
        iterate_trop_define(::phy_engine::model::model_reserve_type_t<bsim3v32_mos<is_pmos>>, bsim3v32_mos<is_pmos>& m, ::phy_engine::MNA::MNA& mna) noexcept
    {
        // Transient operating point: capacitive parts open-circuit -> use DC stamping only.
        return iterate_dc_define(::phy_engine::model::model_reserve_type<bsim3v32_mos<is_pmos>>, m, mna);
    }

    static_assert(::phy_engine::model::defines::can_iterate_trop<bsim3v32_nmos>);
    static_assert(::phy_engine::model::defines::can_iterate_trop<bsim3v32_pmos>);

    template <bool is_pmos>
    inline bool save_op_define(::phy_engine::model::model_reserve_type_t<bsim3v32_mos<is_pmos>>, bsim3v32_mos<is_pmos>& m) noexcept
    {
        // Capture bias-dependent small-signal capacitors at the current (real) operating point.
        auto const nd_ext{m.pins[0].nodes};
        auto const ng_ext{m.pins[1].nodes};
        auto const ns_ext{m.pins[2].nodes};
        auto const nb_raw{m.pins[3].nodes};
        auto const nd{m.nd_int ? m.nd_int : nd_ext};
        auto const ns{m.ns_int ? m.ns_int : ns_ext};
        auto const ng{m.ng_int ? m.ng_int : ng_ext};
        auto const nb{nb_raw ? nb_raw : ns};
        if(nd_ext == nullptr || ng_ext == nullptr || ns_ext == nullptr || nb == nullptr) { return true; }

        auto* const nd_lin = m.mode_swapped ? ns : nd;
        auto* const ns_lin = m.mode_swapped ? nd : ns;

        double const sgn{is_pmos ? -1.0 : 1.0};
        double const vt{details::thermal_voltage(m.Temp)};
        double const Vd{nd_lin->node_information.an.voltage.real()};
        double const Vs{ns_lin->node_information.an.voltage.real()};
        double const Vg{ng->node_information.an.voltage.real()};
        double const Vb{nb->node_information.an.voltage.real()};

        double const Vgs{Vg - Vs};
        double const Vds{Vd - Vs};
        double const Vbs{Vb - Vs};

        details::bsim3v32_meyer_intrinsic_caps(m, sgn * Vgs, sgn * Vds, sgn * Vbs, vt, m.cgs_intr_ac, m.cgd_intr_ac, m.cgb_intr_ac);

        // Intrinsic charge C-matrix (enabled when capMod!=0) captured at the real operating point.
        for(int i{}; i < 4; ++i)
        {
            for(int j{}; j < 4; ++j) { m.cmat_ac[i][j] = 0.0; }
        }
        if(m.capMod != 0.0) { details::bsim3v32_cmatrix_capmod0_simple<is_pmos>(m, Vd, Vg, Vs, Vb, m.cmat_ac); }

        // Depletion caps at the diode biases (real OP). Uses the same diode node attachments.
        m.cbd_ac = 0.0;
        m.cbs_ac = 0.0;
        if(m.cj != 0.0 || m.cjsw != 0.0 || m.cjswg != 0.0)
        {
            details::attach_body_diodes<is_pmos>(nd, ns, nb, m.diode_db, m.diode_sb);
            auto* db_a = m.diode_db.pins[0].nodes;
            auto* db_b = m.diode_db.pins[1].nodes;
            auto* sb_a = m.diode_sb.pins[0].nodes;
            auto* sb_b = m.diode_sb.pins[1].nodes;
            if(db_a && db_b && sb_a && sb_b)
            {
                double const vbd = db_a->node_information.an.voltage.real() - db_b->node_information.an.voltage.real();
                double const vbs = sb_a->node_information.an.voltage.real() - sb_b->node_information.an.voltage.real();

                double const cj_eff = details::bsim3v32_temp_linear_scale_clamped(m.cj, m.tcj, m.Temp, m.tnom, 0.0);
                double const cjsw_eff = details::bsim3v32_temp_linear_scale_clamped(m.cjsw, m.tcjsw, m.Temp, m.tnom, 0.0);
                double const cjswg_eff = details::bsim3v32_temp_linear_scale_clamped(m.cjswg, m.tcjswg, m.Temp, m.tnom, 0.0);
                double const pb_eff = details::bsim3v32_temp_linear_scale_clamped(m.pb, m.tpb, m.Temp, m.tnom, 1e-12);
                double const pbsw0 = (m.pbsw > 0.0) ? m.pbsw : m.pb;
                double const tpbsw0 = (m.pbsw > 0.0) ? m.tpbsw : m.tpb;
                double const pbsw_eff = details::bsim3v32_temp_linear_scale_clamped(pbsw0, tpbsw0, m.Temp, m.tnom, 1e-12);
                double const pbswg0 = (m.pbswg > 0.0) ? m.pbswg : pbsw0;
                double const tpbswg0 = (m.pbswg > 0.0) ? m.tpbswg : tpbsw0;
                double const pbswg_eff = details::bsim3v32_temp_linear_scale_clamped(pbswg0, tpbswg0, m.Temp, m.tnom, 1e-12);

                double const weff{::std::max(m.W - 2.0 * ::std::max(m.dwc, 0.0), 0.0)};
                double const cbd_bottom0 = cj_eff * m.drainArea;
                double const cbs_bottom0 = cj_eff * m.sourceArea;
                double const cbd_side0 = cjsw_eff * m.drainPerimeter;
                double const cbs_side0 = cjsw_eff * m.sourcePerimeter;
                double const cbd_gate0 = cjswg_eff * weff;
                double const cbs_gate0 = cjswg_eff * weff;

                m.cbd_ac = details::bsim3v32_junction_cap(cbd_bottom0, vbd, pb_eff, m.mj, m.fc) +
                           details::bsim3v32_junction_cap(cbd_side0, vbd, pbsw_eff, m.mjsw, m.fc) +
                           details::bsim3v32_junction_cap(cbd_gate0, vbd, pbswg_eff, m.mjswg, m.fc);
                m.cbs_ac = details::bsim3v32_junction_cap(cbs_bottom0, vbs, pb_eff, m.mj, m.fc) +
                           details::bsim3v32_junction_cap(cbs_side0, vbs, pbsw_eff, m.mjsw, m.fc) +
                           details::bsim3v32_junction_cap(cbs_gate0, vbs, pbswg_eff, m.mjswg, m.fc);
            }
        }

        return true;
    }

    static_assert(::phy_engine::model::defines::can_save_op<bsim3v32_nmos>);
    static_assert(::phy_engine::model::defines::can_save_op<bsim3v32_pmos>);

    template <bool is_pmos>
    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<bsim3v32_mos<is_pmos>>,
                                                                            bsim3v32_mos<is_pmos>& m) noexcept
    { return {m.pins, 4}; }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<bsim3v32_nmos>);
    static_assert(::phy_engine::model::defines::can_generate_pin_view<bsim3v32_pmos>);

    template <bool is_pmos>
    inline constexpr ::phy_engine::model::node_view generate_internal_node_define(::phy_engine::model::model_reserve_type_t<bsim3v32_mos<is_pmos>>,
                                                                                  bsim3v32_mos<is_pmos>& m) noexcept
    {
        // Only allocate internal nodes if they are actually used; otherwise they would create floating unknowns.
        m.nd_int = nullptr;
        m.ns_int = nullptr;
        m.ng_int = nullptr;

        double const rd_total{m.Rd + ::std::max(m.rsh, 0.0) * ::std::max(m.nrd, 0.0)};
        double const rs_total{m.Rs + ::std::max(m.rsh, 0.0) * ::std::max(m.nrs, 0.0)};

        ::std::size_t cnt{};
        if(rd_total > 0.0) { m.nd_int = __builtin_addressof(m.internal_nodes[cnt++]); }
        if(rs_total > 0.0) { m.ns_int = __builtin_addressof(m.internal_nodes[cnt++]); }
        if(m.rg > 0.0) { m.ng_int = __builtin_addressof(m.internal_nodes[cnt++]); }
        if(cnt != 0) { return {m.internal_nodes, cnt}; }
        return {};
    }

    static_assert(::phy_engine::model::defines::can_generate_internal_node_view<bsim3v32_nmos>);
    static_assert(::phy_engine::model::defines::can_generate_internal_node_view<bsim3v32_pmos>);

}  // namespace phy_engine::model
