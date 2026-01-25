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
//
// TODO(brief): Remaining work to reach a strict, Berkeley/NGSPICE-equivalent BSIM3v3.2 (clean-room)
// - Parameter coverage: add the full BSIM3v3.2 parameter set (including all model/instance params, defaults, and interactions).
// - Geometry & binning: implement full L/W/NF/M scaling, binning/selector rules, and all geometry-dependent corrections (incl. narrow width, SCE, WPE where
// applicable).
// - Temperature: complete temperature dependencies for all electrical quantities (Vth, mobility, saturation velocity, junction potentials/caps, leakage, etc.)
// per BSIM3v3.2.
// - Charge/C-V (capMod=3): replace the current clean-room stepping-stone with full BSIM3v3.2 charge equations (Qinv/Qb/Qacc/Qdep), including correct
// derivatives for C-matrix.
// - Gate leakage: implement full BSIM3v3.2 gate current models (Ig*, Igb/igs/igd variants, bias/temperature dependence, partitioning, and derivatives) beyond
// the current simplified hooks.
// - GIDL/GISL: extend to full BSIM3v3.2 behavior (bias dependence, temperature, derivatives, and corner cases) beyond the current simplified subset.
// - Junctions: implement full BSIM3v3.2 junction current and capacitance models (incl. sidewall/gate-edge, breakdown behavior, and charge conservation), beyond
// the current SPICE-style subset.
// - NQS: add non-quasi-static support (analysis hooks + model equations) if the engine is expected to support NQS transient/AC behavior.
// - Noise: add BSIM3v3.2 noise models (thermal, flicker, induced gate noise, correlation) and expose them through the simulator’s noise analysis
// infrastructure.
// - Validation: expand regression tests to cover parameter sweeps and operating-region transitions; cross-validate against a trusted reference simulator via
// black-box comparisons (without importing third-party code).

#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>

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
                                                 [[maybe_unused]] ::phy_engine::model::node_t* nb_db,
                                                 [[maybe_unused]] ::phy_engine::model::node_t* nb_sb,
                                                 [[maybe_unused]] PN_junction& db,
                                                 [[maybe_unused]] PN_junction& sb) noexcept
        {
            if constexpr(is_pmos)
            {
                // p+ diffusion to n-well: anode at D/S, cathode at B
                db.pins[0].nodes = nd;
                db.pins[1].nodes = nb_db;
                sb.pins[0].nodes = ns;
                sb.pins[1].nodes = nb_sb;
            }
            else
            {
                // p-sub to n+ diffusion: anode at B, cathode at D/S
                db.pins[0].nodes = nb_db;
                db.pins[1].nodes = nd;
                sb.pins[0].nodes = nb_sb;
                sb.pins[1].nodes = ns;
            }
        }

        template <bool is_pmos>
        inline constexpr void attach_body_diodes([[maybe_unused]] ::phy_engine::model::node_t* nd,
                                                 [[maybe_unused]] ::phy_engine::model::node_t* ns,
                                                 [[maybe_unused]] ::phy_engine::model::node_t* nb,
                                                 [[maybe_unused]] PN_junction& db,
                                                 [[maybe_unused]] PN_junction& sb) noexcept
        { attach_body_diodes<is_pmos>(nd, ns, nb, nb, db, sb); }

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

        // Stamp a general nonlinear current source I(Vd,Vg,Vs,Vb) flowing from terminal a -> terminal b.
        // Linearization uses: I ≈ dI/dVd*Vd + dI/dVg*Vg + dI/dVs*Vs + dI/dVb*Vb + Ieq.
        inline constexpr void stamp_current_4node(::phy_engine::MNA::MNA& mna,
                                                  ::phy_engine::model::node_t* a,
                                                  ::phy_engine::model::node_t* b,
                                                  ::phy_engine::model::node_t* nd,
                                                  ::phy_engine::model::node_t* ng,
                                                  ::phy_engine::model::node_t* ns,
                                                  ::phy_engine::model::node_t* nb,
                                                  double dId_dVd,
                                                  double dId_dVg,
                                                  double dId_dVs,
                                                  double dId_dVb,
                                                  double Ieq) noexcept
        {
            if(a == nullptr || b == nullptr || nd == nullptr || ng == nullptr || ns == nullptr || nb == nullptr) [[unlikely]] { return; }

            auto const ai{a->node_index};
            auto const bi{b->node_index};
            auto const di{nd->node_index};
            auto const gi{ng->node_index};
            auto const si{ns->node_index};
            auto const bi_ctrl{nb->node_index};

            // Row a (current leaving a)
            mna.G_ref(ai, di) += dId_dVd;
            mna.G_ref(ai, gi) += dId_dVg;
            mna.G_ref(ai, si) += dId_dVs;
            mna.G_ref(ai, bi_ctrl) += dId_dVb;
            mna.I_ref(ai) -= Ieq;

            // Row b (current entering b)
            mna.G_ref(bi, di) -= dId_dVd;
            mna.G_ref(bi, gi) -= dId_dVg;
            mna.G_ref(bi, si) -= dId_dVs;
            mna.G_ref(bi, bi_ctrl) -= dId_dVb;
            mna.I_ref(bi) += Ieq;
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

        inline constexpr void stamp_current_2node(::phy_engine::MNA::MNA& mna,
                                                  ::phy_engine::model::node_t* a,
                                                  ::phy_engine::model::node_t* b,
                                                  double dId_dVa,
                                                  double dId_dVb,
                                                  double Ieq) noexcept
        {
            if(a == nullptr || b == nullptr) [[unlikely]] { return; }

            auto const ai{a->node_index};
            auto const bi{b->node_index};

            // Row a (current leaving a)
            mna.G_ref(ai, ai) += dId_dVa;
            mna.G_ref(ai, bi) += dId_dVb;
            mna.I_ref(ai) -= Ieq;

            // Row b (current entering b)
            mna.G_ref(bi, ai) -= dId_dVa;
            mna.G_ref(bi, bi) -= dId_dVb;
            mna.I_ref(bi) += Ieq;
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

        inline double bsim3v32_barrier_v_temp_scale(double barrier_v, double temp_c, double tnom_c) noexcept
        {
            // First-order temperature scaling for "barrier-like" voltage parameters used inside
            // exp(-barrier/voltage) leakage expressions (GIDL/GISL, Ig*).
            //
            // Interpreting a voltage-like barrier as E/q, the exponential argument typically scales
            // with 1/T. To keep behavior anchored at TNOM:
            //   barrier(T) = barrier(Tnom) * (Tnom / T)
            double const t_k{temp_c + k_t0};
            double const tnom_k{tnom_c + k_t0};
            if(!(t_k > 1.0) || !(tnom_k > 1.0)) { return barrier_v; }
            return barrier_v * (tnom_k / t_k);
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

        inline double bsim3v32_exp(double x) noexcept { return limexp(x); }

        inline bsim3v32_dual3 bsim3v32_exp(bsim3v32_dual3 x) noexcept
        {
            // limexp-like behavior to avoid overflow while keeping a reasonable slope.
            // - x > 50: exp(50) * (1 + (x-50)), slope = exp(50)
            // - x < -50: exp(-50), slope = 0
            if(x.val > 50.0)
            {
                double const e50{::std::exp(50.0)};
                double const e{e50 * (1.0 + (x.val - 50.0))};
                return {e, e50 * x.dvgs, e50 * x.dvds, e50 * x.dvbs};
            }
            if(x.val < -50.0)
            {
                double const e{::std::exp(-50.0)};
                return {e, 0.0, 0.0, 0.0};
            }
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

        template <typename Real>
        inline Real bsim3v32_ueff_mobmod2(double u0, double ua, double ub, double uc, Real vgst_eff, Real vbs_eff, double tox) noexcept
        {
            // Clean-room mobMod=2 subset: similar to mobMod=3 but without the "+2*Vt" term in Eeff.
            // Uses the same (ua,ub,uc,tox) parameter units as the mobMod=3 path.
            if(u0 <= 0.0 || tox <= 0.0) { return Real{}; }
            Real const eeff{vgst_eff / tox};
            Real denom{1.0 + (ua * eeff + ub * eeff * eeff) * (1.0 + uc * vbs_eff)};
            if(bsim3v32_value(denom) <= 1e-18) { denom = 1e-18; }
            return u0 / denom;
        }

        template <typename Real>
        inline Real bsim3v32_ueff_mobmod1(double u0, double ua, double ub, double uc, Real vgst_eff, Real vbs_eff) noexcept
        {
            // Clean-room mobMod=1 subset: Vgsteff-based mobility degradation (no tox scaling).
            // Units:
            // - ua: 1/V, ub: 1/V^2, uc: 1/V
            if(u0 <= 0.0) { return Real{}; }
            Real denom{1.0 + ua * vgst_eff + ub * vgst_eff * vgst_eff + uc * vbs_eff};
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
            double ua_eff{details::bsim3v32_lw_scale(m.ua, m.lua, m.wua, m.pua, leff, weff, m.lref, m.wref)};
            double ub_eff{details::bsim3v32_lw_scale(m.ub, m.lub, m.wub, m.pub, leff, weff, m.lref, m.wref)};
            double uc_eff{details::bsim3v32_lw_scale(m.uc, m.luc, m.wuc, m.puc, leff, weff, m.lref, m.wref)};
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
            double const dt_c{m.Temp - m.tnom};
            if(u0 > 0.0 && m.ute != 0.0 && t_k > 1.0 && tnom_k > 1.0)
            {
                double const ratio{t_k / tnom_k};
                // mobility ~ (T/Tnom)^(-ute)
                u0 *= ::std::pow(ratio, -m.ute);
            }
            // Temperature dependence of mobility-degradation coefficients (clean-room subset).
            if(dt_c != 0.0)
            {
                ua_eff += m.ua1 * dt_c;
                ub_eff += m.ub1 * dt_c;
                uc_eff += m.uc1 * dt_c;
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
            double const vth0_t{bsim3v32_vth0_temp_mag(vth0_eff_geom, m.Temp, m.tnom, m.kt1, m.kt2)};
            double const vth0ox{vth0_t - k1_eff * sqrt_phi};

            Real const sqrt_phi_vbs{bsim3v32_sqrt(bsim3v32_max(phi_s - vbseff, 1e-12))};

            // Xdep / lt / lt0 (simplified)
            double const nch{nch_eff > 1.0 ? nch_eff : 1e23};  // 1/m^3
            Real const xdep{bsim3v32_sqrt(2.0 * k_eps_si * bsim3v32_max(phi_s - vbseff, 1e-12) / (k_q * nch))};
            double const xdep0{::std::sqrt(2.0 * k_eps_si * phi_s / (k_q * nch))};
            double const xj_eff{::std::max(m.xj, 0.0)};
            double const lt0{::std::sqrt((k_eps_si / k_eps_ox) * tox * (xj_eff > 0.0 ? xj_eff : xdep0))};
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
            n *= (1.0 + (m.noff > 0.0 ? m.noff : 0.0));
            n += (cd + cit_eff) / cox;
            if(bsim3v32_value(n) < 1.0) { n = 1.0; }

            // Effective (Vgs - Vth - Voff)
            double const voff_eff{details::bsim3v32_lw_scale(m.voff, m.lvoff, m.wvoff, m.pvoff, leff, weff, m.lref, m.wref)};
            Real const vgst{vgs - vth - voff_eff};
            Real const vgsteff{bsim3v32_vgsteff(vgst, n, vt)};
            if(cache) { cache->vgsteff = vgsteff; }

            // Subthreshold is covered by Vgsteff itself (Eq. 3.1.3), so we do not add a separate
            // empirical subthreshold current branch here. This keeps Id(V) and its derivatives
            // smooth and consistent with the charge-based C/V implementation.

            // Mobility (mobMod=3 default)
            Real ueff{};
            if(m.mobMod < 0.5)
            {
                // mobMod==0: constant mobility
                ueff = u0;
            }
            else if(m.mobMod < 1.5)
            {
                // mobMod==1: Vgsteff-based degradation (clean-room subset).
                ueff = bsim3v32_ueff_mobmod1(u0, ua_eff, ub_eff, uc_eff, vgsteff, vbseff);
                if(bsim3v32_value(ueff) <= 0.0) { ueff = u0; }
            }
            else if(m.mobMod < 2.5)
            {
                // mobMod==2: Eeff=Vgsteff/tox based degradation (clean-room subset).
                ueff = bsim3v32_ueff_mobmod2(u0, ua_eff, ub_eff, uc_eff, vgsteff, vbseff, tox);
                if(bsim3v32_value(ueff) <= 0.0) { ueff = u0; }
            }
            else
            {
                // mobMod==3+: default to the mobMod=3 style model (clean-room subset).
                ueff = bsim3v32_ueff_mobmod3(u0, ua_eff, ub_eff, uc_eff, vgsteff, vbseff, tox, vt);
                if(bsim3v32_value(ueff) <= 0.0) { ueff = u0; }
            }

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

            Real rds_eff{};
            bool const rds_enabled{m.rdsMod != 0.0};
            if(rds_enabled) { rds_eff = ::std::max(m.rds, 0.0); }
            double rdsw_eff_geom{details::bsim3v32_lw_scale(m.rdsw, m.lrdsw, m.wrdsw, m.prdsw, leff, weff, m.lref, m.wref)};
            if(rdsw_eff_geom > 0.0 && dt_c != 0.0 && m.prt != 0.0)
            {
                rdsw_eff_geom *= (1.0 + m.prt * dt_c);
                if(!(rdsw_eff_geom > 0.0)) { rdsw_eff_geom = 0.0; }
            }
            if(rds_enabled && rdsw_eff_geom > 0.0)
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
            return ids_inv;
        }

        template <typename Real>
        inline Real bsim3v32_pos_smooth(Real x) noexcept
        {
            Real const ax{bsim3v32_abs_smooth(x)};
            return 0.5 * (x + ax);
        }

        template <typename Mos, typename Real>
        inline Real bsim3v32_gidl_drain_s(Mos const& m, Real vgs_s, Real vds_s, Real vbs_s) noexcept
        {
            // Clean-room GIDL model (subset).
            // Current flows from drain diffusion -> bulk in the signed (n-type) coordinate.
            //
            // Effective fields:
            //   Vdg = max(Vd - Vg - egidl, 0), Vdb = max(Vd - Vb, 0)
            //
            // I_gidl ≈ agidl * Weff * Vdb * exp(-bgidl / (Vdg + cgidl))
            double const weff{::std::max(m.W - 2.0 * ::std::max(m.dwc, 0.0), 0.0)};
            if(!(weff > 0.0)) { return Real{}; }
            if(!(m.agidl > 0.0) || !(m.bgidl > 0.0)) { return Real{}; }

            Real const vgd_s{vgs_s - vds_s};  // Vg - Vd (signed)
            Real const vbd_s{vbs_s - vds_s};  // Vb - Vd (signed)
            Real const vdg_eff{bsim3v32_pos_smooth(-vgd_s - m.egidl)};
            Real const vdb_eff{bsim3v32_pos_smooth(-vbd_s)};

            Real denom{vdg_eff + m.cgidl};
            if(bsim3v32_value(denom) <= 1e-12) { denom = Real{1e-12}; }
            double const bg_eff{bsim3v32_barrier_v_temp_scale(m.bgidl, m.Temp, m.tnom)};
            Real const expo{bsim3v32_exp(Real{-bg_eff} / denom)};
            return Real{m.agidl * weff} * vdb_eff * expo;
        }

        template <typename Mos, typename Real>
        inline Real bsim3v32_gidl_source_s(Mos const& m, Real vgs_s, Real /*vds_s*/, Real vbs_s) noexcept
        {
            // Clean-room GISL model (subset).
            // Current flows from source diffusion -> bulk in the signed (n-type) coordinate.
            double const weff{::std::max(m.W - 2.0 * ::std::max(m.dwc, 0.0), 0.0)};
            if(!(weff > 0.0)) { return Real{}; }

            // If explicit GISL params are unset, fall back to GIDL params.
            double const ag{(m.agisl > 0.0) ? m.agisl : m.agidl};
            double const bg{(m.bgisl > 0.0) ? m.bgisl : m.bgidl};
            double const cg{(m.cgisl >= 0.0) ? m.cgisl : m.cgidl};
            double const eg{(m.egisl >= 0.0) ? m.egisl : m.egidl};
            if(!(ag > 0.0) || !(bg > 0.0)) { return Real{}; }

            Real const vsg_eff{bsim3v32_pos_smooth(-vgs_s - eg)};  // Vs - Vg - egisl
            Real const vsb_eff{bsim3v32_pos_smooth(-vbs_s)};       // Vs - Vb

            Real denom{vsg_eff + cg};
            if(bsim3v32_value(denom) <= 1e-12) { denom = Real{1e-12}; }
            double const bg_eff{bsim3v32_barrier_v_temp_scale(bg, m.Temp, m.tnom)};
            Real const expo{bsim3v32_exp(Real{-bg_eff} / denom)};
            return Real{ag * weff} * vsb_eff * expo;
        }

        template <typename Mos, typename Real>
        inline Real bsim3v32_igb_s(Mos const& m, Real vgb_s) noexcept
        {
            // Clean-room simplified gate-to-bulk leakage (subset):
            // Ig (G -> B) ≈ aigb * Weff * Leff * Vgb^2 * exp(-bigb / (Vgb + cigb)),
            // where Vgb = max(vgb_s - eigb, 0).
            if(!(m.aigb > 0.0)) { return Real{}; }
            double const b{m.bigb};
            if(!(b > 0.0)) { return Real{}; }
            double const weff{::std::max(m.W - 2.0 * ::std::max(m.dwc, 0.0), 0.0)};
            double const leff{::std::max(m.L - 2.0 * ::std::max(m.dlc, 0.0), 1e-18)};
            if(!(weff > 0.0) || !(leff > 0.0)) { return Real{}; }

            Real const vgb_eff{bsim3v32_pos_smooth(vgb_s - m.eigb)};
            Real denom{vgb_eff + m.cigb};
            if(bsim3v32_value(denom) <= 1e-12) { denom = Real{1e-12}; }
            double const b_eff{bsim3v32_barrier_v_temp_scale(b, m.Temp, m.tnom)};
            Real const expo{bsim3v32_exp(Real{-b_eff} / denom)};
            return Real{m.aigb * weff * leff} * vgb_eff * vgb_eff * expo;
        }

        template <typename Mos, typename Real>
        inline Real bsim3v32_igs_s(Mos const& m, Real vgs_s) noexcept
        {
            // Gate-to-source leakage (subset): require the branch amplitude aigs explicitly enabled.
            // If bigs/cigs/eigs are unset, fall back to the corresponding IGB parameters (bigb/cigb/eigb).
            if(!(m.aigs > 0.0)) { return Real{}; }
            double const b{(m.bigs > 0.0) ? m.bigs : m.bigb};
            double const c{(m.cigs != 0.0) ? m.cigs : m.cigb};
            double const e{(m.eigs != 0.0) ? m.eigs : m.eigb};
            if(!(b > 0.0)) { return Real{}; }
            double const weff{::std::max(m.W - 2.0 * ::std::max(m.dwc, 0.0), 0.0)};
            double const leff{::std::max(m.L - 2.0 * ::std::max(m.dlc, 0.0), 1e-18)};
            if(!(weff > 0.0) || !(leff > 0.0)) { return Real{}; }

            Real const vgs_eff{bsim3v32_pos_smooth(vgs_s - e)};
            Real denom{vgs_eff + c};
            if(bsim3v32_value(denom) <= 1e-12) { denom = Real{1e-12}; }
            double const b_eff{bsim3v32_barrier_v_temp_scale(b, m.Temp, m.tnom)};
            Real const expo{bsim3v32_exp(Real{-b_eff} / denom)};
            return Real{m.aigs * weff * leff} * vgs_eff * vgs_eff * expo;
        }

        template <typename Mos, typename Real>
        inline Real bsim3v32_igd_s(Mos const& m, Real vgd_s) noexcept
        {
            // Gate-to-drain leakage (subset): require the branch amplitude aigd explicitly enabled.
            // If bigd/cigd/eigd are unset, fall back to the corresponding IGB parameters (bigb/cigb/eigb).
            if(!(m.aigd > 0.0)) { return Real{}; }
            double const b{(m.bigd > 0.0) ? m.bigd : m.bigb};
            double const c{(m.cigd != 0.0) ? m.cigd : m.cigb};
            double const e{(m.eigd != 0.0) ? m.eigd : m.eigb};
            if(!(b > 0.0)) { return Real{}; }
            double const weff{::std::max(m.W - 2.0 * ::std::max(m.dwc, 0.0), 0.0)};
            double const leff{::std::max(m.L - 2.0 * ::std::max(m.dlc, 0.0), 1e-18)};
            if(!(weff > 0.0) || !(leff > 0.0)) { return Real{}; }

            Real const vgd_eff{bsim3v32_pos_smooth(vgd_s - e)};
            Real denom{vgd_eff + c};
            if(bsim3v32_value(denom) <= 1e-12) { denom = Real{1e-12}; }
            double const b_eff{bsim3v32_barrier_v_temp_scale(b, m.Temp, m.tnom)};
            Real const expo{bsim3v32_exp(Real{-b_eff} / denom)};
            return Real{m.aigd * weff * leff} * vgd_eff * vgd_eff * expo;
        }

        template <typename Mos, typename Real>
        inline Real bsim3v32_impact_ionization_s(Mos const& m, Real ids_s, Real vds_s) noexcept
        {
            // Clean-room impact ionization / substrate current model (subset).
            //
            // Iii is modeled as a drain-to-bulk current source proportional to channel current magnitude.
            // A simple smooth form:
            //   vds_eff = max(Vds - vdsatii, 0)
            //   Iii ≈ |Ids| * alpha0 * vds_eff * exp(-beta0 / vds_eff)
            //
            // alpha0 [1/V], beta0 [V], vdsatii [V]
            if(!(m.alpha0 > 0.0) || !(m.beta0 > 0.0)) { return Real{}; }

            Real const vds_eff{bsim3v32_pos_smooth(vds_s - m.vdsatii)};
            if(!(bsim3v32_value(vds_eff) > 0.0)) { return Real{}; }

            Real denom{vds_eff};
            if(bsim3v32_value(denom) <= 1e-12) { denom = Real{1e-12}; }
            double const beta_eff{bsim3v32_barrier_v_temp_scale(m.beta0, m.Temp, m.tnom)};
            Real const expo{bsim3v32_exp(Real{-beta_eff} / denom)};
            Real const ids_mag{bsim3v32_abs_smooth(ids_s)};
            return Real{m.alpha0} * ids_mag * vds_eff * expo;
        }

        template <bool is_pmos, typename Mos>
        inline void
            bsim3v32_meyer_intrinsic_caps(Mos const& m, double vgs_s, double vds_s, double vbs_s, double vt, double& cgs, double& cgd, double& cgb) noexcept
        {
            // Clean-room "Meyer-style" intrinsic caps computed from the intrinsic charge model:
            //   Cgs = -∂Qg/∂Vs, Cgd = -∂Qg/∂Vd, Cgb = -∂Qg/∂Vb
            // This produces a bias-dependent, smooth small-signal C model that is consistent with the
            // charge-conserving intrinsic Q(V) implementation used by capMod>=3.
            //
            // NOTE: This is still a stepping-stone; full BSIM3v3.2 capMod behavior is more detailed.

            cgs = 0.0;
            cgd = 0.0;
            cgb = 0.0;

            double const sgn{is_pmos ? -1.0 : 1.0};
            auto const q =
                bsim3v32_intrinsic_charges_capmod0_simple_s<is_pmos>(m, bsim3v32_dual3_vgs(vgs_s), bsim3v32_dual3_vds(vds_s), bsim3v32_dual3_vbs(vbs_s), vt);

            // Qg derivatives are w.r.t. signed variables (vgs_s, vds_s, vbs_s).
            // Map to terminal derivatives (Vd,Vg,Vs,Vb) using:
            //   vgs_s = sgn*(Vg-Vs), vds_s = sgn*(Vd-Vs), vbs_s = sgn*(Vb-Vs)
            auto const& qg = q.q[1];
            double const dQg_dvgs{qg.dvgs};
            double const dQg_dvds{qg.dvds};
            double const dQg_dvbs{qg.dvbs};

            double const dQg_dVd{sgn * dQg_dvds};
            double const dQg_dVb{sgn * dQg_dvbs};
            double const dQg_dVs{-sgn * (dQg_dvgs + dQg_dvds + dQg_dvbs)};

            cgd = -dQg_dVd;
            cgb = -dQg_dVb;
            cgs = -dQg_dVs;
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
            double const nch_eff_raw{details::bsim3v32_lw_scale(m.nch, m.lnch, m.wnch, m.pnch, bsim3v32_value(c.leff), bsim3v32_value(c.weff), m.lref, m.wref)};
            double const nch_eff{nch_eff_raw > 1.0 ? nch_eff_raw : (m.nch > 1.0 ? m.nch : 1e23)};
            double const phi0_eff_geom{
                details::bsim3v32_lw_scale(m.phi, m.lphi, m.wphi, m.pphi, bsim3v32_value(c.leff), bsim3v32_value(c.weff), m.lref, m.wref)};
            double const phi_s{bsim3v32_phi_temp(phi0_eff_geom, nch_eff, m.Temp, m.tnom)};
            double const tox{m.tox > 0.0 ? m.tox : 1e-8};
            double const toxm{m.toxm > 0.0 ? m.toxm : tox};
            double const tox_ratio{tox / toxm};

            double const gamma_eff_raw{
                details::bsim3v32_lw_scale(m.gamma, m.lgamma, m.wgamma, m.pgamma, bsim3v32_value(c.leff), bsim3v32_value(c.weff), m.lref, m.wref)};
            double const gamma_eff{gamma_eff_raw > 0.0 ? gamma_eff_raw : 0.0};
            double const k1_base{(m.k1 != 0.0) ? m.k1 : gamma_eff};
            double const k1_eff{details::bsim3v32_lw_scale(k1_base, m.lk1, m.wk1, m.pk1, bsim3v32_value(c.leff), bsim3v32_value(c.weff), m.lref, m.wref)};
            double const k2_eff{details::bsim3v32_lw_scale(m.k2, m.lk2, m.wk2, m.pk2, bsim3v32_value(c.leff), bsim3v32_value(c.weff), m.lref, m.wref)};
            double const k1ox{k1_eff * tox_ratio};
            double const k2ox{k2_eff * tox_ratio};

            double const vbm{(m.vbm < 0.0) ? m.vbm : -3.0};
            double const delta1{(m.delta1 > 0.0) ? m.delta1 : 1e-3};
            Real const vbseff{bsim3v32_vbseff(vbs_s, vbm, delta1)};
            double const sqrt_phi{::std::sqrt(phi_s)};
            Real const sqrt_phi_vbs{bsim3v32_sqrt(bsim3v32_max(phi_s - vbseff, 1e-12))};

            // Bulk depletion charge (clean-room approximation).
            // Include both k1 and k2 body-effect terms to better align Qb with the Vth(Vbs) formulation.
            Real const qb_n{coxwl * (Real{k1ox} * (sqrt_phi_vbs - sqrt_phi) - Real{k2ox} * vbseff)};

            // Inversion charge (clean-room, long-channel baseline), with a smooth linear/saturation blend.
            // - Linear:  Qinv ≈ -CoxWL*(Vgsteff - Abulk*Vdseff/2)
            // - Saturation: Qinv ≈ -(2/3)*CoxWL*Vgsteff
            double const keta_eff{
                details::bsim3v32_lw_scale(m.keta, m.lketa, m.wketa, m.pketa, bsim3v32_value(c.leff), bsim3v32_value(c.weff), m.lref, m.wref)};
            Real const abulk{1.0 + keta_eff * vbseff};

            // Use a CV-specific Vgsteff based on voffcv when provided; this keeps I–V and C–V controls separated
            // (voff affects DC, voffcv affects C/V), which is important for BSIM-style fitting workflows.
            //
            // Recompute n-factor locally (it depends on vbseff via xdep) so vgsteff uses the correct slope.
            double const voff_eff{
                details::bsim3v32_lw_scale(m.voff, m.lvoff, m.wvoff, m.pvoff, bsim3v32_value(c.leff), bsim3v32_value(c.weff), m.lref, m.wref)};
            double const voffcv_eff{
                ::std::isfinite(m.voffcv)
                    ? details::bsim3v32_lw_scale(m.voffcv, m.lvoffcv, m.wvoffcv, m.pvoffcv, bsim3v32_value(c.leff), bsim3v32_value(c.weff), m.lref, m.wref)
                    : voff_eff};
            Real const xdep{bsim3v32_sqrt(2.0 * k_eps_si * bsim3v32_max(phi_s - vbseff, 1e-12) / (k_q * nch_eff))};
            Real const cd{k_eps_si / bsim3v32_max(xdep, 1e-18)};
            double const nfactor_eff_raw{
                details::bsim3v32_lw_scale(m.nfactor, m.lnfactor, m.wnfactor, m.pnfactor, bsim3v32_value(c.leff), bsim3v32_value(c.weff), m.lref, m.wref)};
            double const cit_eff{details::bsim3v32_lw_scale(m.cit, m.lcit, m.wcit, m.pcit, bsim3v32_value(c.leff), bsim3v32_value(c.weff), m.lref, m.wref)};
            Real n{1.0 + (nfactor_eff_raw > 0.0 ? nfactor_eff_raw : 0.0)};
            n *= (1.0 + (m.noff > 0.0 ? m.noff : 0.0));
            n += (cd + cit_eff) / c.cox;
            if(bsim3v32_value(n) < 1.0) { n = 1.0; }

            Real const vgst_cv{vgs_s - c.vth - voffcv_eff};
            Real const vgsteff_cv{bsim3v32_vgsteff(vgst_cv, n, vt)};

            // For C/V, derive the saturation boundary from the same (clean-room) velocity-sat + mobility model
            // used by the DC core, but evaluated at the CV-effective inversion charge (vgsteff_cv).
            // This avoids relying on scaled DC caches and makes the charge Jacobian more self-consistent.
            Real const vds_pos{bsim3v32_pos_smooth(vds_s)};

            // Mobility for the C/V charge model (re-evaluated with vgsteff_cv and vbseff).
            double u0{details::bsim3v32_lw_scale(m.u0, m.lu0, m.wu0, m.pu0, bsim3v32_value(c.leff), bsim3v32_value(c.weff), m.lref, m.wref)};
            double const kp_eff_geom{details::bsim3v32_lw_scale(m.Kp, m.lkp, m.wkp, m.pkp, bsim3v32_value(c.leff), bsim3v32_value(c.weff), m.lref, m.wref)};
            if(u0 <= 0.0 && kp_eff_geom > 0.0) { u0 = kp_eff_geom / bsim3v32_value(c.cox); }

            // Temperature scaling (subset, consistent with DC core).
            double const t_k{m.Temp + k_t0};
            double const tnom_k{m.tnom + k_t0};
            double const dt_c{m.Temp - m.tnom};
            if(u0 > 0.0 && m.ute != 0.0 && t_k > 1.0 && tnom_k > 1.0)
            {
                double const ratio{t_k / tnom_k};
                u0 *= ::std::pow(ratio, -m.ute);
            }

            double ua_eff{details::bsim3v32_lw_scale(m.ua, m.lua, m.wua, m.pua, bsim3v32_value(c.leff), bsim3v32_value(c.weff), m.lref, m.wref)};
            double ub_eff{details::bsim3v32_lw_scale(m.ub, m.lub, m.wub, m.pub, bsim3v32_value(c.leff), bsim3v32_value(c.weff), m.lref, m.wref)};
            double uc_eff{details::bsim3v32_lw_scale(m.uc, m.luc, m.wuc, m.puc, bsim3v32_value(c.leff), bsim3v32_value(c.weff), m.lref, m.wref)};
            if(dt_c != 0.0)
            {
                ua_eff += m.ua1 * dt_c;
                ub_eff += m.ub1 * dt_c;
                uc_eff += m.uc1 * dt_c;
            }
            Real ueff_cv{};
            if(m.mobMod < 1.5)
            {
                ueff_cv = bsim3v32_ueff_mobmod1(u0, ua_eff, ub_eff, uc_eff, vgsteff_cv, vbseff);
                if(bsim3v32_value(ueff_cv) <= 0.0) { ueff_cv = Real{u0}; }
            }
            else if(m.mobMod < 2.5)
            {
                double const tox{m.tox > 0.0 ? m.tox : 1e-8};
                ueff_cv = bsim3v32_ueff_mobmod2(u0, ua_eff, ub_eff, uc_eff, vgsteff_cv, vbseff, tox);
                if(bsim3v32_value(ueff_cv) <= 0.0) { ueff_cv = Real{u0}; }
            }
            else
            {
                double const tox{m.tox > 0.0 ? m.tox : 1e-8};
                ueff_cv = bsim3v32_ueff_mobmod3(u0, ua_eff, ub_eff, uc_eff, vgsteff_cv, vbseff, tox, vt);
                if(bsim3v32_value(ueff_cv) <= 0.0) { ueff_cv = Real{u0}; }
            }

            // Velocity saturation and Vdsat/Vdseff (same functional form as DC core).
            double const vsat_eff_geom{
                details::bsim3v32_lw_scale(m.vsat, m.lvsat, m.wvsat, m.pvsat, bsim3v32_value(c.leff), bsim3v32_value(c.weff), m.lref, m.wref)};
            double vsat{vsat_eff_geom > 0.0 ? vsat_eff_geom : 8e4};
            if(m.at != 0.0)
            {
                vsat *= (1.0 + m.at * dt_c);
                if(vsat <= 1.0) { vsat = 1.0; }
            }
            Real const esat{2.0 * Real{vsat} / bsim3v32_max(ueff_cv, 1e-18)};

            Real const vdsat_cv_raw{vgsteff_cv / (abulk + vgsteff_cv / bsim3v32_max(esat * c.leff, 1e-18))};
            Real const vdsat_cv{bsim3v32_pos_smooth(vdsat_cv_raw)};

            double const delta{(m.delta > 0.0) ? m.delta : 1e-2};
            Real const t1{vdsat_cv - vds_pos - Real{delta}};
            Real const vdseff_cv_raw{vdsat_cv - 0.5 * (t1 + bsim3v32_sqrt(t1 * t1 + 4.0 * Real{delta} * vdsat_cv))};
            Real const vdseff_cv_pos{bsim3v32_pos_smooth(vdseff_cv_raw)};
            Real const dv{vdseff_cv_pos - vds_pos};
            Real const vdseff_cv{0.5 * (vdseff_cv_pos + vds_pos - bsim3v32_abs_smooth(dv))};

            // Smooth linear/saturation blending around Vdsat to keep C(V) continuous.
            Real const s_reg{vds_pos - vdsat_cv};
            double const vds_smooth{(m.delta > 0.0) ? m.delta : 1e-2};
            Real const denom{bsim3v32_sqrt(s_reg * s_reg + Real{vds_smooth} * Real{vds_smooth})};
            Real const f_sat{0.5 * (1.0 + s_reg / bsim3v32_max(denom, 1e-24))};  // 0->linear, 1->sat

            Real const qinv_lin{-coxwl * (vgsteff_cv - abulk * vdseff_cv / 2.0)};
            Real const qinv_sat{-(2.0 / 3.0) * coxwl * vgsteff_cv};
            Real const qinv_n{(1.0 - f_sat) * qinv_lin + f_sat * qinv_sat};

            // Channel inversion charge partition (charge-conserving), smoothly blended across Vdsat.
            // - Linear region: Ward–Dutton partition (bias-dependent).
            // - Saturation: xpart selects 0/100, 50/50, or 40/60 partition.
            Real const qd_lin{-coxwl * (0.5 * vgsteff_cv - (abulk * vdseff_cv) / 3.0)};

            // Saturation partition (BSIM3 xpart):
            // - xpart=0   => 0/100 (all inversion charge to source)
            // - xpart=0.5 => 50/50
            // - xpart=1   => 40/60
            // For intermediate values, use a continuous interpolation that preserves these anchors.
            double const xp_clamped{::std::clamp(m.xpart, 0.0, 1.0)};
            double frac_d{};
            if(xp_clamped <= 0.5) { frac_d = xp_clamped; }
            else
            {
                frac_d = 0.6 - 0.2 * xp_clamped;
            }  // 0.5->0.5, 1.0->0.4
            Real const qd_sat{frac_d * qinv_sat};

            Real const qd_n{(1.0 - f_sat) * qd_lin + f_sat * qd_sat};
            Real const qs_n{qinv_n - qd_n};

            Real const qg_n{-(qinv_n + qb_n)};
            Real qb_adj_n{qb_n};

            // Depletion/accumulation gate-bulk charge (clean-room simplified):
            // - Add an extra G-B term in depletion to avoid near-zero Cgb in cutoff when using the charge-based path.
            // - Keep it smoothly disabled in strong inversion based on Vgs-Vth (signed coordinate).
            //
            // Approximate flatband as Vfb ≈ Vth0(T) - phi_s.
            // Keep geometry scaling consistent with the DC core.
            double const vth0_eff_geom{
                details::bsim3v32_lw_scale(m.Vth0, m.lvth0, m.wvth0, m.pvth0, bsim3v32_value(c.leff), bsim3v32_value(c.weff), m.lref, m.wref)};
            double const vth0_t{bsim3v32_vth0_temp_mag(vth0_eff_geom, m.Temp, m.tnom, m.kt1, m.kt2)};
            double const vfb{::std::isfinite(m.vfbcv) ? m.vfbcv : (vth0_t - phi_s)};
            Real const vgb{vgs_s - vbs_s};
            Real const x{vgb - vfb};
            Real const abs_x{bsim3v32_abs_smooth(x)};
            // min(x,0) and max(x,0) with smooth abs to keep derivatives continuous.
            Real const minx{0.5 * (x - abs_x)};
            Real const maxx{0.5 * (x + abs_x)};

            // Smooth cutoff factor based on Vgs-Vth (signed coordinate from DC cache).
            Real const vgst{vgs_s - c.vth - voffcv_eff};
            Real const abs_vgst{bsim3v32_abs_smooth(vgst)};
            Real const f_cut{0.5 * (1.0 - vgst / bsim3v32_max(abs_vgst, 1e-24))};

            // Use Cox in accumulation, and an effective (Cox||Cdep) in depletion to mimic MOS C-V behavior.
            // This makes CV parameters like vfbcv observable in small-signal analyses.
            Real const cdep_per_area{k_eps_si / bsim3v32_max(xdep, 1e-18)};
            Real const cdep_wl{coxwl * (cdep_per_area / bsim3v32_max(c.cox + cdep_per_area, 1e-24))};

            Real const qacc_g{coxwl * minx};            // <= 0 in accumulation (NMOS)
            Real const qdep_g{cdep_wl * maxx * f_cut};  // >= 0 in depletion/cutoff; fades out in inversion

            qb_adj_n += -(qacc_g + qdep_g);
            Real const qg_adj{qg_n + qacc_g + qdep_g};

            bsim3v32_charge_vec4_t<Real> q{};
            q.q[0] = pol * qd_n;
            q.q[1] = pol * qg_adj;
            q.q[2] = pol * qs_n;
            q.q[3] = pol * qb_adj_n;

            // Overlap charges (acm != 0): model overlap capacitors as charge terms so they are handled
            // consistently by both the C-matrix path (capMod>=3) and the Meyer path (capMod<3).
            //
            // Capacitor between nodes (G,X):
            //   Qg += C * (Vg - Vx),  Qx -= C * (Vg - Vx)
            if(m.acm != 0.0)
            {
                double const weff{::std::max(m.W - 2.0 * ::std::max(m.dwc, 0.0), 0.0)};
                double const leff{::std::max(m.L - 2.0 * ::std::max(m.dlc, 0.0), 1e-18)};
                double const cgs_ovl{m.cgso * weff};
                double const cgd_ovl{m.cgdo * weff};
                double const cgb_ovl{m.cgbo * leff};

                // Map signed variables to physical voltages:
                //   vgs_s = pol*(Vg-Vs), vds_s = pol*(Vd-Vs), vbs_s = pol*(Vb-Vs)
                Real const Vgs{Real{pol} * vgs_s};
                Real const Vgd{Real{pol} * (vgs_s - vds_s)};
                Real const Vgb{Real{pol} * (vgs_s - vbs_s)};

                Real const qg_ovl{Real{cgs_ovl} * Vgs + Real{cgd_ovl} * Vgd + Real{cgb_ovl} * Vgb};
                Real const qd_ovl{-Real{cgd_ovl} * Vgd};
                Real const qs_ovl{-Real{cgs_ovl} * Vgs};
                Real const qb_ovl{-Real{cgb_ovl} * Vgb};

                q.q[0] += qd_ovl;
                q.q[1] += qg_ovl;
                q.q[2] += qs_ovl;
                q.q[3] += qb_ovl;
            }
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

        inline constexpr void add_linear_cap_to_cmat(double cmat[4][4], int a, int b, double c) noexcept
        {
            // Add a two-terminal capacitor between nodes a and b into the charge Jacobian matrix:
            // Q_a += C*(Va - Vb), Q_b -= C*(Va - Vb)
            if(!(c > 0.0)) { return; }
            cmat[a][a] += c;
            cmat[a][b] -= c;
            cmat[b][a] -= c;
            cmat[b][b] += c;
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
        // Optional bulk series resistance (Ohm). When >0 and bulk pin is connected, use internal B' node.
        double Rb{0.0};
        // Optional distributed body resistances (Ohm, clean-room simplified BSIM3-style subset):
        // When >0 and bulk pin is connected, use additional internal nodes near each junction:
        // - rbdb: between B' and the drain-body diode's local body node (B'd)
        // - rbsb: between B' and the source-body diode's local body node (B's)
        double rbdb{0.0};
        double rbsb{0.0};
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
        // Resistance model selectors (clean-room subset, BSIM-like naming):
        // - rgateMod: when 0, disable rg even if rg>0; otherwise enable.
        // - rbodyMod: when 0, disable rbdb/rbsb distributed body resistances; otherwise enable.
        double rgateMod{1.0};
        double rbodyMod{1.0};

        // Charge/capacitance model controls (BSIM3 names).
        // Note: BSIM3/NGSPICE default capMod is typically 3. For now:
        // - capMod < 2.5: use a Meyer-style intrinsic C model (capMod=0/1/2)
        // - capMod >= 2.5: use a charge-based intrinsic C-matrix (capMod=3), clean-room simplified, WIP
        double capMod{3.0};
        double xpart{0.0};  // 0.0=0/100, 0.5=50/50, 1.0=40/60
        double mobMod{3.0};
        double vfbcv{::std::numeric_limits<double>::quiet_NaN()};
        double acm{0.0};  // overlap charge mode (clean-room subset): 0=fixed caps, !=0=charge-based (double-counting avoided)

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
        double lvoffcv{0.0};
        double wvoffcv{0.0};
        double pvoffcv{0.0};
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
        double xj{0.0};      // m (junction depth, used for short-channel/DIBL scaling; 0 -> derived fallback)
        double nch{1.7e23};  // 1/m^3 (channel doping)
        double u0{0.0};      // m^2/Vs (if 0, derived from Kp/Cox)
        double ua{0.0};
        double ub{0.0};
        double uc{0.0};
        // Temperature dependence (clean-room subset, BSIM-like naming):
        // ua/ub/uc are mobility-degradation coefficients; ua1/ub1/uc1 are their linear temperature slopes around TNOM.
        double ua1{0.0};   // 1/(V*°C)
        double ub1{0.0};   // 1/(V*°C^2) in some references; treated here as linear slope in °C
        double uc1{0.0};   // 1/(V*°C)
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
        double noff{0.0};  // clean-room subset: additional subthreshold slope factor
        double cit{0.0};
        double voff{0.0};
        double voffcv{::std::numeric_limits<double>::quiet_NaN()};

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
        // rdsMod: when 0, disable the internal Rds model (rds/rdsw/prwg/prwb).
        double rdsMod{1.0};
        double rds{0.0};  // internal Rds (Ohm), used in Ids expression
                          // BSIM3 Rds model (clean-room subset):
        // - rdsw: source/drain resistance per effective width (Ohm*m). Effective Rds adds rdsw/Weff.
        // - prwg/prwb: bias dependence vs Vgsteff and Vbseff.
        // - prt: linear temperature coefficient for rdsw around TNOM (rdsw(T) ~= rdsw(Tnom)*(1+prt*(T-Tnom))).
        double rdsw{0.0};  // Ohm*m
        double prwg{0.0};  // 1/V
        double prwb{0.0};  // 1/V
        double prt{0.0};   // 1/°C

        double keta{0.0};

        // Gate-induced drain/source leakage (GIDL/GISL) (clean-room subset).
        // Units:
        // - agidl/agisl: A/(m*V) (scaled by Weff and multiplied by Vdb/Vsb)
        // - bgidl/bgisl: V
        // - cgidl/cgisl: V
        // - egidl/egisl: V
        double agidl{0.0};
        double bgidl{0.0};
        double cgidl{0.0};
        double egidl{0.0};
        double agisl{0.0};
        double bgisl{0.0};
        double cgisl{-1.0};  // <0 => fall back to cgidl
        double egisl{-1.0};  // <0 => fall back to egidl

        // Gate-to-bulk leakage (clean-room simplified subset).
        // Units:
        // - aigb: A/V^2 scaled by Weff*Leff
        // - bigb: V
        // - cigb: V
        // - eigb: V (threshold shift)
        double aigb{0.0};
        double bigb{0.0};
        double cigb{0.0};
        double eigb{0.0};

        // Gate-to-source leakage (clean-room simplified subset).
        double aigs{0.0};
        double bigs{0.0};
        double cigs{0.0};
        double eigs{0.0};

        // Gate-to-drain leakage (clean-room simplified subset).
        double aigd{0.0};
        double bigd{0.0};
        double cigd{0.0};
        double eigd{0.0};

        // Impact ionization / substrate current (clean-room subset).
        // alpha0: 1/V, beta0: V, vdsatii: V
        double alpha0{0.0};
        double beta0{0.0};
        double vdsatii{0.0};

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
        // Optional per-junction diode overrides (clean-room subset).
        // Semantics:
        // - For Is/Isr/tt: negative => inherit the common parameter (diode_Is/diode_Isr/tt).
        // - For N/Nr:      <=0 => inherit the common parameter (diode_N/diode_Nr).
        double diode_Isd{-1.0};   // A
        double diode_Iss{-1.0};   // A
        double diode_Nd{-1.0};    // unitless
        double diode_Ns{-1.0};    // unitless
        double diode_Isrd{-1.0};  // A
        double diode_Isrs{-1.0};  // A
        double diode_Nrd{-1.0};   // unitless
        double diode_Nrs{-1.0};   // unitless
        // Optional recombination current (clean-room subset, forwarded to PN_junction Isr/Nr).
        double diode_Isr{0.0};
        double diode_Nr{2.0};
        // Junction breakdown parameters (clean-room subset, forwarded to the internal PN_junction models).
        // Set Bv<=0 or Ibv<=0 to disable breakdown on that junction.
        double bvd{40.0};
        double ibvd{1e-3};
        double bvs{40.0};
        double ibvs{1e-3};
        // Junction transit time (seconds): enables diffusion capacitance in the internal body diodes (tt * dId/dV).
        double tt{0.0};
        // Optional per-junction transit time overrides (seconds). Negative => inherit tt.
        double ttd{-1.0};
        double tts{-1.0};
        // Temperature controls:
        // - Temp:   effective device temperature used for all temperature-dependent calculations (Celsius)
        // - dtemp:  instance temperature offset added on top of environment Temp or overridden Temp (Celsius)
        // - Temp_override: if true, base temperature is Temp_override_value instead of environment temperature
        double Temp{27.0};                 // Celsius (effective)
        double dtemp{0.0};                 // Celsius
        double Temp_override_value{27.0};  // Celsius (base TEMP when override is enabled)
        bool Temp_override{};              // true if instance "Temp" overrides environment temperature

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
        // Optional per-junction diffusion current densities. If 0, fall back to the corresponding global js/jsw/jswg.
        double jsd{0.0};    // A/m^2
        double jss{0.0};    // A/m^2
        double jswd{0.0};   // A/m
        double jsws{0.0};   // A/m
        double jswgd{0.0};  // A/m
        double jswgs{0.0};  // A/m
        // Optional diffusion junction recombination current density parameters (clean-room subset):
        // When any of these are non-zero, they override diode_Isr scaling using geometry similarly to js/jsw/jswg.
        double jsr{0.0};    // A/m^2
        double jsrw{0.0};   // A/m
        double jsrwg{0.0};  // A/m (gate-edge sidewall recombination current density, scaled by Weff)
        // Optional per-junction recombination current densities. If 0, fall back to the corresponding global jsr/jsrw/jsrwg.
        double jsrd{0.0};    // A/m^2
        double jsrs{0.0};    // A/m^2
        double jsrwd{0.0};   // A/m
        double jsrws{0.0};   // A/m
        double jsrwgd{0.0};  // A/m
        double jsrwgs{0.0};  // A/m

        // Optional S/D-B depletion capacitance (simple SPICE-style junction C model).
        // Units:
        // - Cj*Area [F] where Cj is F/m^2 and Area is m^2
        // - Cjsw*Perimeter [F] where Cjsw is F/m and Perimeter is m
        double drainArea{0.0};        // m^2
        double sourceArea{0.0};       // m^2
        double drainPerimeter{0.0};   // m
        double sourcePerimeter{0.0};  // m

        // Optional per-junction depletion parameters (clean-room subset):
        // If set, these override the corresponding common parameters for that junction only.
        // This allows asymmetric drain/source junction C–V behavior while keeping default behavior unchanged.
        double cjd{0.0};  // F/m^2 (drain bottom junction cap density)
        double cjs{0.0};  // F/m^2 (source bottom junction cap density)

        double cj{0.0};    // F/m^2 (bottom junction cap density)
        double cjsw{0.0};  // F/m   (sidewall junction cap density)
        // Optional per-junction sidewall cap densities (F/m). If 0, fall back to cjsw.
        double cjswd{0.0};
        double cjsws{0.0};
        double cjswg{0.0};  // F/m  (gate-edge sidewall junction cap density)
        // Optional per-junction gate-edge sidewall cap densities (F/m). If 0, fall back to cjswg.
        double cjswgd{0.0};
        double cjswgs{0.0};
        double pb{1.0};  // V (junction potential)
        // Optional per-junction bottom-junction potentials (V). If <=0, fall back to pb.
        double pbd{0.0};
        double pbs{0.0};
        // Sidewall junction potential (V). If <=0, fall back to pb.
        double pbsw{0.0};
        // Optional per-junction sidewall junction potentials (V). If <=0, fall back to pbsw/pb.
        double pbswd{0.0};
        double pbsws{0.0};
        // Gate-edge sidewall junction potential (V). If <=0, fall back to pbsw/pb.
        double pbswg{0.0};
        // Optional per-junction gate-edge sidewall junction potentials (V). If <=0, fall back to pbswg/pbsw/pb.
        double pbswgd{0.0};
        double pbswgs{0.0};
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
        // Optional per-junction grading coefficients. Negative => inherit common coefficient.
        double mjd{-1.0};
        double mjs{-1.0};
        double mjswd{-1.0};
        double mjsws{-1.0};
        double mjswgd{-1.0};
        double mjswgs{-1.0};
        double fc{0.5};  // forward-bias coefficient
        // Optional per-junction forward-bias coefficient overrides for depletion C model.
        // Negative => inherit the common fc.
        double fcd{-1.0};
        double fcs{-1.0};

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
        // Physical-terminal bias history (no D/S swapping) used to limit auxiliary branches (GIDL/GISL, Ig*).
        double vgs_s_phys_last{};
        double vds_s_phys_last{};
        double vbs_s_phys_last{};
        bool dc_bias_valid{};
        bool dc_bias_swapped{};  // last bias values correspond to swapped mode
        bool dc_bias_phys_valid{};

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
        ::phy_engine::model::node_t internal_nodes[6]{};  // packed prefix of used nodes
        ::phy_engine::model::node_t* nd_int{};            // D'
        ::phy_engine::model::node_t* ns_int{};            // S'
        ::phy_engine::model::node_t* ng_int{};            // G'
        ::phy_engine::model::node_t* nb_int{};            // B'
        ::phy_engine::model::node_t* nbd_int{};           // B'd (local body near drain junction)
        ::phy_engine::model::node_t* nbs_int{};           // B's (local body near source junction)
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
            case 365: m.W = vi.d; return true;     // w
            case 366: m.L = vi.d; return true;     // l
            case 368: m.Kp = vi.d; return true;    // kp
            case 369: m.Vth0 = vi.d; return true;  // vth0
            case 370:
            {
                // temp (alias of Temp): same override semantics as "Temp".
                if(::std::isfinite(vi.d))
                {
                    m.Temp_override_value = vi.d;
                    m.Temp_override = true;
                    m.Temp = m.Temp_override_value + m.dtemp;
                    m.dc_bias_valid = false;
                    m.dc_bias_phys_valid = false;
                }
                else
                {
                    m.Temp_override = false;
                    m.dc_bias_valid = false;
                    m.dc_bias_phys_valid = false;
                }
                return true;
            }
            case 371: m.Rd = vi.d; return true;   // rd
            case 372: m.Rs = vi.d; return true;   // rs
            case 373: m.Rb = vi.d; return true;   // rb
            case 374: m.Cgs = vi.d; return true;  // cgs
            case 375: m.Cgd = vi.d; return true;  // cgd
            case 376: m.Cgb = vi.d; return true;  // cgb
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
            case 222: m.xj = vi.d; return true;
            case 223: m.mobMod = vi.d; return true;
            case 224: m.vfbcv = vi.d; return true;
            case 225: m.acm = vi.d; return true;
            case 226: m.voffcv = vi.d; return true;
            case 227: m.lvoffcv = vi.d; return true;
            case 228: m.wvoffcv = vi.d; return true;
            case 229: m.pvoffcv = vi.d; return true;
            case 230: m.agidl = vi.d; return true;
            case 231: m.bgidl = vi.d; return true;
            case 232: m.cgidl = vi.d; return true;
            case 233: m.egidl = vi.d; return true;
            case 234: m.agisl = vi.d; return true;
            case 235: m.bgisl = vi.d; return true;
            case 236: m.cgisl = vi.d; return true;
            case 237: m.egisl = vi.d; return true;
            case 238: m.alpha0 = vi.d; return true;
            case 239: m.beta0 = vi.d; return true;
            case 240: m.vdsatii = vi.d; return true;
            case 14: m.Rd = vi.d; return true;
            case 15: m.Rs = vi.d; return true;
            case 241: m.Rb = vi.d; return true;
            case 242: m.noff = vi.d; return true;
            case 97: m.rsh = vi.d; return true;
            case 98: m.nrd = vi.d; return true;
            case 99: m.nrs = vi.d; return true;
            case 78: m.rg = vi.d; return true;
            case 243: m.rbdb = vi.d; return true;
            case 244: m.rbsb = vi.d; return true;
            case 245: m.aigb = vi.d; return true;
            case 246: m.bigb = vi.d; return true;
            case 247: m.cigb = vi.d; return true;
            case 248: m.eigb = vi.d; return true;
            case 249: m.aigs = vi.d; return true;
            case 250: m.bigs = vi.d; return true;
            case 251: m.cigs = vi.d; return true;
            case 252: m.eigs = vi.d; return true;
            case 253: m.aigd = vi.d; return true;
            case 254: m.bigd = vi.d; return true;
            case 255: m.cigd = vi.d; return true;
            case 256: m.eigd = vi.d; return true;
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
            case 337: m.diode_Isd = vi.d; return true;
            case 338: m.diode_Iss = vi.d; return true;
            case 339: m.diode_Nd = vi.d; return true;
            case 340: m.diode_Ns = vi.d; return true;
            case 341: m.diode_Isrd = vi.d; return true;
            case 342: m.diode_Isrs = vi.d; return true;
            case 343: m.diode_Nrd = vi.d; return true;
            case 344: m.diode_Nrs = vi.d; return true;
            case 347: m.diode_Isd = vi.d; return true;   // isd
            case 348: m.diode_Iss = vi.d; return true;   // iss
            case 349: m.diode_Nd = vi.d; return true;    // nd
            case 350: m.diode_Ns = vi.d; return true;    // ns
            case 351: m.diode_Isrd = vi.d; return true;  // isrd
            case 352: m.diode_Isrs = vi.d; return true;  // isrs
            case 353: m.diode_Is = vi.d; return true;    // is
            case 354: m.diode_N = vi.d; return true;     // n
            case 355: m.diode_Isr = vi.d; return true;   // isr
            case 356: m.diode_Nr = vi.d; return true;    // nr
            case 261: m.diode_Isr = vi.d; return true;
            case 262: m.diode_Nr = vi.d; return true;
            case 12:
                if(::std::isfinite(vi.d))
                {
                    m.Temp_override_value = vi.d;
                    m.Temp_override = true;
                    m.Temp = m.Temp_override_value + m.dtemp;
                    m.dc_bias_valid = false;
                    m.dc_bias_phys_valid = false;
                }
                else
                {
                    m.Temp_override = false;
                    m.dc_bias_valid = false;
                    m.dc_bias_phys_valid = false;
                }
                return true;
            case 263:
            {
                double const old_dtemp{m.dtemp};
                m.dtemp = vi.d;
                if(m.Temp_override) { m.Temp = m.Temp_override_value + m.dtemp; }
                else
                {
                    m.Temp += (m.dtemp - old_dtemp);
                }
                m.dc_bias_valid = false;
                m.dc_bias_phys_valid = false;
                return true;
            }
            case 83: m.tt = vi.d; return true;
            case 345: m.ttd = vi.d; return true;
            case 346: m.tts = vi.d; return true;
            case 257: m.bvd = vi.d; return true;
            case 258: m.ibvd = vi.d; return true;
            case 259: m.bvs = vi.d; return true;
            case 260: m.ibvs = vi.d; return true;
            case 357:
                m.bvd = vi.d;
                m.bvs = vi.d;
                return true;  // bv
            case 358:
                m.ibvd = vi.d;
                m.ibvs = vi.d;
                return true;  // ibv
            case 71: m.tnom = vi.d; return true;
            case 72: m.ute = vi.d; return true;
            case 73: m.kt1 = vi.d; return true;
            case 74: m.kt2 = vi.d; return true;
            case 75: m.at = vi.d; return true;
            case 69: m.js = vi.d; return true;
            case 70: m.jsw = vi.d; return true;
            case 101: m.jswg = vi.d; return true;
            case 325: m.jsd = vi.d; return true;
            case 326: m.jss = vi.d; return true;
            case 327: m.jswd = vi.d; return true;
            case 328: m.jsws = vi.d; return true;
            case 329: m.jswgd = vi.d; return true;
            case 330: m.jswgs = vi.d; return true;
            case 304: m.jsr = vi.d; return true;
            case 305: m.jsrw = vi.d; return true;
            case 306: m.jsrwg = vi.d; return true;
            case 331: m.jsrd = vi.d; return true;
            case 332: m.jsrs = vi.d; return true;
            case 333: m.jsrwd = vi.d; return true;
            case 334: m.jsrws = vi.d; return true;
            case 335: m.jsrwgd = vi.d; return true;
            case 336: m.jsrwgs = vi.d; return true;
            case 307: m.rdsMod = vi.d; return true;
            case 308: m.rgateMod = vi.d; return true;
            case 309: m.rbodyMod = vi.d; return true;
            case 310: m.rdsMod = vi.d; return true;    // rdsmod
            case 311: m.rgateMod = vi.d; return true;  // rgatemod
            case 312: m.rbodyMod = vi.d; return true;  // rbodymod
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
            case 313: m.cjd = vi.d; return true;
            case 314: m.cjs = vi.d; return true;
            case 21: m.cjsw = vi.d; return true;
            case 317: m.cjswd = vi.d; return true;
            case 318: m.cjsws = vi.d; return true;
            case 89: m.cjswg = vi.d; return true;
            case 319: m.cjswgd = vi.d; return true;
            case 320: m.cjswgs = vi.d; return true;
            case 22: m.pb = vi.d; return true;
            case 315: m.pbd = vi.d; return true;
            case 316: m.pbs = vi.d; return true;
            case 84: m.pbsw = vi.d; return true;
            case 321: m.pbswd = vi.d; return true;
            case 322: m.pbsws = vi.d; return true;
            case 90: m.pbswg = vi.d; return true;
            case 323: m.pbswgd = vi.d; return true;
            case 324: m.pbswgs = vi.d; return true;
            case 85: m.tcj = vi.d; return true;
            case 86: m.tcjsw = vi.d; return true;
            case 87: m.tpb = vi.d; return true;
            case 88: m.tpbsw = vi.d; return true;
            case 91: m.tcjswg = vi.d; return true;
            case 92: m.tpbswg = vi.d; return true;
            case 23: m.mj = vi.d; return true;
            case 24: m.mjsw = vi.d; return true;
            case 93: m.mjswg = vi.d; return true;
            case 359: m.mjd = vi.d; return true;
            case 360: m.mjs = vi.d; return true;
            case 361: m.mjswd = vi.d; return true;
            case 362: m.mjsws = vi.d; return true;
            case 363: m.mjswgd = vi.d; return true;
            case 364: m.mjswgs = vi.d; return true;
            case 25: m.fc = vi.d; return true;
            case 377: m.fcd = vi.d; return true;
            case 378: m.fcs = vi.d; return true;
            case 26: m.tox = vi.d; return true;
            case 27: m.toxm = vi.d; return true;
            case 28: m.nch = vi.d; return true;
            case 29: m.u0 = vi.d; return true;
            case 30: m.ua = vi.d; return true;
            case 31: m.ub = vi.d; return true;
            case 32: m.uc = vi.d; return true;
            case 300: m.ua1 = vi.d; return true;
            case 301: m.ub1 = vi.d; return true;
            case 302: m.uc1 = vi.d; return true;
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
            case 303: m.prt = vi.d; return true;
            case 61: m.keta = vi.d; return true;
            case 62: m.capMod = vi.d; return true;
            case 367: m.capMod = vi.d; return true;  // capmod
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
            case 365: return {.d{m.W}, .type{::phy_engine::model::variant_type::d}};     // w
            case 366: return {.d{m.L}, .type{::phy_engine::model::variant_type::d}};     // l
            case 368: return {.d{m.Kp}, .type{::phy_engine::model::variant_type::d}};    // kp
            case 369: return {.d{m.Vth0}, .type{::phy_engine::model::variant_type::d}};  // vth0
            case 370: return {.d{m.Temp}, .type{::phy_engine::model::variant_type::d}};  // temp
            case 371: return {.d{m.Rd}, .type{::phy_engine::model::variant_type::d}};    // rd
            case 372: return {.d{m.Rs}, .type{::phy_engine::model::variant_type::d}};    // rs
            case 373: return {.d{m.Rb}, .type{::phy_engine::model::variant_type::d}};    // rb
            case 374: return {.d{m.Cgs}, .type{::phy_engine::model::variant_type::d}};   // cgs
            case 375: return {.d{m.Cgd}, .type{::phy_engine::model::variant_type::d}};   // cgd
            case 376: return {.d{m.Cgb}, .type{::phy_engine::model::variant_type::d}};   // cgb
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
            case 222: return {.d{m.xj}, .type{::phy_engine::model::variant_type::d}};
            case 223: return {.d{m.mobMod}, .type{::phy_engine::model::variant_type::d}};
            case 224: return {.d{m.vfbcv}, .type{::phy_engine::model::variant_type::d}};
            case 225: return {.d{m.acm}, .type{::phy_engine::model::variant_type::d}};
            case 226: return {.d{m.voffcv}, .type{::phy_engine::model::variant_type::d}};
            case 227: return {.d{m.lvoffcv}, .type{::phy_engine::model::variant_type::d}};
            case 228: return {.d{m.wvoffcv}, .type{::phy_engine::model::variant_type::d}};
            case 229: return {.d{m.pvoffcv}, .type{::phy_engine::model::variant_type::d}};
            case 230: return {.d{m.agidl}, .type{::phy_engine::model::variant_type::d}};
            case 231: return {.d{m.bgidl}, .type{::phy_engine::model::variant_type::d}};
            case 232: return {.d{m.cgidl}, .type{::phy_engine::model::variant_type::d}};
            case 233: return {.d{m.egidl}, .type{::phy_engine::model::variant_type::d}};
            case 234: return {.d{m.agisl}, .type{::phy_engine::model::variant_type::d}};
            case 235: return {.d{m.bgisl}, .type{::phy_engine::model::variant_type::d}};
            case 236: return {.d{m.cgisl}, .type{::phy_engine::model::variant_type::d}};
            case 237: return {.d{m.egisl}, .type{::phy_engine::model::variant_type::d}};
            case 238: return {.d{m.alpha0}, .type{::phy_engine::model::variant_type::d}};
            case 239: return {.d{m.beta0}, .type{::phy_engine::model::variant_type::d}};
            case 240: return {.d{m.vdsatii}, .type{::phy_engine::model::variant_type::d}};
            case 14: return {.d{m.Rd}, .type{::phy_engine::model::variant_type::d}};
            case 15: return {.d{m.Rs}, .type{::phy_engine::model::variant_type::d}};
            case 241: return {.d{m.Rb}, .type{::phy_engine::model::variant_type::d}};
            case 242: return {.d{m.noff}, .type{::phy_engine::model::variant_type::d}};
            case 97: return {.d{m.rsh}, .type{::phy_engine::model::variant_type::d}};
            case 98: return {.d{m.nrd}, .type{::phy_engine::model::variant_type::d}};
            case 99: return {.d{m.nrs}, .type{::phy_engine::model::variant_type::d}};
            case 78: return {.d{m.rg}, .type{::phy_engine::model::variant_type::d}};
            case 243: return {.d{m.rbdb}, .type{::phy_engine::model::variant_type::d}};
            case 244: return {.d{m.rbsb}, .type{::phy_engine::model::variant_type::d}};
            case 245: return {.d{m.aigb}, .type{::phy_engine::model::variant_type::d}};
            case 246: return {.d{m.bigb}, .type{::phy_engine::model::variant_type::d}};
            case 247: return {.d{m.cigb}, .type{::phy_engine::model::variant_type::d}};
            case 248: return {.d{m.eigb}, .type{::phy_engine::model::variant_type::d}};
            case 249: return {.d{m.aigs}, .type{::phy_engine::model::variant_type::d}};
            case 250: return {.d{m.bigs}, .type{::phy_engine::model::variant_type::d}};
            case 251: return {.d{m.cigs}, .type{::phy_engine::model::variant_type::d}};
            case 252: return {.d{m.eigs}, .type{::phy_engine::model::variant_type::d}};
            case 253: return {.d{m.aigd}, .type{::phy_engine::model::variant_type::d}};
            case 254: return {.d{m.bigd}, .type{::phy_engine::model::variant_type::d}};
            case 255: return {.d{m.cigd}, .type{::phy_engine::model::variant_type::d}};
            case 256: return {.d{m.eigd}, .type{::phy_engine::model::variant_type::d}};
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
            case 337: return {.d{m.diode_Isd}, .type{::phy_engine::model::variant_type::d}};
            case 338: return {.d{m.diode_Iss}, .type{::phy_engine::model::variant_type::d}};
            case 339: return {.d{m.diode_Nd}, .type{::phy_engine::model::variant_type::d}};
            case 340: return {.d{m.diode_Ns}, .type{::phy_engine::model::variant_type::d}};
            case 341: return {.d{m.diode_Isrd}, .type{::phy_engine::model::variant_type::d}};
            case 342: return {.d{m.diode_Isrs}, .type{::phy_engine::model::variant_type::d}};
            case 343: return {.d{m.diode_Nrd}, .type{::phy_engine::model::variant_type::d}};
            case 344: return {.d{m.diode_Nrs}, .type{::phy_engine::model::variant_type::d}};
            case 347: return {.d{m.diode_Isd}, .type{::phy_engine::model::variant_type::d}};   // isd
            case 348: return {.d{m.diode_Iss}, .type{::phy_engine::model::variant_type::d}};   // iss
            case 349: return {.d{m.diode_Nd}, .type{::phy_engine::model::variant_type::d}};    // nd
            case 350: return {.d{m.diode_Ns}, .type{::phy_engine::model::variant_type::d}};    // ns
            case 351: return {.d{m.diode_Isrd}, .type{::phy_engine::model::variant_type::d}};  // isrd
            case 352: return {.d{m.diode_Isrs}, .type{::phy_engine::model::variant_type::d}};  // isrs
            case 353: return {.d{m.diode_Is}, .type{::phy_engine::model::variant_type::d}};    // is
            case 354: return {.d{m.diode_N}, .type{::phy_engine::model::variant_type::d}};     // n
            case 355: return {.d{m.diode_Isr}, .type{::phy_engine::model::variant_type::d}};   // isr
            case 356: return {.d{m.diode_Nr}, .type{::phy_engine::model::variant_type::d}};    // nr
            case 261: return {.d{m.diode_Isr}, .type{::phy_engine::model::variant_type::d}};
            case 262: return {.d{m.diode_Nr}, .type{::phy_engine::model::variant_type::d}};
            case 12: return {.d{m.Temp}, .type{::phy_engine::model::variant_type::d}};
            case 263: return {.d{m.dtemp}, .type{::phy_engine::model::variant_type::d}};
            case 83: return {.d{m.tt}, .type{::phy_engine::model::variant_type::d}};
            case 345: return {.d{m.ttd}, .type{::phy_engine::model::variant_type::d}};
            case 346: return {.d{m.tts}, .type{::phy_engine::model::variant_type::d}};
            case 257: return {.d{m.bvd}, .type{::phy_engine::model::variant_type::d}};
            case 258: return {.d{m.ibvd}, .type{::phy_engine::model::variant_type::d}};
            case 259: return {.d{m.bvs}, .type{::phy_engine::model::variant_type::d}};
            case 260: return {.d{m.ibvs}, .type{::phy_engine::model::variant_type::d}};
            case 357: return {.d{m.bvd}, .type{::phy_engine::model::variant_type::d}};   // bv
            case 358: return {.d{m.ibvd}, .type{::phy_engine::model::variant_type::d}};  // ibv
            case 71: return {.d{m.tnom}, .type{::phy_engine::model::variant_type::d}};
            case 72: return {.d{m.ute}, .type{::phy_engine::model::variant_type::d}};
            case 73: return {.d{m.kt1}, .type{::phy_engine::model::variant_type::d}};
            case 74: return {.d{m.kt2}, .type{::phy_engine::model::variant_type::d}};
            case 75: return {.d{m.at}, .type{::phy_engine::model::variant_type::d}};
            case 69: return {.d{m.js}, .type{::phy_engine::model::variant_type::d}};
            case 70: return {.d{m.jsw}, .type{::phy_engine::model::variant_type::d}};
            case 101: return {.d{m.jswg}, .type{::phy_engine::model::variant_type::d}};
            case 325: return {.d{m.jsd}, .type{::phy_engine::model::variant_type::d}};
            case 326: return {.d{m.jss}, .type{::phy_engine::model::variant_type::d}};
            case 327: return {.d{m.jswd}, .type{::phy_engine::model::variant_type::d}};
            case 328: return {.d{m.jsws}, .type{::phy_engine::model::variant_type::d}};
            case 329: return {.d{m.jswgd}, .type{::phy_engine::model::variant_type::d}};
            case 330: return {.d{m.jswgs}, .type{::phy_engine::model::variant_type::d}};
            case 304: return {.d{m.jsr}, .type{::phy_engine::model::variant_type::d}};
            case 305: return {.d{m.jsrw}, .type{::phy_engine::model::variant_type::d}};
            case 306: return {.d{m.jsrwg}, .type{::phy_engine::model::variant_type::d}};
            case 331: return {.d{m.jsrd}, .type{::phy_engine::model::variant_type::d}};
            case 332: return {.d{m.jsrs}, .type{::phy_engine::model::variant_type::d}};
            case 333: return {.d{m.jsrwd}, .type{::phy_engine::model::variant_type::d}};
            case 334: return {.d{m.jsrws}, .type{::phy_engine::model::variant_type::d}};
            case 335: return {.d{m.jsrwgd}, .type{::phy_engine::model::variant_type::d}};
            case 336: return {.d{m.jsrwgs}, .type{::phy_engine::model::variant_type::d}};
            case 307: return {.d{m.rdsMod}, .type{::phy_engine::model::variant_type::d}};
            case 308: return {.d{m.rgateMod}, .type{::phy_engine::model::variant_type::d}};
            case 309: return {.d{m.rbodyMod}, .type{::phy_engine::model::variant_type::d}};
            case 310: return {.d{m.rdsMod}, .type{::phy_engine::model::variant_type::d}};    // rdsmod
            case 311: return {.d{m.rgateMod}, .type{::phy_engine::model::variant_type::d}};  // rgatemod
            case 312: return {.d{m.rbodyMod}, .type{::phy_engine::model::variant_type::d}};  // rbodymod
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
            case 313: return {.d{m.cjd}, .type{::phy_engine::model::variant_type::d}};
            case 314: return {.d{m.cjs}, .type{::phy_engine::model::variant_type::d}};
            case 21: return {.d{m.cjsw}, .type{::phy_engine::model::variant_type::d}};
            case 317: return {.d{m.cjswd}, .type{::phy_engine::model::variant_type::d}};
            case 318: return {.d{m.cjsws}, .type{::phy_engine::model::variant_type::d}};
            case 89: return {.d{m.cjswg}, .type{::phy_engine::model::variant_type::d}};
            case 319: return {.d{m.cjswgd}, .type{::phy_engine::model::variant_type::d}};
            case 320: return {.d{m.cjswgs}, .type{::phy_engine::model::variant_type::d}};
            case 22: return {.d{m.pb}, .type{::phy_engine::model::variant_type::d}};
            case 315: return {.d{m.pbd}, .type{::phy_engine::model::variant_type::d}};
            case 316: return {.d{m.pbs}, .type{::phy_engine::model::variant_type::d}};
            case 84: return {.d{m.pbsw}, .type{::phy_engine::model::variant_type::d}};
            case 321: return {.d{m.pbswd}, .type{::phy_engine::model::variant_type::d}};
            case 322: return {.d{m.pbsws}, .type{::phy_engine::model::variant_type::d}};
            case 90: return {.d{m.pbswg}, .type{::phy_engine::model::variant_type::d}};
            case 323: return {.d{m.pbswgd}, .type{::phy_engine::model::variant_type::d}};
            case 324: return {.d{m.pbswgs}, .type{::phy_engine::model::variant_type::d}};
            case 85: return {.d{m.tcj}, .type{::phy_engine::model::variant_type::d}};
            case 86: return {.d{m.tcjsw}, .type{::phy_engine::model::variant_type::d}};
            case 87: return {.d{m.tpb}, .type{::phy_engine::model::variant_type::d}};
            case 88: return {.d{m.tpbsw}, .type{::phy_engine::model::variant_type::d}};
            case 91: return {.d{m.tcjswg}, .type{::phy_engine::model::variant_type::d}};
            case 92: return {.d{m.tpbswg}, .type{::phy_engine::model::variant_type::d}};
            case 23: return {.d{m.mj}, .type{::phy_engine::model::variant_type::d}};
            case 24: return {.d{m.mjsw}, .type{::phy_engine::model::variant_type::d}};
            case 93: return {.d{m.mjswg}, .type{::phy_engine::model::variant_type::d}};
            case 359: return {.d{m.mjd}, .type{::phy_engine::model::variant_type::d}};
            case 360: return {.d{m.mjs}, .type{::phy_engine::model::variant_type::d}};
            case 361: return {.d{m.mjswd}, .type{::phy_engine::model::variant_type::d}};
            case 362: return {.d{m.mjsws}, .type{::phy_engine::model::variant_type::d}};
            case 363: return {.d{m.mjswgd}, .type{::phy_engine::model::variant_type::d}};
            case 364: return {.d{m.mjswgs}, .type{::phy_engine::model::variant_type::d}};
            case 25: return {.d{m.fc}, .type{::phy_engine::model::variant_type::d}};
            case 377: return {.d{m.fcd}, .type{::phy_engine::model::variant_type::d}};
            case 378: return {.d{m.fcs}, .type{::phy_engine::model::variant_type::d}};
            case 26: return {.d{m.tox}, .type{::phy_engine::model::variant_type::d}};
            case 27: return {.d{m.toxm}, .type{::phy_engine::model::variant_type::d}};
            case 28: return {.d{m.nch}, .type{::phy_engine::model::variant_type::d}};
            case 29: return {.d{m.u0}, .type{::phy_engine::model::variant_type::d}};
            case 30: return {.d{m.ua}, .type{::phy_engine::model::variant_type::d}};
            case 31: return {.d{m.ub}, .type{::phy_engine::model::variant_type::d}};
            case 32: return {.d{m.uc}, .type{::phy_engine::model::variant_type::d}};
            case 300: return {.d{m.ua1}, .type{::phy_engine::model::variant_type::d}};
            case 301: return {.d{m.ub1}, .type{::phy_engine::model::variant_type::d}};
            case 302: return {.d{m.uc1}, .type{::phy_engine::model::variant_type::d}};
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
            case 303: return {.d{m.prt}, .type{::phy_engine::model::variant_type::d}};
            case 61: return {.d{m.keta}, .type{::phy_engine::model::variant_type::d}};
            case 62: return {.d{m.capMod}, .type{::phy_engine::model::variant_type::d}};
            case 367: return {.d{m.capMod}, .type{::phy_engine::model::variant_type::d}};  // capmod
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
            case 365: return u8"w";
            case 366: return u8"l";
            case 368: return u8"kp";
            case 369: return u8"vth0";
            case 370: return u8"temp";
            case 371: return u8"rd";
            case 372: return u8"rs";
            case 373: return u8"rb";
            case 374: return u8"cgs";
            case 375: return u8"cgd";
            case 376: return u8"cgb";
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
            case 222: return u8"xj";
            case 223: return u8"mobMod";
            case 224: return u8"vfbcv";
            case 225: return u8"acm";
            case 226: return u8"voffcv";
            case 227: return u8"lvoffcv";
            case 228: return u8"wvoffcv";
            case 229: return u8"pvoffcv";
            case 230: return u8"agidl";
            case 231: return u8"bgidl";
            case 232: return u8"cgidl";
            case 233: return u8"egidl";
            case 234: return u8"agisl";
            case 235: return u8"bgisl";
            case 236: return u8"cgisl";
            case 237: return u8"egisl";
            case 238: return u8"alpha0";
            case 239: return u8"beta0";
            case 240: return u8"vdsatii";
            case 14: return u8"Rd";
            case 15: return u8"Rs";
            case 241: return u8"Rb";
            case 242: return u8"noff";
            case 97: return u8"rsh";
            case 98: return u8"nrd";
            case 99: return u8"nrs";
            case 78: return u8"rg";
            case 243: return u8"rbdb";
            case 244: return u8"rbsb";
            case 245: return u8"aigb";
            case 246: return u8"bigb";
            case 247: return u8"cigb";
            case 248: return u8"eigb";
            case 249: return u8"aigs";
            case 250: return u8"bigs";
            case 251: return u8"cigs";
            case 252: return u8"eigs";
            case 253: return u8"aigd";
            case 254: return u8"bigd";
            case 255: return u8"cigd";
            case 256: return u8"eigd";
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
            case 337: return u8"diode_Isd";
            case 338: return u8"diode_Iss";
            case 339: return u8"diode_Nd";
            case 340: return u8"diode_Ns";
            case 341: return u8"diode_Isrd";
            case 342: return u8"diode_Isrs";
            case 343: return u8"diode_Nrd";
            case 344: return u8"diode_Nrs";
            case 347: return u8"isd";
            case 348: return u8"iss";
            case 349: return u8"nd";
            case 350: return u8"ns";
            case 351: return u8"isrd";
            case 352: return u8"isrs";
            case 353: return u8"is";
            case 354: return u8"n";
            case 355: return u8"isr";
            case 356: return u8"nr";
            case 261: return u8"diode_Isr";
            case 262: return u8"diode_Nr";
            case 12: return u8"Temp";
            case 263: return u8"dtemp";
            case 83: return u8"tt";
            case 345: return u8"ttd";
            case 346: return u8"tts";
            case 257: return u8"bvd";
            case 258: return u8"ibvd";
            case 259: return u8"bvs";
            case 260: return u8"ibvs";
            case 357: return u8"bv";
            case 358: return u8"ibv";
            case 71: return u8"tnom";
            case 72: return u8"ute";
            case 73: return u8"kt1";
            case 74: return u8"kt2";
            case 75: return u8"at";
            case 69: return u8"js";
            case 70: return u8"jsw";
            case 101: return u8"jswg";
            case 325: return u8"jsd";
            case 326: return u8"jss";
            case 327: return u8"jswd";
            case 328: return u8"jsws";
            case 329: return u8"jswgd";
            case 330: return u8"jswgs";
            case 304: return u8"jsr";
            case 305: return u8"jsrw";
            case 306: return u8"jsrwg";
            case 331: return u8"jsrd";
            case 332: return u8"jsrs";
            case 333: return u8"jsrwd";
            case 334: return u8"jsrws";
            case 335: return u8"jsrwgd";
            case 336: return u8"jsrwgs";
            case 307: return u8"rdsMod";
            case 308: return u8"rgateMod";
            case 309: return u8"rbodyMod";
            case 310: return u8"rdsmod";
            case 311: return u8"rgatemod";
            case 312: return u8"rbodymod";
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
            case 313: return u8"cjd";
            case 314: return u8"cjs";
            case 21: return u8"cjsw";
            case 317: return u8"cjswd";
            case 318: return u8"cjsws";
            case 89: return u8"cjswg";
            case 319: return u8"cjswgd";
            case 320: return u8"cjswgs";
            case 22: return u8"pb";
            case 315: return u8"pbd";
            case 316: return u8"pbs";
            case 84: return u8"pbsw";
            case 321: return u8"pbswd";
            case 322: return u8"pbsws";
            case 90: return u8"pbswg";
            case 323: return u8"pbswgd";
            case 324: return u8"pbswgs";
            case 85: return u8"tcj";
            case 86: return u8"tcjsw";
            case 87: return u8"tpb";
            case 88: return u8"tpbsw";
            case 91: return u8"tcjswg";
            case 92: return u8"tpbswg";
            case 23: return u8"mj";
            case 24: return u8"mjsw";
            case 93: return u8"mjswg";
            case 359: return u8"mjd";
            case 360: return u8"mjs";
            case 361: return u8"mjswd";
            case 362: return u8"mjsws";
            case 363: return u8"mjswgd";
            case 364: return u8"mjswgs";
            case 25: return u8"fc";
            case 377: return u8"fcd";
            case 378: return u8"fcs";
            case 26: return u8"tox";
            case 27: return u8"toxm";
            case 28: return u8"nch";
            case 29: return u8"u0";
            case 30: return u8"ua";
            case 31: return u8"ub";
            case 32: return u8"uc";
            case 300: return u8"ua1";
            case 301: return u8"ub1";
            case 302: return u8"uc1";
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
            case 303: return u8"prt";
            case 61: return u8"keta";
            case 62: return u8"capMod";
            case 367: return u8"capmod";
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
        auto const nb{nb_raw ? (m.nb_int ? m.nb_int : nb_raw) : (m.ns_int ? m.ns_int : ns)};
        if(nd_ext == nullptr || ns == nullptr || nb == nullptr) { return true; }

        bool const use_d_int{m.nd_int != nullptr};
        bool const use_s_int{m.ns_int != nullptr};
        auto const nd{use_d_int ? m.nd_int : nd_ext};
        auto const ns_int{use_s_int ? m.ns_int : ns};

        double const scale{(m.m_mult > 0.0 ? m.m_mult : 0.0) * (m.nf > 0.0 ? m.nf : 0.0)};
        // Keep diode internals numerically well-defined even if m<=0 (treat as "almost off").
        double const scale_diode{scale > 0.0 ? scale : 1e-30};
        double const temp_is_scale{details::bsim3v32_is_temp_scale(m.Temp, m.tnom, m.xti, m.eg)};
        double const diode_is_param_d{(m.diode_Isd >= 0.0) ? m.diode_Isd : m.diode_Is};
        double const diode_is_param_s{(m.diode_Iss >= 0.0) ? m.diode_Iss : m.diode_Is};
        double const diode_is_d{(diode_is_param_d > 0.0 ? diode_is_param_d : 1e-30) * temp_is_scale};
        double const diode_is_s{(diode_is_param_s > 0.0 ? diode_is_param_s : 1e-30) * temp_is_scale};

        double const diode_isr_param_d{(m.diode_Isrd >= 0.0) ? m.diode_Isrd : m.diode_Isr};
        double const diode_isr_param_s{(m.diode_Isrs >= 0.0) ? m.diode_Isrs : m.diode_Isr};
        double const diode_isr_d{(diode_isr_param_d > 0.0 ? diode_isr_param_d : 0.0) * temp_is_scale};
        double const diode_isr_s{(diode_isr_param_s > 0.0 ? diode_isr_param_s : 0.0) * temp_is_scale};

        double const diode_n_d{(m.diode_Nd > 0.0) ? m.diode_Nd : m.diode_N};
        double const diode_n_s{(m.diode_Ns > 0.0) ? m.diode_Ns : m.diode_N};
        double const diode_nr_d{(m.diode_Nrd > 0.0) ? m.diode_Nrd : m.diode_Nr};
        double const diode_nr_s{(m.diode_Nrs > 0.0) ? m.diode_Nrs : m.diode_Nr};

        double const tt_d{(m.ttd >= 0.0) ? m.ttd : m.tt};
        double const tt_s{(m.tts >= 0.0) ? m.tts : m.tt};

        // Propagate basic diode params and attach to external nodes.
        m.diode_db.N = (diode_n_d > 0.0 ? diode_n_d : 1.0);
        m.diode_db.Isr = diode_isr_d;
        m.diode_db.Nr = (diode_nr_d > 0.0 ? diode_nr_d : 2.0);
        m.diode_db.Temp = m.Temp;
        m.diode_db.tt = (tt_d > 0.0 ? tt_d : 0.0);
        m.diode_db.Bv = m.bvd;
        m.diode_db.Bv_set = (m.bvd > 0.0 && m.ibvd > 0.0);
        m.diode_sb.N = (diode_n_s > 0.0 ? diode_n_s : 1.0);
        m.diode_sb.Isr = diode_isr_s;
        m.diode_sb.Nr = (diode_nr_s > 0.0 ? diode_nr_s : 2.0);
        m.diode_sb.Temp = m.Temp;
        m.diode_sb.tt = (tt_s > 0.0 ? tt_s : 0.0);
        m.diode_sb.Bv = m.bvs;
        m.diode_sb.Bv_set = (m.bvs > 0.0 && m.ibvs > 0.0);

        bool const use_js{m.js != 0.0 || m.jsw != 0.0 || m.jswg != 0.0 || m.jsd != 0.0 || m.jss != 0.0 || m.jswd != 0.0 || m.jsws != 0.0 || m.jswgd != 0.0 ||
                          m.jswgs != 0.0};
        bool const use_jsr{m.jsr != 0.0 || m.jsrw != 0.0 || m.jsrwg != 0.0 || m.jsrd != 0.0 || m.jsrs != 0.0 || m.jsrwd != 0.0 || m.jsrws != 0.0 ||
                           m.jsrwgd != 0.0 || m.jsrwgs != 0.0};
        bool const use_geom_currents{use_js || use_jsr};

        if(use_geom_currents)
        {
            // Use diffusion geometry scaling (area + sidewall perimeter) for Is and/or Isr.
            double const weff{::std::max(m.W - 2.0 * ::std::max(m.dwc, 0.0), 0.0)};

            double is_db{};
            double is_sb{};
            if(use_js)
            {
                double const js_d{(m.jsd != 0.0) ? m.jsd : m.js};
                double const js_s{(m.jss != 0.0) ? m.jss : m.js};
                double const jsw_d{(m.jswd != 0.0) ? m.jswd : m.jsw};
                double const jsw_s{(m.jsws != 0.0) ? m.jsws : m.jsw};
                double const jswg_d{(m.jswgd != 0.0) ? m.jswgd : m.jswg};
                double const jswg_s{(m.jswgs != 0.0) ? m.jswgs : m.jswg};

                is_db = (js_d * m.drainArea + jsw_d * m.drainPerimeter + jswg_d * weff) * scale * temp_is_scale;
                is_sb = (js_s * m.sourceArea + jsw_s * m.sourcePerimeter + jswg_s * weff) * scale * temp_is_scale;
                if(!(is_db > 0.0)) { is_db = diode_is_d * scale_diode; }
                if(!(is_sb > 0.0)) { is_sb = diode_is_s * scale_diode; }
            }
            else
            {
                // Preserve the legacy scaling behavior for Is when js/jsw/jswg are unset.
                is_db = diode_is_d * scale_diode;
                is_sb = diode_is_s * scale_diode;
            }

            double isr_db{};
            double isr_sb{};
            if(use_jsr)
            {
                double const jsr_d{(m.jsrd != 0.0) ? m.jsrd : m.jsr};
                double const jsr_s{(m.jsrs != 0.0) ? m.jsrs : m.jsr};
                double const jsrw_d{(m.jsrwd != 0.0) ? m.jsrwd : m.jsrw};
                double const jsrw_s{(m.jsrws != 0.0) ? m.jsrws : m.jsrw};
                double const jsrwg_d{(m.jsrwgd != 0.0) ? m.jsrwgd : m.jsrwg};
                double const jsrwg_s{(m.jsrwgs != 0.0) ? m.jsrwgs : m.jsrwg};

                isr_db = (jsr_d * m.drainArea + jsrw_d * m.drainPerimeter + jsrwg_d * weff) * scale * temp_is_scale;
                isr_sb = (jsr_s * m.sourceArea + jsrw_s * m.sourcePerimeter + jsrwg_s * weff) * scale * temp_is_scale;
                if(!(isr_db > 0.0)) { isr_db = diode_isr_d * scale_diode; }
                if(!(isr_sb > 0.0)) { isr_sb = diode_isr_s * scale_diode; }
            }
            else
            {
                // Preserve the legacy scaling behavior for Isr when jsr/jsrw/jsrwg are unset.
                isr_db = diode_isr_d * scale_diode;
                isr_sb = diode_isr_s * scale_diode;
            }

            m.diode_db.Is = is_db;
            m.diode_db.Isr = isr_db;
            m.diode_db.Area = 1.0;
            m.diode_sb.Is = is_sb;
            m.diode_sb.Isr = isr_sb;
            m.diode_sb.Area = 1.0;
        }
        else
        {
            // Legacy: a single diode saturation current scaled only by m.
            m.diode_db.Is = diode_is_d;
            m.diode_db.Area = scale_diode;
            m.diode_sb.Is = diode_is_s;
            m.diode_sb.Area = scale_diode;
        }

        // Scale breakdown current with the same geometry/multiplier factor as Is so the fitted breakdown knee stays scale-invariant.
        // This preserves legacy behavior when geometry scaling is not used, while keeping breakdown consistent when js/jsw/jswg scaling is enabled.
        double const is_eff_db{m.diode_db.Is * m.diode_db.Area};
        double const is_eff_sb{m.diode_sb.Is * m.diode_sb.Area};
        double const ref_is_d{::std::max(diode_is_d, 1e-30)};
        double const ref_is_s{::std::max(diode_is_s, 1e-30)};
        m.diode_db.Ibv = (m.ibvd > 0.0) ? (m.ibvd * is_eff_db / ref_is_d) : 0.0;
        m.diode_sb.Ibv = (m.ibvs > 0.0) ? (m.ibvs * is_eff_sb / ref_is_s) : 0.0;

        details::attach_body_diodes<is_pmos>(nd, ns_int, m.nbd_int ? m.nbd_int : nb, m.nbs_int ? m.nbs_int : nb, m.diode_db, m.diode_sb);
        (void)prepare_foundation_define(::phy_engine::model::model_reserve_type<PN_junction>, m.diode_db);
        (void)prepare_foundation_define(::phy_engine::model::model_reserve_type<PN_junction>, m.diode_sb);
        return true;
    }

    static_assert(::phy_engine::model::defines::can_prepare_foundation<bsim3v32_nmos>);
    static_assert(::phy_engine::model::defines::can_prepare_foundation<bsim3v32_pmos>);

    template <bool is_pmos>
    inline bool load_temperature_define(::phy_engine::model::model_reserve_type_t<bsim3v32_mos<is_pmos>>, bsim3v32_mos<is_pmos>& m, double temp) noexcept
    {
        double const base_temp{m.Temp_override ? m.Temp_override_value : temp};
        m.Temp = base_temp + m.dtemp;
        m.dc_bias_valid = false;
        m.dc_bias_phys_valid = false;
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
        auto const nd{m.nd_int ? m.nd_int : nd_ext};                  // diffusion D'
        auto const ns{m.ns_int ? m.ns_int : ns_ext};                  // diffusion S'
        auto const ng{m.ng_int ? m.ng_int : ng_ext};                  // intrinsic gate
        auto const nb{nb_raw ? (m.nb_int ? m.nb_int : nb_raw) : ns};  // allow 3-terminal usage (bulk tied to source diffusion)
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

        // Bulk resistance (linear stamp): between external B and internal B'.
        double const rb_eff{scale > 0.0 ? (m.Rb / scale) : m.Rb};
        if(m.nb_int && nb_raw) { details::stamp_resistor(mna, nb_raw, nb, rb_eff); }
        double const rbdb_eff{scale > 0.0 ? (m.rbdb / scale) : m.rbdb};
        double const rbsb_eff{scale > 0.0 ? (m.rbsb / scale) : m.rbsb};
        if(m.nbd_int && nb_raw) { details::stamp_resistor(mna, nb, m.nbd_int, ::std::max(rbdb_eff, 0.0)); }
        if(m.nbs_int && nb_raw) { details::stamp_resistor(mna, nb, m.nbs_int, ::std::max(rbsb_eff, 0.0)); }

        // Physical-terminal controlling voltages (no D/S swapping).
        // Apply SPICE-style limiting to improve Newton convergence for all auxiliary branches too (clean-room).
        double const sgn{is_pmos ? -1.0 : 1.0};
        double const Vd_p{nd->node_information.an.voltage.real()};
        double const Vs_p{ns->node_information.an.voltage.real()};
        double const Vg_p{ng->node_information.an.voltage.real()};
        double const Vb_p{nb->node_information.an.voltage.real()};

        double const vto_lim{details::bsim3v32_vth0_temp_mag(m.Vth0, m.Temp, m.tnom, m.kt1, m.kt2)};

        double vgs_s_phys{sgn * (Vg_p - Vs_p)};
        double vds_s_phys{sgn * (Vd_p - Vs_p)};
        double vbs_s_phys{sgn * (Vb_p - Vs_p)};

        double vgs_s_phys_eval{vgs_s_phys};
        double vds_s_phys_eval{vds_s_phys};
        double vbs_s_phys_eval{vbs_s_phys};
        if(m.dc_bias_phys_valid)
        {
            vgs_s_phys_eval = details::bsim3v32_fetlim(vgs_s_phys_eval, m.vgs_s_phys_last, vto_lim);
            vds_s_phys_eval = details::bsim3v32_limvds(vds_s_phys_eval, m.vds_s_phys_last);
            vbs_s_phys_eval = details::bsim3v32_fetlim(vbs_s_phys_eval, m.vbs_s_phys_last, 0.0);
        }
        m.dc_bias_phys_valid = true;
        m.vgs_s_phys_last = vgs_s_phys_eval;
        m.vds_s_phys_last = vds_s_phys_eval;
        m.vbs_s_phys_last = vbs_s_phys_eval;

        double const Vgs_phys_eval{sgn * vgs_s_phys_eval};
        double const Vds_phys_eval{sgn * vds_s_phys_eval};
        double const Vbs_phys_eval{sgn * vbs_s_phys_eval};

        double const vgd_s_phys_eval{vgs_s_phys_eval - vds_s_phys_eval};
        double const vgb_s_phys_eval{vgs_s_phys_eval - vbs_s_phys_eval};
        double const Vgd_phys_eval{sgn * vgd_s_phys_eval};
        double const Vgb_phys_eval{sgn * vgb_s_phys_eval};

        // Off-state leakage currents (GIDL/GISL) between diffusion nodes and bulk.
        if((m.agidl > 0.0 && m.bgidl > 0.0) || (m.agisl > 0.0 && m.bgisl > 0.0))
        {
            // Drain-to-bulk GIDL current source (D -> B).
            auto const idb_dual = details::bsim3v32_gidl_drain_s(m,
                                                                 details::bsim3v32_dual3_vgs(vgs_s_phys_eval),
                                                                 details::bsim3v32_dual3_vds(vds_s_phys_eval),
                                                                 details::bsim3v32_dual3_vbs(vbs_s_phys_eval));
            double const Idb{sgn * idb_dual.val};
            double const gVgs_db{idb_dual.dvgs};
            double const gVds_db{idb_dual.dvds};
            double const gVbs_db{idb_dual.dvbs};

            double const gVd_db{gVds_db};
            double const gVg_db{gVgs_db};
            double const gVb_db{gVbs_db};
            double const gVs_db{-(gVgs_db + gVds_db + gVbs_db)};

            double const Ieq_db{Idb - (gVgs_db * Vgs_phys_eval + gVds_db * Vds_phys_eval + gVbs_db * Vbs_phys_eval)};
            details::stamp_current_4node(mna, nd, nb, nd, ng, ns, nb, gVd_db * scale, gVg_db * scale, gVs_db * scale, gVb_db * scale, Ieq_db * scale);

            // Source-to-bulk GISL current source (S -> B).
            auto const isb_dual = details::bsim3v32_gidl_source_s(m,
                                                                  details::bsim3v32_dual3_vgs(vgs_s_phys_eval),
                                                                  details::bsim3v32_dual3_vds(vds_s_phys_eval),
                                                                  details::bsim3v32_dual3_vbs(vbs_s_phys_eval));
            double const Isb{sgn * isb_dual.val};
            double const gVgs_sb{isb_dual.dvgs};
            double const gVds_sb{isb_dual.dvds};
            double const gVbs_sb{isb_dual.dvbs};

            double const gVd_sb{gVds_sb};
            double const gVg_sb{gVgs_sb};
            double const gVb_sb{gVbs_sb};
            double const gVs_sb{-(gVgs_sb + gVds_sb + gVbs_sb)};

            double const Ieq_sb{Isb - (gVgs_sb * Vgs_phys_eval + gVds_sb * Vds_phys_eval + gVbs_sb * Vbs_phys_eval)};
            details::stamp_current_4node(mna, ns, nb, nd, ng, ns, nb, gVd_sb * scale, gVg_sb * scale, gVs_sb * scale, gVb_sb * scale, Ieq_sb * scale);
        }

        // Gate-to-bulk leakage current source (G -> B). Independent of channel mode.
        if(m.aigb > 0.0 && m.bigb > 0.0)
        {
            auto const igb_dual = details::bsim3v32_igb_s(m, details::bsim3v32_dual3_vgs(vgb_s_phys_eval));
            double const Ig{sgn * igb_dual.val};  // current from G to B (positive means leaving gate)
            double const dId_dVgb{igb_dual.dvgs};

            double const dId_dVg{dId_dVgb};
            double const dId_dVb{-dId_dVgb};
            double const Ieq_gb{Ig - dId_dVgb * Vgb_phys_eval};
            details::stamp_current_2node(mna, ng, nb, dId_dVg * scale, dId_dVb * scale, Ieq_gb * scale);
        }

        // Gate-to-source leakage current source (G -> S) in the physical terminal orientation.
        // Note: when bigs is unset, bsim3v32_igs_s falls back to bigb, so we must not gate on (bigs > 0) here.
        if(m.aigs > 0.0 && ((m.bigs > 0.0) || (m.bigb > 0.0)))
        {
            auto const igs_dual = details::bsim3v32_igs_s(m, details::bsim3v32_dual3_vgs(vgs_s_phys_eval));
            double const Ig{sgn * igs_dual.val};
            double const dId_dVgs{igs_dual.dvgs};

            double const dId_dVg{dId_dVgs};
            double const dId_dVs{-dId_dVgs};
            double const Ieq_gs{Ig - dId_dVgs * Vgs_phys_eval};
            details::stamp_current_2node(mna, ng, ns, dId_dVg * scale, dId_dVs * scale, Ieq_gs * scale);
        }

        // Gate-to-drain leakage current source (G -> D) in the physical terminal orientation.
        // Note: when bigd is unset, bsim3v32_igd_s falls back to bigb, so we must not gate on (bigd > 0) here.
        if(m.aigd > 0.0 && ((m.bigd > 0.0) || (m.bigb > 0.0)))
        {
            auto const igd_dual = details::bsim3v32_igd_s(m, details::bsim3v32_dual3_vgs(vgd_s_phys_eval));
            double const Ig{sgn * igd_dual.val};
            double const dId_dVgd{igd_dual.dvgs};

            double const dId_dVg{dId_dVgd};
            double const dId_dVd{-dId_dVgd};
            double const Ieq_gd{Ig - dId_dVgd * Vgd_phys_eval};
            details::stamp_current_2node(mna, ng, nd, dId_dVg * scale, dId_dVd * scale, Ieq_gd * scale);
        }

        // Channel mode selection (SPICE-style): swap D/S for the intrinsic channel if needed.
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

        // Impact ionization / substrate current (modeled as drain->bulk current).
        if(m.alpha0 > 0.0 && m.beta0 > 0.0)
        {
            auto const ii_dual = details::bsim3v32_impact_ionization_s(m, ids_dual, details::bsim3v32_dual3_vds(vds_s_eval));
            double const Iii{sgn * ii_dual.val};
            double const gVgs{ii_dual.dvgs};
            double const gVds{ii_dual.dvds};
            double const gVbs{ii_dual.dvbs};

            double const gVd{gVds};
            double const gVg{gVgs};
            double const gVb{gVbs};
            double const gVs{-(gVgs + gVds + gVbs)};

            double const Ieq{Iii - (gVd * Vd_ch + gVg * Vg + gVs * Vs_ch + gVb * Vb)};
            details::stamp_current_4node(mna, nd_ch, nb, nd_ch, ng, ns_ch, nb, gVd * scale, gVg * scale, gVs * scale, gVb * scale, Ieq * scale);
        }
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
        auto const nb{nb_raw ? (m.nb_int ? m.nb_int : nb_raw) : ns};
        if(nd_ext && ns_ext && nb) [[likely]]
        {
            details::attach_body_diodes<is_pmos>(nd, ns, m.nbd_int ? m.nbd_int : nb, m.nbs_int ? m.nbs_int : nb, m.diode_db, m.diode_sb);
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
        auto const nb{nb_raw ? (m.nb_int ? m.nb_int : nb_raw) : ns};  // allow 3-terminal usage (bulk tied to source diffusion)
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

            double const rb_eff{scale > 0.0 ? (m.Rb / scale) : m.Rb};
            if(m.nb_int && nb_raw) { details::stamp_resistor(mna, nb_raw, nb, rb_eff); }
            double const rbdb_eff{scale > 0.0 ? (m.rbdb / scale) : m.rbdb};
            double const rbsb_eff{scale > 0.0 ? (m.rbsb / scale) : m.rbsb};
            if(m.nbd_int && nb_raw) { details::stamp_resistor(mna, nb, m.nbd_int, ::std::max(rbdb_eff, 0.0)); }
            if(m.nbs_int && nb_raw) { details::stamp_resistor(mna, nb, m.nbs_int, ::std::max(rbsb_eff, 0.0)); }

            // small-signal conductances
            auto* const nd_lin = m.mode_swapped ? ns : nd;
            auto* const ns_lin = m.mode_swapped ? nd : ns;
            details::stamp_gm_gds_gmb(mna, nd_lin, ng, ns_lin, nb, m.gm * scale, m.gds * scale, m.gmb * scale, 0.0);

            // GIDL/GISL incremental conductances around the real operating point.
            if((m.agidl > 0.0 && m.bgidl > 0.0) || (m.agisl > 0.0 && m.bgisl > 0.0))
            {
                double const sgn{is_pmos ? -1.0 : 1.0};
                double const Vd_p{nd->node_information.an.voltage.real()};
                double const Vs_p{ns->node_information.an.voltage.real()};
                double const Vg_p{ng->node_information.an.voltage.real()};
                double const Vb_p{nb->node_information.an.voltage.real()};

                double const vgs_s_p{sgn * (Vg_p - Vs_p)};
                double const vds_s_p{sgn * (Vd_p - Vs_p)};
                double const vbs_s_p{sgn * (Vb_p - Vs_p)};

                auto const idb_dual = details::bsim3v32_gidl_drain_s(m,
                                                                     details::bsim3v32_dual3_vgs(vgs_s_p),
                                                                     details::bsim3v32_dual3_vds(vds_s_p),
                                                                     details::bsim3v32_dual3_vbs(vbs_s_p));
                double const gVgs_db{idb_dual.dvgs};
                double const gVds_db{idb_dual.dvds};
                double const gVbs_db{idb_dual.dvbs};

                double const gVd_db{gVds_db};
                double const gVg_db{gVgs_db};
                double const gVb_db{gVbs_db};
                double const gVs_db{-(gVgs_db + gVds_db + gVbs_db)};

                details::stamp_current_4node(mna, nd, nb, nd, ng, ns, nb, gVd_db * scale, gVg_db * scale, gVs_db * scale, gVb_db * scale, 0.0);

                auto const isb_dual = details::bsim3v32_gidl_source_s(m,
                                                                      details::bsim3v32_dual3_vgs(vgs_s_p),
                                                                      details::bsim3v32_dual3_vds(vds_s_p),
                                                                      details::bsim3v32_dual3_vbs(vbs_s_p));
                double const gVgs_sb{isb_dual.dvgs};
                double const gVds_sb{isb_dual.dvds};
                double const gVbs_sb{isb_dual.dvbs};

                double const gVd_sb{gVds_sb};
                double const gVg_sb{gVgs_sb};
                double const gVb_sb{gVbs_sb};
                double const gVs_sb{-(gVgs_sb + gVds_sb + gVbs_sb)};

                details::stamp_current_4node(mna, ns, nb, nd, ng, ns, nb, gVd_sb * scale, gVg_sb * scale, gVs_sb * scale, gVb_sb * scale, 0.0);
            }

            // Impact ionization / substrate current incremental conductances around OP.
            if(m.alpha0 > 0.0 && m.beta0 > 0.0)
            {
                double const sgn{is_pmos ? -1.0 : 1.0};
                double const Vd_p{nd_lin->node_information.an.voltage.real()};
                double const Vs_p{ns_lin->node_information.an.voltage.real()};
                double const Vg_p{ng->node_information.an.voltage.real()};
                double const Vb_p{nb->node_information.an.voltage.real()};

                double const vgs_s_p{sgn * (Vg_p - Vs_p)};
                double const vds_s_p{sgn * (Vd_p - Vs_p)};
                double const vbs_s_p{sgn * (Vb_p - Vs_p)};

                double const vt{details::thermal_voltage(m.Temp)};
                auto const vgs_dual = details::bsim3v32_dual3_vgs(vgs_s_p);
                auto const vds_dual = details::bsim3v32_dual3_vds(vds_s_p);
                auto const vbs_dual = details::bsim3v32_dual3_vbs(vbs_s_p);
                auto const ids_dual = details::bsim3v32_ids_core(m,
                                                                 vgs_dual,
                                                                 vds_dual,
                                                                 vbs_dual,
                                                                 vt,
                                                                 static_cast<details::bsim3v32_core_cache_t<details::bsim3v32_dual3>*>(nullptr));
                auto const ii_dual = details::bsim3v32_impact_ionization_s(m, ids_dual, vds_dual);

                double const gVgs{ii_dual.dvgs};
                double const gVds{ii_dual.dvds};
                double const gVbs{ii_dual.dvbs};

                double const gVd{gVds};
                double const gVg{gVgs};
                double const gVb{gVbs};
                double const gVs{-(gVgs + gVds + gVbs)};

                // Small-signal only: Ieq = 0
                details::stamp_current_4node(mna, nd_lin, nb, nd_lin, ng, ns_lin, nb, gVd * scale, gVg * scale, gVs * scale, gVb * scale, 0.0);
            }

            // Gate-to-bulk leakage small-signal conductance around OP (G -> B).
            if(m.aigb > 0.0 && m.bigb > 0.0)
            {
                double const sgn_gb{is_pmos ? -1.0 : 1.0};
                double const Vg_p{ng->node_information.an.voltage.real()};
                double const Vb_p{nb->node_information.an.voltage.real()};
                double const vgb_s_p{sgn_gb * (Vg_p - Vb_p)};

                auto const igb_dual = details::bsim3v32_igb_s(m, details::bsim3v32_dual3_vgs(vgb_s_p));
                double const dId_dVgb{igb_dual.dvgs};
                double const dId_dVg{dId_dVgb};
                double const dId_dVb{-dId_dVgb};
                details::stamp_current_2node(mna, ng, nb, dId_dVg * scale, dId_dVb * scale, 0.0);
            }

            // Gate-to-source leakage small-signal conductance (G -> S), physical orientation.
            // Note: when bigs is unset, bsim3v32_igs_s falls back to bigb, so we must not gate on (bigs > 0) here.
            if(m.aigs > 0.0 && ((m.bigs > 0.0) || (m.bigb > 0.0)))
            {
                double const sgn_gs{is_pmos ? -1.0 : 1.0};
                double const Vg_p{ng->node_information.an.voltage.real()};
                double const Vs_p{ns->node_information.an.voltage.real()};
                double const vgs_s_p{sgn_gs * (Vg_p - Vs_p)};

                auto const igs_dual = details::bsim3v32_igs_s(m, details::bsim3v32_dual3_vgs(vgs_s_p));
                double const dId_dVgs{igs_dual.dvgs};
                double const dId_dVg{dId_dVgs};
                double const dId_dVs{-dId_dVgs};
                details::stamp_current_2node(mna, ng, ns, dId_dVg * scale, dId_dVs * scale, 0.0);
            }

            // Gate-to-drain leakage small-signal conductance (G -> D), physical orientation.
            // Note: when bigd is unset, bsim3v32_igd_s falls back to bigb, so we must not gate on (bigd > 0) here.
            if(m.aigd > 0.0 && ((m.bigd > 0.0) || (m.bigb > 0.0)))
            {
                double const sgn_gd{is_pmos ? -1.0 : 1.0};
                double const Vg_p{ng->node_information.an.voltage.real()};
                double const Vd_p{nd->node_information.an.voltage.real()};
                double const vgd_s_p{sgn_gd * (Vg_p - Vd_p)};

                auto const igd_dual = details::bsim3v32_igd_s(m, details::bsim3v32_dual3_vgs(vgd_s_p));
                double const dId_dVgd{igd_dual.dvgs};
                double const dId_dVg{dId_dVgd};
                double const dId_dVd{-dId_dVgd};
                details::stamp_current_2node(mna, ng, nd, dId_dVg * scale, dId_dVd * scale, 0.0);
            }

            // fixed caps (including overlap)
            double const weff{::std::max(m.W - 2.0 * ::std::max(m.dwc, 0.0), 0.0)};
            double const leff{::std::max(m.L - 2.0 * ::std::max(m.dlc, 0.0), 1e-18)};
            bool const ovl_charge_based{m.acm != 0.0};
            double const cgs_ovl{ovl_charge_based ? 0.0 : (m.cgso * weff)};
            double const cgd_ovl{ovl_charge_based ? 0.0 : (m.cgdo * weff)};
            double const cgb_ovl{ovl_charge_based ? 0.0 : (m.cgbo * leff)};
            double const cgs_fix{m.Cgs + cgs_ovl};
            double const cgd_fix{m.Cgd + cgd_ovl};
            double const cgb_fix{m.Cgb + cgb_ovl};
            details::stamp_cap_ac(mna, ng, ns, cgs_fix * scale, omega);
            details::stamp_cap_ac(mna, ng, nd, cgd_fix * scale, omega);
            details::stamp_cap_ac(mna, ng, nb, cgb_fix * scale, omega);

            // Intrinsic (bias-dependent) caps captured at the real operating point (save_op).
            if(m.capMod >= 2.5)
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
            details::attach_body_diodes<is_pmos>(nd, ns, m.nbd_int ? m.nbd_int : nb, m.nbs_int ? m.nbs_int : nb, m.diode_db, m.diode_sb);
            (void)iterate_ac_define(::phy_engine::model::model_reserve_type<PN_junction>, m.diode_db, mna, omega);
            (void)iterate_ac_define(::phy_engine::model::model_reserve_type<PN_junction>, m.diode_sb, mna, omega);

            // optional depletion caps across the same junctions
            if(m.cj != 0.0 || m.cjd != 0.0 || m.cjs != 0.0 || m.cjsw != 0.0 || m.cjswd != 0.0 || m.cjsws != 0.0 || m.cjswg != 0.0 || m.cjswgd != 0.0 ||
               m.cjswgs != 0.0)
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
        auto const nb{nb_raw ? (m.nb_int ? m.nb_int : nb_raw) : ns};  // allow 3-terminal usage (bulk tied to source diffusion)

        double const scale{(m.m_mult > 0.0 ? m.m_mult : 0.0) * (m.nf > 0.0 ? m.nf : 0.0)};
        double const weff{::std::max(m.W - 2.0 * ::std::max(m.dwc, 0.0), 0.0)};
        double const leff{::std::max(m.L - 2.0 * ::std::max(m.dlc, 0.0), 1e-18)};
        bool const ovl_charge_based{m.acm != 0.0};
        double const cgs_ovl{ovl_charge_based ? 0.0 : (m.cgso * weff)};
        double const cgd_ovl{ovl_charge_based ? 0.0 : (m.cgdo * weff)};
        double const cgb_ovl{ovl_charge_based ? 0.0 : (m.cgbo * leff)};
        double const cgs_fix{m.Cgs + cgs_ovl};
        double const cgd_fix{m.Cgd + cgd_ovl};
        double const cgb_fix{m.Cgb + cgb_ovl};
        details::step_cap_tr(m.cgs_state, ng, ns, cgs_fix * scale, nstep);
        details::step_cap_tr(m.cgd_state, ng, nd, cgd_fix * scale, nstep);
        details::step_cap_tr(m.cgb_state, ng, nb, cgb_fix * scale, nstep);

        // Intrinsic charge/capacitance model (linearized at the previous step's bias).
        m.cap_mode_swapped = m.mode_swapped;
        if(m.capMod >= 2.5)
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
                details::bsim3v32_meyer_intrinsic_caps<is_pmos>(m, sgn * Vgs, sgn * Vds, sgn * Vbs, vt, cgs_i, cgd_i, cgb_i);
                details::step_cap_tr(m.cgs_int_state, ng, ns_lin, cgs_i * scale, nstep);
                details::step_cap_tr(m.cgd_int_state, ng, nd_lin, cgd_i * scale, nstep);
                details::step_cap_tr(m.cgb_int_state, ng, nb, cgb_i * scale, nstep);
            }
        }

        // Keep diode vlimit history aligned with node voltages.
        if(nd && ns && nb)
        {
            details::attach_body_diodes<is_pmos>(nd, ns, m.nbd_int ? m.nbd_int : nb, m.nbs_int ? m.nbs_int : nb, m.diode_db, m.diode_sb);
            (void)step_changed_tr_define(::phy_engine::model::model_reserve_type<PN_junction>, m.diode_db, nlaststep, nstep);
            (void)step_changed_tr_define(::phy_engine::model::model_reserve_type<PN_junction>, m.diode_sb, nlaststep, nstep);

            // optional depletion caps across the same junctions (linearized per step at previous bias)
            if(m.cj != 0.0 || m.cjd != 0.0 || m.cjs != 0.0 || m.cjsw != 0.0 || m.cjswd != 0.0 || m.cjsws != 0.0 || m.cjswg != 0.0 || m.cjswgd != 0.0 ||
               m.cjswgs != 0.0)
            {
                auto* db_a = m.diode_db.pins[0].nodes;
                auto* db_b = m.diode_db.pins[1].nodes;
                auto* sb_a = m.diode_sb.pins[0].nodes;
                auto* sb_b = m.diode_sb.pins[1].nodes;
                if(db_a && db_b && sb_a && sb_b)
                {
                    double const vbd = db_a->node_information.an.voltage.real() - db_b->node_information.an.voltage.real();
                    double const vbs = sb_a->node_information.an.voltage.real() - sb_b->node_information.an.voltage.real();

                    double const cjd0 = (m.cjd != 0.0) ? m.cjd : m.cj;
                    double const cjs0 = (m.cjs != 0.0) ? m.cjs : m.cj;
                    double const cj_d_eff = details::bsim3v32_temp_linear_scale_clamped(cjd0, m.tcj, m.Temp, m.tnom, 0.0);
                    double const cj_s_eff = details::bsim3v32_temp_linear_scale_clamped(cjs0, m.tcj, m.Temp, m.tnom, 0.0);
                    double const cjswd0 = (m.cjswd != 0.0) ? m.cjswd : m.cjsw;
                    double const cjsws0 = (m.cjsws != 0.0) ? m.cjsws : m.cjsw;
                    double const cjswgd0 = (m.cjswgd != 0.0) ? m.cjswgd : m.cjswg;
                    double const cjswgs0 = (m.cjswgs != 0.0) ? m.cjswgs : m.cjswg;
                    double const cjsw_d_eff = details::bsim3v32_temp_linear_scale_clamped(cjswd0, m.tcjsw, m.Temp, m.tnom, 0.0);
                    double const cjsw_s_eff = details::bsim3v32_temp_linear_scale_clamped(cjsws0, m.tcjsw, m.Temp, m.tnom, 0.0);
                    double const cjswg_d_eff = details::bsim3v32_temp_linear_scale_clamped(cjswgd0, m.tcjswg, m.Temp, m.tnom, 0.0);
                    double const cjswg_s_eff = details::bsim3v32_temp_linear_scale_clamped(cjswgs0, m.tcjswg, m.Temp, m.tnom, 0.0);
                    double const pbd0 = (m.pbd > 0.0) ? m.pbd : m.pb;
                    double const pbs0 = (m.pbs > 0.0) ? m.pbs : m.pb;
                    double const pb_d_eff = details::bsim3v32_temp_linear_scale_clamped(pbd0, m.tpb, m.Temp, m.tnom, 1e-12);
                    double const pb_s_eff = details::bsim3v32_temp_linear_scale_clamped(pbs0, m.tpb, m.Temp, m.tnom, 1e-12);
                    double const pbsw_d0 = (m.pbswd > 0.0) ? m.pbswd : ((m.pbsw > 0.0) ? m.pbsw : m.pb);
                    double const pbsw_s0 = (m.pbsws > 0.0) ? m.pbsws : ((m.pbsw > 0.0) ? m.pbsw : m.pb);
                    double const tpbsw_d0 = ((m.pbswd > 0.0) || (m.pbsw > 0.0)) ? m.tpbsw : m.tpb;
                    double const tpbsw_s0 = ((m.pbsws > 0.0) || (m.pbsw > 0.0)) ? m.tpbsw : m.tpb;
                    double const pbsw_d_eff = details::bsim3v32_temp_linear_scale_clamped(pbsw_d0, tpbsw_d0, m.Temp, m.tnom, 1e-12);
                    double const pbsw_s_eff = details::bsim3v32_temp_linear_scale_clamped(pbsw_s0, tpbsw_s0, m.Temp, m.tnom, 1e-12);

                    double const pbswg_d0 = (m.pbswgd > 0.0) ? m.pbswgd : ((m.pbswg > 0.0) ? m.pbswg : pbsw_d0);
                    double const pbswg_s0 = (m.pbswgs > 0.0) ? m.pbswgs : ((m.pbswg > 0.0) ? m.pbswg : pbsw_s0);
                    double const tpbswg_d0 = ((m.pbswgd > 0.0) || (m.pbswg > 0.0)) ? m.tpbswg : tpbsw_d0;
                    double const tpbswg_s0 = ((m.pbswgs > 0.0) || (m.pbswg > 0.0)) ? m.tpbswg : tpbsw_s0;
                    double const pbswg_d_eff = details::bsim3v32_temp_linear_scale_clamped(pbswg_d0, tpbswg_d0, m.Temp, m.tnom, 1e-12);
                    double const pbswg_s_eff = details::bsim3v32_temp_linear_scale_clamped(pbswg_s0, tpbswg_s0, m.Temp, m.tnom, 1e-12);

                    double const cbd_bottom0 = cj_d_eff * m.drainArea;
                    double const cbs_bottom0 = cj_s_eff * m.sourceArea;
                    double const cbd_side0 = cjsw_d_eff * m.drainPerimeter;
                    double const cbs_side0 = cjsw_s_eff * m.sourcePerimeter;
                    double const cbd_gate0 = cjswg_d_eff * weff;
                    double const cbs_gate0 = cjswg_s_eff * weff;

                    double const mj_d{::std::max((m.mjd >= 0.0) ? m.mjd : m.mj, 0.0)};
                    double const mj_s{::std::max((m.mjs >= 0.0) ? m.mjs : m.mj, 0.0)};
                    double const mjsw_d{::std::max((m.mjswd >= 0.0) ? m.mjswd : m.mjsw, 0.0)};
                    double const mjsw_s{::std::max((m.mjsws >= 0.0) ? m.mjsws : m.mjsw, 0.0)};
                    double const mjswg_d{::std::max((m.mjswgd >= 0.0) ? m.mjswgd : m.mjswg, 0.0)};
                    double const mjswg_s{::std::max((m.mjswgs >= 0.0) ? m.mjswgs : m.mjswg, 0.0)};

                    double const fc_d{(m.fcd >= 0.0) ? m.fcd : m.fc};
                    double const fc_s{(m.fcs >= 0.0) ? m.fcs : m.fc};
                    double const cbd = details::bsim3v32_junction_cap(cbd_bottom0, vbd, pb_d_eff, mj_d, fc_d) +
                                       details::bsim3v32_junction_cap(cbd_side0, vbd, pbsw_d_eff, mjsw_d, fc_d) +
                                       details::bsim3v32_junction_cap(cbd_gate0, vbd, pbswg_d_eff, mjswg_d, fc_d);
                    double const cbs = details::bsim3v32_junction_cap(cbs_bottom0, vbs, pb_s_eff, mj_s, fc_s) +
                                       details::bsim3v32_junction_cap(cbs_side0, vbs, pbsw_s_eff, mjsw_s, fc_s) +
                                       details::bsim3v32_junction_cap(cbs_gate0, vbs, pbswg_s_eff, mjswg_s, fc_s);

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
        auto const nb{nb_raw ? (m.nb_int ? m.nb_int : nb_raw) : ns};  // allow 3-terminal usage (bulk tied to source diffusion)
        if(nd_ext && ng_ext && ns_ext && nb) [[likely]]
        {
            details::attach_body_diodes<is_pmos>(nd, ns, m.nbd_int ? m.nbd_int : nb, m.nbs_int ? m.nbs_int : nb, m.diode_db, m.diode_sb);
            (void)iterate_tr_define(::phy_engine::model::model_reserve_type<PN_junction>, m.diode_db, mna, 0.0);
            (void)iterate_tr_define(::phy_engine::model::model_reserve_type<PN_junction>, m.diode_sb, mna, 0.0);

            details::stamp_cap_tr(mna, ng, ns, m.cgs_state);
            details::stamp_cap_tr(mna, ng, nd, m.cgd_state);
            details::stamp_cap_tr(mna, ng, nb, m.cgb_state);

            auto* const nd_lin = m.cap_mode_swapped ? ns : nd;
            auto* const ns_lin = m.cap_mode_swapped ? nd : ns;
            if(m.capMod >= 2.5)
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
        auto const nb2{nb2_raw ? (m.nb_int ? m.nb_int : nb2_raw) : ns_dio};
        if(nd2 && ns2 && nb2)
        {
            details::attach_body_diodes<is_pmos>(nd_dio, ns_dio, m.nbd_int ? m.nbd_int : nb2, m.nbs_int ? m.nbs_int : nb2, m.diode_db, m.diode_sb);
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
        auto const nb{nb_raw ? (m.nb_int ? m.nb_int : nb_raw) : ns};
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

        details::bsim3v32_meyer_intrinsic_caps<is_pmos>(m, sgn * Vgs, sgn * Vds, sgn * Vbs, vt, m.cgs_intr_ac, m.cgd_intr_ac, m.cgb_intr_ac);

        // Intrinsic charge C-matrix (enabled when capMod!=0) captured at the real operating point.
        for(int i{}; i < 4; ++i)
        {
            for(int j{}; j < 4; ++j) { m.cmat_ac[i][j] = 0.0; }
        }
        if(m.capMod >= 2.5) { details::bsim3v32_cmatrix_capmod0_simple<is_pmos>(m, Vd, Vg, Vs, Vb, m.cmat_ac); }

        // Depletion caps at the diode biases (real OP). Uses the same diode node attachments.
        m.cbd_ac = 0.0;
        m.cbs_ac = 0.0;
        if(m.cj != 0.0 || m.cjd != 0.0 || m.cjs != 0.0 || m.cjsw != 0.0 || m.cjswd != 0.0 || m.cjsws != 0.0 || m.cjswg != 0.0 || m.cjswgd != 0.0 ||
           m.cjswgs != 0.0)
        {
            details::attach_body_diodes<is_pmos>(nd, ns, m.nbd_int ? m.nbd_int : nb, m.nbs_int ? m.nbs_int : nb, m.diode_db, m.diode_sb);
            auto* db_a = m.diode_db.pins[0].nodes;
            auto* db_b = m.diode_db.pins[1].nodes;
            auto* sb_a = m.diode_sb.pins[0].nodes;
            auto* sb_b = m.diode_sb.pins[1].nodes;
            if(db_a && db_b && sb_a && sb_b)
            {
                double const vbd = db_a->node_information.an.voltage.real() - db_b->node_information.an.voltage.real();
                double const vbs = sb_a->node_information.an.voltage.real() - sb_b->node_information.an.voltage.real();

                double const cjd0 = (m.cjd != 0.0) ? m.cjd : m.cj;
                double const cjs0 = (m.cjs != 0.0) ? m.cjs : m.cj;
                double const cj_d_eff = details::bsim3v32_temp_linear_scale_clamped(cjd0, m.tcj, m.Temp, m.tnom, 0.0);
                double const cj_s_eff = details::bsim3v32_temp_linear_scale_clamped(cjs0, m.tcj, m.Temp, m.tnom, 0.0);
                double const cjswd0 = (m.cjswd != 0.0) ? m.cjswd : m.cjsw;
                double const cjsws0 = (m.cjsws != 0.0) ? m.cjsws : m.cjsw;
                double const cjswgd0 = (m.cjswgd != 0.0) ? m.cjswgd : m.cjswg;
                double const cjswgs0 = (m.cjswgs != 0.0) ? m.cjswgs : m.cjswg;
                double const cjsw_d_eff = details::bsim3v32_temp_linear_scale_clamped(cjswd0, m.tcjsw, m.Temp, m.tnom, 0.0);
                double const cjsw_s_eff = details::bsim3v32_temp_linear_scale_clamped(cjsws0, m.tcjsw, m.Temp, m.tnom, 0.0);
                double const cjswg_d_eff = details::bsim3v32_temp_linear_scale_clamped(cjswgd0, m.tcjswg, m.Temp, m.tnom, 0.0);
                double const cjswg_s_eff = details::bsim3v32_temp_linear_scale_clamped(cjswgs0, m.tcjswg, m.Temp, m.tnom, 0.0);
                double const pbd0 = (m.pbd > 0.0) ? m.pbd : m.pb;
                double const pbs0 = (m.pbs > 0.0) ? m.pbs : m.pb;
                double const pb_d_eff = details::bsim3v32_temp_linear_scale_clamped(pbd0, m.tpb, m.Temp, m.tnom, 1e-12);
                double const pb_s_eff = details::bsim3v32_temp_linear_scale_clamped(pbs0, m.tpb, m.Temp, m.tnom, 1e-12);
                double const pbsw_d0 = (m.pbswd > 0.0) ? m.pbswd : ((m.pbsw > 0.0) ? m.pbsw : m.pb);
                double const pbsw_s0 = (m.pbsws > 0.0) ? m.pbsws : ((m.pbsw > 0.0) ? m.pbsw : m.pb);
                double const tpbsw_d0 = ((m.pbswd > 0.0) || (m.pbsw > 0.0)) ? m.tpbsw : m.tpb;
                double const tpbsw_s0 = ((m.pbsws > 0.0) || (m.pbsw > 0.0)) ? m.tpbsw : m.tpb;
                double const pbsw_d_eff = details::bsim3v32_temp_linear_scale_clamped(pbsw_d0, tpbsw_d0, m.Temp, m.tnom, 1e-12);
                double const pbsw_s_eff = details::bsim3v32_temp_linear_scale_clamped(pbsw_s0, tpbsw_s0, m.Temp, m.tnom, 1e-12);

                double const pbswg_d0 = (m.pbswgd > 0.0) ? m.pbswgd : ((m.pbswg > 0.0) ? m.pbswg : pbsw_d0);
                double const pbswg_s0 = (m.pbswgs > 0.0) ? m.pbswgs : ((m.pbswg > 0.0) ? m.pbswg : pbsw_s0);
                double const tpbswg_d0 = ((m.pbswgd > 0.0) || (m.pbswg > 0.0)) ? m.tpbswg : tpbsw_d0;
                double const tpbswg_s0 = ((m.pbswgs > 0.0) || (m.pbswg > 0.0)) ? m.tpbswg : tpbsw_s0;
                double const pbswg_d_eff = details::bsim3v32_temp_linear_scale_clamped(pbswg_d0, tpbswg_d0, m.Temp, m.tnom, 1e-12);
                double const pbswg_s_eff = details::bsim3v32_temp_linear_scale_clamped(pbswg_s0, tpbswg_s0, m.Temp, m.tnom, 1e-12);

                double const weff{::std::max(m.W - 2.0 * ::std::max(m.dwc, 0.0), 0.0)};
                double const cbd_bottom0 = cj_d_eff * m.drainArea;
                double const cbs_bottom0 = cj_s_eff * m.sourceArea;
                double const cbd_side0 = cjsw_d_eff * m.drainPerimeter;
                double const cbs_side0 = cjsw_s_eff * m.sourcePerimeter;
                double const cbd_gate0 = cjswg_d_eff * weff;
                double const cbs_gate0 = cjswg_s_eff * weff;

                double const mj_d{::std::max((m.mjd >= 0.0) ? m.mjd : m.mj, 0.0)};
                double const mj_s{::std::max((m.mjs >= 0.0) ? m.mjs : m.mj, 0.0)};
                double const mjsw_d{::std::max((m.mjswd >= 0.0) ? m.mjswd : m.mjsw, 0.0)};
                double const mjsw_s{::std::max((m.mjsws >= 0.0) ? m.mjsws : m.mjsw, 0.0)};
                double const mjswg_d{::std::max((m.mjswgd >= 0.0) ? m.mjswgd : m.mjswg, 0.0)};
                double const mjswg_s{::std::max((m.mjswgs >= 0.0) ? m.mjswgs : m.mjswg, 0.0)};

                double const fc_d{(m.fcd >= 0.0) ? m.fcd : m.fc};
                double const fc_s{(m.fcs >= 0.0) ? m.fcs : m.fc};

                m.cbd_ac = details::bsim3v32_junction_cap(cbd_bottom0, vbd, pb_d_eff, mj_d, fc_d) +
                           details::bsim3v32_junction_cap(cbd_side0, vbd, pbsw_d_eff, mjsw_d, fc_d) +
                           details::bsim3v32_junction_cap(cbd_gate0, vbd, pbswg_d_eff, mjswg_d, fc_d);
                m.cbs_ac = details::bsim3v32_junction_cap(cbs_bottom0, vbs, pb_s_eff, mj_s, fc_s) +
                           details::bsim3v32_junction_cap(cbs_side0, vbs, pbsw_s_eff, mjsw_s, fc_s) +
                           details::bsim3v32_junction_cap(cbs_gate0, vbs, pbswg_s_eff, mjswg_s, fc_s);
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
        m.nb_int = nullptr;
        m.nbd_int = nullptr;
        m.nbs_int = nullptr;

        double const rd_total{m.Rd + ::std::max(m.rsh, 0.0) * ::std::max(m.nrd, 0.0)};
        double const rs_total{m.Rs + ::std::max(m.rsh, 0.0) * ::std::max(m.nrs, 0.0)};

        ::std::size_t cnt{};
        if(rd_total > 0.0) { m.nd_int = __builtin_addressof(m.internal_nodes[cnt++]); }
        if(rs_total > 0.0) { m.ns_int = __builtin_addressof(m.internal_nodes[cnt++]); }
        if(m.rgateMod != 0.0 && m.rg > 0.0) { m.ng_int = __builtin_addressof(m.internal_nodes[cnt++]); }
        bool const bulk_connected{m.pins[3].nodes != nullptr};
        if(bulk_connected && m.Rb > 0.0) { m.nb_int = __builtin_addressof(m.internal_nodes[cnt++]); }
        bool const rbody_enabled{m.rbodyMod != 0.0};
        if(bulk_connected && rbody_enabled && m.rbdb > 0.0) { m.nbd_int = __builtin_addressof(m.internal_nodes[cnt++]); }
        if(bulk_connected && rbody_enabled && m.rbsb > 0.0) { m.nbs_int = __builtin_addressof(m.internal_nodes[cnt++]); }
        if(cnt != 0) { return {m.internal_nodes, cnt}; }
        return {};
    }

    static_assert(::phy_engine::model::defines::can_generate_internal_node_view<bsim3v32_nmos>);
    static_assert(::phy_engine::model::defines::can_generate_internal_node_view<bsim3v32_pmos>);

}  // namespace phy_engine::model
