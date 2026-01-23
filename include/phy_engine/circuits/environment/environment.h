#pragma once
#include <utility>

namespace phy_engine
{

    struct environment
    {
        double V_eps_max{};  // VNTOL
        double V_epsr_max{};
        double I_eps_max{};  // ABSTOL
        double I_epsr_max{};
        double charge_eps_max{};  // CHGTOL
        double g_min{};           // GMIN
        double r_open{};          // ROPEN (open switch/relay equivalent resistance, ohm)
        double t_TOEF{};          // TRTOL
        // SPICE-like defaults:
        // - TEMP defaults to 27C
        // - TNOM defaults to 27C
        double temperature{27.0};       // TEMP
        double norm_temperature{27.0};  // TNOM
    };

    // RELTOL
    inline constexpr double get_rel_tol(environment const& e) noexcept { return ::std::min(e.V_epsr_max, e.I_epsr_max); }

}  // namespace phy_engine
