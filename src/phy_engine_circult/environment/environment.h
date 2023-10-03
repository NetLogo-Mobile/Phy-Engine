#pragma once
#include <utility>

namespace phy_engine {

struct environment {
	double V_eps_max{}; // VNTOL
	double V_epsr_max{};
	double I_eps_max{}; // ABSTOL
	double I_epsr_max{};
	double charge_eps_max{}; // CHGTOL
	double g_min{}; // GMIN
	double t_TOEF{}; // TRTOL
	double temperature{}; // TEMP
	double norm_temperature{}; // TNOM
};

// RELTOL
inline constexpr double get_rel_tol(environment const& e) noexcept {
	return ::std::min(e.V_epsr_max, e.I_epsr_max);
}


}