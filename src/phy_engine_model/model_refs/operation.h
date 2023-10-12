#pragma once
#include "concept.h"

namespace phy_engine::model {

template <::phy_engine::model::model T>
inline constexpr bool init_model(T&& m) noexcept {
	if constexpr (::phy_engine::model::defines::can_init<T>) {
		return init_define(::phy_engine::model::model_reserve_type<T>, m);
	} else {
		return true;
	}
}

template <::phy_engine::model::model T>
inline constexpr bool prepare_ac(T&& m) noexcept {
	if constexpr (::phy_engine::model::defines::can_prepare_ac<T>) {
		return prepare_ac_define(::phy_engine::model::model_reserve_type<T>, m);
	} else {
		return true;
	}
}

template <::phy_engine::model::model T>
inline constexpr bool prepare_dc(T&& m) noexcept {
	if constexpr (::phy_engine::model::defines::can_prepare_dc<T>) {
		return prepare_dc_define(::phy_engine::model::model_reserve_type<T>, m);
	} else {
		return true;
	}
}

template <::phy_engine::model::model T>
inline constexpr bool prepare_tr(T&& m) noexcept {
	if constexpr (::phy_engine::model::defines::can_prepare_tr<T>) {
		return prepare_tr_define(::phy_engine::model::model_reserve_type<T>, m);
	} else {
		return true;
	}
}

template <::phy_engine::model::model T>
inline constexpr bool prepare_op(T&& m) noexcept {
	if constexpr (::phy_engine::model::defines::can_prepare_op<T>) {
		return prepare_op_define(::phy_engine::model::model_reserve_type<T>, m);
	} else if constexpr (::phy_engine::model::defines::can_prepare_dc<T>) {
		return prepare_dc_define(::phy_engine::model::model_reserve_type<T>, m);
	} else {
		return true;
	}
}

template <::phy_engine::model::model T>
inline constexpr bool prepare_trop(T&& m) noexcept {
	if constexpr (::phy_engine::model::defines::can_prepare_trop<T>) {
		return prepare_trop_define(::phy_engine::model::model_reserve_type<T>, m);
	} else if constexpr (::phy_engine::model::defines::can_prepare_tr<T>) {
		return prepare_tr_define(::phy_engine::model::model_reserve_type<T>, m);
	} else {
		return true;
	}
}

template <::phy_engine::model::model T>
inline constexpr bool iterate_ac(T&& m, [[maybe_unused]] double omega) noexcept {
	if constexpr (::phy_engine::model::defines::can_iterate_ac<T>) {
		return iterate_ac_define(::phy_engine::model::model_reserve_type<T>, m, omega);
	} else if constexpr (::phy_engine::model::defines::can_iterate_dc<T>) {
		return iterate_dc_define(::phy_engine::model::model_reserve_type<T>, m);
	} else {
		return false;
	}
}

template <::phy_engine::model::model T>
inline constexpr bool iterate_dc(T&& m) noexcept {
	if constexpr (::phy_engine::model::defines::can_iterate_dc<T>) {
		return iterate_dc_define(::phy_engine::model::model_reserve_type<T>, m);
	} else {
		return false;
	}
}

template <::phy_engine::model::model T>
inline constexpr bool iterate_tr(T&& m, [[maybe_unused]] double tTime) noexcept {
	if constexpr (::phy_engine::model::defines::can_iterate_tr<T>) {
		return iterate_tr_define(::phy_engine::model::model_reserve_type<T>, m, tTime);
	} else if constexpr (::phy_engine::model::defines::can_iterate_dc<T>) {
		return iterate_dc_define(::phy_engine::model::model_reserve_type<T>, m);
	} else {
		return false;
	}
}

template <::phy_engine::model::model T>
inline constexpr bool iterate_op(T&& m) noexcept {
	if constexpr (::phy_engine::model::defines::can_iterate_op<T>) {
		return iterate_op_define(::phy_engine::model::model_reserve_type<T>, m);
	} else if constexpr (::phy_engine::model::defines::can_iterate_dc<T>) {
		return iterate_dc_define(::phy_engine::model::model_reserve_type<T>, m);
	} else {
		return false;
	}
}

template <::phy_engine::model::model T>
inline constexpr bool iterate_trop(T&& m) noexcept {
	if constexpr (::phy_engine::model::defines::can_iterate_trop<T>) {
		return iterate_trop_define(::phy_engine::model::model_reserve_type<T>, m);
	} else if constexpr (::phy_engine::model::defines::can_iterate_tr<T>) {
		return iterate_tr_define(::phy_engine::model::model_reserve_type<T>, m, 0.0);
	} else if constexpr (::phy_engine::model::defines::can_iterate_dc<T>) {
		return iterate_dc_define(::phy_engine::model::model_reserve_type<T>, m);
	} else {
		return false;
	}
}

template <::phy_engine::model::model T>
inline constexpr bool save_op(T&& m) noexcept {
	// not a non-linear device and no need to store operating point
	if constexpr (T::device_type == ::phy_engine::model::model_device_type::non_linear) {
		if constexpr (::phy_engine::model::defines::can_save_op<T>) {
			return save_op_define(::phy_engine::model::model_reserve_type<T>, m);
		} else {
			return true;
		}
	} else {
		return true;
	}
}

template <::phy_engine::model::model T>
inline constexpr bool load_temperature(T&& m, [[maybe_unused]] double temp) noexcept {
	if constexpr (::phy_engine::model::defines::can_load_temperature<T>) {
		return load_temperature_define(::phy_engine::model::model_reserve_type<T>, m, temp);
	} else {
		return true;
	}
}

template <::phy_engine::model::model T>
inline constexpr bool step_changed_tr(T&& m, [[maybe_unused]] double tTemp, [[maybe_unused]] double nstep) noexcept {
	if constexpr (::phy_engine::model::defines::can_step_changed_tr<T>) {
		return step_changed_tr_define(::phy_engine::model::model_reserve_type<T>, m, tTemp, nstep);
	} else {
		return true;
	}
}

template <::phy_engine::model::model T>
inline constexpr bool adapt_step(T&& m, [[maybe_unused]] double& step) noexcept {
	if constexpr (::phy_engine::model::defines::can_adapt_step<T>) {
		return adapt_step_define(::phy_engine::model::model_reserve_type<T>, m, step);
	} else {
		return true;
	}
}

template <::phy_engine::model::model T>
inline constexpr bool check_convergence(T&& m) noexcept {
	// no model-specific checks for convergence
	if constexpr (::phy_engine::model::defines::can_check_convergence<T>) {
		return check_convergence_define(::phy_engine::model::model_reserve_type<T>, m);
	} else {
		return true;
	}
}

}