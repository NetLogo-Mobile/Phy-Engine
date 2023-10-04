#pragma once
#include "concept.h"

namespace phy_engine::model {

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
inline constexpr bool iterate_ac(T&& m, double omega = {}) noexcept {
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
inline constexpr bool iterate_tr(T&& m, double tTime = {}) noexcept {
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
	} else if constexpr (::phy_engine::model::defines::can_iterate_dc<T>) {
		return iterate_dc_define(::phy_engine::model::model_reserve_type<T>, m);
	} else {
		return false;
	}
}

#if 0
template <::phy_engine::model::model T>
inline constexpr ::phy_engine::model::node_view get_nodes(T&& m) noexcept {
	return get_node_view(::phy_engine::model::model_reserve_type<T>, m);
}
#endif  // 0

}