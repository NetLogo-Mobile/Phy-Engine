#pragma once

#include <type_traits>
#include <concepts>

namespace phy_engine::freestanding {
template <typename T>
concept value_transferable =
	::std::is_trivially_copyable_v<::std::remove_cvref_t<T>> &&
#if (defined(_WIN32) && !defined(__WINE__)) || defined(__CYGWIN__)
	sizeof(::std::remove_cvref_t<T>) <= 8u  // ms_abi
#else
	sizeof(::std::remove_cvref_t<T>) <= (sizeof(::std::uintptr_t) * 2)  // sysc_abi
#endif
	;
}  // namespace phy_engine::utils