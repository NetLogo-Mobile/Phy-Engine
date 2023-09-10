/**************
 * Phy Engine *
 *************/

#pragma once
#include "../../phy_engine_utils/fast_io/fast_io.h"

namespace phy_engine::console {

#if __has_cpp_attribute(__gnu__::__always_inline__)
[[__gnu__::__always_inline__]]
#elif __has_cpp_attribute(msvc::forceinline)
[[msvc::forceinline]]
#endif
inline bool set_console_cp_to_utf8() noexcept {
#ifdef _WIN32
	if constexpr (::fast_io::win32_family::native == ::fast_io::win32_family::wide_nt) {
		return ::fast_io::win32::SetConsoleCP(65001u);
	}
#endif
	return true;
}
}