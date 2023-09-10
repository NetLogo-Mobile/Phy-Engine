/**************
 * Phy Engine *
 *************/

#pragma once
#include <cstdint>
#include "../../phy_engine_utils/fast_io/fast_io.h"

namespace phy_engine::console {

// https://learn.microsoft.com/en-us/windows/win32/intl/code-page-identifiers

struct cvt_win32_console_cp_to_utf8 {
	static constexpr ::std::uint_least32_t exe_coding{65001u}; // utf8

	::std::uint_least32_t output{};
	::std::uint_least32_t input{};

#if __has_cpp_attribute(__gnu__::__always_inline__)
	[[__gnu__::__always_inline__]]
#elif __has_cpp_attribute(msvc::forceinline)
	[[msvc::forceinline]]
#endif
	cvt_win32_console_cp_to_utf8() noexcept {
#ifdef _WIN32
		if constexpr (::fast_io::win32_family::native == ::fast_io::win32_family::wide_nt) {
			output = ::fast_io::win32::GetConsolOutputCP();
			input = ::fast_io::win32::GetConsolCP();
			if (output != exe_coding)
				::fast_io::win32::SetConsoleOutputCP(exe_coding);
			if (input != exe_coding) 
				::fast_io::win32::SetConsoleCP(exe_coding);
		}
#endif
	}

#if __has_cpp_attribute(__gnu__::__always_inline__)
	[[__gnu__::__always_inline__]]
#elif __has_cpp_attribute(msvc::forceinline)
	[[msvc::forceinline]]
#endif
	~cvt_win32_console_cp_to_utf8() {
#ifdef _WIN32
		if constexpr (::fast_io::win32_family::native == ::fast_io::win32_family::wide_nt) {
			if (output != exe_coding)
				::fast_io::win32::SetConsoleOutputCP(output);
			if (input != exe_coding)
				::fast_io::win32::SetConsoleCP(input);
		}
#endif
	}
};

}