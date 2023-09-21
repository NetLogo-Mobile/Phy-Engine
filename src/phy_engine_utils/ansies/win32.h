/**************
 * Phy Engine *
 *************/

#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "../fast_io/fast_io.h"

namespace phy_engine::ansi_escape_sequences {

// https://learn.microsoft.com/en-us/windows/win32/intl/code-page-identifiers

struct set_win32_console_io_cp_to_utf8 {
	inline static constexpr ::std::uint_least32_t utf8_coding{65001u};  // utf8

	::std::uint_least32_t output{};
	::std::uint_least32_t input{};

	set_win32_console_io_cp_to_utf8() noexcept {
#if (defined(_WIN32) || defined(__CYGWIN__)) && !defined(_WIN32_WINDOWS)
		output = ::fast_io::win32::GetConsolOutputCP();
		input = ::fast_io::win32::GetConsolCP();
		if (output != utf8_coding)
			::fast_io::win32::SetConsoleOutputCP(utf8_coding);
		if (input != utf8_coding)
			::fast_io::win32::SetConsoleCP(utf8_coding);
#endif
	}

	~set_win32_console_io_cp_to_utf8() {
#if (defined(_WIN32) || defined(__CYGWIN__)) && !defined(_WIN32_WINDOWS)
		if (output != utf8_coding)
			::fast_io::win32::SetConsoleOutputCP(output);
		if (input != utf8_coding)
			::fast_io::win32::SetConsoleCP(input);
#endif
	}
};

struct enable_win32_ansi {
	inline static constexpr ::std::uint_least32_t enable_virtual_terminal_processing{0x0004u};

	::std::uint_least32_t out_omode{};
	::std::uint_least32_t err_omode{};

	void* out_handle{};
	void* err_handle{};

	enable_win32_ansi() noexcept {
#if (defined(_WIN32) || defined(__CYGWIN__)) && !defined(_WIN32_WINDOWS)
		out_handle = ::fast_io::win32::GetStdHandle(::fast_io::win32_stdout_number);
		err_handle = ::fast_io::win32::GetStdHandle(::fast_io::win32_stderr_number);
		::fast_io::win32::GetConsoleMode(out_handle, &out_omode);
		::fast_io::win32::GetConsoleMode(err_handle, &err_omode);
		::fast_io::win32::SetConsoleMode(out_handle, out_omode | enable_virtual_terminal_processing);
		::fast_io::win32::SetConsoleMode(err_handle, err_omode | enable_virtual_terminal_processing);
#endif
	}

	~enable_win32_ansi() {
#if (defined(_WIN32) || defined(__CYGWIN__)) && !defined(_WIN32_WINDOWS)
		::fast_io::win32::SetConsoleMode(out_handle, out_omode);
		::fast_io::win32::SetConsoleMode(err_handle, err_omode);
#endif
	}
};

}