/**************
 * Phy Engine *
 *************/

#include "../phy_engine_utils/fast_io/fast_io.h" // fast_io

#include "devices/native_io.h" // native_io
#include "version/phy_engine.h"
#include "../phy_engine_utils/ansies/impl.h"
#include "../phy_engine_utils/command_line/impl.h"
#include "command_line/impl.h"

#if (defined(_WIN32) || defined(__CYGWIN__)) && !defined(_WIN32_WINDOWS)
[[maybe_unused]] ::phy_engine::ansi_escape_sequences::set_win32_console_io_cp_to_utf8 set_win32_console_var{};
[[maybe_unused]] ::phy_engine::ansi_escape_sequences::enable_win32_ansi enable_win32_ansi_var{};
#endif

int main(int argc, char** argv) noexcept {
	::fast_io::io::println(::phy_engine::u8out,
						 ::phy_engine::ansi_escape_sequences::rst::all,
						 u8"Phy Engine ",
						 ::phy_engine::ansi_escape_sequences::col::green,
						 u8"V",
						 ::phy_engine::phy_engine_version,
						 ::phy_engine::ansi_escape_sequences::rst::color);
	return ::phy_engine::command_line::parsing_parameters(argc, reinterpret_cast<char8_t**>(argv), ::phy_engine::command_line, ::phy_engine::parsing_result);
}