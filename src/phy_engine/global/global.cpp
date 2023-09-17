#include "../../phy_engine_utils/command_line/impl.h"
#include "../command_line/impl.h"
#include "set_timezone.h"

#if (defined(_WIN32) || defined(__CYGWIN__)) && !defined(_WIN32_WINDOWS)
[[maybe_unused]] ::phy_engine::ansi_escape_sequences::set_win32_console_io_cp_to_utf8 set_win32_console_var{};
[[maybe_unused]] ::phy_engine::ansi_escape_sequences::enable_win32_ansi enable_win32_ansi_var{};
#endif

[[maybe_unused]] static ::phy_engine::global::set_timezone stz{};