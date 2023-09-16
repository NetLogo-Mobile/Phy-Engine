#pragma once

#include "../../../phy_engine_utils/fast_io/fast_io.h"
#include "../../../phy_engine_utils/fast_io/fast_io_freestanding.h"

#include "../../devices/native_io.h"

#include "../../../phy_engine_utils/command_line/parameter.h"

namespace phy_engine::parameter {
namespace details {
inline constexpr ::fast_io::freestanding::array<::std::u8string_view, 2> help_alias{u8"-h", u8"-?"};
inline extern bool help_callback(int argc, char8_t** argv, int pos, ::std::u8string_view var) noexcept;
}  // namespace details

inline constexpr ::phy_engine::command_line::parameter help{
	.name{u8"--help"},
	.describe{u8"get help information"},
	.alias{::phy_engine::freestanding::array_view{details::help_alias.data(), details::help_alias.size()}},
	.callback{__builtin_addressof(details::help_callback)},
};
}  // namespace phy_engine::parameter