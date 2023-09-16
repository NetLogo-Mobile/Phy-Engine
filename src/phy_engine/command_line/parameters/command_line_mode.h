#pragma once

#include "../../../phy_engine_utils/command_line/parameter.h"

namespace phy_engine::parameter {
namespace details {
inline constexpr ::std::u8string_view command_line_mode_alias{u8"-c"};
}  // namespace details

inline constexpr ::phy_engine::command_line::parameter command_line_mode{
	.name{u8"--command-line-mode"},
	.describe{u8"command line mode"},
	.alias{::phy_engine::freestanding::array_view{__builtin_addressof(details::command_line_mode_alias), 1}},
};
}  // namespace phy_engine::parameter