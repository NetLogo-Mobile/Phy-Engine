#pragma once

#include "../../../phy_engine_utils/command_line/parameter.h"

namespace phy_engine::parameter {
namespace details {
inline constexpr ::std::u8string_view verson_alias{u8"-v"};
}  // namespace details

inline constexpr ::phy_engine::command_line::parameter version{
	.name{u8"--version"},
	.describe{u8"display version information"},
	.alias{::phy_engine::freestanding::array_view{__builtin_addressof(details::verson_alias), 1}},
};
}  // namespace phy_engine::parameter