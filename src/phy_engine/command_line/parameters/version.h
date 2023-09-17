#pragma once

#include "../../../phy_engine_utils/command_line/parameter.h"

namespace phy_engine::parameter {
namespace details {
inline constexpr ::std::u8string_view verson_alias{u8"-v"};
extern bool version_callback(int argc, char8_t** argv, int pos, ::std::u8string_view var) noexcept;
}  // namespace details

inline constexpr ::phy_engine::command_line::parameter version{
	.name{u8"--version"},
	.describe{u8"display version information"},
	.alias{::phy_engine::freestanding::array_view{__builtin_addressof(details::verson_alias), 1}},
	.callback{__builtin_addressof(details::version_callback)},
};
}  // namespace phy_engine::parameter