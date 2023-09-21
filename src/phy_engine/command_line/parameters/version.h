#pragma once

#include "../../../phy_engine_utils/command_line/parameter.h"

namespace phy_engine::parameter {
namespace details {
inline bool version_is_exist{};
inline constexpr ::std::u8string_view verson_alias{u8"-v"};
extern ::phy_engine::command_line::parameter_return_type version_callback(::std::size_t, ::fast_io::vector<::phy_engine::command_line::parameter_parsing_results>&) noexcept;
}  // namespace details

inline constexpr ::phy_engine::command_line::parameter version{
	.name{u8"--version"},
	.describe{u8"display version information"},
	.alias{::phy_engine::freestanding::array_view{__builtin_addressof(details::verson_alias), 1}},
	.callback{__builtin_addressof(details::version_callback)},
	.is_exist{__builtin_addressof(details::version_is_exist)},
};
}  // namespace phy_engine::parameter