#pragma once

#include "../../../phy_engine_utils/command_line/parameter.h"

namespace phy_engine::parameter {
namespace details {
inline constexpr ::std::u8string_view ignore_invalid_parameters_alias{u8"-v"};
}  // namespace details

inline constexpr ::phy_engine::command_line::parameter ignore_invalid_parameters{
	.name{u8"--ignore-invalid-parameters"},
	.describe{u8"ignore invalid parameters (but it will output warning)"},
	.alias{::phy_engine::freestanding::array_view{__builtin_addressof(details::ignore_invalid_parameters_alias), 1}},
};
}  // namespace phy_engine::parameter