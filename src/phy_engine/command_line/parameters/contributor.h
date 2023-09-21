#pragma once
#include "../../../phy_engine_utils/command_line/parameter.h"

namespace phy_engine::parameter {
namespace details {
inline bool contributor_is_exist{};
::phy_engine::command_line::parameter_return_type contributor_callback(::std::size_t, ::fast_io::vector<::phy_engine::command_line::parameter_parsing_results>&) noexcept;
}  // namespace details

inline constexpr ::phy_engine::command_line::parameter contributor{
	.name{u8"--contributor"},
	.describe{u8"list the contributors to the project"},
	.callback{__builtin_addressof(details::contributor_callback)},
	.is_exist{__builtin_addressof(details::contributor_is_exist)},
};
}  // namespace phy_engine::parameter