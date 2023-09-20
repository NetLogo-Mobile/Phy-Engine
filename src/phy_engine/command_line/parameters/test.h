#pragma once
#include "../../../phy_engine_utils/command_line/parameter.h"

namespace phy_engine::parameter {
namespace details {
inline bool test_is_exist{};
inline constexpr ::std::u8string_view test_alias{u8"-t"};
::phy_engine::command_line::parameter_return_type test_callback(::std::size_t, ::fast_io::vector<::phy_engine::command_line::parameter_parsing_results>&) noexcept;
}  // namespace details

inline constexpr ::phy_engine::command_line::parameter test{
	.name{u8"--test"},
	.describe{u8"test"},
	.alias{::phy_engine::freestanding::array_view{__builtin_addressof(details::test_alias), 1}},
	.callback{__builtin_addressof(details::test_callback)},
	.is_exist{__builtin_addressof(details::test_is_exist)},
};
}  // namespace phy_engine::parameter