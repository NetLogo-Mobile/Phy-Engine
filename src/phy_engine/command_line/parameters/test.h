#pragma once
#include "../../../phy_engine_utils/command_line/parameter.h"

namespace phy_engine::parameter {
namespace details {
inline constexpr ::std::u8string_view test_alias{u8"-t"};
extern bool test_callback(int argc, char8_t** argv, int pos, ::std::u8string_view var) noexcept;

}  // namespace details

inline constexpr ::phy_engine::command_line::parameter test{
	.name{u8"--test"},
	.describe{u8"test"},
	.alias{::phy_engine::freestanding::array_view{__builtin_addressof(details::test_alias), 1}},
	.callback{},
};
}  // namespace phy_engine::parameter