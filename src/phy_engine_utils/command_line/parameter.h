/**************
 * Phy Engine *
 *************/

#pragma once

#include <type_traits>
#include <concepts>
#include <string_view>

#include "../freestanding/array_view.h"

namespace phy_engine::command_line {

struct parameter {
	::std::u8string_view name{};
	::std::u8string_view describe{};
	::phy_engine::freestanding::array_view<::std::u8string_view> alias{};
	::phy_engine::freestanding::array_view<parameter*> prerequisite{};
	::phy_engine::freestanding::array_view<parameter*> clash{};
	bool (*callback)(int argc, char8_t** argv, int pos, ::std::u8string_view var) noexcept {nullptr};
	bool has_value{};
};

}