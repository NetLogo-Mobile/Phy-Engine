/**************
 * Phy Engine *
 *************/

#pragma once

#include <type_traits>
#include <concepts>
#include <string_view>

#include "../freestanding/array_view.h"
#include "parameter.h"
#include "../fast_io/fast_io.h"
#include "../../phy_engine/devices/native_io.h"

namespace phy_engine::command_line {
namespace details {
struct str_parameter {
	::std::u8string_view str{};
	::phy_engine::command_line::parameter* para{};
	int pos{};
};

struct str_var {
	::std::u8string_view str{};
	int pos{};
};
}

struct command_line {
	::phy_engine::freestanding::array_view<::phy_engine::command_line::parameter> parameters{};
	::phy_engine::freestanding::array_view<details::str_parameter> parameter_lookup_table{};
	::phy_engine::freestanding::array_view<details::str_var> variavle_lookup_table{};
};

inline constexpr void parsing_parameters(command_line& cl, int agrc, char8_t** argv) noexcept {
	if (agrc == 0) {
		::fast_io::io::perr(::phy_engine::native_io::u8err, u8"\n");
		return;
	}

}
}