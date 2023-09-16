/**************
 * Phy Engine *
 *************/

#pragma once

#include <type_traits>
#include <concepts>
#include <string_view>

#include "../fast_io/fast_io.h"
#include "../freestanding/array_view.h"
#include "../../phy_engine/devices/native_io.h"
#include "../ansies/impl.h"

#include "parameter.h"

namespace phy_engine::command_line {

namespace details {
struct str_parameter {
	::std::u8string_view str{};
	::phy_engine::command_line::parameter const* para{};
};
}  // namespace details

struct command_line {
	::phy_engine::freestanding::array_view<::phy_engine::command_line::parameter> parameters{};
	::phy_engine::freestanding::array_view<details::str_parameter> parameter_lookup_table{};
};

namespace details {
struct str_parameter_res {
	::std::u8string_view str{};
	::std::u8string_view option{};
	::phy_engine::command_line::parameter const* para{};
	int pos{};
};

struct str_optval_res {
	::std::u8string_view str{};
	int pos{};
};
}  // namespace details

struct cl_parse_res {
	::phy_engine::freestanding::array_view<details::str_parameter_res> parameters_res{};
	::phy_engine::freestanding::array_view<details::str_optval_res> optval_res{};
};

inline constexpr int parsing_parameters(
	int agrc,
	char8_t** argv,
	command_line const& cl,
	cl_parse_res& cpr) noexcept {
	if (agrc == 0) {
		::fast_io::io::perrln(::phy_engine::u8err,
							  ::phy_engine::ansi_escape_sequences::rst::all,
							  ::phy_engine::ansi_escape_sequences::sty::bold,
							  ::phy_engine::ansi_escape_sequences::col::red,
							  u8"[error] ",
							  ::phy_engine::ansi_escape_sequences::rst::all,
							  ::phy_engine::ansi_escape_sequences::col::white,
							  u8"No Parameters.",
							  ::phy_engine::ansi_escape_sequences::rst::all);
		return -1;
	}

}
}