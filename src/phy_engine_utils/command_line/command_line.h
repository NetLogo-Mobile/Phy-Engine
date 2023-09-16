/**************
 * Phy Engine *
 *************/

#pragma once

#include <type_traits>
#include <concepts>
#include <string>
#include <string_view>
#include <map> // replace with other map is subsequent versions

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

inline constexpr ::phy_engine::command_line::parameter const* find_parameter(::std::u8string_view sv, command_line const& cl) noexcept {
	// to do
}

namespace details {
struct str_parameter_res {
#if 0
	::phy_engine::command_line::parameter const* para{};
#endif  // 0
	::std::u8string_view option{};
	int pos{};
};

struct str_optval_res {
#if 0
	::std::u8string_view str{};
#endif  // 0
	int pos{};
};
}  // namespace details

struct cl_parse_res {
	::std::map<::phy_engine::command_line::parameter const*, details::str_parameter_res> parameters_res{};
	::std::map<::std::u8string_view, details::str_optval_res> optval_res{};
};

inline extern int parsing_parameters(int argc, char8_t** argv, command_line const& cl, cl_parse_res& cpr) noexcept;
}