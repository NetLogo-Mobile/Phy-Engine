/**************
 * Phy Engine *
 *************/

#pragma once

#include <type_traits>
#include <concepts>
#include <string_view>

#include "../freestanding/array_view.h"

namespace phy_engine::command_line {

template<::std::integral char_type>
struct basic_parameter {
	::std::basic_string_view<char_type> name{};
	::phy_engine::freestanding::array_view<::std::basic_string_view<char_type>> alias{};
	::phy_engine::freestanding::array_view<basic_parameter*> prerequisite{};
	::phy_engine::freestanding::array_view<basic_parameter*> clash{};
	bool (*callback)(int argc, char** argv, int pos, ::std::basic_string_view<char_type> var) noexcept {};
};

}