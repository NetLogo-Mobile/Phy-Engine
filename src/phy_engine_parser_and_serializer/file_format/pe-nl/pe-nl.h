#pragma once

#include <string_view>
#include "../../refs/base.h"

namespace phy_engine::file_format::pe_nl {
struct pe_nl {
	static constexpr ::std::u8string_view extension{u8"penl"};
};

inline constexpr bool load_file(pe_nl const& penl, ::std::u8string_view sv) noexcept {
	return {};
}
}