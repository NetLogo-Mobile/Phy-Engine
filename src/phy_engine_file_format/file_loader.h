#pragma once

#include <string_view>

#include "../phy_engine_utils/fast_io/fast_io.h"
#include "../phy_engine_utils/fast_io/fast_io_device.h"

namespace phy_engine {
int load_file(::std::u8string_view sv) noexcept {
	auto const extension{::fast_io::details::find_dot_and_sep<false, char8_t, char8_t>(sv.data(), sv.size())};
}
}