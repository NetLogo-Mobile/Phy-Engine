#pragma once
#include <string_view>

namespace phy_engine::model {
struct pin {
	::std::u8string_view name{};
};
}