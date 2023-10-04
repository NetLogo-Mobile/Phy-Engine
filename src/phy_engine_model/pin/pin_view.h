#pragma once
#include <cstddef>
#include "pin.h"

namespace phy_engine::model {
struct pin_view {
	::phy_engine::model::pin* pins{};
	::std::size_t size{};
};
}