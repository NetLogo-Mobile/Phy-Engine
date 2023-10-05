#pragma once
#include <string_view>

#include "../model_refs/variant.h"

namespace phy_engine::model {

struct property {
	::std::u8string_view name{};
	::phy_engine::model::variant var{};
};
}