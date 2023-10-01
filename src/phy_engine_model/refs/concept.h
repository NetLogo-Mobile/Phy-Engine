#pragma once

#include <concepts>

namespace phy_engine::model {
template <typename T>
concept model = requires(T&& t) {
					T::type;
				};

}