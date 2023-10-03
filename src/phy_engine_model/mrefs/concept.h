#pragma once

#include <type_traits>
#include <concepts>
#include <string>
#include <string_view>
#include "../../phy_engine_utils/fast_io/fast_io_dsal/vector.h"

namespace phy_engine::model {
template <typename T>
concept model = requires(T&& t) {
					requires ::std::same_as<::std::remove_cvref_t<decltype(T::model_name)>, ::std::u8string_view>;
					requires ::std::same_as<::std::remove_cvref_t<decltype(t.name)>, ::std::u8string>;

				};

}