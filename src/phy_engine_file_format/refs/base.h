#pragma once

#include <type_traits>
#include <concepts>
#include <string_view>

namespace phy_engine::file_format {
template <typename T>
concept file = requires(T&& t, void* handle) {
				   requires ::std::same_as<::std::remove_cvref_t<decltype(t.extension)>, ::std::u8string_view>;
				   { load_file(t, handle) };
			   };
}  