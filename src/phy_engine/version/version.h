/**************
 * Phy Engine *
 *************/

#pragma once

#include <cstdint>
#include <cstddef>
#include <concepts>
#include <compare>

#include "../../phy_engine_utils/fast_io/fast_io.h"

#include "../../phy_engine_utils/concepts/concepts.h"

namespace phy_engine {
struct version {
	::std::uint_least32_t x{};
	::std::uint_least32_t y{};
	::std::uint_least32_t z{};
	::std::uint_least32_t state{};
};

inline constexpr bool operator==(version v1, version v2) noexcept {
#ifdef _DEBUG
	return v1.x == v2.x && v1.y == v2.y && v1.z == v2.z && v1.state == v2.state;
#else
	if constexpr (4 * sizeof(::std::uint_least32_t) == sizeof(version)) {
#if __cpp_if_consteval >= 202106L
		if consteval
#else
		if (__builtin_is_constant_evaluated())
#endif
		{
			return v1.x == v2.x && v1.y == v2.y && v1.z == v2.z && v1.state == v2.state;
		} else {
			return ::fast_io::freestanding::my_memcmp(__builtin_addressof(v2), __builtin_addressof(v1), sizeof(version)) == 0;
		}
	} else {
		return v1.x == v2.x && v1.y == v2.y && v1.z == v2.z && v1.state == v2.state;
	}
#endif  // _DEBUG
}

inline constexpr ::std::strong_ordering operator<=>(version v1, version v2) noexcept {
	auto const cx{v1.x <=> v2.x};
	if (cx == 0) {
		auto const cy{v1.y <=> v2.y};
		if (cy == 0) {
			auto const cz{v1.z <=> v2.z};
			if (cz == 0) {
				return v1.state <=> v2.state;
			}
			return cz;
		}
		return cy;
	}
	return cx;
}

template <::std::integral char_type>
	requires(sizeof(char_type) == sizeof(char8_t))
inline constexpr ::std::size_t print_reserve_size(::fast_io::io_reserve_type_t<char_type, version>) noexcept {
	constexpr ::std::size_t real_size{print_reserve_size(::fast_io::io_reserve_type<char_type, ::std::uint_least32_t>)};
	constexpr ::std::size_t size{3 + 4 * real_size};
	return size;
}
namespace details {
template <::std::integral char_type>
inline constexpr char_type *version_print_reserve_impl(char_type *iter, ::std::uint_least32_t x, ::std::uint_least32_t y, ::std::uint_least32_t z, ::std::uint_least32_t state) noexcept {
	char_type *curr_pos{print_reserve_define(::fast_io::io_reserve_type<char_type, ::std::uint_least32_t>, iter, x)};
	*(curr_pos++) = static_cast<char_type>('.');
	curr_pos = print_reserve_define(::fast_io::io_reserve_type<char_type, ::std::uint_least32_t>, curr_pos, y);
	*(curr_pos++) = static_cast<char_type>('.');
	curr_pos = print_reserve_define(::fast_io::io_reserve_type<char_type, ::std::uint_least32_t>, curr_pos, z);
	*(curr_pos++) = static_cast<char_type>('.');
	curr_pos = print_reserve_define(::fast_io::io_reserve_type<char_type, ::std::uint_least32_t>, curr_pos, state);
	return curr_pos;
}
}  // namespace details

template <::std::integral char_type>
	requires(::phy_engine::value_transferable<version>)
inline constexpr char_type *print_reserve_define(::fast_io::io_reserve_type_t<char_type, version>, char_type *iter, version ver) noexcept {
	return details::version_print_reserve_impl(iter, ver.x, ver.y, ver.z, ver.state);
}

template <::std::integral char_type>
	requires(!::phy_engine::value_transferable<version>)
inline constexpr char_type *print_reserve_define(::fast_io::io_reserve_type_t<char_type, version>, char_type *iter, version const& ver) noexcept {
	return details::version_print_reserve_impl(iter, ver.x, ver.y, ver.z, ver.state);
}

}