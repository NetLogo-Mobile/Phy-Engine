#pragma once
#include <cstdint>
#include <string_view>

namespace phy_engine {
namespace details {
enum class variant_type : ::std::uint_least8_t {
	i8,
	i16,
	i32,
	i64,

	ui8,
	ui16,
	ui32,
	ui64,

	boolean,
	f,
	d
};

union variant {
	::std::int_least8_t i8;
	::std::int_least16_t i16;
	::std::int_least32_t i32;
	::std::int_least64_t i64;

	::std::uint_least8_t ui8;
	::std::uint_least16_t ui16;
	::std::uint_least32_t ui32;
	::std::uint_least64_t ui64;

	bool boolean;
	float f;
	double d;
};
}  // namespace details

struct property {
	::std::u8string_view name{};
	details::variant var{};
	details::variant_type var_type{};
};
}