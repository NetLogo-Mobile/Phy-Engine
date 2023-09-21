#pragma once

#include <cstddef>
#include <string_view>

namespace phy_engine {
inline constexpr ::std::u8string_view contributor[]{
#include "../../../custom/contributor.h"
};

namespace details {
template <::std::size_t N>
inline consteval ::std::size_t calculate_contributor_size(::std::u8string_view const (&c)[N]) noexcept {
	return N;
}
}  // namespace details
inline constexpr ::std::size_t contributor_size{details::calculate_contributor_size(contributor)};
}