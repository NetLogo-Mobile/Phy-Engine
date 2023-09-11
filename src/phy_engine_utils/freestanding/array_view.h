#pragma once

#include <type_traits>
#include <concepts>

namespace phy_engine::freestanding {

template <typename T>
struct array_view {
	T const* data{};
	::std::size_t size{};

	constexpr array_view() noexcept = default;

	constexpr array_view(decltype(nullptr)) noexcept
		: data{nullptr}, size{0u} {}

	constexpr array_view(T const *t_ptr, ::std::size_t count) noexcept
		: data{t_ptr}, size{count} {}

	template <::std::contiguous_iterator Iter>
		requires ::std::same_as<::std::iter_value_t<Iter>, T>
	constexpr array_view(Iter first, Iter last) noexcept
		: data{::std::to_address(first)}, size{static_cast<::std::size_t>(last - first)} {}

	template <::std::ranges::contiguous_range rg>
		requires(::std::same_as<::std::ranges::range_value_t<rg>, T> && !::std::is_array_v<std::remove_cvref_t<rg>>)
	constexpr array_view(rg &&r) noexcept : array_view(::std::ranges::cbegin(r), ::std::ranges::cend(r)) {}
};
} 