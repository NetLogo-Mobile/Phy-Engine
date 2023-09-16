#pragma once

#include <type_traits>
#include <concepts>

namespace phy_engine::freestanding {

template <typename T>
struct array_view {
	using value_type = T;
	using pointer = T*;
	using const_pointer = T const*;
	using const_iterator = const_pointer;
	using iterator = pointer;
	using reference = T&;
	using const_reference = T const&;
	using size_type = ::std::size_t;
	using difference_type = ::std::ptrdiff_t;
	using const_reverse_iterator = ::std::reverse_iterator<const_iterator>;
	using reverse_iterator = ::std::reverse_iterator<iterator>;

	const_pointer data_ptr{};
	size_type array_size{};

	constexpr array_view() noexcept = default;

	constexpr array_view(decltype(nullptr)) noexcept
		: data_ptr{nullptr}, array_size{0u} {}

	constexpr array_view(const_pointer t_ptr, size_type s) noexcept
		: data_ptr{t_ptr}, array_size{s} {}

	template <::std::contiguous_iterator Iter>
		requires ::std::same_as<::std::iter_value_t<Iter>, value_type>
	constexpr array_view(Iter first, Iter last) noexcept
		: data_ptr{::std::to_address(first)}, array_size{static_cast<size_type>(last - first)} {}

	template <::std::ranges::contiguous_range rg>
		requires(::std::same_as<::std::ranges::range_value_t<rg>, value_type> && !::std::is_array_v<std::remove_cvref_t<rg>>)
	explicit constexpr array_view(rg &&r) noexcept : array_view(::std::ranges::cbegin(r), ::std::ranges::cend(r)) {}

	constexpr const_pointer data() const noexcept {
		return data_ptr;
	}

	constexpr bool empty() const noexcept {
		return array_size == 0;
	}

	constexpr size_type size() const noexcept {
		return array_size;
	}

	constexpr size_type max_size() const noexcept {
		return SIZE_MAX;
	}

	constexpr const_iterator cbegin() const noexcept {
		return data_ptr;
	}

	constexpr const_iterator begin() const noexcept {
		return data_ptr;
	}

	constexpr const_iterator cend() const noexcept {
		return data_ptr + array_size;
	}

	constexpr const_iterator end() const noexcept {
		return data_ptr + array_size;
	}

	constexpr const_reverse_iterator crbegin() const noexcept {
		return const_reverse_iterator{data_ptr + array_size};
	}

	constexpr const_reverse_iterator rbegin() const noexcept {
		return const_reverse_iterator{data_ptr + array_size};
	}

	constexpr const_reverse_iterator crend() const noexcept {
		return const_reverse_iterator{data_ptr};
	}

	constexpr const_reverse_iterator rend() const noexcept {
		return const_reverse_iterator{data_ptr};
	}

	constexpr const_reference front() const noexcept {
		return *data_ptr;
	}
	constexpr const_reference back() const noexcept {
		return *(data_ptr + array_size - 1u);
	}

	inline constexpr const_reference operator[](size_type s) const noexcept {
		return data_ptr[s];
	}
};
} 