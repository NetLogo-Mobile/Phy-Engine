#pragma once

#include <type_traits>
#include <concepts>

#include "../../phy_engine_utils/fast_io/fast_io_core.h"
#include "base.h"
#include "file_type.h"

namespace phy_engine::file_format {
namespace details {
using Alloc = ::fast_io::native_global_allocator;

struct file_base_impl {
	virtual constexpr ~file_base_impl() noexcept = default;
	virtual constexpr file_base_impl *clone() const noexcept = 0;

	virtual constexpr ::std::u8string_view get_ext_impl() const noexcept = 0;
	virtual constexpr bool load_file_impl(::std::u8string_view) const noexcept = 0;
};

template <::phy_engine::file_format::file T>
struct file_derv_impl : file_base_impl {
	T t{};

	constexpr file_derv_impl(T &&tt) noexcept : t{::std::forward<T>(tt)} {}

	virtual constexpr file_base_impl *clone() const noexcept override {
#if (__cpp_if_consteval >= 202106L || __cpp_lib_is_constant_evaluated >= 201811L) && __cpp_constexpr_dynamic_alloc >= 201907L
#if __cpp_if_consteval >= 202106L
		if consteval
#else
		if (__builtin_is_constant_evaluated())
#endif
		{
			return new file_derv_impl<T>{*this};
		} else
#endif
		{
			file_base_impl *ptr{reinterpret_cast<file_base_impl *>(Alloc::allocate(sizeof(file_derv_impl<T>)))};
			new (ptr) file_derv_impl<T>{*this};
			return ptr;
		}
	}

	virtual constexpr ::std::u8string_view get_ext_impl() const noexcept override {
		return T::extension;
	}

	virtual constexpr bool load_file_impl(::std::u8string_view sv) const noexcept override {
		return load_file(sv);
	}
};
}  // namespace details

struct
#if __has_cpp_attribute(__gnu__::__packed__)
[[__gnu__::__packed__]]
#endif
	file_format {
	::phy_engine::file_format::file_type type{};
	details::file_base_impl *ptr{};

	constexpr file_format() noexcept = default;

	template <::phy_engine::file_format::file T>
	constexpr file_format(T &&tt) noexcept {
#if (__cpp_if_consteval >= 202106L || __cpp_lib_is_constant_evaluated >= 201811L) && __cpp_constexpr_dynamic_alloc >= 201907L
#if __cpp_if_consteval >= 202106L
		if consteval
#else
		if (__builtin_is_constant_evaluated())
#endif
		{
			ptr = new details::file_derv_impl<T>{tt};
		} else
#endif
		{
			ptr = reinterpret_cast<details::file_base_impl*>(details::Alloc::allocate(sizeof(details::file_derv_impl<T>)));
			new (ptr) details::file_derv_impl<T>{tt};
		}
	}

	constexpr file_format(file_format const &other) : ptr{other.ptr->clone()} {}

	constexpr file_format &operator=(file_format const &other) {
		auto temp{other.ptr->clone()};
		delete this->ptr;
		this->ptr = temp;
		return *this;
	}

	constexpr file_format(file_format &&other) noexcept : ptr{other.ptr} {
		other.ptr = nullptr;
	}

	constexpr file_format &operator=(file_format &&other) noexcept {
		delete this->ptr;
		this->ptr = other.ptr;
		other.ptr = nullptr;
		return *this;
	}

	~file_format() {
		delete_file();
	}

	constexpr void delete_file() noexcept {
#if (__cpp_if_consteval >= 202106L || __cpp_lib_is_constant_evaluated >= 201811L) && __cpp_constexpr_dynamic_alloc >= 201907L
#if __cpp_if_consteval >= 202106L
		if consteval
#else
		if (__builtin_is_constant_evaluated())
#endif
		{
			delete ptr;
		} else
#endif
		{
			ptr->~file_base_impl();
			details::Alloc::deallocate(ptr);
		}
	}
};

inline constexpr ::phy_engine::file_format::file_type get_file_type(file_format const &f) noexcept {
	return f.type;
}

inline constexpr ::std::u8string_view get_ext(file_format const &f) noexcept {
	return f.ptr->get_ext_impl();
}

inline constexpr bool load_file(file_format const &f, ::std::u8string_view sv) noexcept {
	return f.ptr->load_file_impl(sv);
}

}  // namespace phy_engine::file_format