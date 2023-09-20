/**************
 * Phy Engine *
 *************/

#pragma once

#include <type_traits>
#include <concepts>
#include <string_view>

#include "../fast_io/fast_io.h"
#include "../freestanding/array_view.h"

namespace phy_engine::command_line {
inline constexpr ::std::size_t dp(::std::u8string_view x, ::std::u8string_view y) noexcept {
	::std::size_t const lena{x.size()};
	::std::size_t const lenb{y.size()};

	::std::size_t* d{};

#if __cpp_if_consteval >= 202106L
	if consteval
#else
	if (__builtin_is_constant_evaluated())
#endif
	{
		d = new ::std::size_t[lenb + 1];
	} else {
		d = ::fast_io::native_typed_global_allocator<::std::size_t>::allocate(lenb + 1);
	}

	::std::size_t i{}, j{}, old{}, temp{};

	for (j = 0; j <= lenb; j++) {
		d[j] = j;
	}

	for (i = 1; i <= lena; i++) {
		old = i - 1;
		d[0] = i;
		for (j = 1; j <= lenb; j++) {
			temp = d[j];
			if (x[i - 1] == y[j - 1]) {
				d[j] = old;
			} else {
				size_t min = d[j] + 1;
				if (d[j - 1] + 1 < min)
					min = d[j - 1] + 1;
				if (old + 1 < min)
					min = old + 1;
				d[j] = min;
			}
			old = temp;
		}
	}

	const size_t ret{d[lenb]};

#if __cpp_if_consteval >= 202106L
	if consteval
#else
	if (__builtin_is_constant_evaluated())
#endif
	{
		delete[] d;
	} else {
		if constexpr (::fast_io::details::has_deallocate_n_impl<::fast_io::native_global_allocator>) {
			::fast_io::native_global_allocator::deallocate_n(d, lenb + 1);
		} else {
			::fast_io::native_global_allocator::deallocate(d);
		}
	}

	return ret;
}
}