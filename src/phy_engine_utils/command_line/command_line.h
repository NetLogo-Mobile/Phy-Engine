/**************
 * Phy Engine *
 *************/

#pragma once

#include <type_traits>
#include <concepts>
#include <string>
#include <string_view>
#include <map> // replace with other map is subsequent versions

#include "../fast_io/fast_io.h"
#include "../freestanding/array_view.h"
#include "../../phy_engine/devices/native_io.h"
#include "../ansies/impl.h"

#include "parameter.h"

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

namespace details {
struct str_parameter {
	::std::u8string_view str{};
	::phy_engine::command_line::parameter const* para{};
};
}  // namespace details

struct command_line {
	::phy_engine::freestanding::array_view<::phy_engine::command_line::parameter const*> parameters{};
	::phy_engine::freestanding::array_view<details::str_parameter> parameter_lookup_table{};
};

struct find_parameter_res {
	::phy_engine::command_line::parameter const* par{};
	::std::u8string_view str{};
};

inline constexpr find_parameter_res find_parameter(::std::u8string_view sv, bool h, command_line const& cl) noexcept {
	::phy_engine::freestanding::array_view<details::str_parameter> plt{cl.parameter_lookup_table};
	::std::size_t left{};
	::std::size_t right{plt.size() - 1u};

#if __has_cpp_attribute(assume)
	[[assume(left <= right)]];
#endif

	if (left == right) 
		return {};

	::std::size_t mid{};

	::std::u8string_view d_str{sv};
	::std::u8string_view optval{};

	::std::size_t res{sv.find(u8'=')};
	if (res != ::std::u8string_view::npos) {
		if (h) {
			optval = d_str.substr(res + 1u);
			d_str = d_str.substr(0u, res);
		} else {
			return {}; // no optval required
		}
	}

	while (left <= right) {
		mid = (left + right) >> 1u;

		if (plt[mid].str > d_str) {
			right = mid - 1;
		} else if (plt[mid].str < d_str) {
			left = mid + 1;
		} else {
			return {plt[mid].para, optval};
		}
	}

#if __has_cpp_attribute(assume)
	[[assume(d_str.size() <= 0xffff'ffff'ffff'ffff / 4u)]];
#endif
	::std::size_t const test_size{d_str.size() * 4u / 10u};
	::std::size_t f_test_size{test_size};
	::std::u8string_view f_test_str{};
	for (auto &i : plt) {
		if (auto const dp_res{dp(i.str, d_str)}; dp_res <= test_size) {
			f_test_str = i.str;
			f_test_size = dp_res;
		}
	}

	if (!f_test_str.empty()) {
		return {nullptr, f_test_str};
	}

	return {};
}

namespace details {
struct str_parameter_res {
#if 0
	::phy_engine::command_line::parameter const* para{};
#endif  // 0
	::std::u8string_view option{};
	int pos{};
};

struct str_optval_res {
#if 0
	::std::u8string_view str{};
#endif  // 0
	int pos{};
};
}  // namespace details

struct cl_parse_res {
	::std::map<::phy_engine::command_line::parameter const*, details::str_parameter_res> parameters_res{};
	::std::map<::std::u8string_view, details::str_optval_res> optval_res{};
};

extern int parsing_parameters(int argc, char8_t** argv, command_line const& cl, cl_parse_res& cpr) noexcept;
}