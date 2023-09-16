#pragma once

#include <algorithm>

#include "../../phy_engine_utils/fast_io/fast_io_freestanding.h"

#include "../../phy_engine_utils/command_line/impl.h"

namespace phy_engine {

namespace details {
inline constexpr ::phy_engine::command_line::parameter parameter_unsort[] {
#include "parainc.h"
};

template <::std::size_t N>
inline constexpr auto parameter_sort(::phy_engine::command_line::parameter const (&punsort)[N]) noexcept {
	::fast_io::freestanding::array<::phy_engine::command_line::parameter, N> res{};
	for (::std::size_t i{}; i < N; i++) {
		res[i] = punsort[i];
	}
	::std::ranges::sort(res, [](::phy_engine::command_line::parameter const& a, ::phy_engine::command_line::parameter const& b) -> bool {
		return a.name < b.name;
	});
	return res;
}

template <::std::size_t N>
inline constexpr ::std::size_t calculate_str_parameter_size(::fast_io::freestanding::array<::phy_engine::command_line::parameter, N> const& punsort) noexcept {
	::std::size_t res{};
	for (::std::size_t i{}; i < N; i++) {
		res++;
		res += punsort[i].alias.size();
	}
	return res;
}

template <::std::size_t Nres, ::std::size_t N>
inline constexpr auto generate_str_parameter_array(::fast_io::freestanding::array<::phy_engine::command_line::parameter, N> const& punsort) noexcept {
	::fast_io::freestanding::array<::phy_engine::command_line::details::str_parameter, Nres> res{};
	::std::size_t res_pos{};
	for (::std::size_t i{}; i < N; i++) {
		res[res_pos++] = {punsort[i].name, __builtin_addressof(punsort[i])};
		for (auto j : punsort[i].alias) {
			res[res_pos++] = {j, __builtin_addressof(punsort[i])};
		}
	}
	::std::ranges::sort(res, [](::phy_engine::command_line::details::str_parameter const& a, ::phy_engine::command_line::details::str_parameter const& b) -> bool {
		return a.str < b.str;
	});
	return res;
}
}  // namespace details

inline constexpr auto parameters{details::parameter_sort(details::parameter_unsort)};
inline constexpr ::std::size_t parameter_lookup_table_size{details::calculate_str_parameter_size(parameters)};
inline constexpr auto parameter_lookup_table{details::generate_str_parameter_array<parameter_lookup_table_size>(parameters)};
inline constexpr ::phy_engine::command_line::command_line command_line{
	::phy_engine::freestanding::array_view{parameters.data(), parameters.size()},
	::phy_engine::freestanding::array_view{parameter_lookup_table.data(), parameter_lookup_table.size()}
};

}