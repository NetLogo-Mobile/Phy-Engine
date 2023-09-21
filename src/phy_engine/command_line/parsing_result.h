#pragma once

#include <utility>

#include "../../phy_engine_utils/command_line/command_line.h"
#include "../../phy_engine_utils/ansies/impl.h"
#include "../devices/native_io.h"

namespace phy_engine {
inline ::fast_io::vector<::phy_engine::command_line::parameter_parsing_results> parsing_result{};

template <::std::size_t hash_table_size, ::std::size_t conflict_size>
inline constexpr int parsing(int argc, char8_t** argv, ::fast_io::vector<::phy_engine::command_line::parameter_parsing_results>& pr, ::phy_engine::command_line::parameters_hash_table<hash_table_size, conflict_size> const& ht) noexcept {
	
	if (argc == 0) {
		::fast_io::io::perrln(::phy_engine::u8err,
							  ::phy_engine::ansi_escape_sequences::rst::all,
							  ::phy_engine::ansi_escape_sequences::col::red,
							  u8"[error] ",
							  ::phy_engine::ansi_escape_sequences::rst::all,
							  ::phy_engine::ansi_escape_sequences::col::white,
							  u8"No Parameters.",
							  ::phy_engine::ansi_escape_sequences::rst::all);
		return -1;
	}

	pr.reserve(static_cast<::std::size_t>(argc));

	if (*argv != nullptr) {
		pr.emplace_back_unchecked(::std::u8string_view{*argv}, nullptr, ::phy_engine::command_line::parameter_parsing_results_type::dir);
	}

	for (int i{1}; i < argc; i++) {
		if (argv[i] == nullptr)
			continue;
		::std::u8string_view argv_str{argv[i]};

		if (argv_str.empty())
			continue;

		if (argv_str.front() == u8'-') {
			auto para{::phy_engine::command_line::find_from_hash_table<hash_table_size, conflict_size>(ht, argv_str)};
			if (para == nullptr) {
				pr.emplace_back_unchecked(argv_str, nullptr, ::phy_engine::command_line::parameter_parsing_results_type::invalid_parameter);
			} else {
				if (*para->is_exist) {
					pr.emplace_back_unchecked(argv_str, nullptr, ::phy_engine::command_line::parameter_parsing_results_type::duplicate_parameter);
				} else {
					*para->is_exist = true;
					pr.emplace_back_unchecked(argv_str, para, ::phy_engine::command_line::parameter_parsing_results_type::parameter);
				}
			}
		} else {
			pr.emplace_back_unchecked(argv_str, nullptr, ::phy_engine::command_line::parameter_parsing_results_type::arg);
		}
	}

	{
		::fast_io::basic_obuf<::fast_io::u8native_io_observer> buf_u8err{::phy_engine::u8err};
		constexpr auto ign_invpm{__builtin_addressof(::phy_engine::parameter::ignore_invalid_parameters)};
		bool const ign_invpm_b{*ign_invpm->is_exist};

		bool shouldreturn{};

		for (auto& i : pr) {
			if (i.type == ::phy_engine::command_line::parameter_parsing_results_type::invalid_parameter) {
				if (ign_invpm_b) {
					::fast_io::io::perr(buf_u8err,
										::phy_engine::ansi_escape_sequences::rst::all,
										::phy_engine::ansi_escape_sequences::col::lt_red,
										u8"[warning] ",
										::phy_engine::ansi_escape_sequences::col::white,
										u8"ignore ");
				} else {
					shouldreturn = true;
					::fast_io::io::perr(buf_u8err,
										::phy_engine::ansi_escape_sequences::rst::all,
										::phy_engine::ansi_escape_sequences::col::red,
										u8"[error] ",
										::phy_engine::ansi_escape_sequences::col::white);
				}

				::fast_io::io::perr(buf_u8err,
									u8"invalid parameter: ",
									i.str);

				if (ign_invpm_b) {
					::fast_io::io::perrln(buf_u8err, ::phy_engine::ansi_escape_sequences::rst::all);
				} else {
					::std::u8string_view f_test_str{};

					::std::size_t const str_size{i.str.size()};
#if __has_cpp_attribute(assume)
					::std::size_t constexpr smax{::std::numeric_limits<::std::size_t>::max() / 4u};
					[[assume(str_size <= smax)]];
#endif
					::std::size_t const test_size{str_size * 4u / 10u};
					::std::size_t f_test_size{test_size};

					for (auto& j : ::phy_engine::parameter_lookup_table) {
						if (j.str.size() < str_size - f_test_size || j.str.size() > str_size + f_test_size)
							continue;
						if (auto const dp_res{::phy_engine::command_line::dp(i.str, j.str)}; dp_res <= test_size) {
							f_test_str = j.str;
							f_test_size = dp_res;
						}
					}

					if (f_test_str.empty()) {
						::fast_io::io::perrln(buf_u8err, ::phy_engine::ansi_escape_sequences::rst::all);
					} else {
						::fast_io::io::perrln(buf_u8err,
											  u8" (did you mean: ",
											  ::phy_engine::ansi_escape_sequences::col::purple,
											  f_test_str,
											  ::phy_engine::ansi_escape_sequences::col::white,
											  ::fast_io::mnp::chvw(u8')'),
											  ::phy_engine::ansi_escape_sequences::rst::all);
					}
				}
			} else if (i.type == ::phy_engine::command_line::parameter_parsing_results_type::duplicate_parameter) {
				if (ign_invpm_b) {
					::fast_io::io::perrln(buf_u8err,
										  ::phy_engine::ansi_escape_sequences::rst::all,
										  ::phy_engine::ansi_escape_sequences::col::lt_red,
										  u8"[warning] ",
										  ::phy_engine::ansi_escape_sequences::col::white,
										  u8"ignore duplicate parameter: ",
										  i.str,
										  ::phy_engine::ansi_escape_sequences::rst::all);
				} else {
					shouldreturn = true;
					::fast_io::io::perrln(buf_u8err,
										  ::phy_engine::ansi_escape_sequences::rst::all,
										  ::phy_engine::ansi_escape_sequences::col::red,
										  u8"[error] ",
										  ::phy_engine::ansi_escape_sequences::col::white,
										  u8"duplicate parameter: ",
										  i.str,
										  ::phy_engine::ansi_escape_sequences::rst::all);
				}
			}
		}

		if (shouldreturn) {
			return -1;
		}
	}

	bool needexit{};
	for (::size_t i{}; i < pr.size(); i++) {
		if (pr[i].para == nullptr)
			continue;
		
		if (auto const cb{pr[i].para->callback}; cb != nullptr) {
			::phy_engine::command_line::parameter_return_type const res{cb(i, pr)};
			switch (res) {
			case ::phy_engine::command_line::parameter_return_type::def:
				break;
			case ::phy_engine::command_line::parameter_return_type::return_imme :
				return -1;
			case ::phy_engine::command_line::parameter_return_type::return_soon:
				needexit = true;
			default:
				::std::unreachable();
			}
		}
	}

	if (needexit)
		return -1;
	
	return 0;
}
}