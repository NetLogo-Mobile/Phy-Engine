#include "../fast_io/fast_io_dsal/vector.h"

#include "command_line.h"
#include "../../phy_engine/command_line/parameters.h"

#include "../../phy_engine/command_line/inc.h"

struct pair_str_fpr {
	::std::u8string_view str{};
	::phy_engine::command_line::find_parameter_res fpr{};
};

int ::phy_engine::command_line::parsing_parameters(
	int argc,
	char8_t** argv,
	command_line const& cl,
	cl_parse_res& cpr) noexcept {
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

	::fast_io::vector<pair_str_fpr> inv_pars{};
	::fast_io::vector<::phy_engine::command_line::find_parameter_res> has_prerequisite_or_clash{};

	for (int i{1}; i < argc; i++) {
		if (argv[i] == nullptr)
			continue;
		char8_t* str{argv[i]};
		::std::size_t length{::std::char_traits<char8_t>::length(str)};
		if (length == 0)
			continue;

		::std::u8string_view str_vw{str, length};

		if (*str == u8'-') {
			bool h{};

			auto const fres{str_vw.find(u8'=')};

			if (fres != ::std::u8string_view::npos) {
				h = true;
			}

			auto res{::phy_engine::command_line::find_parameter(str_vw, h, cl)};
			if (res.para == nullptr) {
				inv_pars.emplace_back(str_vw, res);
				continue;
			} else {
				if (res.para->has_value != h) {
					inv_pars.emplace_back(str_vw);
				} else {
					cpr.parameters_res.emplace(res.para, ::phy_engine::command_line::details::str_parameter_res{res.str, i});
					if (res.para->prerequisite.data_ptr != nullptr || res.para->clash.data_ptr != nullptr) 
						has_prerequisite_or_clash.emplace_back(str_vw.substr(0u, fres), res.para);
				}
			}

		} else {
			cpr.optval_res.emplace(str_vw, ::phy_engine::command_line::details::str_optval_res{i});
		}
	}

	{
		::fast_io::basic_obuf<::fast_io::u8native_io_observer> buf_u8err{::phy_engine::u8err};

		if (!inv_pars.empty()) {
			constexpr auto ign_invpm{__builtin_addressof(::phy_engine::parameter::ignore_invalid_parameters)};
			bool const ign_invpm_b{cpr.parameters_res.contains(ign_invpm)};

			for (auto& [par, fpr] : inv_pars) {
				if (ign_invpm_b) {
					::fast_io::io::perr(buf_u8err,
										::phy_engine::ansi_escape_sequences::rst::all,
										::phy_engine::ansi_escape_sequences::col::lt_red,
										u8"[warning] ");
				} else {
					::fast_io::io::perr(buf_u8err,
										::phy_engine::ansi_escape_sequences::rst::all,
										::phy_engine::ansi_escape_sequences::col::red,
										u8"[error] ");
				}
				::fast_io::io::perr(buf_u8err,
									::phy_engine::ansi_escape_sequences::col::white,
									u8"invalid parameter: ",
									par);
				if (fpr.str.empty()) {
					::fast_io::io::perrln(buf_u8err, ::phy_engine::ansi_escape_sequences::rst::all);
				} else {
					::fast_io::io::perrln(buf_u8err,
										  u8" (did you mean: ",
										  fpr.str,
										  ::fast_io::mnp::chvw(u8')'),
										  ::phy_engine::ansi_escape_sequences::rst::all);
				}
			}

			if (!ign_invpm_b) {
				return -1;
			}
		}

		if (!has_prerequisite_or_clash.empty()) {
			bool should_return{};
			for (auto &i : has_prerequisite_or_clash) {
				if (i.para->prerequisite.data_ptr != nullptr) {
					for (auto j : i.para->prerequisite) {
						if (!cpr.parameters_res.contains(j)) {
							should_return = true;
							::fast_io::io::perrln(buf_u8err,
												::phy_engine::ansi_escape_sequences::rst::all,
												::phy_engine::ansi_escape_sequences::col::red,
												u8"[error] ",
												::phy_engine::ansi_escape_sequences::col::white,
												u8"Parameter ",
												::phy_engine::ansi_escape_sequences::col::purple,
												u8"(",
												i.str,
												u8")",
												::phy_engine::ansi_escape_sequences::col::white,
												u8" depends on parameter ",
												::phy_engine::ansi_escape_sequences::col::purple,
												u8"(",
												j->name,
												u8")",
												::phy_engine::ansi_escape_sequences::rst::all);
						}
					}
				} else {
					for (auto j : i.para->clash) {
						if (auto fres{cpr.parameters_res.find(j)}; fres != cpr.parameters_res.end()) {
							should_return = true;
							::std::u8string_view str{argv[fres->second.pos]};
							if (fres->first->has_value)
								str = str.substr(0u, str.find(u8'='));
							::fast_io::io::perrln(buf_u8err,
												::phy_engine::ansi_escape_sequences::rst::all,
												::phy_engine::ansi_escape_sequences::col::red,
												u8"[error] ",
												::phy_engine::ansi_escape_sequences::col::white,
												u8"Parameter ",
												::phy_engine::ansi_escape_sequences::col::purple,
												u8"(",
												i.str,
												u8")",
												::phy_engine::ansi_escape_sequences::col::white,
												u8" conflicts with parameter ",
												::phy_engine::ansi_escape_sequences::col::purple,
												u8"(",
												str,
												u8")",
												::phy_engine::ansi_escape_sequences::rst::all);
						}
					}

				}
			}

			if (should_return) {
				return -1;
			}
		}
	}

	for (auto& [f, s] : cpr.parameters_res) {
		if (f->callback != nullptr) {
			if (!f->callback(argc, argv, s.pos, s.option)) {
				return -1;
			}
		}
	}

	return 0;
}
