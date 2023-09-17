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
							  ::phy_engine::ansi_escape_sequences::sty::bold,
							  ::phy_engine::ansi_escape_sequences::col::red,
							  u8"[error] ",
							  ::phy_engine::ansi_escape_sequences::rst::all,
							  ::phy_engine::ansi_escape_sequences::col::white,
							  u8"No Parameters.",
							  ::phy_engine::ansi_escape_sequences::rst::all);
		return -1;
	}

	::fast_io::vector<pair_str_fpr> inv_pars{};

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
			if (str_vw.find(u8'=') != ::std::u8string_view::npos) {
				h = true;
			}

			auto res{::phy_engine::command_line::find_parameter(str_vw, h, cl)};
			if (res.par == nullptr) {
				inv_pars.emplace_back(str_vw, res);
				continue;
			} else {
				cpr.parameters_res.emplace(res.par, ::phy_engine::command_line::details::str_parameter_res{res.str, i});
			}

		} else {
			cpr.optval_res.emplace(str_vw, ::phy_engine::command_line::details::str_optval_res{i});
		}
	}

	if (!inv_pars.empty()) {
		constexpr auto ign_invpm{__builtin_addressof(::phy_engine::parameter::ignore_invalid_parameters)};
		bool const ign_invpm_b{cpr.parameters_res.contains(ign_invpm)};
		::fast_io::basic_obuf<::fast_io::u8native_io_observer> buf_u8err{::phy_engine::u8err};

		for (auto& [par, fpr] : inv_pars) {
			if (ign_invpm_b) {			
				::fast_io::io::perr(buf_u8err,
									::phy_engine::ansi_escape_sequences::rst::all,
									::phy_engine::ansi_escape_sequences::sty::bold,
									::phy_engine::ansi_escape_sequences::col::lt_red,
									u8"[warning] ",
									::phy_engine::ansi_escape_sequences::rst::all);
			} else {
				::fast_io::io::perr(buf_u8err,
									::phy_engine::ansi_escape_sequences::rst::all,
									::phy_engine::ansi_escape_sequences::col::red,
									u8"[error] ",
									::phy_engine::ansi_escape_sequences::rst::all);
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

	for (auto& [f, s] : cpr.parameters_res) {
		if (f->callback != nullptr) {
			f->callback(argc, argv, s.pos, s.option);
		}
	}

	return 0;
}
