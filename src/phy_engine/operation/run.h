#pragma once

namespace phy_engine {

inline int run() noexcept {
#if 0
	auto& parse_res{::phy_engine::parsing_result};

	if (parse_res.size() > 1) {
		if (parse_res[1].type == ::phy_engine::command_line::parameter_parsing_results_type::file) {
			if (::phy_engine::load_file(parse_res[1].str) != 0) 
				return -2;
		} else {
			constexpr auto& version_para{::phy_engine::parameter::version};
			constexpr auto& help_para{::phy_engine::parameter::help};
			constexpr auto& contributor_para{::phy_engine::parameter::contributor};
			if (!(*version_para.is_exist || *help_para.is_exist || *contributor_para.is_exist)) {
				::fast_io::io::perr(::phy_engine::u8err,
									::phy_engine::ansi_escape_sequences::rst::all,
									::phy_engine::ansi_escape_sequences::col::white,
									u8"Phy Engine: ",
									::phy_engine::ansi_escape_sequences::col::bd_red,
									u8"[fatal] ",
									::phy_engine::ansi_escape_sequences::col::white,
									u8"no input files\n"
									u8"Usage: $",
									::phy_engine::ansi_escape_sequences::col::orange,
									u8"phy_engine <file> [options]\n",
									::phy_engine::ansi_escape_sequences::rst::all);
				return -2;
			}
		}
	} else {
		::fast_io::io::perr(::phy_engine::u8err,
							::phy_engine::ansi_escape_sequences::rst::all,
							::phy_engine::ansi_escape_sequences::col::white,
							u8"Phy Engine: ",
							::phy_engine::ansi_escape_sequences::col::bd_red,
							u8"[fatal] ",
							::phy_engine::ansi_escape_sequences::col::white,
							u8"no input files\n"
							u8"Usage: $",
							::phy_engine::ansi_escape_sequences::col::orange,
							u8"phy_engine <file> [options]\n",
							::phy_engine::ansi_escape_sequences::rst::all);
		return -2;
	}

#endif  // 0

}
}