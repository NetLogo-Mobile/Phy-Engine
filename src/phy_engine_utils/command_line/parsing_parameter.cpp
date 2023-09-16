#include "command_line.h"

#include "../../phy_engine/command_line/parameters.h"

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
	for (int i{}; i < argc; i++) {
		if (argv == nullptr)
			continue;
		char8_t* str{*argv};
		::std::size_t length{::std::char_traits<char8_t>::length(str)};
		if (length == 0)
			continue;
		if (*str == u8'-') {
			// auto res{::fast_io::freestanding::find_if(parameter_lookup_table.begin(), parameter_lookup_table.end(), )};
			// to do
		} else {
			cpr.optval_res.emplace(::std::u8string_view{str, length}, i);
		}
	}
}
