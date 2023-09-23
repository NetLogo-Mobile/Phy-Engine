/**************
 * Phy Engine *
 *************/

#include "../phy_engine_utils/fast_io/fast_io.h" // fast_io

#include "devices/native_io.h" // native_io
#include "version/phy_engine.h"
#include "../phy_engine_utils/ansies/impl.h"
#include "../phy_engine_utils/command_line/impl.h"
#include "command_line/impl.h"
#include "command_line/parsing_result.h"
#include "devices/file_loader.h"

int main(int argc, char** argv) noexcept {

#ifdef _DEBUG
	::fast_io::io::println(::phy_engine::u8out,
						 ::phy_engine::ansi_escape_sequences::rst::all,
						 u8"Phy Engine Dev ",
						 ::phy_engine::ansi_escape_sequences::col::green,
						 u8"V",
						 ::phy_engine::phy_engine_version,
						 ::phy_engine::ansi_escape_sequences::rst::color);
#endif

	auto& parse_res{::phy_engine::parsing_result};

	int pr{::phy_engine::parsing(argc, reinterpret_cast<char8_t**>(argv), parse_res, ::phy_engine::hash_table)};

	if (pr != 0) {
		return pr;
	}

	if (parse_res.size() >= 2) {
		bool has_file{};
		for (::std::size_t i{}; i < parse_res.size(); i++) {
			if (parse_res[i].type == ::phy_engine::command_line::parameter_parsing_results_type::arg) {
				if (has_file) {
					::fast_io::io::perr(::phy_engine::u8err,
										::phy_engine::ansi_escape_sequences::rst::all,
										::phy_engine::ansi_escape_sequences::col::white,
										u8"Phy Engine: ",
										::phy_engine::ansi_escape_sequences::col::lt_red,
										u8"[warning] ",
										::phy_engine::ansi_escape_sequences::col::white,
										u8"Only one file can be opened simultaneously, subsequent files \"",
										::phy_engine::ansi_escape_sequences::col::dk_gray,
										parse_res[i].str,
										::phy_engine::ansi_escape_sequences::col::white,
										u8"\" will be ignored.\n",
										::phy_engine::ansi_escape_sequences::rst::all);
					continue;
				}
				has_file = true;
				if (auto const ret{::phy_engine::file::file_loader(parse_res[i].str)}; ret != 0) {
					return ret;
				}
			}
		}

		if (!has_file) {
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
									u8"no input files\n",
									::phy_engine::ansi_escape_sequences::rst::all,
									u8"\nPhy Engine terminated.\n");
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
							u8"no input files\n",
							::phy_engine::ansi_escape_sequences::rst::all,
							u8"\nPhy Engine terminated.\n");
		return -2;
	}

	// run

	return 0;
}