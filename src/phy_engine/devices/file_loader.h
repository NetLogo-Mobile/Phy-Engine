#pragma once

#include "../../phy_engine_utils/fast_io/fast_io.h"
#include "../../phy_engine_utils/fast_io/fast_io_device.h"
#include "../../phy_engine_utils/fast_io/fast_io_dsal/vector.h"

#include "native_io.h"
#include "../command_line/parsing_result.h"

namespace phy_engine::file {

inline int file_loader(::std::u8string_view const dir) noexcept {

	// inline file ...

	try {
		::fast_io::u8ibuf_file_mutex files{dir};
		// lock the file
		// check file format
		// scan
	} catch (const ::fast_io::error e) {
		::fast_io::io::perr(::phy_engine::u8err,
							::phy_engine::ansi_escape_sequences::rst::all,
							::phy_engine::ansi_escape_sequences::col::white,
							u8"Phy Engine: ",
							::phy_engine::ansi_escape_sequences::col::bd_red,
							u8"[fatal] ",
							::phy_engine::ansi_escape_sequences::col::white,
							u8"file loaded failed: \"",
							::phy_engine::ansi_escape_sequences::col::dk_gray,
							dir,
							::phy_engine::ansi_escape_sequences::col::white,
							u8"\", reason: ",
							e,
							::phy_engine::ansi_escape_sequences::rst::all,
							u8"\nPhy Engine terminated.\n");
		return -2;
	}

#if 1 // defined(_DEBUG)
	::fast_io::io::print(::phy_engine::u8out,
						 ::phy_engine::ansi_escape_sequences::rst::all,
						 ::phy_engine::ansi_escape_sequences::col::white,
						 u8"Phy Engine: ",
						 ::phy_engine::ansi_escape_sequences::col::lt_cyan,
						 u8"[debug] ",
						 ::phy_engine::ansi_escape_sequences::rst::all,
						 u8"file loaded successfully: \"",
						 ::phy_engine::ansi_escape_sequences::col::dk_gray,
						 dir,
						 ::phy_engine::ansi_escape_sequences::rst::all,
						 u8"\"\n");
#endif 

	return 0;
}
}