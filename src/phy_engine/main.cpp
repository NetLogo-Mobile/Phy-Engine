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
#include "operation/run.h"

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

	return ::phy_engine::run();
}