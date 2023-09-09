/**************
 * Phy Engine *
 *************/

#include "../phy_engine_utils/fast_io/fast_io.h" // fast_io
#include "devices/native_io.h" // native_io

int main(int argc, char** argv) noexcept {
	if (argc != 2) {
		if (argc == 0) {
			return 1;
		}
		::fast_io::io::perr(::phy_engine::native_io::u8err, u8"Usage: ", fast_io::mnp::code_cvt_os_c_str(*argv), u8" <file>\n");
		return 1;
	}

	::fast_io::io::println(::phy_engine::native_io::u8out, u8"Phy Engine");
	
	return 0;
}