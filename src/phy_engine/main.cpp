/**************
 * Phy Engine *
 *************/

#include "../phy_engine_utils/fast_io/fast_io.h" // fast_io
#include "devices/native_io.h" // native_io
#include "console/impl.h" // set_console_cp_to_utf8

int main(int argc, char** argv) noexcept {
	return ::phy_engine::console::command_line(argc, argv);
}