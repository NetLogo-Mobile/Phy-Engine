#include "help.h"
#include "../parameters.h"

bool (::phy_engine::parameter::details::help_callback)(int argc, char8_t** argv, int pos, ::std::u8string_view var) noexcept {
	::fast_io::io::print(::phy_engine::u8out, u8"Arguments:\n");
	for (auto& p : ::phy_engine::parameters) {
		::fast_io::io::print(::phy_engine::u8out, p.name);
		if (p.alias.array_size != 0) {
			::fast_io::io::print(::phy_engine::u8out, u8" (aka: ");
			for (::std::size_t i{}; i < p.alias.array_size - 1; i++) {
				::fast_io::io::print(::phy_engine::u8out, p.alias[i], u8", ");
			}
			::fast_io::io::print(::phy_engine::u8out, p.alias.back(), ::fast_io::mnp::chvw(u8')'));
		}
		::fast_io::io::println(::phy_engine::u8out, u8" --- ", p.describe);
	}
	return true;
}