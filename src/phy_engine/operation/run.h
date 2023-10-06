#pragma once
#include <cstdlib>
#include <string>

#include "../../phy_engine_utils/fast_io/fast_io.h"
#include "../../phy_engine_utils/fast_io/fast_io_device.h"

#include "../devices/native_io.h"
#include "../../phy_engine_utils/ansies/impl.h"

namespace phy_engine {

inline int run() noexcept {
#if 0
	::fast_io::basic_ibuf<::fast_io::u8native_io_observer> ibuf{::phy_engine::u8in};
	while (true) {
		::fast_io::io::print(::phy_engine::u8out, u8"\nphy_engine >");
		while (true) {
			::std::u8string str;
			::fast_io::io::scan<true>(ibuf, str);
			if (!str.empty()) {
				break;
			}
		}

	}
#endif  // 0

	return 0;
}
}