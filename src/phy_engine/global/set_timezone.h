#pragma once
#include "../../phy_engine_utils/fast_io/fast_io.h"

namespace phy_engine::global {
struct set_timezone {
	set_timezone() noexcept {
		::fast_io::posix_tzset();
	}
};
}