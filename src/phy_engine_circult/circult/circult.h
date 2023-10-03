#pragma once
#include <cstdint>
#include <utility>

#include "../environment/environment.h"

namespace phy_engine {

enum class CKT_mode_type : ::std::uint_least8_t {
	DC,
	AC,
	TR
};

struct
#if __has_cpp_attribute(__gnu__::__packed__)
[[__gnu__::__packed__]]
#endif
circult {
	CKT_mode_type CKT_mode{};  // CKTmode
	::phy_engine::environment env{};
};


}  // namespace phy_engine