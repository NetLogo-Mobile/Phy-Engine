#pragma once
#include <cstdint>
#include <utility>

#include "../environment/environment.h"
#include "../../phy_engine_netlist/netlist.h"

namespace phy_engine {

enum class CKT_mode_type : ::std::uint_fast8_t {
	DC,
	AC,
	TR,
	OP,
	TrOP,
};

struct
#if __has_cpp_attribute(__gnu__::__packed__)
[[__gnu__::__packed__]]
#endif
circult {
	CKT_mode_type ckt_mode{};  // CKTmode
	::phy_engine::environment env{};
	::phy_engine::netlist nl{};
};


}  // namespace phy_engine