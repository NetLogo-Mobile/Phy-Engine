#pragma once

#include "version.h"

namespace phy_engine {

inline constexpr version phy_engine_version{
#include "../../../custom/version.h"
};

}