#pragma once
#include <cstdint>

namespace phy_engine
{

    enum class analyze_type : ::std::uint_fast32_t
    {
        OP = 0,
        DC,
        AC,
        ACOP,
        TR, //transient
        TROP
    };
}
