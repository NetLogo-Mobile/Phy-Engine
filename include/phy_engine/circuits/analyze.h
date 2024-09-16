#pragma once
#include <cstdint>

namespace phy_engine
{

    enum analyze_type : ::std::uint_fast8_t
    {
        OP,
        DC,
        AC,
        ACOP,
        TR, //transient
        TROP
    };
}
