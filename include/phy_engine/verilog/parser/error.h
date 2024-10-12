#pragma once
#include <fast_io/fast_io.h>

namespace phy_engine::verilog
{
    enum class error_type : ::std::size_t
    {
        invaild_pretreatment
    };

    struct error_t
    {
        char8_t const* pos{};
        error_type error_code{};
    };
}
