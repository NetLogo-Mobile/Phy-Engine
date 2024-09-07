#pragma once

#include <fast_io/fast_io.h>

namespace phy_engine
{
    struct tz_set_s
    {
#if __has_cpp_attribute(__gnu__::__cold__)
        [[__gnu__::__cold__]]
#endif
        tz_set_s() noexcept
        {
            ::fast_io::posix_tzset();
        }
    };
}  // namespace phy_engine
