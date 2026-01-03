#pragma once
#include <cstddef>
#include <cstdint>

namespace phy_engine::analyzer
{
    struct AC
    {
        enum class sweep_type : ::std::uint_fast8_t
        {
            single = 0,
            linear = 1,
            log = 2,
        };

        sweep_type sweep{sweep_type::single};

        // Angular frequency (rad/s). For sweep mode this will be updated per-point.
        double omega{};

        // Sweep settings (rad/s)
        double omega_start{};
        double omega_stop{};
        ::std::size_t points{};
    };
}  // namespace phy_engine::analyzer
