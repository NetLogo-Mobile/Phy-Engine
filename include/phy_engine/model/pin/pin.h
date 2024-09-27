#pragma once
#include <fast_io/fast_io_dsal/string_view.h>

namespace phy_engine::model
{
    struct node_t;

    enum pin_type : ::std::uint_fast8_t
    {
        analog = 0,
        digital_in = 1,
        digital_out = 2
    };

    struct pin
    {
        ::fast_io::u8string_view name{};
        node_t* nodes{};

#if 0
        pin_type type{};

        // digital
        double Ll{0.0};  // low_level
        double Hl{5.0};  // high_level
        double Tsu{1e-9};    // unsteady_state_setup_time
        double Th{5e-10};     // unsteady_state_hold_time

        // private:
        double last_duration{}; // calculate unsteady state
#endif
    };
}  // namespace phy_engine::model
