#pragma once
#include <fast_io/fast_io_dsal/string_view.h>

namespace phy_engine::model
{
    struct node_t;
    struct model_base;

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
        model_base* model{};
    };
}  // namespace phy_engine::model
