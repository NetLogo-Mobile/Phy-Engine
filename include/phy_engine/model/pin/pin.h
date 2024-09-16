#pragma once
#include <fast_io/fast_io_dsal/string_view.h>

namespace phy_engine::model
{
    struct node_t;

    struct pin
    {
        ::fast_io::u8string_view name{};
        node_t* nodes{};

        // storage
        double current{};
    };
}  // namespace phy_engine::model
