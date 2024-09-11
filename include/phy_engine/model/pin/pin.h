#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include "../node/node.h"
namespace phy_engine::model
{
    struct pin
    {
        ::fast_io::u8string_view name{};
        ::phy_engine::model::node_t* nodes{};
    };
}  // namespace phy_engine::model
