#pragma once
#include <cstddef>
#include "node.h"

namespace phy_engine::model
{
    struct node_view
    {
        ::phy_engine::model::node_t* nodes{};
        ::std::size_t size{};
    };
}  // namespace phy_engine::model
