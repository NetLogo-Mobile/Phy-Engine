#pragma once
#include <cstddef>
#include "branch.h"

namespace phy_engine::model
{
    struct branch_view
    {
        ::phy_engine::model::branch* branches{};
        ::std::size_t size{};
    };
}  // namespace phy_engine::model
