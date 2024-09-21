#pragma once
#include <cstddef>
#include "integral_history.h"

namespace phy_engine::solver
{
    struct integral_history_view
    {
        ::phy_engine::solver::integral_history* X{};
        ::phy_engine::solver::integral_history* Y{};
        ::std::size_t size{};

    };
}
