#pragma once
#include <cstddef>
namespace phy_engine::optimizer
{
    struct Gmin
    {
        double m_gmin;
        ::std::size_t m_lastFixRow;
        bool m_gminEnabled;
    };
}  // namespace phy_engine::optimizer
