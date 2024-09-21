#pragma once
#include <cstddef>
#include <complex>

namespace phy_engine::model
{
    struct branch
    {
        ::std::size_t index{};
        ::std::complex<double> current{};
    };
}  // namespace phy_engine::model
