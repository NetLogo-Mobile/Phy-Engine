#pragma once
#include <cstddef>
#include <cstdint>
#include <fast_io/fast_io_core.h>

namespace phy_engine::solver
{

    // #define ITEM 0.001 //Calculation accuracy
    struct Longkuta
    {
        double progess() const noexcept
        {
            if(fun == nullptr) [[unlikely]] { ::fast_io::fast_terminate(); }

            double x0{x};
            double y0{y};
            double K1, K2, K3, K4, y1, x1;

            for(;;)
            {
                x1 = x0 + h;
                if(x1 > y) [[unlikely]] { break; }

                K1 = fun(x0, y0);
                K2 = fun(x0 + h / 2.0, y0 + K1 * h / 2.0);
                K3 = fun(x0 + h / 2.0, y0 + K2 * h / 2.0);
                K4 = fun(x1, y0 + h * K3);
                y1 = y0 + h * (K1 + 2 * K2 + 2 * K3 + K4) / 6.0;
                x0 = x1;
                y0 = y1;
#if 0
                if(::std::abs(x0 - y1) < 0.001) { break; }
#endif
            }

            return y0;
        }

        double (*fun)(double x0, double y0) noexcept {};  // Define function here
        double x{};
        double y{};
        double h{};
    };

}  // namespace phy_engine::solver
