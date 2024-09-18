#include <fast_io/fast_io.h>

#include <phy_engine/circuits/solver/longkuta.h>

inline constexpr double fun(double x0, double y0) noexcept  // Function expression
{
    double dy{y0 - (2.0 * x0) / y0};
    return dy;
}

int main()
{
    ::phy_engine::solver::Longkuta lk{.fun{fun}, .x{0.0}, .y{1.0}, .h{0.0005}};
    double res{lk.progess()};

    ::fast_io::io::perrln(res);

    return 0;
}
