#include <algorithm>
#include <cmath>
#include <limits>

#include <phy_engine/model/models/non-linear/bsim3v32.h>

static bool check_bias(double vd, double vg, double vs, double vb) noexcept
{
    ::phy_engine::model::bsim3v32_nmos m{};
    m.W = 1e-6;
    m.L = 1e-6;
    m.tox = 1e-8;
    m.toxm = 1e-8;
    m.Vth0 = 0.7;
    m.phi = 0.7;
    m.capMod = 3.0;
    m.xpart = 0.5;
    m.keta = 0.0;
    m.voff = 0.0;
    m.u0 = 0.05;
    m.ua = 0.0;
    m.ub = 0.0;
    m.uc = 0.0;
    m.mobMod = 3.0;
    m.delta = 1e-2;

    double c[4][4]{};
    ::phy_engine::model::details::bsim3v32_cmatrix_capmod0_simple<false>(m, vd, vg, vs, vb, c);

    double max_abs{};
    for(int i{}; i < 4; ++i)
    {
        for(int j{}; j < 4; ++j)
        {
            if(!std::isfinite(c[i][j])) { return false; }
            max_abs = std::max(max_abs, std::abs(c[i][j]));
        }
    }

    double const cox = ::phy_engine::model::details::k_eps_ox / m.tox;
    double const cgg = cox * m.W * m.L;
    if(!(cgg > 0.0) || !std::isfinite(cgg)) { return false; }

    // Row sums (∑j dQi/dVj) must be ~0 by construction (charge invariance to common-mode).
    for(int i{}; i < 4; ++i)
    {
        double sum{};
        for(int j{}; j < 4; ++j) { sum += c[i][j]; }
        if(!(std::abs(sum) < 1e-12 + 1e-6 * std::max(cgg, max_abs))) { return false; }
    }

    // Column sums (∑i dQi/dVj) must be ~0 if total charge is conserved (Qd+Qg+Qs+Qb = 0).
    for(int j{}; j < 4; ++j)
    {
        double sum{};
        for(int i{}; i < 4; ++i) { sum += c[i][j]; }
        if(!(std::abs(sum) < 1e-12 + 1e-6 * std::max(cgg, max_abs))) { return false; }
    }

    return true;
}

int main()
{
    // A few representative biases (cutoff, linear, saturation-ish, accumulation-ish).
    if(!check_bias(0.0, 0.0, 0.0, 0.0)) { return 1; }
    if(!check_bias(0.05, 1.2, 0.0, 0.0)) { return 2; }
    if(!check_bias(2.0, 2.0, 0.0, 0.0)) { return 3; }
    if(!check_bias(0.0, -1.0, 0.0, 0.0)) { return 4; }
    if(!check_bias(1.0, 1.2, 0.0, -0.5)) { return 5; }
    return 0;
}

