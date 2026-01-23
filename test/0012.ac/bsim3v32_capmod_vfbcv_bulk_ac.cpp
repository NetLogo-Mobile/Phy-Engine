#include <algorithm>
#include <cmath>
#include <limits>

#include <phy_engine/model/models/non-linear/bsim3v32.h>

int main()
{
    ::phy_engine::model::bsim3v32_nmos m{};
    m.W = 1e-6;
    m.L = 1e-6;
    m.tox = 1e-8;
    m.toxm = 1e-8;
    m.Vth0 = 0.7;
    m.phi = 0.7;
    m.capMod = 3.0;

    // Keep charge model numerically well-defined.
    m.u0 = 0.05;
    m.ua = 0.0;
    m.ub = 0.0;
    m.uc = 0.0;
    m.mobMod = 3.0;

    // At Vg=Vd=Vs=Vb=0:
    // - vfbcv=+0.5 => x = vgb - vfb = -0.5 -> accumulation => ~CoxWL coupling to bulk
    // - vfbcv=-0.5 => x = +0.5 -> depletion => ~(Cox||Cdep)WL coupling to bulk (smaller than CoxWL)
    double c0[4][4]{};
    double c1[4][4]{};

    m.vfbcv = +0.5;
    ::phy_engine::model::details::bsim3v32_cmatrix_capmod0_simple<false>(m, 0.0, 0.0, 0.0, 0.0, c0);

    m.vfbcv = -0.5;
    ::phy_engine::model::details::bsim3v32_cmatrix_capmod0_simple<false>(m, 0.0, 0.0, 0.0, 0.0, c1);

    for(int i{}; i < 4; ++i)
    {
        for(int j{}; j < 4; ++j)
        {
            if(!std::isfinite(c0[i][j]) || !std::isfinite(c1[i][j])) { return 1; }
        }
    }

    // Bulk charge sensitivity to gate voltage (Cbg) should be larger in accumulation than depletion.
    double const cbg0 = std::abs(c0[3][1]);
    double const cbg1 = std::abs(c1[3][1]);
    if(!(cbg0 > 0.0)) { return 2; }
    if(!(cbg0 > cbg1 * 1.1)) { return 3; }
    return 0;
}
