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
    m.xpart = 0.0;
    m.keta = 0.0;
    m.voff = 0.0;
    m.u0 = 0.05;
    m.ua = 0.0;
    m.ub = 0.0;
    m.uc = 0.0;
    m.mobMod = 3.0;

    double const vt = ::phy_engine::model::details::thermal_voltage(m.Temp);
    ::phy_engine::model::details::bsim3v32_core_cache cache{};
    (void)::phy_engine::model::details::bsim3v32_ids_core(m, 1.2, 1.0, 0.0, vt, __builtin_addressof(cache));
    double const vdsat = cache.vdsat;
    if(!std::isfinite(vdsat) || !(vdsat > 1e-6)) { return 1; }

    // Evaluate the intrinsic C-matrix just below and above Vdsat; it should be continuous (no big jump).
    double const eps = 1e-6;
    double const vds1 = vdsat * (1.0 - eps);
    double const vds2 = vdsat * (1.0 + eps);

    double c1[4][4]{};
    double c2[4][4]{};
    ::phy_engine::model::details::bsim3v32_cmatrix_capmod0_simple<false>(m, vds1, 1.2, 0.0, 0.0, c1);
    ::phy_engine::model::details::bsim3v32_cmatrix_capmod0_simple<false>(m, vds2, 1.2, 0.0, 0.0, c2);

    double max_diff{};
    for(int i{}; i < 4; ++i)
    {
        for(int j{}; j < 4; ++j)
        {
            if(!std::isfinite(c1[i][j]) || !std::isfinite(c2[i][j])) { return 2; }
            max_diff = std::max(max_diff, std::abs(c1[i][j] - c2[i][j]));
        }
    }

    double const cox = ::phy_engine::model::details::k_eps_ox / m.tox;
    double const cgg = cox * m.W * m.L;
    if(!(cgg > 0.0) || !std::isfinite(cgg)) { return 3; }

    if(!(max_diff < 0.1 * cgg)) { return 4; }
    return 0;
}
