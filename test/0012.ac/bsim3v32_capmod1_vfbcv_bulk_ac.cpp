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
    m.capMod = 1.0;

    // Keep charge model numerically well-defined.
    m.u0 = 0.05;
    m.ua = 0.0;
    m.ub = 0.0;
    m.uc = 0.0;
    m.mobMod = 3.0;

    // At Vg=Vd=Vs=Vb=0 (signed variables vgs_s=vds_s=vbs_s=0):
    // - vfbcv=+0.5 => x = vgb - vfb = -0.5 -> accumulation => ~CoxWL coupling to bulk
    // - vfbcv=-0.5 => x = +0.5 -> depletion => ~(Cox||Cdep)WL coupling to bulk (smaller)
    double cgs0{}, cgd0{}, cgb0{};
    double cgs1{}, cgd1{}, cgb1{};

    double const vt = ::phy_engine::model::details::thermal_voltage(m.Temp);

    m.vfbcv = +0.5;
    ::phy_engine::model::details::bsim3v32_meyer_intrinsic_caps<false>(m, 0.0, 0.0, 0.0, vt, cgs0, cgd0, cgb0);

    m.vfbcv = -0.5;
    ::phy_engine::model::details::bsim3v32_meyer_intrinsic_caps<false>(m, 0.0, 0.0, 0.0, vt, cgs1, cgd1, cgb1);

    if(!std::isfinite(cgs0) || !std::isfinite(cgd0) || !std::isfinite(cgb0)) { return 1; }
    if(!std::isfinite(cgs1) || !std::isfinite(cgd1) || !std::isfinite(cgb1)) { return 1; }

    // Gate-bulk coupling should be larger in accumulation than in depletion.
    if(!(std::abs(cgb0) > 0.0)) { return 2; }
    if(!(std::abs(cgb0) > std::abs(cgb1) * 1.1)) { return 3; }
    return 0;
}

