#include <fast_io/fast_io.h>
#include <phy_engine/model/models/non-linear/PN_junction.h>

int main()
{
    ::phy_engine::model::PN_junction pn{};
    pn.Is = 1e-14;
    pn.Isr = 2e-14;
    pn.Area = 10.0;
    pn.Bv = 40.0;
    pn.Bv_set = true;

    double const is_before{pn.Is};
    double const isr_before{pn.Isr};
    double const bv_before{pn.Bv};

    prepare_foundation_define(::phy_engine::model::model_reserve_type<::phy_engine::model::PN_junction>, pn);
    prepare_foundation_define(::phy_engine::model::model_reserve_type<::phy_engine::model::PN_junction>, pn);

    if(pn.Is != is_before || pn.Isr != isr_before || pn.Bv != bv_before)
    {
        ::fast_io::io::perr("pn_prepare_idempotent: public parameters mutated by prepare_foundation_define\n");
        return 1;
    }

    if(pn.Is_eff != pn.Is * pn.Area || pn.Isr_eff != pn.Isr * pn.Area)
    {
        ::fast_io::io::perr("pn_prepare_idempotent: effective parameters not updated correctly\n");
        return 1;
    }

    return 0;
}

