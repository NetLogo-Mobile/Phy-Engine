#include <fast_io/fast_io.h>
#include <phy_engine/netlist/impl.h>
#include <phy_engine/model/models/linear/resistance.h>

int main()
{
    ::phy_engine::model::resistance r{};

    constexpr auto is_model = ::phy_engine::model::model<::phy_engine::model::resistance>;
    constexpr auto can_iterate_dc = ::phy_engine::model::defines::can_iterate_dc<phy_engine::model::resistance>;
    constexpr auto has_attribute = ::phy_engine::model::defines::has_attribute<::phy_engine::model::resistance>;
}
