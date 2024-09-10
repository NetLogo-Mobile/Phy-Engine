#include <fast_io/fast_io.h>
#include <phy_engine/netlist/impl.h>
#include <phy_engine/model/models/custom_chip/chip.h>

int main()
{
    ::phy_engine::netlist::netlist nl{};
    ::phy_engine::model::custom_clip cl{};
    constexpr auto is_model = ::phy_engine::model::model<::phy_engine::model::custom_clip>;
    constexpr auto can_iterate_dc = ::phy_engine::model::defines::can_iterate_dc<::phy_engine::model::custom_clip>;

}
