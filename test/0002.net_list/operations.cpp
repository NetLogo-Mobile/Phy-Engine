#include <fast_io/fast_io.h>
#include <phy_engine/netlist/impl.h>
#include <phy_engine/model/models/custom_chip/chip.h>

int main()
{
    ::phy_engine::netlist::netlist nl{};
    ::phy_engine::model::custom_clip cl{};
    constexpr auto is_model = ::phy_engine::model::model<::phy_engine::model::custom_clip>;
    constexpr auto can_iterate_dc = ::phy_engine::model::defines::can_iterate_dc<::phy_engine::model::custom_clip>;
    for(::std::size_t i{}; i < ::phy_engine::netlist::details::netlist_block::chunk_module_size * 2; i++) 
    { 
        auto ret = add_model(nl, cl); 
        ::fast_io::io::perrln(ret.mod_pos.vec_pos, " ", ret.mod_pos.chunk_pos);
    }
    //prepare(nl);
}
