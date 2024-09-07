#pragma once
#include <cstdint>
#include <utility>

#include "environment/environment.h"
#include "../netlist/netlist.h"

namespace phy_engine
{

    enum class CKT_mode_type : ::std::uint_fast8_t
    {
        DC,
        AC,
        TR,
        OP,
        TrOP,
    };

    struct
#if __has_cpp_attribute(__gnu__::__packed__)
        [[__gnu__::__packed__]]
#endif
        circult
    {
        CKT_mode_type ckt_mode{};  // CKTmode
        ::phy_engine::environment env{};
        ::phy_engine::model::netlist nl{};

        constexpr ::std::size_t get_component_size() noexcept {}

        template <::phy_engine::model::model mod>
        constexpr void add_component(mod&& m) noexcept
        {
        }

        constexpr bool delete_component(::std::size_t pos) noexcept {}

        constexpr ::phy_engine::model::model_base* get_component(::std::size_t pos) noexcept {}

        constexpr bool add_wire() noexcept {}

        constexpr bool delete_wire() noexcept {}

        constexpr bool add_netlist(::phy_engine::model::netlist const& nl_add) noexcept {}

        constexpr ::phy_engine::model::netlist get_netlist(::std::size_t* pos_view, ::std::size_t size) noexcept {}

        constexpr void optimize_memory() noexcept {}

        constexpr ::phy_engine::model::model_base* find_component_from_mtype(::phy_engine::model::model_type type, ::std::size_t identification) noexcept {}

        constexpr ::fast_io::vector<::fast_io::u8string> find_component_from_name(::std::u8string_view str) noexcept {}
    };

}  // namespace phy_engine
