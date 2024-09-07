#pragma once
#include <type_traits>
#include <concepts>

#include "netlist.h"

namespace phy_engine::model
{
    template <typename mod>
    concept can_generate_netlist = requires(mod&& m) {
        { generate_netlist_define(model_reserve_type<::std::remove_cvref_t<mod>>) } -> ::std::same_as<netlist>;
    };

    template <typename mod>
    concept can_generate_netlist_const_lvalue = requires(mod&& m) {
        { generate_netlist_define(model_reserve_type<::std::remove_cvref_t<mod>>) } -> ::std::same_as<netlist const&>;
    };

}  // namespace phy_engine::model
