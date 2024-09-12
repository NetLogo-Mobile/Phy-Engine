#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    struct resistance
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"Resistance"};
        inline static constexpr ::fast_io::u8string_view model_description{u8"Resistance"};

        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::linear};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"R"};

        double r{10.0};
        ::phy_engine::model::pin pins[2]{{::fast_io::u8string_view{u8"A"}}, {::fast_io::u8string_view{u8"B"}}};
    };

    inline constexpr bool iterate_dc_define(::phy_engine::model::model_reserve_type_t<resistance>, resistance&) noexcept
    { 
        // add to MNA
        return true; 
    }

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<resistance>, resistance& r) noexcept
    {
        return {r.pins, 2};
    }

}  // namespace phy_engine::model
