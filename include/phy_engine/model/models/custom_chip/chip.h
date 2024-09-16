#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    struct custom_clip
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"Verilog Module"};
        inline static constexpr ::fast_io::u8string_view model_description{u8"You can compile and run verilog module in custom clip."};
        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::digital};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"VM"};
    };


    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<custom_clip>, custom_clip&) noexcept
    {
        return {};
    }

}  // namespace phy_engine::model
