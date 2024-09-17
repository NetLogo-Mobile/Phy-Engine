#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    struct CCCS
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"CCCS"};

        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::linear};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"CCCS"};

        double m_alpha{10.0};
        ::phy_engine::model::pin pins[4]{{{u8"S"}}, {{u8"T"}}, {{u8"P"}}, {{u8"Q"}}};
    };

    inline constexpr bool
        set_attribute_define(::phy_engine::model::model_reserve_type_t<CCCS>, CCCS& cccs, ::std::size_t n, ::phy_engine::model::variant vi) noexcept
    {
        switch(n)
        {
            case 0:
            {
                // Set resistance value
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                cccs.m_alpha = vi.d;
                return true;
            }
            default:
            {
                return false;
            }
        }
        return false;
    }

    inline constexpr ::phy_engine::model::variant get_attribute_define(::phy_engine::model::model_reserve_type_t<CCCS>, CCCS const& cccs, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                // Get resistance value
                return {.d{cccs.m_alpha}, .type{::phy_engine::model::variant_type::d}};
            }
            default:
            {
                return {};
            }
        }
        return {};
    }

    inline constexpr ::fast_io::u8string_view
        get_attribute_name_define(::phy_engine::model::model_reserve_type_t<CCCS>, CCCS const& cccs, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                // m_kZimag
                return {u8"m_alpha"};
            }
            default:
            {
                return {};
            }
        }
        return {};
    }

    inline constexpr bool iterate_dc_define(::phy_engine::model::model_reserve_type_t<CCCS>, CCCS const& cccs, ::phy_engine::MNA::MNA& mna) noexcept
    {
        return true;
    }

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<CCCS>, CCCS& c) noexcept
    {
        return {c.pins, 4};
    }

}  // namespace phy_engine::model
