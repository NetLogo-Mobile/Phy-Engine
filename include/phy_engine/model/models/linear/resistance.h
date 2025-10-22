#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    struct resistance
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"Resistance"};
        // inline static constexpr ::fast_io::u8string_view model_description{u8"Resistance"};

        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::linear};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"R"};

        double r{10.0};
        ::phy_engine::model::pin pins[2]{{{u8"A"}}, {{u8"B"}}};
    };

    static_assert(::phy_engine::model::model<resistance>);

    inline constexpr bool
        set_attribute_define(::phy_engine::model::model_reserve_type_t<resistance>, resistance& r, ::std::size_t n, ::phy_engine::model::variant vi) noexcept
    {
        switch(n)
        {
            case 0:
            {
                // Set resistance value
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                r.r = vi.d;
                return true;
            }
            default:
            {
                return false;
            }
        }
        return false;
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<resistance>);

    inline constexpr ::phy_engine::model::variant
        get_attribute_define(::phy_engine::model::model_reserve_type_t<resistance>, resistance const& r, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                // Get resistance value
                return {.d = r.r, .type = ::phy_engine::model::variant_type::d};
            }
            default:
            {
                return {};
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<resistance>);

    inline constexpr ::fast_io::u8string_view get_attribute_name_define(::phy_engine::model::model_reserve_type_t<resistance>, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                // resistance
                return {u8"R"};
            }
            default:
            {
                return {};
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<resistance>);

    inline constexpr bool iterate_dc_define(::phy_engine::model::model_reserve_type_t<resistance>, resistance const& r, ::phy_engine::MNA::MNA& mna) noexcept
    {
        auto const node_0{r.pins[0].nodes};
        auto const node_1{r.pins[1].nodes};
        if(node_0 && node_1) [[likely]]
        {
            auto const m_G{1.0 / r.r};
            mna.G_ref(node_0->node_index, node_0->node_index) += m_G;
            mna.G_ref(node_0->node_index, node_1->node_index) -= m_G;
            mna.G_ref(node_1->node_index, node_0->node_index) -= m_G;
            mna.G_ref(node_1->node_index, node_1->node_index) += m_G;
        }

        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_dc<resistance>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<resistance>, resistance& r) noexcept
    {
        return {r.pins, 2};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<resistance>);

}  // namespace phy_engine::model
