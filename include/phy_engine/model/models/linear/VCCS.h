#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    struct VCCS
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"VCCS"};

        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::linear};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"VCCS"};

        double m_g{1.0};
        ::phy_engine::model::pin pins[4]{{{u8"S"}}, {{u8"T"}}, {{u8"P"}}, {{u8"Q"}}};
    };

    static_assert(::phy_engine::model::model<VCCS>);

    inline constexpr bool
        set_attribute_define(::phy_engine::model::model_reserve_type_t<VCCS>, VCCS& vccs, ::std::size_t n, ::phy_engine::model::variant vi) noexcept
    {
        switch(n)
        {
            case 0:
            {
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                vccs.m_g = vi.d;
                return true;
            }
            default:
            {
                return false;
            }
        }
        return false;
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<VCCS>);

    inline constexpr ::phy_engine::model::variant
        get_attribute_define(::phy_engine::model::model_reserve_type_t<VCCS>, VCCS const& vccs, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                return {.d{vccs.m_g}, .type{::phy_engine::model::variant_type::d}};
            }
            default:
            {
                return {};
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<VCCS>);

    inline constexpr ::fast_io::u8string_view
        get_attribute_name_define(::phy_engine::model::model_reserve_type_t<VCCS>, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                // m_kZimag
                return {u8"G"};
            }
            default:
            {
                return {};
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<VCCS>);

    inline constexpr bool iterate_dc_define(::phy_engine::model::model_reserve_type_t<VCCS>, VCCS const& vccs, ::phy_engine::MNA::MNA& mna) noexcept
    {
        auto const node_S{vccs.pins[0].nodes};
        auto const node_T{vccs.pins[1].nodes};
        auto const node_P{vccs.pins[2].nodes};
        auto const node_Q{vccs.pins[3].nodes};
        if(node_S && node_T && node_P && node_Q) [[likely]]
        {

            mna.G_ref(node_S->node_index, node_P->node_index) += vccs.m_g;
            mna.G_ref(node_S->node_index, node_Q->node_index) -= vccs.m_g;
            mna.G_ref(node_T->node_index, node_P->node_index) -= vccs.m_g;
            mna.G_ref(node_T->node_index, node_Q->node_index) += vccs.m_g;
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_dc<VCCS>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<VCCS>, VCCS& vccs) noexcept
    {
        return {vccs.pins, 4};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<VCCS>);

}  // namespace phy_engine::model
