#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    struct single_pole_switch
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"switch"};
        // inline static constexpr ::fast_io::u8string_view model_description{u8"Resistance"};

        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::linear};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"switch"};

        bool cut_through{};
        ::phy_engine::model::pin pins[2]{{{u8"A"}}, {{u8"B"}}};
    };

    static_assert(::phy_engine::model::model<single_pole_switch>);

    inline constexpr bool set_attribute_define(::phy_engine::model::model_reserve_type_t<single_pole_switch>,
                                               single_pole_switch& sps,
                                               ::std::size_t n,
                                               ::phy_engine::model::variant vi) noexcept
    {
        switch(n)
        {
            case 0:
            {
                // Set single_pole_switch value
                if(vi.type != ::phy_engine::model::variant_type::boolean) [[unlikely]] { return false; }
                sps.cut_through = vi.boolean;
                return true;
            }
            default:
            {
                return false;
            }
        }
        return false;
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<single_pole_switch>);

    inline constexpr ::phy_engine::model::variant
        get_attribute_define(::phy_engine::model::model_reserve_type_t<single_pole_switch>, single_pole_switch const& sps, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                // Get single_pole_switch value
                return {.boolean{sps.cut_through}, .type{::phy_engine::model::variant_type::boolean}};
            }
            default:
            {
                return {};
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<single_pole_switch>);

    inline constexpr ::fast_io::u8string_view
        get_attribute_name_define(::phy_engine::model::model_reserve_type_t<single_pole_switch>, single_pole_switch const& sps, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                // single_pole_switch
                return {u8"Cut Through"};
            }
            default:
            {
                return {};
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<single_pole_switch>);

    inline constexpr bool
        iterate_dc_define(::phy_engine::model::model_reserve_type_t<single_pole_switch>, single_pole_switch const& sps, ::phy_engine::MNA::MNA& mna) noexcept
    {
        auto const node_0{sps.pins[0].nodes};
        auto const node_1{sps.pins[1].nodes};
        if(node_0 && node_1) [[likely]]
        {

            constexpr auto G_min{::std::numeric_limits<double>::min()};
            constexpr auto G_max{::std::numeric_limits<double>::max()};
            auto const m_G{sps.cut_through ? G_max : G_min};

            mna.G_ref(node_0->node_index, node_0->node_index) = m_G;
            mna.G_ref(node_0->node_index, node_1->node_index) = -m_G;
            mna.G_ref(node_1->node_index, node_0->node_index) = -m_G;
            mna.G_ref(node_1->node_index, node_1->node_index) = m_G;

        }

        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_dc<single_pole_switch>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<single_pole_switch>,
                                                                            single_pole_switch& sps) noexcept
    {
        return {sps.pins, 2};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<single_pole_switch>);

}  // namespace phy_engine::model
