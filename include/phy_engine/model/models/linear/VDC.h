#pragma once
#include <numbers>
#include <complex>
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    struct VDC
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"VDC"};

        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::linear};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"VDC"};

        double V{5.0};

        ::phy_engine::model::pin pins[2]{{{u8"+"}}, {{u8"-"}}};
        ::phy_engine::model::branch branchs{};
    };

    static_assert(::phy_engine::model::model<VDC>);

    inline constexpr bool
        set_attribute_define(::phy_engine::model::model_reserve_type_t<VDC>, VDC& vdc, ::std::size_t n, ::phy_engine::model::variant vi) noexcept
    {
        switch(n)
        {
            case 0:
            {
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                vdc.V = vi.d;
                return true;
            }
            default:
            {
                return false;
            }
        }
        return false;
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<VDC>);

    inline constexpr ::phy_engine::model::variant get_attribute_define(::phy_engine::model::model_reserve_type_t<VDC>, VDC const& vdc, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                return {.d{vdc.V}, .type{::phy_engine::model::variant_type::d}};
            }
            default:
            {
                return {};
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<VDC>);

    inline constexpr ::fast_io::u8string_view
        get_attribute_name_define(::phy_engine::model::model_reserve_type_t<VDC>, VDC const& vdc, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                return {u8"V"};
            }
            default:
            {
                return {};
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<VDC>);

    inline constexpr bool iterate_dc_define(::phy_engine::model::model_reserve_type_t<VDC>, VDC const& vdc, ::phy_engine::MNA::MNA& mna) noexcept
    {
        auto const node_A{vdc.pins[0].nodes};
        auto const node_B{vdc.pins[1].nodes};
        if(node_A && node_B) [[likely]]
        {
            auto const k{vdc.branchs.index};
            mna.B_ref(node_A->node_index, k) = 1.0;
            mna.B_ref(node_B->node_index, k) = -1.0;
            mna.C_ref(k, node_A->node_index) = 1.0;
            mna.C_ref(k, node_B->node_index) = -1.0;
            mna.E_ref(k) = vdc.V;
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_dc<VDC>);

    inline constexpr bool
        iterate_ac_define(::phy_engine::model::model_reserve_type_t<VDC>, VDC const& vdc, ::phy_engine::MNA::MNA& mna, [[maybe_unused]] double omega) noexcept
    {
        auto const node_A{vdc.pins[0].nodes};
        auto const node_B{vdc.pins[1].nodes};
        if(node_A && node_B) [[likely]]
        {
            auto const k{vdc.branchs.index};
            mna.B_ref(node_A->node_index, k) = 1.0;
            mna.B_ref(node_B->node_index, k) = -1.0;
            mna.C_ref(k, node_A->node_index) = 1.0;
            mna.C_ref(k, node_B->node_index) = -1.0;
            // mna.E_ref(k) += 0.0;
        }

        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_ac<VDC>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<VDC>, VDC& vdc) noexcept
    {
        return {vdc.pins, 2};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<VDC>);

    inline constexpr ::phy_engine::model::branch_view generate_branch_view_define(::phy_engine::model::model_reserve_type_t<VDC>, VDC& vdc) noexcept
    {
        return {__builtin_addressof(vdc.branchs), 1};
    }

    static_assert(::phy_engine::model::defines::can_generate_branch_view<VDC>);

}  // namespace phy_engine::model
