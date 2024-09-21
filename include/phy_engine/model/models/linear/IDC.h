#pragma once
#include <numbers>
#include <complex>
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    struct IDC
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"IDC"};

        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::linear};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"IDC"};

        double I{0.2};

        ::phy_engine::model::pin pins[2]{{{u8"A"}}, {{u8"B"}}};

        // private:
        ::std::complex<double> m_I{};
    };

    static_assert(::phy_engine::model::model<IDC>);

    inline constexpr bool
        set_attribute_define(::phy_engine::model::model_reserve_type_t<IDC>, IDC& idc, ::std::size_t n, ::phy_engine::model::variant vi) noexcept
    {
        switch(n)
        {
            case 0:
            {
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                idc.I = vi.d;
                return true;
            }
            default:
            {
                return false;
            }
        }
        return false;
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<IDC>);

    inline constexpr ::phy_engine::model::variant get_attribute_define(::phy_engine::model::model_reserve_type_t<IDC>, IDC const& idc, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                return {.d{idc.I}, .type{::phy_engine::model::variant_type::d}};
            }
            default:
            {
                return {};
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<IDC>);

    inline constexpr ::fast_io::u8string_view
        get_attribute_name_define(::phy_engine::model::model_reserve_type_t<IDC>, IDC const& idc, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                return {u8"I"};
            }
            default:
            {
                return {};
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<IDC>);

    inline constexpr bool iterate_dc_define(::phy_engine::model::model_reserve_type_t<IDC>, IDC const& idc, ::phy_engine::MNA::MNA& mna) noexcept
    {
        auto const node_A{idc.pins[0].nodes};
        auto const node_B{idc.pins[1].nodes};
        if(node_A && node_B) [[likely]]
        {
            mna.I_ref(node_A->node_index) -= idc.I;
            mna.I_ref(node_B->node_index) += idc.I;
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_dc<IDC>);

    inline constexpr bool
        iterate_ac_define(::phy_engine::model::model_reserve_type_t<IDC>, IDC const& idc, ::phy_engine::MNA::MNA& mna, [[maybe_unused]] double omega) noexcept
    {
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_ac<IDC>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<IDC>, IDC& idc) noexcept
    {
        return {idc.pins, 2};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<IDC>);

}  // namespace phy_engine::model
