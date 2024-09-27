#pragma once
#include <numbers>
#include <complex>
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    struct IAC
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"IAC"};

        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::linear};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"IAC"};

        double m_Ip{0.2};
        double m_omega{50.0};
        double m_phase{0.0};

        ::phy_engine::model::pin pins[2]{{{u8"+"}}, {{u8"-"}}};

        // private:
        ::std::complex<double> m_I{};
    };

    static_assert(::phy_engine::model::model<IAC>);

    inline constexpr bool
        set_attribute_define(::phy_engine::model::model_reserve_type_t<IAC>, IAC& iac, ::std::size_t n, ::phy_engine::model::variant vi) noexcept
    {
        switch(n)
        {
            case 0:
            {
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                iac.m_Ip = vi.d;
                return true;
            }
            case 1:
            {
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                iac.m_omega = vi.d * (2.0 * ::std::numbers::pi);
                return true;
            }
            case 2:
            {
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                iac.m_phase = vi.d * (::std::numbers::pi / 180.0);
                return true;
            }
            default:
            {
                return false;
            }
        }
        return false;
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<IAC>);

    inline constexpr ::phy_engine::model::variant get_attribute_define(::phy_engine::model::model_reserve_type_t<IAC>, IAC const& iac, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                return {.d{iac.m_Ip}, .type{::phy_engine::model::variant_type::d}};
            }
            case 1:
            {
                return {.d{iac.m_omega / (2.0 * ::std::numbers::pi)}, .type{::phy_engine::model::variant_type::d}};
            }
            case 2:
            {
                return {.d{iac.m_phase / (::std::numbers::pi / 180.0)}, .type{::phy_engine::model::variant_type::d}};
            }
            default:
            {
                return {};
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<IAC>);

    inline constexpr ::fast_io::u8string_view
        get_attribute_name_define(::phy_engine::model::model_reserve_type_t<IAC>, IAC const& iac, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                return {u8"Ip"};
            }
            case 1:
            {
                return {u8"freq"};
            }
            case 2:
            {
                return {u8"phase"};
            }
            default:
            {
                return {};
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<IAC>);

    inline constexpr bool prepare_foundation_define(::phy_engine::model::model_reserve_type_t<IAC>, IAC& iac) noexcept
    {
        iac.m_I.real(iac.m_Ip * ::std::cos(iac.m_phase));
        iac.m_I.imag(iac.m_Ip * ::std::sin(iac.m_phase));

        return true;
    }

    static_assert(::phy_engine::model::defines::can_prepare_foundation<IAC>);

    inline constexpr bool iterate_dc_define(::phy_engine::model::model_reserve_type_t<IAC>, IAC const& iac, ::phy_engine::MNA::MNA& mna) noexcept
    {
        auto const node_A{iac.pins[0].nodes};
        auto const node_B{iac.pins[1].nodes};
        if(node_A && node_B) [[likely]]
        {
            mna.I_ref(node_A->node_index) -= iac.m_Ip;
            mna.I_ref(node_B->node_index) += iac.m_Ip;
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_dc<IAC>);

    inline constexpr bool
        iterate_ac_define(::phy_engine::model::model_reserve_type_t<IAC>, IAC const& iac, ::phy_engine::MNA::MNA& mna, [[maybe_unused]] double omega) noexcept
    {
        auto const node_A{iac.pins[0].nodes};
        auto const node_B{iac.pins[1].nodes};
        if(node_A && node_B) [[likely]]
        {
            mna.I_ref(node_A->node_index) -= iac.m_I;
            mna.I_ref(node_B->node_index) += iac.m_I;
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_ac<IAC>);

    inline constexpr bool
        iterate_tr_define(::phy_engine::model::model_reserve_type_t<IAC>, IAC const& iac, ::phy_engine::MNA::MNA& mna, [[maybe_unused]] double t_time) noexcept
    {
        auto const node_A{iac.pins[0].nodes};
        auto const node_B{iac.pins[1].nodes};
        if(node_A && node_B) [[likely]]
        {
            double I{iac.m_Ip * ::std::sin(iac.m_omega * t_time + iac.m_phase)};

            mna.I_ref(node_A->node_index) -= I;
            mna.I_ref(node_B->node_index) += I;
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_tr<IAC>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<IAC>, IAC& iac) noexcept
    {
        return {iac.pins, 2};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<IAC>);

}  // namespace phy_engine::model
