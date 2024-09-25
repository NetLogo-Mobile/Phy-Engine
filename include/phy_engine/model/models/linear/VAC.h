#pragma once
#include <numbers>
#include <complex>
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    struct VAC
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"VAC"};

        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::linear};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"VAC"};

        double m_Vp{5.0};
        double m_omega{50.0};
        double m_phase{0.0};

        ::phy_engine::model::pin pins[2]{{{u8"A"}}, {{u8"B"}}};
        ::phy_engine::model::branch branches{};

        // private:
        ::std::complex<double> m_E{};
    };

    static_assert(::phy_engine::model::model<VAC>);

    inline constexpr bool
        set_attribute_define(::phy_engine::model::model_reserve_type_t<VAC>, VAC& vac, ::std::size_t n, ::phy_engine::model::variant vi) noexcept
    {
        switch(n)
        {
            case 0:
            {
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                vac.m_Vp = vi.d;
                return true;
            }
            case 1:
            {
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                vac.m_omega = vi.d * (2.0 * ::std::numbers::pi);
                return true;
            }
            case 2:
            {
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                vac.m_phase = vi.d * (::std::numbers::pi / 180.0);
                return true;
            }
            default:
            {
                return false;
            }
        }
        return false;
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<VAC>);

    inline constexpr ::phy_engine::model::variant get_attribute_define(::phy_engine::model::model_reserve_type_t<VAC>, VAC const& vac, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                return {.d{vac.m_Vp}, .type{::phy_engine::model::variant_type::d}};
            }
            case 1:
            {
                return {.d{vac.m_omega / (2.0 * ::std::numbers::pi)}, .type{::phy_engine::model::variant_type::d}};
            }
            case 2:
            {
                return {.d{vac.m_phase / (::std::numbers::pi / 180.0)}, .type{::phy_engine::model::variant_type::d}};
            }
            default:
            {
                return {};
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<VAC>);

    inline constexpr ::fast_io::u8string_view
        get_attribute_name_define(::phy_engine::model::model_reserve_type_t<VAC>, VAC const& vac, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                return {u8"Vp"};
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

    static_assert(::phy_engine::model::defines::has_get_attribute_name<VAC>);

    inline constexpr bool prepare_dc_define(::phy_engine::model::model_reserve_type_t<VAC>, VAC& vac) noexcept
    {
        vac.m_E.real(vac.m_Vp * ::std::cos(vac.m_phase));
        vac.m_E.imag(vac.m_Vp * ::std::sin(vac.m_phase));

        return true;
    }

    static_assert(::phy_engine::model::defines::can_prepare_dc<VAC>);

    inline constexpr bool iterate_dc_define(::phy_engine::model::model_reserve_type_t<VAC>, VAC const& vac, ::phy_engine::MNA::MNA& mna) noexcept
    {
        auto const node_A{vac.pins[0].nodes};
        auto const node_B{vac.pins[1].nodes};
        if(node_A && node_B) [[likely]]
        {
            auto const k{vac.branches.index};
            mna.B_ref(node_A->node_index, k) = 1.0;
            mna.B_ref(node_B->node_index, k) = -1.0;
            mna.C_ref(k, node_A->node_index) = 1.0;
            mna.C_ref(k, node_B->node_index) = -1.0;
            // mna.E_ref(k) += 0.0;
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_dc<VAC>);

    inline constexpr bool
        iterate_ac_define(::phy_engine::model::model_reserve_type_t<VAC>, VAC const& vac, ::phy_engine::MNA::MNA& mna, [[maybe_unused]] double omega) noexcept
    {
        auto const node_A{vac.pins[0].nodes};
        auto const node_B{vac.pins[1].nodes};
        if(node_A && node_B) [[likely]]
        {
            auto const k{vac.branches.index};
            mna.B_ref(node_A->node_index, k) = 1.0;
            mna.B_ref(node_B->node_index, k) = -1.0;
            mna.C_ref(k, node_A->node_index) = 1.0;
            mna.C_ref(k, node_B->node_index) = -1.0;
            mna.E_ref(k) = vac.m_E;
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_ac<VAC>);

    inline constexpr bool iterate_tr_define(::phy_engine::model::model_reserve_type_t<VAC>,
                                            VAC const& vac,
                                            ::phy_engine::MNA::MNA& mna,
                                            [[maybe_unused]] double t_time) noexcept
    {
        auto const node_A{vac.pins[0].nodes};
        auto const node_B{vac.pins[1].nodes};
        if(node_A && node_B) [[likely]]
        {
            auto const k{vac.branches.index};
            mna.B_ref(node_A->node_index, k) = 1.0;
            mna.B_ref(node_B->node_index, k) = -1.0;
            mna.C_ref(k, node_A->node_index) = 1.0;
            mna.C_ref(k, node_B->node_index) = -1.0;
            mna.E_ref(k) = vac.m_Vp * ::std::sin(vac.m_omega * t_time + vac.m_phase);
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_tr<VAC>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<VAC>, VAC& vac) noexcept
    {
        return {vac.pins, 2};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<VAC>);

    inline constexpr ::phy_engine::model::branch_view generate_branch_view_define(::phy_engine::model::model_reserve_type_t<VAC>, VAC& vac) noexcept
    {
        return {__builtin_addressof(vac.branches), 1};
    }

    static_assert(::phy_engine::model::defines::can_generate_branch_view<VAC>);

}  // namespace phy_engine::model
