#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    struct VCVS
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"VCVS"};

        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::linear};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"VCVS"};

        double m_mu{1.0};
        ::phy_engine::model::pin pins[4]{{{u8"S"}}, {{u8"T"}}, {{u8"P"}}, {{u8"Q"}}};
        ::phy_engine::model::branch branchs{};
    };

    static_assert(::phy_engine::model::model<VCVS>);

    inline constexpr bool
        set_attribute_define(::phy_engine::model::model_reserve_type_t<VCVS>, VCVS& vcvs, ::std::size_t n, ::phy_engine::model::variant vi) noexcept
    {
        switch(n)
        {
            case 0:
            {
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                vcvs.m_mu = vi.d;
                return true;
            }
            default:
            {
                return false;
            }
        }
        return false;
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<VCVS>);

    inline constexpr ::phy_engine::model::variant
        get_attribute_define(::phy_engine::model::model_reserve_type_t<VCVS>, VCVS const& vcvs, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                return {.d{vcvs.m_mu}, .type{::phy_engine::model::variant_type::d}};
            }
            default:
            {
                return {};
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<VCVS>);

    inline constexpr ::fast_io::u8string_view
        get_attribute_name_define(::phy_engine::model::model_reserve_type_t<VCVS>, VCVS const& vcvs, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                // m_kZimag
                return {u8"Mu"};
            }
            default:
            {
                return {};
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<VCVS>);

    inline constexpr bool iterate_dc_define(::phy_engine::model::model_reserve_type_t<VCVS>, VCVS const& vcvs, ::phy_engine::MNA::MNA& mna) noexcept
    {

        auto const node_S{vcvs.pins[0].nodes};
        auto const node_T{vcvs.pins[1].nodes};
        auto const node_P{vcvs.pins[2].nodes};
        auto const node_Q{vcvs.pins[3].nodes};
        if(node_S && node_T && node_P && node_Q) [[likely]]
        {
            ::std::size_t k{vcvs.branchs.index};

            mna.B_ref(node_S->node_index, k) += 1.0;
            mna.B_ref(node_T->node_index, k) -= 1.0;

            mna.C_ref(k, node_S->node_index) += 1.0;
            mna.C_ref(k, node_T->node_index) -= 1.0;
            mna.C_ref(k, node_P->node_index) -= vcvs.m_mu;
            mna.C_ref(k, node_Q->node_index) += vcvs.m_mu;
            // mna.E_ref(k) += 0.0;
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_dc<VCVS>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<VCVS>, VCVS& vcvs) noexcept
    {
        return {vcvs.pins, 4};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<VCVS>);

    inline constexpr ::phy_engine::model::branch_view generate_branch_view_define(::phy_engine::model::model_reserve_type_t<VCVS>, VCVS& vcvs) noexcept
    {
        return {__builtin_addressof(vcvs.branchs), 1};
    }

    static_assert(::phy_engine::model::defines::can_generate_branch_view<VCVS>);

}  // namespace phy_engine::model
