#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    struct CCVS
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"CCVS"};

        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::linear};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"CCVS"};

        double m_r{10.0};
        ::phy_engine::model::pin pins[4]{{{u8"S"}}, {{u8"T"}}, {{u8"P"}}, {{u8"Q"}}};
        ::phy_engine::model::branch branches[2]{};
    };

    static_assert(::phy_engine::model::model<CCVS>);

    inline constexpr bool
        set_attribute_define(::phy_engine::model::model_reserve_type_t<CCVS>, CCVS& ccvs, ::std::size_t n, ::phy_engine::model::variant vi) noexcept
    {
        switch(n)
        {
            case 0:
            {
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                ccvs.m_r = vi.d;
                return true;
            }
            default:
            {
                return false;
            }
        }
        return false;
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<CCVS>);

    inline constexpr ::phy_engine::model::variant
        get_attribute_define(::phy_engine::model::model_reserve_type_t<CCVS>, CCVS const& ccvs, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                return {.d{ccvs.m_r}, .type{::phy_engine::model::variant_type::d}};
            }
            default:
            {
                return {};
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<CCVS>);

    inline constexpr ::fast_io::u8string_view
        get_attribute_name_define(::phy_engine::model::model_reserve_type_t<CCVS>, CCVS const& ccvs, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                // m_kZimag
                return {u8"r"};
            }
            default:
            {
                return {};
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<CCVS>);

    inline constexpr bool iterate_dc_define(::phy_engine::model::model_reserve_type_t<CCVS>, CCVS const& ccvs, ::phy_engine::MNA::MNA& mna) noexcept
    {

        auto const node_S{ccvs.pins[0].nodes};
        auto const node_T{ccvs.pins[1].nodes};
        auto const node_P{ccvs.pins[2].nodes};
        auto const node_Q{ccvs.pins[3].nodes};
        if(node_S && node_T && node_P && node_Q) [[likely]]
        {
            ::std::size_t k{ccvs.branches[0].index};
            ::std::size_t c{ccvs.branches[1].index};

            mna.B_ref(node_S->node_index, k) = 1.0;
            mna.B_ref(node_T->node_index, k) = -1.0;
            mna.B_ref(node_P->node_index, c) = 1.0;
            mna.B_ref(node_Q->node_index, c) = -1.0;
            mna.C_ref(k, node_S->node_index) = 1.0;
            mna.C_ref(k, node_T->node_index) = -1.0;
            mna.C_ref(c, node_P->node_index) = 1.0;
            mna.C_ref(c, node_Q->node_index) = -1.0;
            mna.D_ref(k, c) = -ccvs.m_r;
            // mna.D_ref(c, c) += 0.0;
            // mna.E_ref(k) += 0.0;
            // mna.E_ref(c) += 0.0;
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_dc<CCVS>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<CCVS>, CCVS& ccvs) noexcept
    {
        return {ccvs.pins, 4};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<CCVS>);

    inline constexpr ::phy_engine::model::branch_view generate_branch_view_define(::phy_engine::model::model_reserve_type_t<CCVS>, CCVS& ccvs) noexcept
    {
        return {ccvs.branches, 2};
    }

    static_assert(::phy_engine::model::defines::can_generate_branch_view<CCVS>);

}  // namespace phy_engine::model
