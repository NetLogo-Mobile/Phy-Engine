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
        ::phy_engine::model::branch branchs{};
    };

    static_assert(::phy_engine::model::model<CCCS>);

    inline constexpr bool
        set_attribute_define(::phy_engine::model::model_reserve_type_t<CCCS>, CCCS& cccs, ::std::size_t n, ::phy_engine::model::variant vi) noexcept
    {
        switch(n)
        {
            case 0:
            {
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

    static_assert(::phy_engine::model::defines::has_set_attribute<CCCS>);

    inline constexpr ::phy_engine::model::variant
        get_attribute_define(::phy_engine::model::model_reserve_type_t<CCCS>, CCCS const& cccs, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                return {.d{cccs.m_alpha}, .type{::phy_engine::model::variant_type::d}};
            }
            default:
            {
                return {};
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<CCCS>);

    inline constexpr ::fast_io::u8string_view
        get_attribute_name_define(::phy_engine::model::model_reserve_type_t<CCCS>, CCCS const& cccs, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                // m_kZimag
                return {u8"alpha"};
            }
            default:
            {
                return {};
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<CCCS>);

    inline constexpr bool iterate_dc_define(::phy_engine::model::model_reserve_type_t<CCCS>, CCCS const& cccs, ::phy_engine::MNA::MNA& mna) noexcept
    {
        auto const node_S{cccs.pins[0].nodes};
        auto const node_T{cccs.pins[1].nodes};
        auto const node_P{cccs.pins[2].nodes};
        auto const node_Q{cccs.pins[3].nodes};
        if(node_S && node_T && node_P && node_Q) [[likely]]
        {
            ::std::size_t c{cccs.branchs.index};
            mna.B_ref(node_S->node_index, c) = cccs.m_alpha;
            mna.B_ref(node_T->node_index, c) = -cccs.m_alpha;
            mna.B_ref(node_P->node_index, c) = 1.0;
            mna.B_ref(node_Q->node_index, c) = -1.0;
            mna.C_ref(c, node_P->node_index) = 1.0;
            mna.C_ref(c, node_Q->node_index) = -1.0;
            // mna.D_ref(c, c) = 0.0;
            // mna.E_ref(c) = 0.0;
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_dc<CCCS>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<CCCS>, CCCS& cccs) noexcept
    {
        return {cccs.pins, 4};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<CCCS>);

    inline constexpr ::phy_engine::model::branch_view generate_branch_view_define(::phy_engine::model::model_reserve_type_t<CCCS>, CCCS& cccs) noexcept
    {
        return {__builtin_addressof(cccs.branchs), 1};
    }

    static_assert(::phy_engine::model::defines::can_generate_branch_view<CCCS>);

}  // namespace phy_engine::model
