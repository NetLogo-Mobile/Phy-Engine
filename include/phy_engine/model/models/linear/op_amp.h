#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    struct op_amp
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"OpAmp"};

        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::linear};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"OPAMP"};

        double mu{1.0e5};
        ::phy_engine::model::pin pins[4]{{{u8"+"}}, {{u8"-"}}, {{u8"OUT+"}}, {{u8"OUT-"}}};
        ::phy_engine::model::branch branches{};
    };

    static_assert(::phy_engine::model::model<op_amp>);

    inline constexpr bool set_attribute_define(::phy_engine::model::model_reserve_type_t<op_amp>, op_amp& oa, ::std::size_t idx, ::phy_engine::model::variant vi) noexcept
    {
        switch(idx)
        {
            case 0:
            {
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                oa.mu = vi.d;
                return true;
            }
            default: return false;
        }
        return false;
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<op_amp>);

    inline constexpr ::phy_engine::model::variant get_attribute_define(::phy_engine::model::model_reserve_type_t<op_amp>, op_amp const& oa, ::std::size_t idx) noexcept
    {
        switch(idx)
        {
            case 0: return {.d{oa.mu}, .type{::phy_engine::model::variant_type::d}};
            default: return {};
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<op_amp>);

    inline constexpr ::fast_io::u8string_view get_attribute_name_define(::phy_engine::model::model_reserve_type_t<op_amp>, ::std::size_t idx) noexcept
    {
        switch(idx)
        {
            case 0: return u8"mu";
            default: return {};
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<op_amp>);

    inline constexpr bool iterate_dc_define(::phy_engine::model::model_reserve_type_t<op_amp>, op_amp const& oa, ::phy_engine::MNA::MNA& mna) noexcept
    {
        auto const node_S{oa.pins[0].nodes};
        auto const node_T{oa.pins[1].nodes};
        auto const node_P{oa.pins[2].nodes};
        auto const node_Q{oa.pins[3].nodes};
        if(node_S && node_T && node_P && node_Q) [[likely]]
        {
            ::std::size_t k{oa.branches.index};

            mna.B_ref(node_P->node_index, k) = 1.0;
            mna.B_ref(node_Q->node_index, k) = -1.0;

            mna.C_ref(k, node_P->node_index) = 1.0;
            mna.C_ref(k, node_Q->node_index) = -1.0;
            mna.C_ref(k, node_S->node_index) -= oa.mu;
            mna.C_ref(k, node_T->node_index) += oa.mu;
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_dc<op_amp>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<op_amp>, op_amp& oa) noexcept
    {
        return {oa.pins, 4};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<op_amp>);

    inline constexpr ::phy_engine::model::branch_view generate_branch_view_define(::phy_engine::model::model_reserve_type_t<op_amp>, op_amp& oa) noexcept
    {
        return {__builtin_addressof(oa.branches), 1};
    }

    static_assert(::phy_engine::model::defines::can_generate_branch_view<op_amp>);
}


