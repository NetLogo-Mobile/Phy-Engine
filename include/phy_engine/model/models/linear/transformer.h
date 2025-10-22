#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    struct transformer
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"Transformer"};

        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::linear};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"TX"};

        // Voltage ratio n = Vp / Vs
        double n{1.0};
        ::phy_engine::model::pin pins[4]{{{u8"P"}}, {{u8"Q"}}, {{u8"S"}}, {{u8"T"}}};  // Primary P-Q, Secondary S-T
        ::phy_engine::model::branch branches[2]{};                                     // kP for primary, kS for secondary
    };

    static_assert(::phy_engine::model::model<transformer>);

    inline constexpr bool set_attribute_define(::phy_engine::model::model_reserve_type_t<transformer>,
                                               transformer& tx,
                                               ::std::size_t nidx,
                                               ::phy_engine::model::variant vi) noexcept
    {
        switch(nidx)
        {
            case 0:
            {
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                tx.n = vi.d;
                return true;
            }
            default: return false;
        }
        return false;
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<transformer>);

    inline constexpr ::phy_engine::model::variant
        get_attribute_define(::phy_engine::model::model_reserve_type_t<transformer>, transformer const& tx, ::std::size_t nidx) noexcept
    {
        switch(nidx)
        {
            case 0: return {.d{tx.n}, .type{::phy_engine::model::variant_type::d}};
            default: return {};
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<transformer>);

    inline constexpr ::fast_io::u8string_view get_attribute_name_define(::phy_engine::model::model_reserve_type_t<transformer>, ::std::size_t nidx) noexcept
    {
        switch(nidx)
        {
            case 0: return u8"n";  // Vp/Vs
            default: return {};
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<transformer>);

    inline constexpr bool iterate_dc_define(::phy_engine::model::model_reserve_type_t<transformer>, transformer const& tx, ::phy_engine::MNA::MNA& mna) noexcept
    {
        auto const node_P{tx.pins[0].nodes};
        auto const node_Q{tx.pins[1].nodes};
        auto const node_S{tx.pins[2].nodes};
        auto const node_T{tx.pins[3].nodes};

        if(node_P && node_Q && node_S && node_T) [[likely]]
        {
            auto const kP{tx.branches[0].index};
            auto const kS{tx.branches[1].index};

            // Primary branch
            mna.B_ref(node_P->node_index, kP) = 1.0;
            mna.B_ref(node_Q->node_index, kP) = -1.0;
            mna.C_ref(kP, node_P->node_index) = 1.0;
            mna.C_ref(kP, node_Q->node_index) = -1.0;
            // Vp - n*Vs = 0
            mna.C_ref(kP, node_S->node_index) -= tx.n;
            mna.C_ref(kP, node_T->node_index) += tx.n;

            // Secondary branch
            mna.B_ref(node_S->node_index, kS) = 1.0;
            mna.B_ref(node_T->node_index, kS) = -1.0;
            mna.C_ref(kS, node_S->node_index) = 1.0;
            mna.C_ref(kS, node_T->node_index) = -1.0;
            // Vs - (1/n)*Vp = 0
            if(tx.n != 0.0)
            {
                double const invn{1.0 / tx.n};
                mna.C_ref(kS, node_P->node_index) -= invn;
                mna.C_ref(kS, node_Q->node_index) += invn;
            }
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_dc<transformer>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<transformer>, transformer& tx) noexcept
    {
        return {tx.pins, 4};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<transformer>);

    inline constexpr ::phy_engine::model::branch_view generate_branch_view_define(::phy_engine::model::model_reserve_type_t<transformer>,
                                                                                  transformer& tx) noexcept
    {
        return {tx.branches, 2};
    }

    static_assert(::phy_engine::model::defines::can_generate_branch_view<transformer>);
}  // namespace phy_engine::model

