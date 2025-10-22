#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    struct transformer_center_tap
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"Transformer Center Tap"};

        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::linear};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"TXCT"};

        // Voltage ratio across total secondary: n_total = Vp / Vst (S1-S2)
        double n_total{1.0};
        double n_half{};  // computed as 2*n_total

        // Primary P-Q; Secondary split S1-CT and CT-S2
        ::phy_engine::model::pin pins[5]{{{u8"P"}}, {{u8"Q"}}, {{u8"S1"}}, {{u8"CT"}}, {{u8"S2"}}};
        ::phy_engine::model::branch branches[3]{};  // kP, kH1, kH2
    };

    static_assert(::phy_engine::model::model<transformer_center_tap>);

    inline constexpr bool set_attribute_define(::phy_engine::model::model_reserve_type_t<transformer_center_tap>,
                                               transformer_center_tap& tx,
                                               ::std::size_t idx,
                                               ::phy_engine::model::variant vi) noexcept
    {
        switch(idx)
        {
            case 0:
            {
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                tx.n_total = vi.d;
                return true;
            }
            default: return false;
        }
        return false;
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<transformer_center_tap>);

    inline constexpr ::phy_engine::model::variant
        get_attribute_define(::phy_engine::model::model_reserve_type_t<transformer_center_tap>, transformer_center_tap const& tx, ::std::size_t idx) noexcept
    {
        switch(idx)
        {
            case 0: return {.d{tx.n_total}, .type{::phy_engine::model::variant_type::d}};
            default: return {};
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<transformer_center_tap>);

    inline constexpr ::fast_io::u8string_view get_attribute_name_define(::phy_engine::model::model_reserve_type_t<transformer_center_tap>,
                                                                        ::std::size_t idx) noexcept
    {
        switch(idx)
        {
            case 0: return u8"n_total";
            default: return {};
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<transformer_center_tap>);

    inline bool prepare_foundation_define(::phy_engine::model::model_reserve_type_t<transformer_center_tap>, transformer_center_tap& tx) noexcept
    {
        tx.n_half = 2.0 * tx.n_total;
        return true;
    }

    static_assert(::phy_engine::model::defines::can_prepare_foundation<transformer_center_tap>);

    inline constexpr bool iterate_dc_define(::phy_engine::model::model_reserve_type_t<transformer_center_tap>,
                                            transformer_center_tap const& tx,
                                            ::phy_engine::MNA::MNA& mna) noexcept
    {
        auto const node_P{tx.pins[0].nodes};
        auto const node_Q{tx.pins[1].nodes};
        auto const node_S1{tx.pins[2].nodes};
        auto const node_CT{tx.pins[3].nodes};
        auto const node_S2{tx.pins[4].nodes};

        if(node_P && node_Q && node_S1 && node_CT && node_S2) [[likely]]
        {
            auto const kP{tx.branches[0].index};
            auto const kH1{tx.branches[1].index};
            auto const kH2{tx.branches[2].index};

            // Branch connections (for KCL)
            mna.B_ref(node_P->node_index, kP) = 1.0;
            mna.B_ref(node_Q->node_index, kP) = -1.0;
            mna.B_ref(node_S1->node_index, kH1) = 1.0;
            mna.B_ref(node_CT->node_index, kH1) = -1.0;
            mna.B_ref(node_CT->node_index, kH2) = 1.0;
            mna.B_ref(node_S2->node_index, kH2) = -1.0;

            // KVL for half 1: V(S1-CT) - (1/n_half)*V(P-Q) = 0
            mna.C_ref(kH1, node_S1->node_index) = 1.0;
            mna.C_ref(kH1, node_CT->node_index) = -1.0;
            if(tx.n_half != 0.0)
            {
                double const invnh{1.0 / tx.n_half};
                mna.C_ref(kH1, node_P->node_index) -= invnh;
                mna.C_ref(kH1, node_Q->node_index) += invnh;
            }

            // KVL for half 2: V(CT-S2) - (1/n_half)*V(P-Q) = 0
            mna.C_ref(kH2, node_CT->node_index) = 1.0;
            mna.C_ref(kH2, node_S2->node_index) = -1.0;
            if(tx.n_half != 0.0)
            {
                double const invnh{1.0 / tx.n_half};
                mna.C_ref(kH2, node_P->node_index) -= invnh;
                mna.C_ref(kH2, node_Q->node_index) += invnh;
            }

            // Ampere-turns current relation: I_p + (1/n_half)*I_h1 + (1/n_half)*I_h2 = 0
            if(tx.n_half != 0.0)
            {
                double const invnh{1.0 / tx.n_half};
                mna.D_ref(kP, kP) = 1.0;
                mna.D_ref(kP, kH1) = invnh;
                mna.D_ref(kP, kH2) = invnh;
            }
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_dc<transformer_center_tap>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<transformer_center_tap>,
                                                                            transformer_center_tap& tx) noexcept
    {
        return {tx.pins, 5};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<transformer_center_tap>);

    inline constexpr ::phy_engine::model::branch_view generate_branch_view_define(::phy_engine::model::model_reserve_type_t<transformer_center_tap>,
                                                                                  transformer_center_tap& tx) noexcept
    {
        return {tx.branches, 3};
    }

    static_assert(::phy_engine::model::defines::can_generate_branch_view<transformer_center_tap>);
}  // namespace phy_engine::model

