#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    struct relay
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"Relay"};

        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::linear};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"RELAY"};

        // Control coil: C+ and C-; Contact: A and B
        ::phy_engine::model::pin pins[4]{{{u8"C+"}}, {{u8"C-"}}, {{u8"A"}}, {{u8"B"}}};
        ::phy_engine::model::branch branches{};

        double Von{5.0};
        double Voff{3.0};
        bool engaged{};
    };

    static_assert(::phy_engine::model::model<relay>);

    inline constexpr bool
        set_attribute_define(::phy_engine::model::model_reserve_type_t<relay>, relay& r, ::std::size_t idx, ::phy_engine::model::variant vi) noexcept
    {
        switch(idx)
        {
            case 0:
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                r.Von = vi.d;
                return true;
            case 1:
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                r.Voff = vi.d;
                return true;
            default: return false;
        }
        return false;
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<relay>);

    inline constexpr ::phy_engine::model::variant
        get_attribute_define(::phy_engine::model::model_reserve_type_t<relay>, relay const& r, ::std::size_t idx) noexcept
    {
        switch(idx)
        {
            case 0: return {.d{r.Von}, .type{::phy_engine::model::variant_type::d}};
            case 1: return {.d{r.Voff}, .type{::phy_engine::model::variant_type::d}};
            case 2: return {.boolean{r.engaged}, .type{::phy_engine::model::variant_type::boolean}};
            default: return {};
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<relay>);

    inline constexpr ::fast_io::u8string_view get_attribute_name_define(::phy_engine::model::model_reserve_type_t<relay>, ::std::size_t idx) noexcept
    {
        switch(idx)
        {
            case 0: return u8"Von";
            case 1: return u8"Voff";
            case 2: return u8"Engaged";
            default: return {};
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<relay>);

    inline constexpr bool iterate_dc_define(::phy_engine::model::model_reserve_type_t<relay>, relay& r, ::phy_engine::MNA::MNA& mna) noexcept
    {
        auto const node_Cp{r.pins[0].nodes};
        auto const node_Cn{r.pins[1].nodes};
        auto const node_A{r.pins[2].nodes};
        auto const node_B{r.pins[3].nodes};

        if(node_Cp && node_Cn && node_A && node_B) [[likely]]
        {
            double const vctrl{node_Cp->node_information.an.voltage.real() - node_Cn->node_information.an.voltage.real()};
            if(!r.engaged)
            {
                if(vctrl >= r.Von) { r.engaged = true; }
            }
            else
            {
                if(vctrl <= r.Voff) { r.engaged = false; }
            }

            // Always stamp a (very) large resistance when open, to keep MNA well-conditioned.
            double const r_contact{r.engaged ? 0.0 : mna.r_open};

            auto const k{r.branches.index};
            mna.B_ref(node_A->node_index, k) = 1.0;
            mna.B_ref(node_B->node_index, k) = -1.0;
            mna.C_ref(k, node_A->node_index) = 1.0;
            mna.C_ref(k, node_B->node_index) = -1.0;
            mna.D_ref(k, k) = -r_contact;
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_dc<relay>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<relay>, relay& r) noexcept
    {
        return {r.pins, 4};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<relay>);

    inline constexpr ::phy_engine::model::branch_view generate_branch_view_define(::phy_engine::model::model_reserve_type_t<relay>, relay& r) noexcept
    {
        return {__builtin_addressof(r.branches), 1};
    }

    static_assert(::phy_engine::model::defines::can_generate_branch_view<relay>);
}  // namespace phy_engine::model
