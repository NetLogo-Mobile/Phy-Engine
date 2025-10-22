#pragma once
#include <numbers>
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    struct triangle_gen
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"Triangle Wave Generator"};

        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::linear};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"TRIANGLE"};

        double Vh{5.0};
        double Vl{0.0};
        double freq{1e3};
        double phase{0.0}; // radians

        ::phy_engine::model::pin pins[2]{{{u8"+"}}, {{u8"-"}}};
        ::phy_engine::model::branch branches{};
    };

    static_assert(::phy_engine::model::model<triangle_gen>);

    inline constexpr bool set_attribute_define(::phy_engine::model::model_reserve_type_t<triangle_gen>, triangle_gen& g, ::std::size_t idx, ::phy_engine::model::variant vi) noexcept
    {
        switch(idx)
        {
            case 0: if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] return false; g.Vh = vi.d; return true;
            case 1: if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] return false; g.Vl = vi.d; return true;
            case 2: if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] return false; g.freq = vi.d; return true;
            case 3: if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] return false; g.phase = vi.d; return true;
            default: return false;
        }
        return false;
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<triangle_gen>);

    inline constexpr ::phy_engine::model::variant get_attribute_define(::phy_engine::model::model_reserve_type_t<triangle_gen>, triangle_gen const& g, ::std::size_t idx) noexcept
    {
        switch(idx)
        {
            case 0: return {.d{g.Vh}, .type{::phy_engine::model::variant_type::d}};
            case 1: return {.d{g.Vl}, .type{::phy_engine::model::variant_type::d}};
            case 2: return {.d{g.freq}, .type{::phy_engine::model::variant_type::d}};
            case 3: return {.d{g.phase}, .type{::phy_engine::model::variant_type::d}};
            default: return {};
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<triangle_gen>);

    inline constexpr ::fast_io::u8string_view get_attribute_name_define(::phy_engine::model::model_reserve_type_t<triangle_gen>, ::std::size_t idx) noexcept
    {
        switch(idx)
        {
            case 0: return u8"Vh";
            case 1: return u8"Vl";
            case 2: return u8"freq";
            case 3: return u8"phase";
            default: return {};
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<triangle_gen>);

    inline constexpr bool iterate_tr_define(::phy_engine::model::model_reserve_type_t<triangle_gen>, triangle_gen const& g, ::phy_engine::MNA::MNA& mna, double tTime) noexcept
    {
        auto const node_P{g.pins[0].nodes};
        auto const node_M{g.pins[1].nodes};
        if(node_P && node_M) [[likely]]
        {
            double const T{1.0 / g.freq};
            double const t0{tTime + g.phase/(2.0*::std::numbers::pi)/g.freq};
            double const t{::std::fmod(t0, T)};
            double const amp{g.Vh - g.Vl};
            double val{};
            if(t < 0.5 * T)
            {
                val = g.Vl + (2.0 * amp / T) * t;
            }
            else
            {
                val = g.Vh - (2.0 * amp / T) * (t - 0.5 * T);
            }
            auto const k{g.branches.index};
            mna.B_ref(node_P->node_index, k) = 1.0;
            mna.B_ref(node_M->node_index, k) = -1.0;
            mna.C_ref(k, node_P->node_index) = 1.0;
            mna.C_ref(k, node_M->node_index) = -1.0;
            mna.E_ref(k) = val;
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_tr<triangle_gen>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<triangle_gen>, triangle_gen& g) noexcept
    {
        return {g.pins, 2};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<triangle_gen>);

    inline constexpr ::phy_engine::model::branch_view generate_branch_view_define(::phy_engine::model::model_reserve_type_t<triangle_gen>, triangle_gen& g) noexcept
    {
        return {__builtin_addressof(g.branches), 1};
    }

    static_assert(::phy_engine::model::defines::can_generate_branch_view<triangle_gen>);
}
