#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    struct BJT_NPN
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"NPN BJT"};

        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::non_linear};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"QNP"};

        // Parameters
        double Is{1e-16};
        double N{1.0};
        double BetaF{100.0};
        double Temp{27.0};
        double Area{1.0};

        // Pins: B, C, E
        ::phy_engine::model::pin pins[3]{{{u8"B"}}, {{u8"C"}}, {{u8"E"}}};

        // Private state
        double Ut{};  // thermal voltage
        double Vbe_last{};
        double geq_be{};  // diode conductance between B and E
        double Ieq_be{};  // diode current source equivalent
        double gm{};      // transconductance Ic ≈ gm * Vbe
        double Ieq_c{};   // collector current source equivalent
    };

    static_assert(::phy_engine::model::model<BJT_NPN>);

    inline constexpr bool
        set_attribute_define(::phy_engine::model::model_reserve_type_t<BJT_NPN>, BJT_NPN& q, ::std::size_t n, ::phy_engine::model::variant vi) noexcept
    {
        switch(n)
        {
            case 0:
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                q.Is = vi.d;
                return true;
            case 1:
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                q.N = vi.d;
                return true;
            case 2:
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                q.BetaF = vi.d;
                return true;
            case 3:
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                q.Temp = vi.d;
                return true;
            case 4:
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                q.Area = vi.d;
                return true;
            default: return false;
        }
        return false;
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<BJT_NPN>);

    inline constexpr ::phy_engine::model::variant
        get_attribute_define(::phy_engine::model::model_reserve_type_t<BJT_NPN>, BJT_NPN const& q, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0: return {.d{q.Is}, .type{::phy_engine::model::variant_type::d}};
            case 1: return {.d{q.N}, .type{::phy_engine::model::variant_type::d}};
            case 2: return {.d{q.BetaF}, .type{::phy_engine::model::variant_type::d}};
            case 3: return {.d{q.Temp}, .type{::phy_engine::model::variant_type::d}};
            case 4: return {.d{q.Area}, .type{::phy_engine::model::variant_type::d}};
            default: return {};
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<BJT_NPN>);

    inline constexpr ::fast_io::u8string_view get_attribute_name_define(::phy_engine::model::model_reserve_type_t<BJT_NPN>, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0: return u8"Is";
            case 1: return u8"N";
            case 2: return u8"BetaF";
            case 3: return u8"Temp";
            case 4: return u8"Area";
            default: return {};
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<BJT_NPN>);

    inline bool prepare_foundation_define(::phy_engine::model::model_reserve_type_t<BJT_NPN>, BJT_NPN& q) noexcept
    {
        constexpr double kKelvin{-273.15};
        constexpr double qElement{1.6021765314e-19};
        constexpr double kBoltzmann{1.380650524e-23};
        q.Ut = kBoltzmann * (q.Temp - kKelvin) / qElement;

        auto const node_B{q.pins[0].nodes};
        auto const node_C{q.pins[1].nodes};
        auto const node_E{q.pins[2].nodes};
        if(node_B && node_C && node_E) [[likely]] { q.Vbe_last = node_B->node_information.an.voltage.real() - node_E->node_information.an.voltage.real(); }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_prepare_foundation<BJT_NPN>);

    inline constexpr bool iterate_dc_define(::phy_engine::model::model_reserve_type_t<BJT_NPN>, BJT_NPN& q, ::phy_engine::MNA::MNA& mna) noexcept
    {
        auto const node_B{q.pins[0].nodes};
        auto const node_C{q.pins[1].nodes};
        auto const node_E{q.pins[2].nodes};
        if(node_B && node_C && node_E) [[likely]]
        {
            double const Is_eff{q.Is * q.Area};
            double const Vbe{node_B->node_information.an.voltage.real() - node_E->node_information.an.voltage.real()};
            q.Vbe_last = Vbe;

            double const Ute{q.N * q.Ut};
            double e{::std::exp(Vbe / Ute)};

            // Base-emitter diode linearization
            q.geq_be = Is_eff * e / Ute;
            double const Ibe{Is_eff * (e - 1.0)};
            q.Ieq_be = Ibe - Vbe * q.geq_be;

            // Stamp BE diode between B and E
            mna.G_ref(node_B->node_index, node_B->node_index) += q.geq_be;
            mna.G_ref(node_B->node_index, node_E->node_index) -= q.geq_be;
            mna.G_ref(node_E->node_index, node_B->node_index) -= q.geq_be;
            mna.G_ref(node_E->node_index, node_E->node_index) += q.geq_be;
            mna.I_ref(node_B->node_index) -= q.Ieq_be;
            mna.I_ref(node_E->node_index) += q.Ieq_be;

            // Collector current linearization Ic ≈ gm*Vbe + Ieq_c
            q.gm = q.BetaF * q.geq_be;
            double const Ic{q.BetaF * Ibe};
            q.Ieq_c = Ic - q.gm * Vbe;

            // Stamp VCCS from C to E controlled by Vb - Ve with transconductance gm
            mna.G_ref(node_C->node_index, node_B->node_index) += q.gm;
            mna.G_ref(node_C->node_index, node_E->node_index) -= q.gm;
            mna.G_ref(node_E->node_index, node_B->node_index) -= q.gm;
            mna.G_ref(node_E->node_index, node_E->node_index) += q.gm;

            // Stamp collector current source equivalent oriented C -> E
            mna.I_ref(node_C->node_index) -= q.Ieq_c;
            mna.I_ref(node_E->node_index) += q.Ieq_c;
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_dc<BJT_NPN>);

    inline constexpr bool
        iterate_ac_define(::phy_engine::model::model_reserve_type_t<BJT_NPN>, BJT_NPN& q, ::phy_engine::MNA::MNA& mna, [[maybe_unused]] double omega) noexcept
    {
        auto const node_B{q.pins[0].nodes};
        auto const node_C{q.pins[1].nodes};
        auto const node_E{q.pins[2].nodes};
        if(node_B && node_C && node_E) [[likely]]
        {
            // Small-signal: diode conductance and gm only
            mna.G_ref(node_B->node_index, node_B->node_index) += q.geq_be;
            mna.G_ref(node_B->node_index, node_E->node_index) -= q.geq_be;
            mna.G_ref(node_E->node_index, node_B->node_index) -= q.geq_be;
            mna.G_ref(node_E->node_index, node_E->node_index) += q.geq_be;

            mna.G_ref(node_C->node_index, node_B->node_index) += q.gm;
            mna.G_ref(node_C->node_index, node_E->node_index) -= q.gm;
            mna.G_ref(node_E->node_index, node_B->node_index) -= q.gm;
            mna.G_ref(node_E->node_index, node_E->node_index) += q.gm;
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_ac<BJT_NPN>);

    inline constexpr bool step_changed_tr_define(::phy_engine::model::model_reserve_type_t<BJT_NPN>,
                                                 BJT_NPN& q,
                                                 [[maybe_unused]] double nlaststep,
                                                 [[maybe_unused]] double nstep) noexcept
    {
        auto const node_B{q.pins[0].nodes};
        auto const node_E{q.pins[2].nodes};
        if(node_B && node_E) [[likely]] { q.Vbe_last = node_B->node_information.an.voltage.real() - node_E->node_information.an.voltage.real(); }
        return true;
    }

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<BJT_NPN>, BJT_NPN& q) noexcept
    {
        return {q.pins, 3};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<BJT_NPN>);
}  // namespace phy_engine::model
