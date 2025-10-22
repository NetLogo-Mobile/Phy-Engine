#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    struct BJT_PNP
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"PNP BJT"};

        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::non_linear};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"QPN"};

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
        double Veb_last{};
        double geq_eb{};  // diode conductance between E and B (emitter-base, opposite polarity of NPN)
        double Ieq_eb{};  // diode current source equivalent
        double gm{};      // transconductance Ic ≈ gm * Veb (PNP sign handled in stamps)
        double Ieq_c{};   // collector current source equivalent
    };

    static_assert(::phy_engine::model::model<BJT_PNP>);

    inline constexpr bool
        set_attribute_define(::phy_engine::model::model_reserve_type_t<BJT_PNP>, BJT_PNP& q, ::std::size_t n, ::phy_engine::model::variant vi) noexcept
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

    static_assert(::phy_engine::model::defines::has_set_attribute<BJT_PNP>);

    inline constexpr ::phy_engine::model::variant
        get_attribute_define(::phy_engine::model::model_reserve_type_t<BJT_PNP>, BJT_PNP const& q, ::std::size_t n) noexcept
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

    static_assert(::phy_engine::model::defines::has_get_attribute<BJT_PNP>);

    inline constexpr ::fast_io::u8string_view get_attribute_name_define(::phy_engine::model::model_reserve_type_t<BJT_PNP>, ::std::size_t n) noexcept
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

    static_assert(::phy_engine::model::defines::has_get_attribute_name<BJT_PNP>);

    inline bool prepare_foundation_define(::phy_engine::model::model_reserve_type_t<BJT_PNP>, BJT_PNP& q) noexcept
    {
        q.Is *= q.Area;
        constexpr double kKelvin{-273.15};
        constexpr double qElement{1.6021765314e-19};
        constexpr double kBoltzmann{1.380650524e-23};
        q.Ut = kBoltzmann * (q.Temp - kKelvin) / qElement;

        auto const node_B{q.pins[0].nodes};
        auto const node_C{q.pins[1].nodes};
        auto const node_E{q.pins[2].nodes};
        if(node_B && node_C && node_E) [[likely]] { q.Veb_last = node_E->node_information.an.voltage.real() - node_B->node_information.an.voltage.real(); }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_prepare_foundation<BJT_PNP>);

    inline constexpr bool iterate_dc_define(::phy_engine::model::model_reserve_type_t<BJT_PNP>, BJT_PNP& q, ::phy_engine::MNA::MNA& mna) noexcept
    {
        auto const node_B{q.pins[0].nodes};
        auto const node_C{q.pins[1].nodes};
        auto const node_E{q.pins[2].nodes};
        if(node_B && node_C && node_E) [[likely]]
        {
            double const Veb{node_E->node_information.an.voltage.real() - node_B->node_information.an.voltage.real()};
            q.Veb_last = Veb;

            double const Ute{q.N * q.Ut};
            double e{::std::exp(Veb / Ute)};

            // Emitter-base diode
            q.geq_eb = q.Is * e / Ute;
            double const Ieb{q.Is * (e - 1.0)};
            q.Ieq_eb = Ieb - Veb * q.geq_eb;

            // Stamp EB diode between E and B (current from E->B)
            mna.G_ref(node_E->node_index, node_E->node_index) += q.geq_eb;
            mna.G_ref(node_E->node_index, node_B->node_index) -= q.geq_eb;
            mna.G_ref(node_B->node_index, node_E->node_index) -= q.geq_eb;
            mna.G_ref(node_B->node_index, node_B->node_index) += q.geq_eb;
            mna.I_ref(node_E->node_index) -= q.Ieq_eb;
            mna.I_ref(node_B->node_index) += q.Ieq_eb;

            // Collector current (flows from E to C in PNP when forward active)
            q.gm = q.BetaF * q.geq_eb;
            double const Ic{q.BetaF * Ieb};
            q.Ieq_c = Ic - q.gm * Veb;

            // VCCS from E to C controlled by Veb with transconductance gm
            mna.G_ref(node_E->node_index, node_E->node_index) += q.gm;
            mna.G_ref(node_E->node_index, node_B->node_index) -= q.gm;
            mna.G_ref(node_C->node_index, node_E->node_index) -= q.gm;
            mna.G_ref(node_C->node_index, node_B->node_index) += q.gm;

            // Current source equivalent oriented E -> C
            mna.I_ref(node_E->node_index) -= q.Ieq_c;
            mna.I_ref(node_C->node_index) += q.Ieq_c;
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_dc<BJT_PNP>);

    inline constexpr bool
        iterate_ac_define(::phy_engine::model::model_reserve_type_t<BJT_PNP>, BJT_PNP& q, ::phy_engine::MNA::MNA& mna, [[maybe_unused]] double omega) noexcept
    {
        auto const node_B{q.pins[0].nodes};
        auto const node_C{q.pins[1].nodes};
        auto const node_E{q.pins[2].nodes};
        if(node_B && node_C && node_E) [[likely]]
        {
            // EB small-signal and gm only
            mna.G_ref(node_E->node_index, node_E->node_index) += q.geq_eb;
            mna.G_ref(node_E->node_index, node_B->node_index) -= q.geq_eb;
            mna.G_ref(node_B->node_index, node_E->node_index) -= q.geq_eb;
            mna.G_ref(node_B->node_index, node_B->node_index) += q.geq_eb;

            mna.G_ref(node_E->node_index, node_E->node_index) += q.gm;
            mna.G_ref(node_E->node_index, node_B->node_index) -= q.gm;
            mna.G_ref(node_C->node_index, node_E->node_index) -= q.gm;
            mna.G_ref(node_C->node_index, node_B->node_index) += q.gm;
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_ac<BJT_PNP>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<BJT_PNP>, BJT_PNP& q) noexcept
    {
        return {q.pins, 3};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<BJT_PNP>);
}  // namespace phy_engine::model

