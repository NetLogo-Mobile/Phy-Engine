#pragma once
#include <cmath>
#include <complex>
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    struct coupled_inductors
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"Coupled Inductors"};

        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::linear};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"K"};

        // Parameters: L1, L2 in Henry, coupling factor k in [0,1]
        double L1{1e-3};
        double L2{1e-3};
        double k{0.99};

        // Trapezoidal integration state for transient analysis
        double m_tr_step{};   // dt
        double m_tr_req11{};  // (2/dt)*L1
        double m_tr_req12{};  // (2/dt)*M
        double m_tr_req22{};  // (2/dt)*L2
        double m_tr_Ueq1{};   // history source for winding1 KVL
        double m_tr_Ueq2{};   // history source for winding2 KVL

        ::phy_engine::model::pin pins[4]{{{u8"P1"}}, {{u8"P2"}}, {{u8"S1"}}, {{u8"S2"}}};  // winding1 P1-P2, winding2 S1-S2
        ::phy_engine::model::branch branches[2]{};                                         // k1 for primary, k2 for secondary
    };

    static_assert(::phy_engine::model::model<coupled_inductors>);

    inline constexpr bool set_attribute_define(::phy_engine::model::model_reserve_type_t<coupled_inductors>,
                                               coupled_inductors& kL,
                                               ::std::size_t idx,
                                               ::phy_engine::model::variant vi) noexcept
    {
        switch(idx)
        {
            case 0:
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                kL.L1 = vi.d;
                return true;
            case 1:
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                kL.L2 = vi.d;
                return true;
            case 2:
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                kL.k = vi.d;
                return true;
            default: return false;
        }
        return false;
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<coupled_inductors>);

    inline constexpr ::phy_engine::model::variant
        get_attribute_define(::phy_engine::model::model_reserve_type_t<coupled_inductors>, coupled_inductors const& kL, ::std::size_t idx) noexcept
    {
        switch(idx)
        {
            case 0: return {.d{kL.L1}, .type{::phy_engine::model::variant_type::d}};
            case 1: return {.d{kL.L2}, .type{::phy_engine::model::variant_type::d}};
            case 2: return {.d{kL.k}, .type{::phy_engine::model::variant_type::d}};
            default: return {};
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<coupled_inductors>);

    inline constexpr ::fast_io::u8string_view get_attribute_name_define(::phy_engine::model::model_reserve_type_t<coupled_inductors>,
                                                                        ::std::size_t idx) noexcept
    {
        switch(idx)
        {
            case 0: return u8"L1";
            case 1: return u8"L2";
            case 2: return u8"k";
            default: return {};
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<coupled_inductors>);

    inline constexpr bool
        iterate_dc_define(::phy_engine::model::model_reserve_type_t<coupled_inductors>, coupled_inductors const& kL, ::phy_engine::MNA::MNA& mna) noexcept
    {
        // In DC, inductors behave as short circuits
        auto const p1{kL.pins[0].nodes};
        auto const p2{kL.pins[1].nodes};
        auto const s1{kL.pins[2].nodes};
        auto const s2{kL.pins[3].nodes};
        if(p1 && p2 && s1 && s2) [[likely]]
        {
            auto const k1{kL.branches[0].index};
            auto const k2{kL.branches[1].index};
            // winding 1 short
            mna.B_ref(p1->node_index, k1) = 1.0;
            mna.B_ref(p2->node_index, k1) = -1.0;
            mna.C_ref(k1, p1->node_index) = 1.0;
            mna.C_ref(k1, p2->node_index) = -1.0;
            // winding 2 short
            mna.B_ref(s1->node_index, k2) = 1.0;
            mna.B_ref(s2->node_index, k2) = -1.0;
            mna.C_ref(k2, s1->node_index) = 1.0;
            mna.C_ref(k2, s2->node_index) = -1.0;
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_dc<coupled_inductors>);

    inline constexpr bool iterate_ac_define(::phy_engine::model::model_reserve_type_t<coupled_inductors>,
                                            coupled_inductors const& kL,
                                            ::phy_engine::MNA::MNA& mna,
                                            double omega) noexcept
    {
        // Z-matrix: [V1 V2]^T = j*omega * [[L1 M],[M L2]] * [I1 I2]^T
        auto const p1{kL.pins[0].nodes};
        auto const p2{kL.pins[1].nodes};
        auto const s1{kL.pins[2].nodes};
        auto const s2{kL.pins[3].nodes};
        if(p1 && p2 && s1 && s2) [[likely]]
        {
            auto const k1{kL.branches[0].index};
            auto const k2{kL.branches[1].index};

            double const M{kL.k * ::std::sqrt(kL.L1 * kL.L2)};
            ::std::complex<double> jomegaL1{0.0, omega * kL.L1};
            ::std::complex<double> jomegaL2{0.0, omega * kL.L2};
            ::std::complex<double> jomegaM{0.0, omega * M};

            // Branch connections
            mna.B_ref(p1->node_index, k1) = 1.0;
            mna.B_ref(p2->node_index, k1) = -1.0;
            mna.B_ref(s1->node_index, k2) = 1.0;
            mna.B_ref(s2->node_index, k2) = -1.0;

            // KVL rows: V1 - jωL1 I1 - jωM I2 = 0; V2 - jωM I1 - jωL2 I2 = 0
            mna.C_ref(k1, p1->node_index) = 1.0;
            mna.C_ref(k1, p2->node_index) = -1.0;
            mna.C_ref(k2, s1->node_index) = 1.0;
            mna.C_ref(k2, s2->node_index) = -1.0;
            mna.D_ref(k1, k1) = -jomegaL1;
            mna.D_ref(k1, k2) = -jomegaM;
            mna.D_ref(k2, k1) = -jomegaM;
            mna.D_ref(k2, k2) = -jomegaL2;
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_ac<coupled_inductors>);

    inline constexpr bool step_changed_tr_define(::phy_engine::model::model_reserve_type_t<coupled_inductors>,
                                                 coupled_inductors& kL,
                                                 [[maybe_unused]] double nlaststep,
                                                 double nstep) noexcept
    {
        kL.m_tr_step = nstep;
        kL.m_tr_req11 = 0.0;
        kL.m_tr_req12 = 0.0;
        kL.m_tr_req22 = 0.0;
        kL.m_tr_Ueq1 = 0.0;
        kL.m_tr_Ueq2 = 0.0;

        if(nstep <= 0.0) { return true; }

        auto const p1{kL.pins[0].nodes};
        auto const p2{kL.pins[1].nodes};
        auto const s1{kL.pins[2].nodes};
        auto const s2{kL.pins[3].nodes};

        if(p1 && p2 && s1 && s2) [[likely]]
        {
            double const M{kL.k * ::std::sqrt(kL.L1 * kL.L2)};

            double const req_scale{2.0 / nstep};
            kL.m_tr_req11 = req_scale * kL.L1;
            kL.m_tr_req12 = req_scale * M;
            kL.m_tr_req22 = req_scale * kL.L2;

            double const v1_prev{p1->node_information.an.voltage.real() - p2->node_information.an.voltage.real()};
            double const v2_prev{s1->node_information.an.voltage.real() - s2->node_information.an.voltage.real()};
            double const i1_prev{kL.branches[0].current.real()};
            double const i2_prev{kL.branches[1].current.real()};

            // v(n) - Req*i(n) = -v(n-1) - Req*i(n-1)
            kL.m_tr_Ueq1 = -v1_prev - (kL.m_tr_req11 * i1_prev + kL.m_tr_req12 * i2_prev);
            kL.m_tr_Ueq2 = -v2_prev - (kL.m_tr_req12 * i1_prev + kL.m_tr_req22 * i2_prev);
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_step_changed_tr<coupled_inductors>);

    inline constexpr bool iterate_tr_define(::phy_engine::model::model_reserve_type_t<coupled_inductors>,
                                            coupled_inductors& kL,
                                            ::phy_engine::MNA::MNA& mna,
                                            [[maybe_unused]] double t_time) noexcept
    {
        // Trapezoidal companion model (Thevenin):
        // [v1 v2]^T - Req * [i1 i2]^T = Ueq, where Req = (2/dt)*L_matrix and Ueq = -v_prev - Req*i_prev.
        //
        // For dt<=0 (e.g. TROP), fall back to DC behavior: both windings are short circuits.
        if(kL.m_tr_step <= 0.0) { return iterate_dc_define(::phy_engine::model::model_reserve_type<coupled_inductors>, kL, mna); }

        auto const p1{kL.pins[0].nodes};
        auto const p2{kL.pins[1].nodes};
        auto const s1{kL.pins[2].nodes};
        auto const s2{kL.pins[3].nodes};
        if(p1 && p2 && s1 && s2) [[likely]]
        {
            auto const k1{kL.branches[0].index};
            auto const k2{kL.branches[1].index};

            // Branch connections (KCL)
            mna.B_ref(p1->node_index, k1) = 1.0;
            mna.B_ref(p2->node_index, k1) = -1.0;
            mna.B_ref(s1->node_index, k2) = 1.0;
            mna.B_ref(s2->node_index, k2) = -1.0;

            // KVL rows (C)
            mna.C_ref(k1, p1->node_index) = 1.0;
            mna.C_ref(k1, p2->node_index) = -1.0;
            mna.C_ref(k2, s1->node_index) = 1.0;
            mna.C_ref(k2, s2->node_index) = -1.0;

            // Req matrix (D) and history sources (E)
            mna.D_ref(k1, k1) = -kL.m_tr_req11;
            mna.D_ref(k1, k2) = -kL.m_tr_req12;
            mna.D_ref(k2, k1) = -kL.m_tr_req12;
            mna.D_ref(k2, k2) = -kL.m_tr_req22;

            mna.E_ref(k1) = kL.m_tr_Ueq1;
            mna.E_ref(k2) = kL.m_tr_Ueq2;
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_tr<coupled_inductors>);

    inline constexpr bool iterate_trop_define(::phy_engine::model::model_reserve_type_t<coupled_inductors>,
                                              coupled_inductors const& kL,
                                              ::phy_engine::MNA::MNA& mna) noexcept
    {
        // For transient operating point, inductors behave as short circuits (DC behavior).
        return iterate_dc_define(::phy_engine::model::model_reserve_type<coupled_inductors>, kL, mna);
    }

    static_assert(::phy_engine::model::defines::can_iterate_trop<coupled_inductors>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<coupled_inductors>,
                                                                            coupled_inductors& kL) noexcept
    {
        return {kL.pins, 4};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<coupled_inductors>);

    inline constexpr ::phy_engine::model::branch_view generate_branch_view_define(::phy_engine::model::model_reserve_type_t<coupled_inductors>,
                                                                                  coupled_inductors& kL) noexcept
    {
        return {kL.branches, 2};
    }

    static_assert(::phy_engine::model::defines::can_generate_branch_view<coupled_inductors>);
}  // namespace phy_engine::model
