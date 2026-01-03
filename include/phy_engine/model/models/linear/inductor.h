#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    struct inductor
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"Inductor"};

        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::linear};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"I"};

        double m_kZimag{1e-5};
        double m_tr_step{};  // dt for TR (set by step_changed_tr)
        double m_tr_req{};
        double m_tr_Ueq{};
        ::phy_engine::model::pin pins[2]{{{u8"A"}}, {{u8"B"}}};
        ::phy_engine::model::branch branches{};
    };

    static_assert(::phy_engine::model::model<inductor>);

    inline constexpr bool
        set_attribute_define(::phy_engine::model::model_reserve_type_t<inductor>, inductor& i, ::std::size_t n, ::phy_engine::model::variant vi) noexcept
    {
        switch(n)
        {
            case 0:
            {
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                i.m_kZimag = vi.d;
                return true;
            }
            default:
            {
                return false;
            }
        }
        return false;
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<inductor>);

    inline constexpr ::phy_engine::model::variant
        get_attribute_define(::phy_engine::model::model_reserve_type_t<inductor>, inductor const& i, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                return {.d{i.m_kZimag}, .type{::phy_engine::model::variant_type::d}};
            }
            default:
            {
                return {};
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<inductor>);

    inline constexpr ::fast_io::u8string_view get_attribute_name_define(::phy_engine::model::model_reserve_type_t<inductor>, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                // m_kZimag
                return {u8"L"};
            }
            default:
            {
                return {};
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<inductor>);

    inline constexpr bool iterate_dc_define(::phy_engine::model::model_reserve_type_t<inductor>, inductor const& i, ::phy_engine::MNA::MNA& mna) noexcept
    {

        auto const node_0{i.pins[0].nodes};
        auto const node_1{i.pins[1].nodes};
        if(node_0 && node_1) [[likely]]
        {
            auto const k{i.branches.index};
            mna.B_ref(node_0->node_index, k) = 1.0;
            mna.B_ref(node_1->node_index, k) = -1.0;
            mna.C_ref(k, node_0->node_index) = 1.0;
            mna.C_ref(k, node_1->node_index) = -1.0;
            // mna.E_ref(k) += 0.0;
        }

        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_dc<inductor>);

    inline constexpr bool
        iterate_ac_define(::phy_engine::model::model_reserve_type_t<inductor>, inductor const& i, ::phy_engine::MNA::MNA& mna, double omega) noexcept
    {

        auto const node_0{i.pins[0].nodes};
        auto const node_1{i.pins[1].nodes};

        if(node_0 && node_1) [[likely]]
        {
            auto const k{i.branches.index};
            mna.B_ref(node_0->node_index, k) = 1.0;
            mna.B_ref(node_1->node_index, k) = -1.0;
            mna.C_ref(k, node_0->node_index) = 1.0;
            mna.C_ref(k, node_1->node_index) = -1.0;

            if(i.m_kZimag == 0.0 || omega == 0.0)
            {
                // Short circuit at DC
            }
            else
            {
                // V = j*omega*L * I  =>  V - (j*omega*L)*I = 0
                mna.D_ref(k, k) = ::std::complex<double>{0.0, -omega * i.m_kZimag};
            }
        }

        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_ac<inductor>);

    inline constexpr bool step_changed_tr_define(::phy_engine::model::model_reserve_type_t<inductor>,
                                                 inductor& i,
                                                 [[maybe_unused]] double nlaststep,
                                                 double nstep) noexcept
    {
        i.m_tr_step = nstep;
        i.m_tr_req = 0.0;
        i.m_tr_Ueq = 0.0;
        if(nstep <= 0.0) { return true; }

        auto const node_0{i.pins[0].nodes};
        auto const node_1{i.pins[1].nodes};

        if(node_0 && node_1) [[likely]]
        {
            // previous step values
            double const v_prev{node_0->node_information.an.voltage.real() - node_1->node_information.an.voltage.real()};
            double const i_prev{i.branches.current.real()};

            // Trapezoidal integrator companion model (Thevenin):
            // req = 2*L/dt, Ueq = -v_prev - req*i_prev
            double const req{2.0 * i.m_kZimag / nstep};
            i.m_tr_req = req;
            i.m_tr_Ueq = -v_prev - req * i_prev;
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_step_changed_tr<inductor>);

    inline constexpr bool iterate_tr_define(::phy_engine::model::model_reserve_type_t<inductor>,
                                            inductor& i,
                                            ::phy_engine::MNA::MNA& mna,
                                            [[maybe_unused]] double t_time) noexcept
    {
        auto const node_0{i.pins[0].nodes};
        auto const node_1{i.pins[1].nodes};

        if(node_0 && node_1) [[likely]]
        {
            if(i.m_tr_step <= 0.0)
            {
                // No dynamic contribution at TROP: treat as short (DC)
                auto const k{i.branches.index};
                mna.B_ref(node_0->node_index, k) = 1.0;
                mna.B_ref(node_1->node_index, k) = -1.0;
                mna.C_ref(k, node_0->node_index) = 1.0;
                mna.C_ref(k, node_1->node_index) = -1.0;
                return true;
            }

            auto const k{i.branches.index};
            mna.B_ref(node_0->node_index, k) = 1.0;
            mna.B_ref(node_1->node_index, k) = -1.0;
            mna.C_ref(k, node_0->node_index) = 1.0;
            mna.C_ref(k, node_1->node_index) = -1.0;
            mna.D_ref(k, k) = -i.m_tr_req;
            mna.E_ref(k) = i.m_tr_Ueq;
        }

        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_tr<inductor>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<inductor>, inductor& c) noexcept
    {
        return {c.pins, 2};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<inductor>);

    inline constexpr ::phy_engine::model::branch_view generate_branch_view_define(::phy_engine::model::model_reserve_type_t<inductor>, inductor& i) noexcept
    {
        return {__builtin_addressof(i.branches), 1};
    }

    static_assert(::phy_engine::model::defines::can_generate_branch_view<inductor>);

    inline constexpr bool iterate_trop_define(::phy_engine::model::model_reserve_type_t<inductor>, inductor const& i, ::phy_engine::MNA::MNA& mna) noexcept
    {
        // For transient operating point, treat inductor as a short (DC behavior)
        auto const node_0{i.pins[0].nodes};
        auto const node_1{i.pins[1].nodes};
        if(node_0 && node_1)
        {
            auto const k{i.branches.index};
            mna.B_ref(node_0->node_index, k) = 1.0;
            mna.B_ref(node_1->node_index, k) = -1.0;
            mna.C_ref(k, node_0->node_index) = 1.0;
            mna.C_ref(k, node_1->node_index) = -1.0;
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_trop<inductor>);

}  // namespace phy_engine::model
