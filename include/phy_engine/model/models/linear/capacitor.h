#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    struct capacitor
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"Capacitor"};

        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::linear};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"C"};

        double m_kZimag{1e-5};
        
        // Trapezoidal integration state
        double m_tr_hist_current{};   // history current (Norton equivalent) used by TR: i = g*v + m_tr_hist_current
        double m_tr_prev_g{};         // g used in the previous step (2*C/dt)

        ::phy_engine::model::pin pins[2]{{{u8"A"}}, {{u8"B"}}};
    };

    static_assert(::phy_engine::model::model<capacitor>);

    inline constexpr bool
        set_attribute_define(::phy_engine::model::model_reserve_type_t<capacitor>, capacitor& c, ::std::size_t n, ::phy_engine::model::variant vi) noexcept
    {
        switch(n)
        {
            case 0:
            {
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                c.m_kZimag = vi.d;
                return true;
            }
            default:
            {
                return false;
            }
        }
        return false;
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<capacitor>);

    inline constexpr ::phy_engine::model::variant
        get_attribute_define(::phy_engine::model::model_reserve_type_t<capacitor>, capacitor const& c, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                return {.d{c.m_kZimag}, .type{::phy_engine::model::variant_type::d}};
            }
            default:
            {
                return {};
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<capacitor>);

    inline constexpr ::fast_io::u8string_view
        get_attribute_name_define(::phy_engine::model::model_reserve_type_t<capacitor>, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                // m_kZimag
                return {u8"C"};
            }
            default:
            {
                return {};
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<capacitor>);

    inline constexpr bool
        iterate_ac_define(::phy_engine::model::model_reserve_type_t<capacitor>, capacitor const& c, ::phy_engine::MNA::MNA& mna, double omega) noexcept
    {
        auto const node_0{c.pins[0].nodes};
        auto const node_1{c.pins[1].nodes};

        if(node_0 && node_1) [[likely]]
        {
            ::std::complex<double> z{0.0, c.m_kZimag * omega};

            mna.G_ref(node_0->node_index, node_0->node_index) += z;
            mna.G_ref(node_0->node_index, node_1->node_index) -= z;
            mna.G_ref(node_1->node_index, node_0->node_index) -= z;
            mna.G_ref(node_1->node_index, node_1->node_index) += z;
        }

        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_ac<capacitor>);

    inline constexpr bool iterate_tr_define(::phy_engine::model::model_reserve_type_t<capacitor>,
                                            capacitor& c,
                                            ::phy_engine::MNA::MNA& mna,
                                            [[maybe_unused]] double t_time) noexcept
    {
        auto const node_0{c.pins[0].nodes};
        auto const node_1{c.pins[1].nodes};

        if(node_0 && node_1) [[likely]]
        {
            // previous step voltage across capacitor (node_0 - node_1)
            double const v_prev{node_0->node_information.an.voltage.real() - node_1->node_information.an.voltage.real()};

            if(t_time <= 0.0)
            {
                // No contribution at TROP (capacitor open circuit)
                return true;
            }

            // Trapezoidal integrator companion model (Norton):
            // geq = 2*C/dt, Ieq = history current h^n
            double const g_new{2.0 * c.m_kZimag / t_time};

            // Update history current to the value for this step using last and new conductances:
            // h^n = -(g_new + g_prev)*v_prev - h^{n-1}; if no previous step, assume i^0 ≈ 0 => h^0 = -g_new*v_prev
            if(c.m_tr_prev_g > 0.0)
            {
                c.m_tr_hist_current = -(g_new + c.m_tr_prev_g) * v_prev - c.m_tr_hist_current;
            }
            else
            {
                c.m_tr_hist_current = -g_new * v_prev;
            }
            c.m_tr_prev_g = g_new;

            // Stamp Norton equivalent
            mna.G_ref(node_0->node_index, node_0->node_index) += g_new;
            mna.G_ref(node_0->node_index, node_1->node_index) -= g_new;
            mna.G_ref(node_1->node_index, node_0->node_index) -= g_new;
            mna.G_ref(node_1->node_index, node_1->node_index) += g_new;
            mna.I_ref(node_0->node_index) -= c.m_tr_hist_current;
            mna.I_ref(node_1->node_index) += c.m_tr_hist_current;
        }

        return true;
    }

    inline constexpr bool iterate_trop_define(::phy_engine::model::model_reserve_type_t<capacitor>,
                                              capacitor& /*c*/,
                                              ::phy_engine::MNA::MNA& /*mna*/) noexcept
    {
        // Capacitor is open-circuit for transient operating point; no stamp needed
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_tr<capacitor>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<capacitor>, capacitor& c) noexcept
    {
        return {c.pins, 2};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<capacitor>);

    static_assert(::phy_engine::model::defines::can_iterate_trop<capacitor>);

}  // namespace phy_engine::model
