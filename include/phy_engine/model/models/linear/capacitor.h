#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../model_refs/base.h"
#include "../../../circuits/solver/integral_history.h"
#include "../../../circuits/solver/integral_corrector_gear.h"

namespace phy_engine::model
{
    struct capacitor
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"Capacitor"};

        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::linear};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"C"};

        double m_kZimag{1e-5};
        ::phy_engine::model::pin pins[2]{{{u8"A"}}, {{u8"B"}}};

        // private:
        ::phy_engine::solver::integral_history m_historyX{};
        ::phy_engine::solver::integral_history m_historyY{};
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
        get_attribute_name_define(::phy_engine::model::model_reserve_type_t<capacitor>, capacitor const& c, ::std::size_t n) noexcept
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

    inline bool
        prepare_tr_define(::phy_engine::model::model_reserve_type_t<capacitor>, capacitor const& c, ::phy_engine::solver::integral_corrector_gear& icg) noexcept
    {
        auto const node_0{c.pins[0].nodes};
        auto const node_1{c.pins[1].nodes};

        if(node_0 && node_1) [[likely]]
        {
            icg.m_integralU.push_back(node_0->node_index);
            icg.m_integralU.push_back(node_1->node_index);
        }

        return true;
    }

    static_assert(::phy_engine::model::defines::can_prepare_tr<capacitor>);

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
                                            [[maybe_unused]] double t_time,
                                            ::phy_engine::solver::integral_corrector_gear& icg) noexcept
    {
        auto const node_0{c.pins[0].nodes};
        auto const node_1{c.pins[1].nodes};
        if(node_0 && node_1) [[likely]]
        {
            double voltage{node_0->node_information.an.voltage.real() - node_1->node_information.an.voltage.real()};

            double geq{};
            double Ieq{};

            c.m_historyX.set(0, voltage);
            icg.integrate(c.m_historyX, c.m_historyY, c.m_kZimag, geq, Ieq);

            mna.G_ref(node_0->node_index, node_0->node_index) += geq;
            mna.G_ref(node_0->node_index, node_1->node_index) -= geq;
            mna.G_ref(node_1->node_index, node_0->node_index) -= geq;
            mna.G_ref(node_1->node_index, node_1->node_index) += geq;
            mna.I_ref(node_0->node_index) -= Ieq;
            mna.I_ref(node_1->node_index) += Ieq;
        }

        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_tr<capacitor>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<capacitor>, capacitor& c) noexcept
    {
        return {c.pins, 2};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<capacitor>);

    inline constexpr ::phy_engine::solver::integral_history_view generate_integral_history_view_define(::phy_engine::model::model_reserve_type_t<capacitor>,
                                                                                                       capacitor& c) noexcept
    {
        return {__builtin_addressof(c.m_historyX), __builtin_addressof(c.m_historyY), 1};
    }

    static_assert(::phy_engine::model::defines::can_generate_integral_history_view<capacitor>);

}  // namespace phy_engine::model
