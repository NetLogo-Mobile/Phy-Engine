#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    // Shichman–Hodges Level-1 NMOS (bulk tied to source)
    struct nmosfet
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"NMOSFET"};

        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::non_linear};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"NMOS"};

        // Pins: D, G, S
        ::phy_engine::model::pin pins[3]{{{u8"D"}}, {{u8"G"}}, {{u8"S"}}};

        // Parameters
        double Kp{1e-3};     // A/V^2
        double lambda{0.0};  // 1/V
        double Vth{1.0};     // V

        // Linearization state
        double gm{};   // ∂Id/∂Vgs
        double gds{};  // ∂Id/∂Vds
        double Ieq{};  // Id - gm*Vgs - gds*Vds (drain->source)
    };

    static_assert(::phy_engine::model::model<nmosfet>);

    inline constexpr bool
        set_attribute_define(::phy_engine::model::model_reserve_type_t<nmosfet>, nmosfet& m, ::std::size_t idx, ::phy_engine::model::variant vi) noexcept
    {
        switch(idx)
        {
            case 0:
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                m.Kp = vi.d;
                return true;
            case 1:
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                m.lambda = vi.d;
                return true;
            case 2:
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                m.Vth = vi.d;
                return true;
            default: return false;
        }
        return false;
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<nmosfet>);

    inline constexpr ::phy_engine::model::variant
        get_attribute_define(::phy_engine::model::model_reserve_type_t<nmosfet>, nmosfet const& m, ::std::size_t idx) noexcept
    {
        switch(idx)
        {
            case 0: return {.d{m.Kp}, .type{::phy_engine::model::variant_type::d}};
            case 1: return {.d{m.lambda}, .type{::phy_engine::model::variant_type::d}};
            case 2: return {.d{m.Vth}, .type{::phy_engine::model::variant_type::d}};
            default: return {};
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<nmosfet>);

    inline constexpr ::fast_io::u8string_view get_attribute_name_define(::phy_engine::model::model_reserve_type_t<nmosfet>, ::std::size_t idx) noexcept
    {
        switch(idx)
        {
            case 0: return u8"Kp";
            case 1: return u8"lambda";
            case 2: return u8"Vth";
            default: return {};
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<nmosfet>);

    inline constexpr bool iterate_dc_define(::phy_engine::model::model_reserve_type_t<nmosfet>, nmosfet& m, ::phy_engine::MNA::MNA& mna) noexcept
    {
        auto const node_D{m.pins[0].nodes};
        auto const node_G{m.pins[1].nodes};
        auto const node_S{m.pins[2].nodes};
        if(node_D && node_G && node_S) [[likely]]
        {
            double const Vd{node_D->node_information.an.voltage.real()};
            double const Vg{node_G->node_information.an.voltage.real()};
            double const Vs{node_S->node_information.an.voltage.real()};
            double const Vgs{Vg - Vs};
            double const Vds{Vd - Vs};

            double Id{};
            m.gm = 0.0;
            m.gds = 0.0;
            double const Vov{Vgs - m.Vth};
            if(Vov <= 0.0)
            {
                // cutoff
                Id = 0.0;
            }
            else if(Vds < Vov)
            {
                // triode
                double const B{Vov * Vds - 0.5 * Vds * Vds};
                Id = m.Kp * B * (1.0 + m.lambda * Vds);
                m.gm = m.Kp * Vds * (1.0 + m.lambda * Vds);
                m.gds = m.Kp * ((Vov - Vds) * (1.0 + m.lambda * Vds) + B * m.lambda);
            }
            else
            {
                // saturation
                Id = 0.5 * m.Kp * Vov * Vov * (1.0 + m.lambda * Vds);
                m.gm = m.Kp * Vov * (1.0 + m.lambda * Vds);
                m.gds = 0.5 * m.Kp * Vov * Vov * m.lambda;
            }

            m.Ieq = Id - m.gm * Vgs - m.gds * Vds;

            // Stamp gds between D and S
            mna.G_ref(node_D->node_index, node_D->node_index) += m.gds;
            mna.G_ref(node_D->node_index, node_S->node_index) -= m.gds;
            mna.G_ref(node_S->node_index, node_D->node_index) -= m.gds;
            mna.G_ref(node_S->node_index, node_S->node_index) += m.gds;

            // Stamp gm: VCCS controlled by Vg-Vs between D and S
            mna.G_ref(node_D->node_index, node_G->node_index) += m.gm;
            mna.G_ref(node_D->node_index, node_S->node_index) -= m.gm;
            mna.G_ref(node_S->node_index, node_G->node_index) -= m.gm;
            mna.G_ref(node_S->node_index, node_S->node_index) += m.gm;

            // Current source equivalent Ids from D->S
            mna.I_ref(node_D->node_index) -= m.Ieq;
            mna.I_ref(node_S->node_index) += m.Ieq;
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_dc<nmosfet>);

    inline constexpr bool iterate_ac_define(::phy_engine::model::model_reserve_type_t<nmosfet>,
                                            nmosfet const& m,
                                            ::phy_engine::MNA::MNA& mna,
                                            [[maybe_unused]] double omega) noexcept
    {
        auto const node_D{m.pins[0].nodes};
        auto const node_G{m.pins[1].nodes};
        auto const node_S{m.pins[2].nodes};
        if(node_D && node_G && node_S) [[likely]]
        {
            // gds between D and S
            mna.G_ref(node_D->node_index, node_D->node_index) += m.gds;
            mna.G_ref(node_D->node_index, node_S->node_index) -= m.gds;
            mna.G_ref(node_S->node_index, node_D->node_index) -= m.gds;
            mna.G_ref(node_S->node_index, node_S->node_index) += m.gds;

            // gm controlled by Vg-Vs
            mna.G_ref(node_D->node_index, node_G->node_index) += m.gm;
            mna.G_ref(node_D->node_index, node_S->node_index) -= m.gm;
            mna.G_ref(node_S->node_index, node_G->node_index) -= m.gm;
            mna.G_ref(node_S->node_index, node_S->node_index) += m.gm;
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_ac<nmosfet>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<nmosfet>, nmosfet& m) noexcept
    {
        return {m.pins, 3};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<nmosfet>);

}  // namespace phy_engine::model

