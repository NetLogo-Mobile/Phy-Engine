#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    // Shichman–Hodges Level-1 PMOS (bulk tied to source)
    struct pmosfet
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"PMOSFET"};

        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::non_linear};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"PMOS"};

        // Pins: D, G, S
        ::phy_engine::model::pin pins[3]{{{u8"D"}}, {{u8"G"}}, {{u8"S"}}};

        // Parameters
        double Kp{1e-3};     // A/V^2 (use positive; polarity handled)
        double lambda{0.0};  // 1/V
        double Vth{1.0};     // V (|Vth|)

        // Linearization state
        double gm{};         // ∂Id/∂Vsg (Id defined positive from D->S; PMOS sign handled)
        double gds{};        // ∂Id/∂Vds
        double Ieq{};        // Id - gm*Vsg - gds*Vds
    };

    static_assert(::phy_engine::model::model<pmosfet>);

    inline constexpr bool set_attribute_define(::phy_engine::model::model_reserve_type_t<pmosfet>, pmosfet& m, ::std::size_t idx, ::phy_engine::model::variant vi) noexcept
    {
        switch(idx)
        {
            case 0: if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] return false; m.Kp = vi.d; return true;
            case 1: if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] return false; m.lambda = vi.d; return true;
            case 2: if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] return false; m.Vth = vi.d; return true;
            default: return false;
        }
        return false;
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<pmosfet>);

    inline constexpr ::phy_engine::model::variant get_attribute_define(::phy_engine::model::model_reserve_type_t<pmosfet>, pmosfet const& m, ::std::size_t idx) noexcept
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

    static_assert(::phy_engine::model::defines::has_get_attribute<pmosfet>);

    inline constexpr ::fast_io::u8string_view get_attribute_name_define(::phy_engine::model::model_reserve_type_t<pmosfet>, ::std::size_t idx) noexcept
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

    static_assert(::phy_engine::model::defines::has_get_attribute_name<pmosfet>);

    inline constexpr bool iterate_dc_define(::phy_engine::model::model_reserve_type_t<pmosfet>, pmosfet& m, ::phy_engine::MNA::MNA& mna) noexcept
    {
        auto const node_D{m.pins[0].nodes};
        auto const node_G{m.pins[1].nodes};
        auto const node_S{m.pins[2].nodes};
        if(node_D && node_G && node_S) [[likely]]
        {
            double const Vd{node_D->node_information.an.voltage.real()};
            double const Vg{node_G->node_information.an.voltage.real()};
            double const Vs{node_S->node_information.an.voltage.real()};
            double const Vsg{Vs - Vg};
            double const Vds{Vd - Vs}; // note: Id defined D->S

            double Id{}; m.gm = 0.0; m.gds = 0.0;
            double const Vov{Vsg - m.Vth};
            if(Vov <= 0.0)
            {
                Id = 0.0;
            }
            else if((-Vds) < Vov) // triode condition mirrored for PMOS
            {
                double const Vsd{-Vds};
                double const B{Vov * Vsd - 0.5 * Vsd * Vsd};
                double const Id_s{m.Kp * B * (1.0 + m.lambda * Vsd)}; // source->drain
                Id = -Id_s; // convert to D->S sign
                m.gm = m.Kp * Vsd * (1.0 + m.lambda * Vsd);
                double const dIddVsd{m.Kp * ((Vov - Vsd) * (1.0 + m.lambda * Vsd) + B * m.lambda)};
                m.gds = -dIddVsd; // dId/dVds = - dIds/dVsd
            }
            else
            {
                double const Id_s{0.5 * m.Kp * Vov * Vov * (1.0 + m.lambda * (-Vds))};
                Id = -Id_s;
                m.gm = m.Kp * Vov * (1.0 + m.lambda * (-Vds));
                m.gds = 0.5 * m.Kp * Vov * Vov * (-m.lambda);
            }

            m.Ieq = Id - m.gm * Vsg - m.gds * Vds;

            // gds between D and S
            mna.G_ref(node_D->node_index, node_D->node_index) += m.gds;
            mna.G_ref(node_D->node_index, node_S->node_index) -= m.gds;
            mna.G_ref(node_S->node_index, node_D->node_index) -= m.gds;
            mna.G_ref(node_S->node_index, node_S->node_index) += m.gds;

            // gm controlled by Vsg: note control is Vs - Vg
            mna.G_ref(node_D->node_index, node_S->node_index) += m.gm;
            mna.G_ref(node_D->node_index, node_G->node_index) -= m.gm;
            mna.G_ref(node_S->node_index, node_S->node_index) -= m.gm;
            mna.G_ref(node_S->node_index, node_G->node_index) += m.gm;

            // Ieq from D->S
            mna.I_ref(node_D->node_index) -= m.Ieq;
            mna.I_ref(node_S->node_index) += m.Ieq;
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_dc<pmosfet>);

    inline constexpr bool iterate_ac_define(::phy_engine::model::model_reserve_type_t<pmosfet>, pmosfet const& m, ::phy_engine::MNA::MNA& mna, [[maybe_unused]] double omega) noexcept
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

            // gm controlled by Vsg i.e., Vs - Vg
            mna.G_ref(node_D->node_index, node_S->node_index) += m.gm;
            mna.G_ref(node_D->node_index, node_G->node_index) -= m.gm;
            mna.G_ref(node_S->node_index, node_S->node_index) -= m.gm;
            mna.G_ref(node_S->node_index, node_G->node_index) += m.gm;
        }
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_ac<pmosfet>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<pmosfet>, pmosfet& m) noexcept
    {
        return {m.pins, 3};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<pmosfet>);

}


