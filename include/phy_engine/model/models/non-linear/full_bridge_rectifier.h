#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../model_refs/base.h"
#include "PN_junction.h"

namespace phy_engine::model
{
    // Full-bridge rectifier composed by 4 ideal diodes modeled with PN_junction small model
    struct full_bridge_rectifier
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"Full Bridge Rectifier"};

        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::non_linear};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"FBR"};

        // Pins: AC input A,B and DC output +,-
        ::phy_engine::model::pin pins[4]{{{u8"A"}}, {{u8"B"}}, {{u8"+"}}, {{u8"-"}}};

        // Internally reuse four PN junctions: D1: A->+, D2: B->+, D3: - -> A, D4: - -> B
        PN_junction D1{};
        PN_junction D2{};
        PN_junction D3{};
        PN_junction D4{};
    };

    static_assert(::phy_engine::model::model<full_bridge_rectifier>);

    inline bool prepare_foundation_define(::phy_engine::model::model_reserve_type_t<full_bridge_rectifier>, full_bridge_rectifier& r) noexcept
    {
        // bind internal diodes to external nodes
        // A->+
        r.D1.pins[0].nodes = r.pins[0].nodes;
        r.D1.pins[1].nodes = r.pins[2].nodes;
        // B->+
        r.D2.pins[0].nodes = r.pins[1].nodes;
        r.D2.pins[1].nodes = r.pins[2].nodes;
        // - -> A
        r.D3.pins[0].nodes = r.pins[3].nodes;
        r.D3.pins[1].nodes = r.pins[0].nodes;
        // - -> B
        r.D4.pins[0].nodes = r.pins[3].nodes;
        r.D4.pins[1].nodes = r.pins[1].nodes;

        prepare_foundation_define(::phy_engine::model::model_reserve_type<PN_junction>, r.D1);
        prepare_foundation_define(::phy_engine::model::model_reserve_type<PN_junction>, r.D2);
        prepare_foundation_define(::phy_engine::model::model_reserve_type<PN_junction>, r.D3);
        prepare_foundation_define(::phy_engine::model::model_reserve_type<PN_junction>, r.D4);

        return true;
    }

    static_assert(::phy_engine::model::defines::can_prepare_foundation<full_bridge_rectifier>);

    inline constexpr bool
        iterate_dc_define(::phy_engine::model::model_reserve_type_t<full_bridge_rectifier>, full_bridge_rectifier& r, ::phy_engine::MNA::MNA& mna) noexcept
    {
        iterate_dc_define(::phy_engine::model::model_reserve_type<PN_junction>, r.D1, mna);
        iterate_dc_define(::phy_engine::model::model_reserve_type<PN_junction>, r.D2, mna);
        iterate_dc_define(::phy_engine::model::model_reserve_type<PN_junction>, r.D3, mna);
        iterate_dc_define(::phy_engine::model::model_reserve_type<PN_junction>, r.D4, mna);
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_dc<full_bridge_rectifier>);

    inline constexpr bool iterate_ac_define(::phy_engine::model::model_reserve_type_t<full_bridge_rectifier>,
                                            full_bridge_rectifier& r,
                                            ::phy_engine::MNA::MNA& mna,
                                            double omega) noexcept
    {
        iterate_ac_define(::phy_engine::model::model_reserve_type<PN_junction>, r.D1, mna, omega);
        iterate_ac_define(::phy_engine::model::model_reserve_type<PN_junction>, r.D2, mna, omega);
        iterate_ac_define(::phy_engine::model::model_reserve_type<PN_junction>, r.D3, mna, omega);
        iterate_ac_define(::phy_engine::model::model_reserve_type<PN_junction>, r.D4, mna, omega);
        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_ac<full_bridge_rectifier>);

    inline constexpr bool step_changed_tr_define(::phy_engine::model::model_reserve_type_t<full_bridge_rectifier>,
                                                 full_bridge_rectifier& r,
                                                 [[maybe_unused]] double nlaststep,
                                                 [[maybe_unused]] double nstep) noexcept
    {
        step_changed_tr_define(::phy_engine::model::model_reserve_type<PN_junction>, r.D1, nlaststep, nstep);
        step_changed_tr_define(::phy_engine::model::model_reserve_type<PN_junction>, r.D2, nlaststep, nstep);
        step_changed_tr_define(::phy_engine::model::model_reserve_type<PN_junction>, r.D3, nlaststep, nstep);
        step_changed_tr_define(::phy_engine::model::model_reserve_type<PN_junction>, r.D4, nlaststep, nstep);
        return true;
    }

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<full_bridge_rectifier>,
                                                                            full_bridge_rectifier& r) noexcept
    {
        return {r.pins, 4};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<full_bridge_rectifier>);
}  // namespace phy_engine::model

