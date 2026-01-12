#pragma once

#include <fast_io/fast_io_dsal/string_view.h>

#include "../../../../circuits/digital/update_table.h"
#include "../../../model_refs/base.h"

namespace phy_engine::model
{
    struct RESOLVE2
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"RESOLVE2"};
        inline static constexpr ::phy_engine::model::digital_update_method_t digital_update_method{::phy_engine::model::digital_update_method_t::update_table};
        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::digital};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"RESOLVE2"};

        ::phy_engine::model::pin pins[3]{{{u8"a"}}, {{u8"b"}}, {{u8"o"}}};

        double Ll{0.0};
        double Hl{5.0};
    };

    static_assert(::phy_engine::model::model<RESOLVE2>);

    inline constexpr ::phy_engine::digital::need_operate_analog_node_t
        update_digital_clk_define(::phy_engine::model::model_reserve_type_t<RESOLVE2>,
                                  RESOLVE2& clip,
                                  ::phy_engine::digital::digital_node_update_table& table,
                                  double /*tr_duration*/,
                                  ::phy_engine::model::digital_update_method_t /*method*/) noexcept
    {
        auto const na{clip.pins[0].nodes};
        auto const nb{clip.pins[1].nodes};
        auto const no{clip.pins[2].nodes};
        if(na == nullptr || nb == nullptr || no == nullptr) { return {}; }

        auto read_dn = [&](::phy_engine::model::node_t* n) constexpr noexcept -> ::phy_engine::model::digital_node_statement_t
        {
            if(n == nullptr) { return ::phy_engine::model::digital_node_statement_t::indeterminate_state; }
            if(n->num_of_analog_node == 0) { return n->node_information.dn.state; }
            double const v{n->node_information.an.voltage.real()};
            if(v >= clip.Hl) { return ::phy_engine::model::digital_node_statement_t::true_state; }
            if(v <= clip.Ll) { return ::phy_engine::model::digital_node_statement_t::false_state; }
            return ::phy_engine::model::digital_node_statement_t::indeterminate_state;
        };

        auto const a{read_dn(na)};
        auto const b{read_dn(nb)};

        auto resolve = [](auto a0, auto b0) constexpr noexcept -> ::phy_engine::model::digital_node_statement_t
        {
            using dn = ::phy_engine::model::digital_node_statement_t;
            if(a0 == dn::high_impedence_state) { return b0; }
            if(b0 == dn::high_impedence_state) { return a0; }
            if(a0 == b0) { return a0; }
            return dn::indeterminate_state;
        };

        auto const out{resolve(a, b)};

        if(no->num_of_analog_node == 0)
        {
            if(no->node_information.dn.state != out)
            {
                no->node_information.dn.state = out;
                table.tables.insert(no);
            }
            return {};
        }

        // Analog drive (best-effort; Z/X drive Ll).
        switch(out)
        {
            case ::phy_engine::model::digital_node_statement_t::false_state: return {clip.Ll, no};
            case ::phy_engine::model::digital_node_statement_t::true_state: return {clip.Hl, no};
            default: return {clip.Ll, no};
        }
    }

    static_assert(::phy_engine::model::defines::can_update_digital_clk<RESOLVE2>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<RESOLVE2>, RESOLVE2& clip) noexcept
    {
        return {clip.pins, 3};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<RESOLVE2>);
}  // namespace phy_engine::model

