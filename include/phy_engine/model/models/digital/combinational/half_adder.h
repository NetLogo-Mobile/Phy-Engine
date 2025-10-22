#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../../../circuits/digital/update_table.h"
#include "../../../model_refs/base.h"

namespace phy_engine::model
{
    struct HALF_ADDER
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"HALF_ADDER"};
        inline static constexpr ::phy_engine::model::digital_update_method_t digital_update_method{::phy_engine::model::digital_update_method_t::update_table};
        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::digital};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"HA"};

        ::phy_engine::model::pin pins[4]{{{u8"ia"}}, {{u8"ib"}}, {{u8"s"}}, {{u8"c"}}};

        double Ll{0.0};
        double Hl{5.0};

        ::phy_engine::model::digital_node_statement_t last_s{::phy_engine::model::digital_node_statement_t::X};
        ::phy_engine::model::digital_node_statement_t last_c{::phy_engine::model::digital_node_statement_t::X};
    };

    static_assert(::phy_engine::model::model<HALF_ADDER>);

    inline constexpr ::phy_engine::digital::need_operate_analog_node_t
        update_digital_clk_define(::phy_engine::model::model_reserve_type_t<HALF_ADDER>,
                                  HALF_ADDER& clip,
                                  ::phy_engine::digital::digital_node_update_table& table,
                                  double /*tr_duration*/,
                                  ::phy_engine::model::digital_update_method_t /*method*/) noexcept
    {
        auto const node_a{clip.pins[0].nodes};
        auto const node_b{clip.pins[1].nodes};
        auto const node_s{clip.pins[2].nodes};
        auto const node_c{clip.pins[3].nodes};

        if(node_a && node_b && node_s && node_c) [[likely]]
        {
            auto read_dn = [&](::phy_engine::model::node_t* n) constexpr noexcept
            {
                if(n->num_of_analog_node == 0)
                {
                    auto const s{n->node_information.dn.state};
                    return s == ::phy_engine::model::digital_node_statement_t::high_impedence_state
                               ? ::phy_engine::model::digital_node_statement_t::indeterminate_state
                               : s;
                }
                double const v{n->node_information.an.voltage.real()};
                if(v >= clip.Hl) { return ::phy_engine::model::digital_node_statement_t::true_state; }
                if(v <= clip.Ll) { return ::phy_engine::model::digital_node_statement_t::false_state; }
                return ::phy_engine::model::digital_node_statement_t::indeterminate_state;
            };

            auto const A{read_dn(node_a)};
            auto const B{read_dn(node_b)};

            bool const any_unknown{A == ::phy_engine::model::digital_node_statement_t::indeterminate_state ||
                                   B == ::phy_engine::model::digital_node_statement_t::indeterminate_state};

            auto const S{any_unknown ? ::phy_engine::model::digital_node_statement_t::indeterminate_state : (A ^ B)};
            auto const C{any_unknown ? ::phy_engine::model::digital_node_statement_t::indeterminate_state : (A & B)};

            bool changed{};
            if(node_s->num_of_analog_node == 0)
            {
                if(clip.last_s != S)
                {
                    changed = true;
                    clip.last_s = S;
                    node_s->node_information.dn.state = S;
                    table.tables.insert(node_s);
                }
            }
            if(node_c->num_of_analog_node == 0)
            {
                if(clip.last_c != C)
                {
                    changed = true;
                    clip.last_c = C;
                    node_c->node_information.dn.state = C;
                    table.tables.insert(node_c);
                }
            }

            if(node_s->num_of_analog_node != 0)
            {
                switch(S)
                {
                    case ::phy_engine::model::digital_node_statement_t::false_state: return {clip.Ll, node_s};
                    case ::phy_engine::model::digital_node_statement_t::true_state: return {clip.Hl, node_s};
                    default: return {clip.Ll, node_s};
                }
            }
            if(node_c->num_of_analog_node != 0)
            {
                switch(C)
                {
                    case ::phy_engine::model::digital_node_statement_t::false_state: return {clip.Ll, node_c};
                    case ::phy_engine::model::digital_node_statement_t::true_state: return {clip.Hl, node_c};
                    default: return {clip.Ll, node_c};
                }
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::can_update_digital_clk<HALF_ADDER>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<HALF_ADDER>, HALF_ADDER& clip) noexcept
    {
        return {clip.pins, 4};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<HALF_ADDER>);
}  // namespace phy_engine::model

