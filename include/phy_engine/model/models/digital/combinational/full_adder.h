#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../../circuits/digital/update_table.h"
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    struct FULL_ADDER
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"FULL_ADDER"};
        inline static constexpr ::phy_engine::model::digital_update_method_t digital_update_method{::phy_engine::model::digital_update_method_t::update_table};
        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::digital};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"FA"};

        ::phy_engine::model::pin pins[5]{{{u8"ia"}}, {{u8"ib"}}, {{u8"cin"}}, {{u8"s"}}, {{u8"cout"}}};

        double Ll{0.0};
        double Hl{5.0};

        ::phy_engine::model::digital_node_statement_t last_s{::phy_engine::model::digital_node_statement_t::X};
        ::phy_engine::model::digital_node_statement_t last_c{::phy_engine::model::digital_node_statement_t::X};
    };

    static_assert(::phy_engine::model::model<FULL_ADDER>);

    inline constexpr ::phy_engine::digital::need_operate_analog_node_t
        update_digital_clk_define(::phy_engine::model::model_reserve_type_t<FULL_ADDER>,
                                  FULL_ADDER& clip,
                                  ::phy_engine::digital::digital_node_update_table& table,
                                  double /*tr_duration*/,
                                  ::phy_engine::model::digital_update_method_t /*method*/) noexcept
    {
        auto const node_a{clip.pins[0].nodes};
        auto const node_b{clip.pins[1].nodes};
        auto const node_ci{clip.pins[2].nodes};
        auto const node_s{clip.pins[3].nodes};
        auto const node_co{clip.pins[4].nodes};

        if(node_a && node_b && node_ci && node_s && node_co) [[likely]]
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
            auto const CI{read_dn(node_ci)};

            bool const any_unknown{A == ::phy_engine::model::digital_node_statement_t::indeterminate_state ||
                                   B == ::phy_engine::model::digital_node_statement_t::indeterminate_state ||
                                   CI == ::phy_engine::model::digital_node_statement_t::indeterminate_state};

            auto const S{any_unknown ? ::phy_engine::model::digital_node_statement_t::indeterminate_state : (A ^ B ^ CI)};
            auto const CO{any_unknown ? ::phy_engine::model::digital_node_statement_t::indeterminate_state : ((A & B) | (A & CI) | (B & CI))};

            if(node_s->num_of_analog_node == 0)
            {
                if(clip.last_s != S)
                {
                    clip.last_s = S;
                    node_s->node_information.dn.state = S;
                    table.tables.insert(node_s);
                }
            }
            if(node_co->num_of_analog_node == 0)
            {
                if(clip.last_c != CO)
                {
                    clip.last_c = CO;
                    node_co->node_information.dn.state = CO;
                    table.tables.insert(node_co);
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
            if(node_co->num_of_analog_node != 0)
            {
                switch(CO)
                {
                    case ::phy_engine::model::digital_node_statement_t::false_state: return {clip.Ll, node_co};
                    case ::phy_engine::model::digital_node_statement_t::true_state: return {clip.Hl, node_co};
                    default: return {clip.Ll, node_co};
                }
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::can_update_digital_clk<FULL_ADDER>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<FULL_ADDER>, FULL_ADDER& clip) noexcept
    {
        return {clip.pins, 5};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<FULL_ADDER>);
}  // namespace phy_engine::model

