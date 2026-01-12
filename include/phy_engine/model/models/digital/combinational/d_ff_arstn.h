#pragma once

#include <fast_io/fast_io_dsal/string_view.h>

#include "../../../../circuits/digital/update_table.h"
#include "../../../model_refs/base.h"

namespace phy_engine::model
{
    struct DFF_ARSTN
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"DFF_ARSTN"};
        inline static constexpr ::phy_engine::model::digital_update_method_t digital_update_method{::phy_engine::model::digital_update_method_t::update_table};
        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::digital};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"DFF_ARSTN"};

        ::phy_engine::model::pin pins[4]{{{u8"d"}}, {{u8"clk"}}, {{u8"arst_n"}}, {{u8"q"}}};

        double Ll{0.0};
        double Hl{5.0};

        ::phy_engine::model::digital_node_statement_t q{::phy_engine::model::digital_node_statement_t::false_state};
        ::phy_engine::model::digital_node_statement_t reset_value{::phy_engine::model::digital_node_statement_t::false_state};

        ::phy_engine::model::digital_node_statement_t last_clk{::phy_engine::model::digital_node_statement_t::false_state};
    };

    static_assert(::phy_engine::model::model<DFF_ARSTN>);

    inline constexpr ::phy_engine::digital::need_operate_analog_node_t
        update_digital_clk_define(::phy_engine::model::model_reserve_type_t<DFF_ARSTN>,
                                  DFF_ARSTN& clip,
                                  ::phy_engine::digital::digital_node_update_table& table,
                                  double /*tr_duration*/,
                                  ::phy_engine::model::digital_update_method_t /*method*/) noexcept
    {
        auto const nd{clip.pins[0].nodes};
        auto const nclk{clip.pins[1].nodes};
        auto const narst{clip.pins[2].nodes};
        auto const nq{clip.pins[3].nodes};

        if(nd && nclk && narst && nq)
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

            auto const D{read_dn(nd)};
            auto const CLK{read_dn(nclk)};
            auto const ARSTN{read_dn(narst)};

            if(ARSTN == ::phy_engine::model::digital_node_statement_t::indeterminate_state)
            {
                clip.q = ::phy_engine::model::digital_node_statement_t::indeterminate_state;
            }
            else if(ARSTN == ::phy_engine::model::digital_node_statement_t::false_state)
            {
                clip.q = clip.reset_value;
            }
            else if(clip.last_clk == ::phy_engine::model::digital_node_statement_t::false_state && CLK == ::phy_engine::model::digital_node_statement_t::true_state)
            {
                clip.q = D;
            }

            if(CLK == ::phy_engine::model::digital_node_statement_t::false_state || CLK == ::phy_engine::model::digital_node_statement_t::true_state)
            {
                clip.last_clk = CLK;
            }

            if(nq->num_of_analog_node == 0)
            {
                if(nq->node_information.dn.state != clip.q)
                {
                    nq->node_information.dn.state = clip.q;
                    table.tables.insert(nq);
                }
            }
            else
            {
                switch(clip.q)
                {
                    case ::phy_engine::model::digital_node_statement_t::false_state: return {clip.Ll, nq};
                    case ::phy_engine::model::digital_node_statement_t::true_state: return {clip.Hl, nq};
                    default: return {clip.Ll, nq};
                }
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::can_update_digital_clk<DFF_ARSTN>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<DFF_ARSTN>, DFF_ARSTN& clip) noexcept
    {
        return {clip.pins, 4};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<DFF_ARSTN>);
}  // namespace phy_engine::model

