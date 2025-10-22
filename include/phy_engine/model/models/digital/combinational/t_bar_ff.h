#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../../circuits/digital/update_table.h"
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    struct T_BAR_FF
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"T_BAR_FF"};
        inline static constexpr ::phy_engine::model::digital_update_method_t digital_update_method{::phy_engine::model::digital_update_method_t::update_table};
        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::digital};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"T_BAR_FF"};

        ::phy_engine::model::pin pins[3]{{{u8"t_bar"}}, {{u8"clk"}}, {{u8"q"}}};

        double Ll{0.0};
        double Hl{5.0};

        ::phy_engine::model::digital_node_statement_t q{::phy_engine::model::digital_node_statement_t::false_state};
        ::phy_engine::model::digital_node_statement_t last_clk{::phy_engine::model::digital_node_statement_t::false_state};
    };

    static_assert(::phy_engine::model::model<T_BAR_FF>);

    inline constexpr ::phy_engine::digital::need_operate_analog_node_t update_digital_clk_define(::phy_engine::model::model_reserve_type_t<T_BAR_FF>,
                                                                                                 T_BAR_FF& clip,
                                                                                                 ::phy_engine::digital::digital_node_update_table& table,
                                                                                                 double /*tr_duration*/,
                                                                                                 ::phy_engine::model::digital_update_method_t /*method*/) noexcept
    {
        auto const ntb{clip.pins[0].nodes};
        auto const nclk{clip.pins[1].nodes};
        auto const nq{clip.pins[2].nodes};

        if(ntb && nclk && nq)
        {
            auto read_dn = [&](::phy_engine::model::node_t* n) constexpr noexcept {
                if(n->num_of_analog_node == 0) { return n->node_information.dn.state; }
                double const v{n->node_information.an.voltage.real()};
                if(v >= clip.Hl) { return ::phy_engine::model::digital_node_statement_t::true_state; }
                if(v <= clip.Ll) { return ::phy_engine::model::digital_node_statement_t::false_state; }
                return ::phy_engine::model::digital_node_statement_t::indeterminate_state;
            };

            auto const TB{read_dn(ntb)}; // active low toggle
            auto const CLK{read_dn(nclk)};

            if(clip.last_clk == ::phy_engine::model::digital_node_statement_t::false_state && CLK == ::phy_engine::model::digital_node_statement_t::true_state)
            {
                if(TB == ::phy_engine::model::digital_node_statement_t::false_state) // low enables toggle
                {
                    clip.q = static_cast<::phy_engine::model::digital_node_statement_t>(!static_cast<bool>(clip.q));
                }
            }
            clip.last_clk = CLK;

            if(nq->num_of_analog_node == 0)
            {
                if(nq->node_information.dn.state != clip.q) { nq->node_information.dn.state = clip.q; table.tables.insert(nq); }
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

    static_assert(::phy_engine::model::defines::can_update_digital_clk<T_BAR_FF>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<T_BAR_FF>, T_BAR_FF& clip) noexcept
    {
        return {clip.pins, 3};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<T_BAR_FF>);
}  // namespace phy_engine::model



