#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../../circuits/digital/update_table.h"
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    struct FULL_SUB
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"FULL_SUB"};
        inline static constexpr ::phy_engine::model::digital_update_method_t digital_update_method{::phy_engine::model::digital_update_method_t::update_table};
        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::digital};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"FS"};

        ::phy_engine::model::pin pins[5]{{{u8"ia"}}, {{u8"ib"}}, {{u8"bin"}}, {{u8"d"}}, {{u8"bout"}}};

        double Ll{0.0};
        double Hl{5.0};

        ::phy_engine::model::digital_node_statement_t last_d{::phy_engine::model::digital_node_statement_t::X};
        ::phy_engine::model::digital_node_statement_t last_b{::phy_engine::model::digital_node_statement_t::X};
    };

    static_assert(::phy_engine::model::model<FULL_SUB>);

    inline constexpr ::phy_engine::digital::need_operate_analog_node_t
        update_digital_clk_define(::phy_engine::model::model_reserve_type_t<FULL_SUB>,
                                  FULL_SUB& clip,
                                  ::phy_engine::digital::digital_node_update_table& table,
                                  double /*tr_duration*/,
                                  ::phy_engine::model::digital_update_method_t /*method*/) noexcept
    {
        auto const node_a{clip.pins[0].nodes};
        auto const node_b{clip.pins[1].nodes};
        auto const node_bi{clip.pins[2].nodes};
        auto const node_d{clip.pins[3].nodes};
        auto const node_bo{clip.pins[4].nodes};

        if(node_a && node_b && node_bi && node_d && node_bo) [[likely]]
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
            auto const BI{read_dn(node_bi)};

            auto inv = [](auto v) constexpr noexcept
            {
                using dns = ::phy_engine::model::digital_node_statement_t;
                switch(v)
                {
                    case dns::false_state: return dns::true_state;
                    case dns::true_state: return dns::false_state;
                    default: return dns::indeterminate_state;
                }
            };

            bool const any_unknown{A == ::phy_engine::model::digital_node_statement_t::indeterminate_state ||
                                   B == ::phy_engine::model::digital_node_statement_t::indeterminate_state ||
                                   BI == ::phy_engine::model::digital_node_statement_t::indeterminate_state};

            auto const D{any_unknown ? ::phy_engine::model::digital_node_statement_t::indeterminate_state : (A ^ B ^ BI)};
            auto const BO{any_unknown ? ::phy_engine::model::digital_node_statement_t::indeterminate_state : ((inv(A) & B) | (inv(A) & BI) | (B & BI))};

            if(node_d->num_of_analog_node == 0)
            {
                if(clip.last_d != D)
                {
                    clip.last_d = D;
                    node_d->node_information.dn.state = D;
                    table.tables.insert(node_d);
                }
            }
            if(node_bo->num_of_analog_node == 0)
            {
                if(clip.last_b != BO)
                {
                    clip.last_b = BO;
                    node_bo->node_information.dn.state = BO;
                    table.tables.insert(node_bo);
                }
            }

            if(node_d->num_of_analog_node != 0)
            {
                switch(D)
                {
                    case ::phy_engine::model::digital_node_statement_t::false_state: return {clip.Ll, node_d};
                    case ::phy_engine::model::digital_node_statement_t::true_state: return {clip.Hl, node_d};
                    default: return {clip.Ll, node_d};
                }
            }
            if(node_bo->num_of_analog_node != 0)
            {
                switch(BO)
                {
                    case ::phy_engine::model::digital_node_statement_t::false_state: return {clip.Ll, node_bo};
                    case ::phy_engine::model::digital_node_statement_t::true_state: return {clip.Hl, node_bo};
                    default: return {clip.Ll, node_bo};
                }
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::can_update_digital_clk<FULL_SUB>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<FULL_SUB>, FULL_SUB& clip) noexcept
    {
        return {clip.pins, 5};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<FULL_SUB>);
}  // namespace phy_engine::model

