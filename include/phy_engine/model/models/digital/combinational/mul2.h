#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../../circuits/digital/update_table.h"
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    struct MUL2
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"MUL2"};
        inline static constexpr ::phy_engine::model::digital_update_method_t digital_update_method{::phy_engine::model::digital_update_method_t::update_table};
        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::digital};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"M2"};

        // Inputs: a0,a1,b0,b1; Outputs: p0,p1,p2,p3
        ::phy_engine::model::pin pins[8]{{{u8"a0"}}, {{u8"a1"}}, {{u8"b0"}}, {{u8"b1"}}, {{u8"p0"}}, {{u8"p1"}}, {{u8"p2"}}, {{u8"p3"}}};

        double Ll{0.0};
        double Hl{5.0};

        ::phy_engine::model::digital_node_statement_t last_p[4]{::phy_engine::model::digital_node_statement_t::X,
                                                                ::phy_engine::model::digital_node_statement_t::X,
                                                                ::phy_engine::model::digital_node_statement_t::X,
                                                                ::phy_engine::model::digital_node_statement_t::X};
    };

    static_assert(::phy_engine::model::model<MUL2>);

    inline constexpr ::phy_engine::digital::need_operate_analog_node_t update_digital_clk_define(::phy_engine::model::model_reserve_type_t<MUL2>,
                                                                                                 MUL2& clip,
                                                                                                 ::phy_engine::digital::digital_node_update_table& table,
                                                                                                 double /*tr_duration*/,
                                                                                                 ::phy_engine::model::digital_update_method_t /*method*/) noexcept
    {
        auto const a0{clip.pins[0].nodes};
        auto const a1{clip.pins[1].nodes};
        auto const b0{clip.pins[2].nodes};
        auto const b1{clip.pins[3].nodes};
        auto const p0{clip.pins[4].nodes};
        auto const p1{clip.pins[5].nodes};
        auto const p2{clip.pins[6].nodes};
        auto const p3{clip.pins[7].nodes};

        if(a0 && a1 && b0 && b1 && p0 && p1 && p2 && p3) [[likely]]
        {
            auto read_dn = [&](::phy_engine::model::node_t* n) constexpr noexcept {
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

            auto const A0{read_dn(a0)};
            auto const A1{read_dn(a1)};
            auto const B0{read_dn(b0)};
            auto const B1{read_dn(b1)};

            bool const any_unknown{
                A0 == ::phy_engine::model::digital_node_statement_t::indeterminate_state ||
                A1 == ::phy_engine::model::digital_node_statement_t::indeterminate_state ||
                B0 == ::phy_engine::model::digital_node_statement_t::indeterminate_state ||
                B1 == ::phy_engine::model::digital_node_statement_t::indeterminate_state
            };

            ::phy_engine::model::digital_node_statement_t p0v{};
            ::phy_engine::model::digital_node_statement_t p1v{};
            ::phy_engine::model::digital_node_statement_t p2v{};
            ::phy_engine::model::digital_node_statement_t p3v{};

            if(any_unknown)
            {
                p0v = p1v = p2v = p3v = ::phy_engine::model::digital_node_statement_t::indeterminate_state;
            }
            else
            {
                // Partial products
                auto const p0vv{A0 & B0};
                auto const p1_t1{A0 & B1};
                auto const p1_t2{A1 & B0};
                auto const p1vv{p1_t1 ^ p1_t2};
                auto const c1{p1_t1 & p1_t2};
                auto const p2_t1{A1 & B1};
                auto const p2vv{p2_t1 ^ c1};
                auto const p3vv{p2_t1 & c1};
                p0v = p0vv; p1v = p1vv; p2v = p2vv; p3v = p3vv;
            }

            ::phy_engine::model::digital_node_statement_t vals[4]{p0v, p1v, p2v, p3v};
            ::phy_engine::model::node_t* outs[4]{p0, p1, p2, p3};

            for(int i = 0; i < 4; ++i)
            {
                auto const v{vals[i]};
                auto* const n{outs[i]};
                if(n->num_of_analog_node == 0)
                {
                    if(clip.last_p[i] != v) { clip.last_p[i] = v; n->node_information.dn.state = v; table.tables.insert(n); }
                }
                else
                {
                    switch(v)
                    {
                        case ::phy_engine::model::digital_node_statement_t::false_state: return {clip.Ll, n};
                        case ::phy_engine::model::digital_node_statement_t::true_state: return {clip.Hl, n};
                        default: return {clip.Ll, n};
                    }
                }
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::can_update_digital_clk<MUL2>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<MUL2>, MUL2& clip) noexcept
    {
        return {clip.pins, 8};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<MUL2>);
}  // namespace phy_engine::model



