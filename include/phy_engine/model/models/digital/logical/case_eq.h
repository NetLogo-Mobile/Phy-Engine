#pragma once

#include <fast_io/fast_io_dsal/string_view.h>

#include "../../../../circuits/digital/update_table.h"
#include "../../../model_refs/base.h"

namespace phy_engine::model
{
    // 1-bit 4-state case equality: outputs 1 iff inputs are exactly equal (including X and Z), else 0.
    // Output is always 0/1 (never X).
    struct CASE_EQ
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"CASE_EQ"};
        inline static constexpr ::phy_engine::model::digital_update_method_t digital_update_method{::phy_engine::model::digital_update_method_t::update_table};
        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::digital};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"CASE_EQ"};

        ::phy_engine::model::pin pins[3]{{{u8"ia"}}, {{u8"ib"}}, {{u8"o"}}};

        double Ll{0.0};
        double Hl{5.0};

        // private
        ::phy_engine::model::digital_node_statement_t last_out{::phy_engine::model::digital_node_statement_t::X};
    };

    static_assert(::phy_engine::model::model<CASE_EQ>);

    inline constexpr bool set_attribute_define(::phy_engine::model::model_reserve_type_t<CASE_EQ>, CASE_EQ& clip, ::std::size_t n, ::phy_engine::model::variant vi) noexcept
    {
        switch(n)
        {
            case 0:
            {
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                clip.Ll = vi.d;
                return true;
            }
            case 1:
            {
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                clip.Hl = vi.d;
                return true;
            }
            default: return false;
        }
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<CASE_EQ>);

    inline constexpr ::phy_engine::model::variant get_attribute_define(::phy_engine::model::model_reserve_type_t<CASE_EQ>, CASE_EQ const& clip, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0: return {.d{clip.Ll}, .type{::phy_engine::model::variant_type::d}};
            case 1: return {.d{clip.Hl}, .type{::phy_engine::model::variant_type::d}};
            default: return {};
        }
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<CASE_EQ>);

    inline constexpr ::fast_io::u8string_view get_attribute_name_define(::phy_engine::model::model_reserve_type_t<CASE_EQ>, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0: return u8"Ll";
            case 1: return u8"Hl";
            default: return {};
        }
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<CASE_EQ>);

    inline constexpr ::phy_engine::digital::need_operate_analog_node_t update_digital_clk_define(::phy_engine::model::model_reserve_type_t<CASE_EQ>,
                                                                                                 CASE_EQ& clip,
                                                                                                 ::phy_engine::digital::digital_node_update_table& table,
                                                                                                 double /*tr_duration*/,
                                                                                                 ::phy_engine::model::digital_update_method_t /*method*/) noexcept
    {
        auto read_dn = [&](::phy_engine::model::node_t* n) constexpr noexcept -> ::phy_engine::model::digital_node_statement_t
        {
            if(n == nullptr) { return ::phy_engine::model::digital_node_statement_t::indeterminate_state; }
            if(n->num_of_analog_node == 0)
            {
                return n->node_information.dn.state;
            }
            double const v{n->node_information.an.voltage.real()};
            if(v >= clip.Hl) { return ::phy_engine::model::digital_node_statement_t::true_state; }
            if(v <= clip.Ll) { return ::phy_engine::model::digital_node_statement_t::false_state; }
            return ::phy_engine::model::digital_node_statement_t::indeterminate_state;
        };

        auto const node_a{clip.pins[0].nodes};
        auto const node_b{clip.pins[1].nodes};
        auto const node_o{clip.pins[2].nodes};
        if(node_a == nullptr || node_b == nullptr || node_o == nullptr) { return {}; }

        auto const a = read_dn(node_a);
        auto const b = read_dn(node_b);
        auto const out = (a == b) ? ::phy_engine::model::digital_node_statement_t::true_state : ::phy_engine::model::digital_node_statement_t::false_state;

        if(node_o->num_of_analog_node == 0)
        {
            if(node_o->node_information.dn.state != out)
            {
                node_o->node_information.dn.state = out;
                table.tables.insert(node_o);
            }
            clip.last_out = out;
            return {};
        }

        if(clip.last_out == out) { return {}; }
        clip.last_out = out;
        return {out == ::phy_engine::model::digital_node_statement_t::true_state ? clip.Hl : clip.Ll, node_o};
    }

    static_assert(::phy_engine::model::defines::can_update_digital_clk<CASE_EQ>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<CASE_EQ>, CASE_EQ& clip) noexcept
    {
        return {clip.pins, 3};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<CASE_EQ>);
}  // namespace phy_engine::model

