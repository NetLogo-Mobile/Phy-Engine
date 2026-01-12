#pragma once

#include <cstdint>

#include <fast_io/fast_io_dsal/string_view.h>

#include "../../../../circuits/digital/update_table.h"
#include "../../../model_refs/base.h"

namespace phy_engine::model
{
    struct EIGHT_BIT_DISPLAY
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"EIGHT_BIT_DISPLAY"};
        inline static constexpr ::phy_engine::model::digital_update_method_t digital_update_method{::phy_engine::model::digital_update_method_t::update_table};
        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::digital};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"EIGHT_BIT_DISPLAY"};

        // pins: 0..7 are bits b7..b0 (MSB..LSB)
        ::phy_engine::model::pin pins[8]{{{u8"b7"}}, {{u8"b6"}}, {{u8"b5"}}, {{u8"b4"}}, {{u8"b3"}}, {{u8"b2"}}, {{u8"b1"}}, {{u8"b0"}}};

        double Ll{0.0};
        double Hl{5.0};

        // sampled (4-state) value
        ::std::uint8_t value{};
        ::std::uint8_t unknown_mask{};  // bit=1 means X/Z
    };

    static_assert(::phy_engine::model::model<EIGHT_BIT_DISPLAY>);

    inline constexpr bool
        set_attribute_define(::phy_engine::model::model_reserve_type_t<EIGHT_BIT_DISPLAY>, EIGHT_BIT_DISPLAY& /*clip*/, ::std::size_t /*n*/, ::phy_engine::model::variant /*vi*/) noexcept
    {
        return false;
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<EIGHT_BIT_DISPLAY>);

    inline constexpr ::phy_engine::model::variant
        get_attribute_define(::phy_engine::model::model_reserve_type_t<EIGHT_BIT_DISPLAY>, EIGHT_BIT_DISPLAY const& clip, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0: return {.ui8{clip.value}, .type{::phy_engine::model::variant_type::ui8}};
            case 1: return {.ui8{clip.unknown_mask}, .type{::phy_engine::model::variant_type::ui8}};
            default: return {};
        }
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<EIGHT_BIT_DISPLAY>);

    inline constexpr ::fast_io::u8string_view get_attribute_name_define(::phy_engine::model::model_reserve_type_t<EIGHT_BIT_DISPLAY>, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0: return {u8"value"};
            case 1: return {u8"unknown_mask"};
            default: return {};
        }
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<EIGHT_BIT_DISPLAY>);

    inline constexpr ::phy_engine::digital::need_operate_analog_node_t update_digital_clk_define(::phy_engine::model::model_reserve_type_t<EIGHT_BIT_DISPLAY>,
                                                                                                 EIGHT_BIT_DISPLAY& clip,
                                                                                                 ::phy_engine::digital::digital_node_update_table& /*table*/,
                                                                                                 double /*tr_duration*/,
                                                                                                 ::phy_engine::model::digital_update_method_t /*method*/) noexcept
    {
        auto read_dn = [&](::phy_engine::model::node_t* n) constexpr noexcept -> ::phy_engine::model::digital_node_statement_t
        {
            if(n == nullptr) { return ::phy_engine::model::digital_node_statement_t::X; }
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

        ::std::uint8_t v{};
        ::std::uint8_t um{};

        for(int pin = 0; pin < 8; ++pin)
        {
            auto* n = clip.pins[pin].nodes;
            int const bit = 7 - pin;
            auto const st = read_dn(n);
            switch(st)
            {
                case ::phy_engine::model::digital_node_statement_t::false_state: break;
                case ::phy_engine::model::digital_node_statement_t::true_state: v |= static_cast<::std::uint8_t>(1u << static_cast<unsigned>(bit)); break;
                case ::phy_engine::model::digital_node_statement_t::indeterminate_state:
                case ::phy_engine::model::digital_node_statement_t::high_impedence_state:
                    um |= static_cast<::std::uint8_t>(1u << static_cast<unsigned>(bit));
                    break;
                default: break;
            }
        }

        clip.value = v;
        clip.unknown_mask = um;

        return {};
    }

    static_assert(::phy_engine::model::defines::can_update_digital_clk<EIGHT_BIT_DISPLAY>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<EIGHT_BIT_DISPLAY>, EIGHT_BIT_DISPLAY& clip) noexcept
    {
        return {clip.pins, 8};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<EIGHT_BIT_DISPLAY>);
}  // namespace phy_engine::model

