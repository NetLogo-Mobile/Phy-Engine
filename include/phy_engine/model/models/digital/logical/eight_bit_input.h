#pragma once

#include <cstdint>

#include <fast_io/fast_io_dsal/string_view.h>

#include "../../../../circuits/digital/update_table.h"
#include "../../../model_refs/base.h"

namespace phy_engine::model
{
    struct EIGHT_BIT_INPUT
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"EIGHT_BIT_INPUT"};
        inline static constexpr ::phy_engine::model::digital_update_method_t digital_update_method{::phy_engine::model::digital_update_method_t::before_all_clk};
        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::digital};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"EIGHT_BIT_INPUT"};

        // pins: 0..7 are bits b7..b0 (MSB..LSB)
        ::phy_engine::model::pin pins[8]{{{u8"b7"}}, {{u8"b6"}}, {{u8"b5"}}, {{u8"b4"}}, {{u8"b3"}}, {{u8"b2"}}, {{u8"b1"}}, {{u8"b0"}}};

        double Ll{0.0};
        double Hl{5.0};

        ::std::uint8_t value{};

        // private
        ::std::uint8_t last_value{0xFF};
    };

    static_assert(::phy_engine::model::model<EIGHT_BIT_INPUT>);

    inline constexpr bool
        set_attribute_define(::phy_engine::model::model_reserve_type_t<EIGHT_BIT_INPUT>, EIGHT_BIT_INPUT& clip, ::std::size_t n, ::phy_engine::model::variant vi) noexcept
    {
        switch(n)
        {
            case 0:
            {
                if(vi.type != ::phy_engine::model::variant_type::ui8) [[unlikely]] { return false; }
                clip.value = static_cast<::std::uint8_t>(vi.ui8);
                return true;
            }
            default: return false;
        }
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<EIGHT_BIT_INPUT>);

    inline constexpr ::phy_engine::model::variant
        get_attribute_define(::phy_engine::model::model_reserve_type_t<EIGHT_BIT_INPUT>, EIGHT_BIT_INPUT const& clip, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0: return {.ui8{clip.value}, .type{::phy_engine::model::variant_type::ui8}};
            default: return {};
        }
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<EIGHT_BIT_INPUT>);

    inline constexpr ::fast_io::u8string_view get_attribute_name_define(::phy_engine::model::model_reserve_type_t<EIGHT_BIT_INPUT>, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0: return {u8"value"};
            default: return {};
        }
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<EIGHT_BIT_INPUT>);

    inline constexpr ::phy_engine::digital::need_operate_analog_node_t update_digital_clk_define(::phy_engine::model::model_reserve_type_t<EIGHT_BIT_INPUT>,
                                                                                                 EIGHT_BIT_INPUT& clip,
                                                                                                 ::phy_engine::digital::digital_node_update_table& table,
                                                                                                 double /*tr_duration*/,
                                                                                                 ::phy_engine::model::digital_update_method_t /*method*/) noexcept
    {
        bool const changed = (clip.last_value != clip.value);
        ::phy_engine::digital::need_operate_analog_node_t analog_drive{};

        for(int pin = 0; pin < 8; ++pin)
        {
            auto* n = clip.pins[pin].nodes;
            if(n == nullptr) { continue; }

            int const bit = 7 - pin;
            auto const out = ((clip.value >> static_cast<unsigned>(bit)) & 1u) ? ::phy_engine::model::digital_node_statement_t::true_state
                                                                                : ::phy_engine::model::digital_node_statement_t::false_state;

            if(n->num_of_analog_node == 0)
            {
                if(changed && n->node_information.dn.state != out)
                {
                    n->node_information.dn.state = out;
                    table.tables.insert(n);
                }
            }
            else if(analog_drive.need_to_operate_analog_node == nullptr)
            {
                analog_drive = {out == ::phy_engine::model::digital_node_statement_t::true_state ? clip.Hl : clip.Ll, n};
            }
        }

        clip.last_value = clip.value;
        return analog_drive;
    }

    static_assert(::phy_engine::model::defines::can_update_digital_clk<EIGHT_BIT_INPUT>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<EIGHT_BIT_INPUT>, EIGHT_BIT_INPUT& clip) noexcept
    {
        return {clip.pins, 8};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<EIGHT_BIT_INPUT>);
}  // namespace phy_engine::model

