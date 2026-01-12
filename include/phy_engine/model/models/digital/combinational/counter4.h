#pragma once

#include <cstdint>

#include <fast_io/fast_io_dsal/string_view.h>

#include "../../../../circuits/digital/update_table.h"
#include "../../../model_refs/base.h"

namespace phy_engine::model
{
    struct COUNTER4
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"COUNTER4"};
        inline static constexpr ::phy_engine::model::digital_update_method_t digital_update_method{::phy_engine::model::digital_update_method_t::update_table};
        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::digital};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"COUNTER4"};

        // pins:
        //   0..3: q3..q0 (MSB..LSB)
        //   4: clk
        //   5: en (Z => treated as H)
        ::phy_engine::model::pin pins[6]{{{u8"q3"}}, {{u8"q2"}}, {{u8"q1"}}, {{u8"q0"}}, {{u8"clk"}}, {{u8"en"}}};

        double Ll{0.0};
        double Hl{5.0};

        // attributes/state
        ::std::uint8_t value{};  // low 4 bits used
        bool unknown{};

        // private
        ::phy_engine::model::digital_node_statement_t last_clk{::phy_engine::model::digital_node_statement_t::false_state};
        ::std::uint8_t last_value{0xFF};
        bool last_unknown{true};
    };

    static_assert(::phy_engine::model::model<COUNTER4>);

    inline constexpr bool
        set_attribute_define(::phy_engine::model::model_reserve_type_t<COUNTER4>, COUNTER4& clip, ::std::size_t n, ::phy_engine::model::variant vi) noexcept
    {
        switch(n)
        {
            case 0:
            {
                if(vi.type != ::phy_engine::model::variant_type::ui8) [[unlikely]] { return false; }
                clip.value = static_cast<::std::uint8_t>(vi.ui8 & 0x0F);
                clip.unknown = false;
                return true;
            }
            case 1:
            {
                if(vi.type != ::phy_engine::model::variant_type::boolean) [[unlikely]] { return false; }
                clip.unknown = vi.boolean;
                return true;
            }
            default: return false;
        }
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<COUNTER4>);

    inline constexpr ::phy_engine::model::variant
        get_attribute_define(::phy_engine::model::model_reserve_type_t<COUNTER4>, COUNTER4 const& clip, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0: return {.ui8{static_cast<::std::uint_least8_t>(clip.value & 0x0F)}, .type{::phy_engine::model::variant_type::ui8}};
            case 1: return {.boolean{clip.unknown}, .type{::phy_engine::model::variant_type::boolean}};
            default: return {};
        }
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<COUNTER4>);

    inline constexpr ::fast_io::u8string_view get_attribute_name_define(::phy_engine::model::model_reserve_type_t<COUNTER4>, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0: return {u8"value"};
            case 1: return {u8"unknown"};
            default: return {};
        }
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<COUNTER4>);

    inline constexpr ::phy_engine::digital::need_operate_analog_node_t
        update_digital_clk_define(::phy_engine::model::model_reserve_type_t<COUNTER4>,
                                  COUNTER4& clip,
                                  ::phy_engine::digital::digital_node_update_table& table,
                                  double /*tr_duration*/,
                                  ::phy_engine::model::digital_update_method_t /*method*/) noexcept
    {
        auto read_dn_logic = [&](::phy_engine::model::node_t* n) constexpr noexcept -> ::phy_engine::model::digital_node_statement_t
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

        auto const n_clk{clip.pins[4].nodes};
        auto const n_en{clip.pins[5].nodes};
        if(n_clk == nullptr) { return {}; }

        auto const clk{read_dn_logic(n_clk)};
        auto en = (n_en == nullptr) ? ::phy_engine::model::digital_node_statement_t::high_impedence_state
                                    : (n_en->num_of_analog_node == 0 ? n_en->node_information.dn.state : read_dn_logic(n_en));
        if(en == ::phy_engine::model::digital_node_statement_t::high_impedence_state) { en = ::phy_engine::model::digital_node_statement_t::true_state; }

        bool value_changed{};
        bool unknown_changed{};

        if(clip.last_clk == ::phy_engine::model::digital_node_statement_t::false_state && clk == ::phy_engine::model::digital_node_statement_t::true_state)
        {
            if(en == ::phy_engine::model::digital_node_statement_t::true_state)
            {
                if(clip.unknown)
                {
                    // already unknown
                }
                else
                {
                    clip.value = static_cast<::std::uint8_t>((clip.value + 1u) & 0x0F);
                }
            }
            else if(en == ::phy_engine::model::digital_node_statement_t::false_state)
            {
                // hold
            }
            else
            {
                clip.unknown = true;
            }
        }

        if(clk == ::phy_engine::model::digital_node_statement_t::false_state || clk == ::phy_engine::model::digital_node_statement_t::true_state) { clip.last_clk = clk; }

        if(clip.last_value != clip.value) { value_changed = true; }
        if(clip.last_unknown != clip.unknown) { unknown_changed = true; }

        auto q_state = [&](int bit) constexpr noexcept -> ::phy_engine::model::digital_node_statement_t
        {
            if(clip.unknown) { return ::phy_engine::model::digital_node_statement_t::indeterminate_state; }
            return ((clip.value >> static_cast<unsigned>(bit)) & 1u) ? ::phy_engine::model::digital_node_statement_t::true_state
                                                                      : ::phy_engine::model::digital_node_statement_t::false_state;
        };

        ::phy_engine::digital::need_operate_analog_node_t analog_drive{};
        for(int pin = 0; pin < 4; ++pin)
        {
            auto* nq = clip.pins[pin].nodes;
            if(nq == nullptr) { continue; }

            // pin0=q3, pin1=q2, pin2=q1, pin3=q0
            int const bit = 3 - pin;
            auto const out = q_state(bit);

            if(nq->num_of_analog_node == 0)
            {
                if(value_changed || unknown_changed || nq->node_information.dn.state != out)
                {
                    if(nq->node_information.dn.state != out)
                    {
                        nq->node_information.dn.state = out;
                        table.tables.insert(nq);
                    }
                }
            }
            else if(analog_drive.need_to_operate_analog_node == nullptr)
            {
                switch(out)
                {
                    case ::phy_engine::model::digital_node_statement_t::false_state: analog_drive = {clip.Ll, nq}; break;
                    case ::phy_engine::model::digital_node_statement_t::true_state: analog_drive = {clip.Hl, nq}; break;
                    default: analog_drive = {clip.Ll, nq}; break;
                }
            }
        }

        clip.last_value = clip.value;
        clip.last_unknown = clip.unknown;

        return analog_drive;
    }

    static_assert(::phy_engine::model::defines::can_update_digital_clk<COUNTER4>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<COUNTER4>, COUNTER4& clip) noexcept
    {
        return {clip.pins, 6};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<COUNTER4>);
}  // namespace phy_engine::model
