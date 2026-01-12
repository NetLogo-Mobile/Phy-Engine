#pragma once

#include <cstdint>

#include <fast_io/fast_io_dsal/string_view.h>

#include "../../../../circuits/digital/update_table.h"
#include "../../../model_refs/base.h"

namespace phy_engine::model
{
    struct RANDOM_GENERATOR4
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"RANDOM_GENERATOR4"};
        inline static constexpr ::phy_engine::model::digital_update_method_t digital_update_method{::phy_engine::model::digital_update_method_t::update_table};
        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::digital};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"RANDOM_GENERATOR4"};

        // pins:
        //   0..3: q3..q0 (MSB..LSB)
        //   4: clk
        //   5: seed (Z => treated as H)
        ::phy_engine::model::pin pins[6]{{{u8"q3"}}, {{u8"q2"}}, {{u8"q1"}}, {{u8"q0"}}, {{u8"clk"}}, {{u8"seed"}}};

        double Ll{0.0};
        double Hl{5.0};

        // attributes/state
        ::std::uint8_t state{1u};  // 4-bit shift register state; default non-zero
        bool unknown{};

        // private
        ::phy_engine::model::digital_node_statement_t last_clk{::phy_engine::model::digital_node_statement_t::false_state};
        ::std::uint8_t last_state{0xFF};
        bool last_unknown{true};
    };

    static_assert(::phy_engine::model::model<RANDOM_GENERATOR4>);

    inline constexpr bool set_attribute_define(::phy_engine::model::model_reserve_type_t<RANDOM_GENERATOR4>,
                                               RANDOM_GENERATOR4& clip,
                                               ::std::size_t n,
                                               ::phy_engine::model::variant vi) noexcept
    {
        switch(n)
        {
            case 0:
            {
                if(vi.type != ::phy_engine::model::variant_type::ui8) [[unlikely]] { return false; }
                clip.state = static_cast<::std::uint8_t>(vi.ui8 & 0x0F);
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

    static_assert(::phy_engine::model::defines::has_set_attribute<RANDOM_GENERATOR4>);

    inline constexpr ::phy_engine::model::variant
        get_attribute_define(::phy_engine::model::model_reserve_type_t<RANDOM_GENERATOR4>, RANDOM_GENERATOR4 const& clip, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0: return {.ui8{static_cast<::std::uint_least8_t>(clip.state & 0x0F)}, .type{::phy_engine::model::variant_type::ui8}};
            case 1: return {.boolean{clip.unknown}, .type{::phy_engine::model::variant_type::boolean}};
            default: return {};
        }
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<RANDOM_GENERATOR4>);

    inline constexpr ::fast_io::u8string_view get_attribute_name_define(::phy_engine::model::model_reserve_type_t<RANDOM_GENERATOR4>, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0: return {u8"state"};
            case 1: return {u8"unknown"};
            default: return {};
        }
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<RANDOM_GENERATOR4>);

    inline constexpr ::phy_engine::digital::need_operate_analog_node_t
        update_digital_clk_define(::phy_engine::model::model_reserve_type_t<RANDOM_GENERATOR4>,
                                  RANDOM_GENERATOR4& clip,
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
        auto const n_seed{clip.pins[5].nodes};
        if(n_clk == nullptr) { return {}; }

        auto const clk{read_dn_logic(n_clk)};
        auto seed = (n_seed == nullptr) ? ::phy_engine::model::digital_node_statement_t::high_impedence_state
                                        : (n_seed->num_of_analog_node == 0 ? n_seed->node_information.dn.state : read_dn_logic(n_seed));
        if(seed == ::phy_engine::model::digital_node_statement_t::high_impedence_state) { seed = ::phy_engine::model::digital_node_statement_t::true_state; }

        if(clip.last_clk == ::phy_engine::model::digital_node_statement_t::false_state && clk == ::phy_engine::model::digital_node_statement_t::true_state)
        {
            if(seed == ::phy_engine::model::digital_node_statement_t::indeterminate_state)
            {
                clip.unknown = true;
            }
            else
            {
                if(!clip.unknown)
                {
                    bool const seed_bit = (seed == ::phy_engine::model::digital_node_statement_t::true_state);
                    bool const b3 = ((clip.state >> 3u) & 1u) != 0u;
                    bool const b2 = ((clip.state >> 2u) & 1u) != 0u;
                    bool const feedback = (b3 ^ b2) ^ seed_bit;
                    clip.state = static_cast<::std::uint8_t>(((clip.state << 1u) & 0x0E) | static_cast<::std::uint8_t>(feedback));
                }
            }
        }

        if(clk == ::phy_engine::model::digital_node_statement_t::false_state || clk == ::phy_engine::model::digital_node_statement_t::true_state) { clip.last_clk = clk; }

        bool const changed = (clip.last_state != clip.state) || (clip.last_unknown != clip.unknown);

        auto q_state = [&](int bit) constexpr noexcept -> ::phy_engine::model::digital_node_statement_t
        {
            if(clip.unknown) { return ::phy_engine::model::digital_node_statement_t::indeterminate_state; }
            return ((clip.state >> static_cast<unsigned>(bit)) & 1u) ? ::phy_engine::model::digital_node_statement_t::true_state
                                                                      : ::phy_engine::model::digital_node_statement_t::false_state;
        };

        ::phy_engine::digital::need_operate_analog_node_t analog_drive{};
        for(int pin = 0; pin < 4; ++pin)
        {
            auto* nq = clip.pins[pin].nodes;
            if(nq == nullptr) { continue; }
            int const bit = 3 - pin;
            auto const out = q_state(bit);

            if(nq->num_of_analog_node == 0)
            {
                if(changed || nq->node_information.dn.state != out)
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

        clip.last_state = clip.state;
        clip.last_unknown = clip.unknown;

        return analog_drive;
    }

    static_assert(::phy_engine::model::defines::can_update_digital_clk<RANDOM_GENERATOR4>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<RANDOM_GENERATOR4>, RANDOM_GENERATOR4& clip) noexcept
    {
        return {clip.pins, 6};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<RANDOM_GENERATOR4>);
}  // namespace phy_engine::model
