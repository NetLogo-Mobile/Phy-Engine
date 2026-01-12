#pragma once

#include <fast_io/fast_io_dsal/string_view.h>

#include "../../../../circuits/digital/update_table.h"
#include "../../../model_refs/base.h"

namespace phy_engine::model
{
    struct SCHMITT_TRIGGER
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"SCHMITT_TRIGGER"};
        inline static constexpr ::phy_engine::model::digital_update_method_t digital_update_method{::phy_engine::model::digital_update_method_t::update_table};
        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::digital};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"SCHMITT_TRIGGER"};

        ::phy_engine::model::pin pins[2]{{{u8"i"}}, {{u8"o"}}};

        // Drive levels for analog output nodes
        double Ll{0.0};
        double Hl{5.0};

        // Hysteresis thresholds (in volts, compared against input voltage when input is analog).
        double Vth_low{1.6666666666666666666666};
        double Vth_high{3.333333333333333333333};

        bool inverted{};

        // private
        ::phy_engine::model::digital_node_statement_t last_out{::phy_engine::model::digital_node_statement_t::false_state};
        ::phy_engine::model::digital_node_statement_t last_driven{::phy_engine::model::digital_node_statement_t::X};
    };

    static_assert(::phy_engine::model::model<SCHMITT_TRIGGER>);

    inline constexpr bool
        set_attribute_define(::phy_engine::model::model_reserve_type_t<SCHMITT_TRIGGER>, SCHMITT_TRIGGER& clip, ::std::size_t n, ::phy_engine::model::variant vi) noexcept
    {
        switch(n)
        {
            case 0:
            {
                if(vi.type != ::phy_engine::model::variant_type::boolean) [[unlikely]] { return false; }
                clip.inverted = vi.boolean;
                return true;
            }
            case 1:
            {
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                clip.Vth_low = vi.d;
                return true;
            }
            case 2:
            {
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                clip.Vth_high = vi.d;
                return true;
            }
            default: return false;
        }
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<SCHMITT_TRIGGER>);

    inline constexpr ::phy_engine::model::variant
        get_attribute_define(::phy_engine::model::model_reserve_type_t<SCHMITT_TRIGGER>, SCHMITT_TRIGGER const& clip, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0: return {.boolean{clip.inverted}, .type{::phy_engine::model::variant_type::boolean}};
            case 1: return {.d{clip.Vth_low}, .type{::phy_engine::model::variant_type::d}};
            case 2: return {.d{clip.Vth_high}, .type{::phy_engine::model::variant_type::d}};
            case 3: return {.digital{clip.last_out}, .type{::phy_engine::model::variant_type::digital}};
            default: return {};
        }
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<SCHMITT_TRIGGER>);

    inline constexpr ::fast_io::u8string_view get_attribute_name_define(::phy_engine::model::model_reserve_type_t<SCHMITT_TRIGGER>, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0: return {u8"inverted"};
            case 1: return {u8"Vth_low"};
            case 2: return {u8"Vth_high"};
            case 3: return {u8"out"};
            default: return {};
        }
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<SCHMITT_TRIGGER>);

    inline constexpr ::phy_engine::digital::need_operate_analog_node_t update_digital_clk_define(::phy_engine::model::model_reserve_type_t<SCHMITT_TRIGGER>,
                                                                                                 SCHMITT_TRIGGER& clip,
                                                                                                 ::phy_engine::digital::digital_node_update_table& table,
                                                                                                 double /*tr_duration*/,
                                                                                                 ::phy_engine::model::digital_update_method_t /*method*/) noexcept
    {
        auto const n_i{clip.pins[0].nodes};
        auto const n_o{clip.pins[1].nodes};
        if(n_i == nullptr || n_o == nullptr) { return {}; }

        auto decide_from_digital = [&](::phy_engine::model::digital_node_statement_t st) constexpr noexcept
        {
            switch(st)
            {
                case ::phy_engine::model::digital_node_statement_t::false_state: clip.last_out = ::phy_engine::model::digital_node_statement_t::false_state; break;
                case ::phy_engine::model::digital_node_statement_t::true_state: clip.last_out = ::phy_engine::model::digital_node_statement_t::true_state; break;
                case ::phy_engine::model::digital_node_statement_t::high_impedence_state:
                case ::phy_engine::model::digital_node_statement_t::indeterminate_state:
                default: clip.last_out = ::phy_engine::model::digital_node_statement_t::indeterminate_state; break;
            }
        };

        if(n_i->num_of_analog_node == 0)
        {
            decide_from_digital(n_i->node_information.dn.state);
        }
        else
        {
            double const v{n_i->node_information.an.voltage.real()};
            switch(clip.last_out)
            {
                case ::phy_engine::model::digital_node_statement_t::false_state:
                {
                    if(v >= clip.Vth_high) { clip.last_out = ::phy_engine::model::digital_node_statement_t::true_state; }
                    break;
                }
                case ::phy_engine::model::digital_node_statement_t::true_state:
                {
                    if(v <= clip.Vth_low) { clip.last_out = ::phy_engine::model::digital_node_statement_t::false_state; }
                    break;
                }
                case ::phy_engine::model::digital_node_statement_t::indeterminate_state:
                case ::phy_engine::model::digital_node_statement_t::high_impedence_state:
                default:
                {
                    if(v >= clip.Vth_high) { clip.last_out = ::phy_engine::model::digital_node_statement_t::true_state; }
                    else if(v <= clip.Vth_low)
                    {
                        clip.last_out = ::phy_engine::model::digital_node_statement_t::false_state;
                    }
                    break;
                }
            }
        }

        auto driven = clip.last_out;
        if(clip.inverted)
        {
            if(driven == ::phy_engine::model::digital_node_statement_t::false_state) { driven = ::phy_engine::model::digital_node_statement_t::true_state; }
            else if(driven == ::phy_engine::model::digital_node_statement_t::true_state)
            {
                driven = ::phy_engine::model::digital_node_statement_t::false_state;
            }
        }

        if(n_o->num_of_analog_node == 0)
        {
            if(n_o->node_information.dn.state != driven)
            {
                n_o->node_information.dn.state = driven;
                table.tables.insert(n_o);
            }
            clip.last_driven = driven;
            return {};
        }

        if(clip.last_driven == driven) { return {}; }
        clip.last_driven = driven;
        switch(driven)
        {
            case ::phy_engine::model::digital_node_statement_t::false_state: return {clip.Ll, n_o};
            case ::phy_engine::model::digital_node_statement_t::true_state: return {clip.Hl, n_o};
            default: return {clip.Ll, n_o};
        }
    }

    static_assert(::phy_engine::model::defines::can_update_digital_clk<SCHMITT_TRIGGER>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<SCHMITT_TRIGGER>, SCHMITT_TRIGGER& clip) noexcept
    {
        return {clip.pins, 2};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<SCHMITT_TRIGGER>);
}  // namespace phy_engine::model

