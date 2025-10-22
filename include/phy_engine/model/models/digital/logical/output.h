#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../../circuits/digital/update_table.h"
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    struct OUTPUT
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"OUTPUT"};

        inline static constexpr ::phy_engine::model::digital_update_method_t digital_update_method{::phy_engine::model::digital_update_method_t::update_table};

        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::digital};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"OUTPUT"};

        ::phy_engine::model::pin pins{{u8"i"}};

        // digital
        double Ll{0.0};    // low_level
        double Hl{5.0};    // high_level
        double Tsu{1e-9};  // unsteady_state_setup_time
        double Th{5e-10};  // unsteady_state_hold_time

        // private:

        ::phy_engine::model::digital_node_statement_t inputA{::phy_engine::model::digital_node_statement_t::X};
        ::phy_engine::model::digital_node_statement_t USRA{::phy_engine::model::digital_node_statement_t::X};
        double duration_A{};  // calculate unsteady state
    };

    static_assert(::phy_engine::model::model<OUTPUT>);

    inline constexpr bool
        set_attribute_define(::phy_engine::model::model_reserve_type_t<OUTPUT>, OUTPUT& clip, ::std::size_t n, ::phy_engine::model::variant vi) noexcept
    {
        return false;
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<OUTPUT>);

    inline constexpr ::phy_engine::model::variant
        get_attribute_define(::phy_engine::model::model_reserve_type_t<OUTPUT>, OUTPUT const& clip, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                return {.digital{clip.inputA}, .type{::phy_engine::model::variant_type::digital}};
            }
            default:
            {
                return {};
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<OUTPUT>);

    inline constexpr ::fast_io::u8string_view
        get_attribute_name_define(::phy_engine::model::model_reserve_type_t<OUTPUT>, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                return {u8"value"};
            }
            default:
            {
                return {};
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<OUTPUT>);

    inline ::phy_engine::digital::need_operate_analog_node_t update_digital_clk_define(::phy_engine::model::model_reserve_type_t<OUTPUT>,
                                                                                       OUTPUT& clip,
                                                                                       ::phy_engine::digital::digital_node_update_table& table,
                                                                                       double tr_duration,
                                                                                       ::phy_engine::model::digital_update_method_t method) noexcept
    {
        auto const node_i{clip.pins.nodes};

        if(node_i->num_of_analog_node != 0)  // analog
        {
            double const voltage{node_i->node_information.an.voltage.real()};

            switch(clip.inputA)
            {
                case ::phy_engine::model::digital_node_statement_t::false_state:
                {
                    if(voltage >= clip.Hl)
                    {
                        if(clip.Tsu > 0.0)
                        {
                            clip.inputA = ::phy_engine::model::digital_node_statement_t::indeterminate_state;
                            clip.USRA = ::phy_engine::model::digital_node_statement_t::true_state;
                            clip.duration_A = tr_duration;
                        }
                        else { clip.inputA = ::phy_engine::model::digital_node_statement_t::true_state; }
                    }
                    break;
                }
                case ::phy_engine::model::digital_node_statement_t::true_state:
                {
                    if(voltage <= clip.Ll)
                    {
                        if(clip.Th > 0.0)
                        {
                            clip.inputA = ::phy_engine::model::digital_node_statement_t::indeterminate_state;
                            clip.USRA = ::phy_engine::model::digital_node_statement_t::false_state;
                            clip.duration_A = tr_duration;
                        }
                        else { clip.inputA = ::phy_engine::model::digital_node_statement_t::false_state; }
                    }
                    break;
                }
                case ::phy_engine::model::digital_node_statement_t::indeterminate_state:
                {
                    switch(clip.USRA)
                    {
                        case ::phy_engine::model::digital_node_statement_t::false_state:
                        {
                            if(voltage <= clip.Ll)
                            {
                                if(tr_duration - clip.duration_A >= clip.Tsu) { clip.inputA = ::phy_engine::model::digital_node_statement_t::false_state; }
                            }
                            else { clip.inputA = ::phy_engine::model::digital_node_statement_t::true_state; }
                            break;
                        }
                        case ::phy_engine::model::digital_node_statement_t::true_state:
                        {
                            if(voltage >= clip.Hl)
                            {
                                if(tr_duration - clip.duration_A >= clip.Th) { clip.inputA = ::phy_engine::model::digital_node_statement_t::true_state; }
                            }
                            else { clip.inputA = ::phy_engine::model::digital_node_statement_t::false_state; }
                            break;
                        }
                        case ::phy_engine::model::digital_node_statement_t::indeterminate_state:
                        {
                            if(voltage >= clip.Hl)
                            {
                                if(tr_duration - clip.duration_A >= clip.Th) { clip.inputA = ::phy_engine::model::digital_node_statement_t::true_state; }
                            }
                            else if(voltage <= clip.Ll)
                            {
                                if(tr_duration - clip.duration_A >= clip.Tsu) { clip.inputA = ::phy_engine::model::digital_node_statement_t::false_state; }
                            }
                            else { clip.duration_A = tr_duration; }

                            break;
                        }
                        default: ::fast_io::unreachable();
                    }

                    break;
                }
                case ::phy_engine::model::digital_node_statement_t::high_impedence_state: break;
                default: ::fast_io::unreachable();
            }
        }
        else { clip.inputA = node_i->node_information.dn.state; }

        return {};
    }

    static_assert(::phy_engine::model::defines::can_update_digital_clk<OUTPUT>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<OUTPUT>, OUTPUT& clip) noexcept
    {
        return {__builtin_addressof(clip.pins), 1};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<OUTPUT>);

}  // namespace phy_engine::model
