#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../../circuits/digital/update_table.h"
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    struct OR
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"OR"};

        inline static constexpr ::phy_engine::model::digital_update_method_t digital_update_method{::phy_engine::model::digital_update_method_t::update_table};

        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::digital};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"OR"};

        ::phy_engine::model::pin pins[3]{{{u8"ia"}}, {{u8"ib"}}, {{u8"o"}}};

        // digital
        double Ll{0.0};    // low_level
        double Hl{5.0};    // high_level
        double Tsu{1e-9};  // unsteady_state_setup_time
        double Th{5e-10};  // unsteady_state_hold_time

        // private:

        ::phy_engine::model::digital_node_statement_t inputA{::phy_engine::model::digital_node_statement_t::X};
        ::phy_engine::model::digital_node_statement_t USRA{::phy_engine::model::digital_node_statement_t::X};
        double duration_A{};  // calculate unsteady state

        ::phy_engine::model::digital_node_statement_t inputB{::phy_engine::model::digital_node_statement_t::X};
        ::phy_engine::model::digital_node_statement_t USRB{::phy_engine::model::digital_node_statement_t::X};
        double duration_B{};  // calculate unsteady state

        ::phy_engine::model::digital_node_statement_t last_outputA{::phy_engine::model::digital_node_statement_t::X};
    };

    static_assert(::phy_engine::model::model<OR>);

    inline constexpr bool
        set_attribute_define(::phy_engine::model::model_reserve_type_t<OR>, OR& clip, ::std::size_t n, ::phy_engine::model::variant vi) noexcept
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
            case 2:
            {
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                clip.Tsu = vi.d;
                return true;
            }
            case 3:
            {
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                clip.Th = vi.d;
                return true;
            }
            default:
            {
                return false;
            }
        }
        return false;
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<OR>);

    inline constexpr ::phy_engine::model::variant get_attribute_define(::phy_engine::model::model_reserve_type_t<OR>, OR const& clip, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                return {.d{clip.Ll}, .type{::phy_engine::model::variant_type::d}};
            }
            case 1:
            {
                return {.d{clip.Hl}, .type{::phy_engine::model::variant_type::d}};
            }
            case 2:
            {
                return {.d{clip.Tsu}, .type{::phy_engine::model::variant_type::d}};
            }
            case 3:
            {
                return {.d{clip.Th}, .type{::phy_engine::model::variant_type::d}};
            }
            default:
            {
                return {};
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<OR>);

    inline constexpr ::fast_io::u8string_view get_attribute_name_define(::phy_engine::model::model_reserve_type_t<OR>, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                return {u8"Ll"};
            }
            case 1:
            {
                return {u8"Hl"};
            }
            case 2:
            {
                return {u8"Tsu"};
            }
            case 3:
            {
                return {u8"Th"};
            }
            default:
            {
                return {};
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<OR>);

    inline bool prepare_tr_define(::phy_engine::model::model_reserve_type_t<OR>, OR& clip) noexcept
    {
        clip.duration_A = 0.0;
        clip.inputA = ::phy_engine::model::digital_node_statement_t::indeterminate_state;
        clip.USRA = {};

        clip.duration_B = 0.0;
        clip.inputB = ::phy_engine::model::digital_node_statement_t::indeterminate_state;
        clip.USRB = {};

        return true;
    }

    static_assert(::phy_engine::model::defines::can_prepare_tr<OR>);

    inline constexpr ::phy_engine::digital::need_operate_analog_node_t update_digital_clk_define(::phy_engine::model::model_reserve_type_t<OR>,
                                                                                                 OR& clip,
                                                                                                 ::phy_engine::digital::digital_node_update_table& table,
                                                                                                 double tr_duration,
                                                                                                 ::phy_engine::model::digital_update_method_t method) noexcept
    {
        auto const node_ia{clip.pins[0].nodes};
        auto const node_ib{clip.pins[1].nodes};
        auto const node_o{clip.pins[2].nodes};

        if(node_ia && node_ib && node_o) [[likely]]
        {
            if(node_ia->num_of_analog_node != 0)  // analog
            {
                double const voltage{node_ia->node_information.an.voltage.real()};

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
            else { clip.inputA = node_ia->node_information.dn.state; }

            if(node_ib->num_of_analog_node != 0)  // analog
            {
                double const voltage{node_ib->node_information.an.voltage.real()};

                switch(clip.inputB)
                {
                    case ::phy_engine::model::digital_node_statement_t::false_state:
                    {
                        if(voltage >= clip.Hl)
                        {
                            if(clip.Tsu > 0.0)
                            {
                                clip.inputB = ::phy_engine::model::digital_node_statement_t::indeterminate_state;
                                clip.USRB = ::phy_engine::model::digital_node_statement_t::true_state;
                                clip.duration_B = tr_duration;
                            }
                            else { clip.inputB = ::phy_engine::model::digital_node_statement_t::true_state; }
                        }
                        break;
                    }
                    case ::phy_engine::model::digital_node_statement_t::true_state:
                    {
                        if(voltage <= clip.Ll)
                        {
                            if(clip.Th > 0.0)
                            {
                                clip.inputB = ::phy_engine::model::digital_node_statement_t::indeterminate_state;
                                clip.USRB = ::phy_engine::model::digital_node_statement_t::false_state;
                                clip.duration_B = tr_duration;
                            }
                            else { clip.inputB = ::phy_engine::model::digital_node_statement_t::false_state; }
                        }
                        break;
                    }
                    case ::phy_engine::model::digital_node_statement_t::indeterminate_state:
                    {
                        switch(clip.USRB)
                        {
                            case ::phy_engine::model::digital_node_statement_t::false_state:
                            {
                                if(voltage <= clip.Ll)
                                {
                                    if(tr_duration - clip.duration_B >= clip.Tsu) { clip.inputB = ::phy_engine::model::digital_node_statement_t::false_state; }
                                }
                                else { clip.inputB = ::phy_engine::model::digital_node_statement_t::true_state; }
                                break;
                            }
                            case ::phy_engine::model::digital_node_statement_t::true_state:
                            {
                                if(voltage >= clip.Hl)
                                {
                                    if(tr_duration - clip.duration_B >= clip.Th) { clip.inputB = ::phy_engine::model::digital_node_statement_t::true_state; }
                                }
                                else { clip.inputB = ::phy_engine::model::digital_node_statement_t::false_state; }
                                break;
                            }
                            case ::phy_engine::model::digital_node_statement_t::indeterminate_state:
                            {
                                if(voltage >= clip.Hl)
                                {
                                    if(tr_duration - clip.duration_B >= clip.Th) { clip.inputB = ::phy_engine::model::digital_node_statement_t::true_state; }
                                }
                                else if(voltage <= clip.Ll)
                                {
                                    if(tr_duration - clip.duration_B >= clip.Tsu) { clip.inputB = ::phy_engine::model::digital_node_statement_t::false_state; }
                                }
                                else { clip.duration_B = tr_duration; }

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
            else { clip.inputB = node_ib->node_information.dn.state; }

            auto const output_res{clip.inputA | clip.inputB};
            bool output_change{};
            if(clip.last_outputA != output_res)
            {
                output_change = true;
                clip.last_outputA = output_res;
            }

            if(node_o->num_of_analog_node != 0)  // analog
            {
                switch(output_res)
                {
                    case ::phy_engine::model::digital_node_statement_t::false_state:
                    {
                        return {clip.Ll, node_o};
                    }
                    case ::phy_engine::model::digital_node_statement_t::true_state:
                    {
                        return {clip.Hl, node_o};
                    }
                    case ::phy_engine::model::digital_node_statement_t::indeterminate_state:
                    {
                        // UB
                        return {clip.Ll, node_o};
                    }
                    case ::phy_engine::model::digital_node_statement_t::high_impedence_state: break;
                    default: ::fast_io::unreachable();
                }
            }
            else
            {
                node_o->node_information.dn.state = output_res;
                if(output_change) { table.tables.insert(node_o); }
            }
        }

        return {};
    }

    static_assert(::phy_engine::model::defines::can_update_digital_clk<OR>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<OR>, OR& clip) noexcept
    {
        return {clip.pins, 3};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<OR>);

}  // namespace phy_engine::model
