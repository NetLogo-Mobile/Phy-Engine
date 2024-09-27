#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../../circuits/digital/update_table.h"
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    struct INPUT
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"INPUT"};

        inline static constexpr ::phy_engine::model::digital_update_method_t digital_update_method{::phy_engine::model::digital_update_method_t::before_all_clk};

        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::digital};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"INPUT"};

        ::phy_engine::model::pin pins{{u8"o"}};

        // digital
        double Ll{0.0};  // low_level
        double Hl{5.0};  // high_level

        ::phy_engine::model::digital_node_statement_t outputA{};

        // private:
        ::phy_engine::model::digital_node_statement_t last_outputA{::phy_engine::model::digital_node_statement_t::X};
    };

    static_assert(::phy_engine::model::model<INPUT>);

    inline constexpr bool
        set_attribute_define(::phy_engine::model::model_reserve_type_t<INPUT>, INPUT& clip, ::std::size_t n, ::phy_engine::model::variant vi) noexcept
    {
        switch(n)
        {
            case 0:
            {
                if(vi.type != ::phy_engine::model::variant_type::digital) [[unlikely]] { return false; }
                clip.outputA = vi.digital;
                return true;
            }
            default:
            {
                return false;
            }
        }
        return false;
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<INPUT>);

    inline constexpr ::phy_engine::model::variant
        get_attribute_define(::phy_engine::model::model_reserve_type_t<INPUT>, INPUT const& clip, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                return {.digital{clip.outputA}, .type{::phy_engine::model::variant_type::digital}};
            }
            default:
            {
                return {};
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<INPUT>);

    inline constexpr ::fast_io::u8string_view
        get_attribute_name_define(::phy_engine::model::model_reserve_type_t<INPUT>, INPUT const& vac, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                return {u8"boolean"};
            }
            default:
            {
                return {};
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<INPUT>);

    inline constexpr ::phy_engine::digital::need_operate_analog_node_t update_digital_clk_define(::phy_engine::model::model_reserve_type_t<INPUT>,
                                                                                                 INPUT& clip,
                                                                                                 ::phy_engine::digital::digital_node_update_table& table,
                                                                                                 double tr_duration,
                                                                                                 ::phy_engine::model::digital_update_method_t method) noexcept
    {
        auto const node_o{clip.pins.nodes};

        if(node_o) [[likely]]
        {

            auto const output_res{clip.outputA};
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
                    default: ::std::unreachable();
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

    static_assert(::phy_engine::model::defines::can_update_digital_clk<INPUT>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<INPUT>, INPUT& clip) noexcept
    {
        return {__builtin_addressof(clip.pins), 1};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<INPUT>);

}  // namespace phy_engine::model
