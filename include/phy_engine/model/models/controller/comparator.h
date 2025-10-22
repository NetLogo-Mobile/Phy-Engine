#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    struct comparator
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"Comparator"};

        inline static constexpr ::phy_engine::model::digital_update_method_t digital_update_method{::phy_engine::model::digital_update_method_t::update_table};

        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::digital};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"CMP"};

        // A, B are analog inputs; o is output (digital/analog driven)
        ::phy_engine::model::pin pins[3]{{{u8"A"}}, {{u8"B"}}, {{u8"o"}}};

        double Ll{0.0};
        double Hl{5.0};
    };

    static_assert(::phy_engine::model::model<comparator>);

    inline constexpr bool
        set_attribute_define(::phy_engine::model::model_reserve_type_t<comparator>, comparator& c, ::std::size_t idx, ::phy_engine::model::variant vi) noexcept
    {
        switch(idx)
        {
            case 0:
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                c.Ll = vi.d;
                return true;
            case 1:
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                c.Hl = vi.d;
                return true;
            default: return false;
        }
        return false;
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<comparator>);

    inline constexpr ::phy_engine::model::variant
        get_attribute_define(::phy_engine::model::model_reserve_type_t<comparator>, comparator const& c, ::std::size_t idx) noexcept
    {
        switch(idx)
        {
            case 0: return {.d{c.Ll}, .type{::phy_engine::model::variant_type::d}};
            case 1: return {.d{c.Hl}, .type{::phy_engine::model::variant_type::d}};
            default: return {};
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<comparator>);

    inline constexpr ::fast_io::u8string_view get_attribute_name_define(::phy_engine::model::model_reserve_type_t<comparator>, ::std::size_t idx) noexcept
    {
        switch(idx)
        {
            case 0: return u8"Ll";
            case 1: return u8"Hl";
            default: return {};
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<comparator>);

    inline constexpr ::phy_engine::digital::need_operate_analog_node_t
        update_digital_clk_define(::phy_engine::model::model_reserve_type_t<comparator>,
                                  comparator& c,
                                  ::phy_engine::digital::digital_node_update_table& table,
                                  double /*tr_duration*/,
                                  ::phy_engine::model::digital_update_method_t /*method*/) noexcept
    {
        auto const node_A{c.pins[0].nodes};
        auto const node_B{c.pins[1].nodes};
        auto const node_o{c.pins[2].nodes};

        if(node_A && node_B && node_o) [[likely]]
        {
            double const vA{node_A->node_information.an.voltage.real()};
            double const vB{node_B->node_information.an.voltage.real()};

            if(node_o->num_of_analog_node != 0)
            {
                if(vA >= vB) { return {c.Hl, node_o}; }
                else
                {
                    return {c.Ll, node_o};
                }
            }
            else
            {
                node_o->node_information.dn.state =
                    vA >= vB ? ::phy_engine::model::digital_node_statement_t::true_state : ::phy_engine::model::digital_node_statement_t::false_state;
                table.tables.insert(node_o);
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::can_update_digital_clk<comparator>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<comparator>, comparator& c) noexcept
    {
        return {c.pins, 3};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<comparator>);
}  // namespace phy_engine::model

