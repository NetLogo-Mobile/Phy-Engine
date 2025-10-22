#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../../circuits/digital/update_table.h"
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    struct TRI
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"TRI"};

        inline static constexpr ::phy_engine::model::digital_update_method_t digital_update_method{::phy_engine::model::digital_update_method_t::update_table};

        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::digital};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"TRI"};

        ::phy_engine::model::pin pins[3]{{{u8"i"}}, {{u8"en"}}, {{u8"o"}}};

        double Ll{0.0};
        double Hl{5.0};
    };

    static_assert(::phy_engine::model::model<TRI>);

    inline constexpr bool
        set_attribute_define(::phy_engine::model::model_reserve_type_t<TRI>, TRI& t, ::std::size_t idx, ::phy_engine::model::variant vi) noexcept
    {
        switch(idx)
        {
            case 0:
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                t.Ll = vi.d;
                return true;
            case 1:
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                t.Hl = vi.d;
                return true;
            default: return false;
        }
        return false;
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<TRI>);

    inline constexpr ::phy_engine::model::variant get_attribute_define(::phy_engine::model::model_reserve_type_t<TRI>, TRI const& t, ::std::size_t idx) noexcept
    {
        switch(idx)
        {
            case 0: return {.d{t.Ll}, .type{::phy_engine::model::variant_type::d}};
            case 1: return {.d{t.Hl}, .type{::phy_engine::model::variant_type::d}};
            default: return {};
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<TRI>);

    inline constexpr ::fast_io::u8string_view get_attribute_name_define(::phy_engine::model::model_reserve_type_t<TRI>, ::std::size_t idx) noexcept
    {
        switch(idx)
        {
            case 0: return u8"Ll";
            case 1: return u8"Hl";
            default: return {};
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<TRI>);

    inline constexpr ::phy_engine::digital::need_operate_analog_node_t
        update_digital_clk_define(::phy_engine::model::model_reserve_type_t<TRI>,
                                  TRI& t,
                                  ::phy_engine::digital::digital_node_update_table& table,
                                  double /*tr_duration*/,
                                  ::phy_engine::model::digital_update_method_t /*method*/) noexcept
    {
        auto const node_i{t.pins[0].nodes};
        auto const node_en{t.pins[1].nodes};
        auto const node_o{t.pins[2].nodes};

        if(node_i && node_en && node_o) [[likely]]
        {
            bool enabled{};
            if(node_en->num_of_analog_node != 0) { enabled = node_en->node_information.an.voltage.real() >= t.Hl; }
            else
            {
                enabled = node_en->node_information.dn.state == ::phy_engine::model::digital_node_statement_t::true_state;
            }

            if(!enabled)
            {
                if(node_o->num_of_analog_node == 0)
                {
                    node_o->node_information.dn.state = ::phy_engine::model::digital_node_statement_t::high_impedence_state;
                    table.tables.insert(node_o);
                }
                return {};
            }

            ::phy_engine::model::digital_node_statement_t inputState{};
            if(node_i->num_of_analog_node != 0)
            {
                double const v{node_i->node_information.an.voltage.real()};
                if(v >= t.Hl) { inputState = ::phy_engine::model::digital_node_statement_t::true_state; }
                else if(v <= t.Ll) { inputState = ::phy_engine::model::digital_node_statement_t::false_state; }
                else
                {
                    inputState = ::phy_engine::model::digital_node_statement_t::indeterminate_state;
                }
            }
            else
            {
                inputState = node_i->node_information.dn.state;
            }

            if(node_o->num_of_analog_node != 0)
            {
                switch(inputState)
                {
                    case ::phy_engine::model::digital_node_statement_t::true_state: return {t.Hl, node_o};
                    case ::phy_engine::model::digital_node_statement_t::false_state: return {t.Ll, node_o};
                    default: return {t.Ll, node_o};
                }
            }
            else
            {
                node_o->node_information.dn.state = inputState;
                table.tables.insert(node_o);
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::can_update_digital_clk<TRI>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<TRI>, TRI& t) noexcept
    {
        return {t.pins, 3};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<TRI>);
}  // namespace phy_engine::model

