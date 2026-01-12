#pragma once

#include <cstddef>

#include <fast_io/fast_io_dsal/string_view.h>
#include <fast_io/fast_io_dsal/vector.h>

#include "../../../../circuits/digital/update_table.h"
#include "../../../model_refs/base.h"

namespace phy_engine::model
{
    // A 1-bit tick-based delay line for digital nodes.
    // Each `digital_clk()` outputs the value from `ticks` cycles ago, then shifts in the current input.
    struct TICK_DELAY
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"TICK_DELAY"};
        inline static constexpr ::phy_engine::model::digital_update_method_t digital_update_method{::phy_engine::model::digital_update_method_t::before_all_clk};
        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::digital};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"TICK_DELAY"};

        ::phy_engine::model::pin pins[2]{{{u8"i"}}, {{u8"o"}}};

        // digital
        double Ll{0.0};
        double Hl{5.0};

        ::std::size_t ticks{1};

        // private
        ::fast_io::vector<::phy_engine::model::digital_node_statement_t> pipe{};
        ::phy_engine::model::digital_node_statement_t last_out{::phy_engine::model::digital_node_statement_t::X};

        TICK_DELAY() = default;
        explicit TICK_DELAY(::std::size_t t) : ticks{t} {}
    };

    static_assert(::phy_engine::model::model<TICK_DELAY>);

    inline constexpr bool set_attribute_define(::phy_engine::model::model_reserve_type_t<TICK_DELAY>,
                                               TICK_DELAY& /*m*/,
                                               ::std::size_t /*index*/,
                                               ::phy_engine::model::variant /*vi*/) noexcept
    {
        return false;
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<TICK_DELAY>);

    inline constexpr ::phy_engine::model::variant get_attribute_define(::phy_engine::model::model_reserve_type_t<TICK_DELAY>,
                                                                       TICK_DELAY const& /*m*/,
                                                                       ::std::size_t /*index*/) noexcept
    {
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<TICK_DELAY>);

    inline constexpr ::fast_io::u8string_view get_attribute_name_define(::phy_engine::model::model_reserve_type_t<TICK_DELAY>, ::std::size_t /*index*/) noexcept
    {
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<TICK_DELAY>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<TICK_DELAY>, TICK_DELAY& m) noexcept
    {
        return {m.pins, 2};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<TICK_DELAY>);

    inline constexpr ::phy_engine::digital::need_operate_analog_node_t update_digital_clk_define(::phy_engine::model::model_reserve_type_t<TICK_DELAY>,
                                                                                                 TICK_DELAY& m,
                                                                                                 ::phy_engine::digital::digital_node_update_table& table,
                                                                                                 double /*tr_duration*/,
                                                                                                 ::phy_engine::model::digital_update_method_t method) noexcept
    {
        // This model is stateful; only advance once per `digital_clk()` cycle.
        // The engine may call digital models again during the update-table phase; ignore those calls.
        if(method != ::phy_engine::model::digital_update_method_t::before_all_clk) { return {}; }

        auto read_dn = [&](::phy_engine::model::node_t* n) constexpr noexcept -> ::phy_engine::model::digital_node_statement_t
        {
            if(n == nullptr) { return ::phy_engine::model::digital_node_statement_t::indeterminate_state; }
            if(n->num_of_analog_node == 0) { return n->node_information.dn.state; }
            double const v{n->node_information.an.voltage.real()};
            if(v >= m.Hl) { return ::phy_engine::model::digital_node_statement_t::true_state; }
            if(v <= m.Ll) { return ::phy_engine::model::digital_node_statement_t::false_state; }
            return ::phy_engine::model::digital_node_statement_t::indeterminate_state;
        };

        auto const node_i{m.pins[0].nodes};
        auto const node_o{m.pins[1].nodes};
        if(node_o == nullptr) { return {}; }

        auto const in = read_dn(node_i);

        auto const n_ticks = m.ticks;
        if(n_ticks == 0)
        {
            // passthrough
            if(node_o->num_of_analog_node == 0)
            {
                if(m.last_out != in)
                {
                    m.last_out = in;
                    node_o->node_information.dn.state = in;
                    table.tables.insert(node_o);
                }
                else
                {
                    node_o->node_information.dn.state = in;
                }
                return {};
            }

            if(in == ::phy_engine::model::digital_node_statement_t::false_state) { return {m.Ll, node_o}; }
            if(in == ::phy_engine::model::digital_node_statement_t::true_state) { return {m.Hl, node_o}; }
            return {};
        }

        if(m.pipe.size() != n_ticks)
        {
            m.pipe.clear();
            // Assume the input was stable before simulation starts (common for `#delay` usage),
            // so pre-fill the delay line with the current input instead of X.
            m.pipe.resize(n_ticks, in);
            m.last_out = ::phy_engine::model::digital_node_statement_t::indeterminate_state;
        }

        // Output the value from `ticks` cycles ago (before shifting this cycle).
        auto const out = m.pipe[n_ticks - 1];

        // Shift register: pipe[0] is newest (after shifting).
        if(n_ticks > 1)
        {
            for(::std::size_t i{n_ticks - 1}; i > 0; --i) { m.pipe[i] = m.pipe[i - 1]; }
        }
        m.pipe[0] = in;

        if(node_o->num_of_analog_node == 0)
        {
            if(m.last_out != out)
            {
                m.last_out = out;
                node_o->node_information.dn.state = out;
                table.tables.insert(node_o);
            }
            else
            {
                node_o->node_information.dn.state = out;
            }
            return {};
        }

        if(out == ::phy_engine::model::digital_node_statement_t::false_state) { return {m.Ll, node_o}; }
        if(out == ::phy_engine::model::digital_node_statement_t::true_state) { return {m.Hl, node_o}; }
        return {};
    }

    static_assert(::phy_engine::model::defines::can_update_digital_clk<TICK_DELAY>);
}  // namespace phy_engine::model
