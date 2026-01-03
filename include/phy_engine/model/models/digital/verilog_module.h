#pragma once

#include <utility>
#include <memory>

#include <fast_io/fast_io_dsal/string_view.h>
#include <fast_io/fast_io_dsal/string.h>
#include <fast_io/fast_io_dsal/vector.h>

#include "../../../circuits/digital/update_table.h"
#include "../../model_refs/base.h"
#include "../../../verilog/digital/digital.h"

namespace phy_engine::model
{
    struct VERILOG_MODULE
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"VERILOG_MODULE"};
        inline static constexpr ::phy_engine::model::digital_update_method_t digital_update_method{::phy_engine::model::digital_update_method_t::before_all_clk};
        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::digital};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"VERILOG"};

        // Analog I/O thresholds (used when a port is connected to an analog/hybrid node).
        double Ll{0.0};
        double Hl{5.0};

        // Source and top module name (for debug/introspection; not interpreted by the engine after construction).
        ::fast_io::u8string source{};
        ::fast_io::u8string top{};

        // Compiled representation and runtime state (supports hierarchy/instances).
        ::std::shared_ptr<::phy_engine::verilog::digital::compiled_design> design{};
        ::phy_engine::verilog::digital::instance_state top_instance{};
        ::std::uint64_t tick{};

        // Ports are exposed as pins in the same order as the Verilog module port list.
        ::fast_io::vector<::fast_io::u8string> pin_name_storage{};
        ::fast_io::vector<::phy_engine::model::pin> pins{};

        // Round-robin emitter for analog drives (because the digital API returns one drive per call).
        ::std::size_t analog_emit_cursor{};
    };

    static_assert(::phy_engine::model::model<VERILOG_MODULE>);

    inline constexpr bool set_attribute_define(::phy_engine::model::model_reserve_type_t<VERILOG_MODULE>,
                                               VERILOG_MODULE& m,
                                               ::std::size_t idx,
                                               ::phy_engine::model::variant vi) noexcept
    {
        switch(idx)
        {
            case 0:
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                m.Ll = vi.d;
                return true;
            case 1:
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                m.Hl = vi.d;
                return true;
            default: return false;
        }
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<VERILOG_MODULE>);

    inline constexpr ::phy_engine::model::variant get_attribute_define(::phy_engine::model::model_reserve_type_t<VERILOG_MODULE>,
                                                                       VERILOG_MODULE const& m,
                                                                       ::std::size_t idx) noexcept
    {
        switch(idx)
        {
            case 0: return {.d{m.Ll}, .type{::phy_engine::model::variant_type::d}};
            case 1: return {.d{m.Hl}, .type{::phy_engine::model::variant_type::d}};
            default: return {};
        }
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<VERILOG_MODULE>);

    inline constexpr ::fast_io::u8string_view get_attribute_name_define(::phy_engine::model::model_reserve_type_t<VERILOG_MODULE>,
                                                                        ::std::size_t idx) noexcept
    {
        switch(idx)
        {
            case 0: return u8"Ll";
            case 1: return u8"Hl";
            default: return {};
        }
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<VERILOG_MODULE>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<VERILOG_MODULE>, VERILOG_MODULE& m) noexcept
    {
        return {m.pins.data(), m.pins.size()};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<VERILOG_MODULE>);

    inline constexpr ::phy_engine::digital::need_operate_analog_node_t update_digital_clk_define(::phy_engine::model::model_reserve_type_t<VERILOG_MODULE>,
                                                                                                 VERILOG_MODULE& m,
                                                                                                 ::phy_engine::digital::digital_node_update_table& table,
                                                                                                 double /*tr_duration*/,
                                                                                                 ::phy_engine::model::digital_update_method_t method) noexcept
    {
        if(m.design == nullptr || m.top_instance.mod == nullptr) [[unlikely]] { return {}; }
        auto const& cm{*m.top_instance.mod};

        auto read_dn = [&](::phy_engine::model::node_t* n) noexcept -> ::phy_engine::verilog::digital::logic_t
        {
            if(n == nullptr) { return ::phy_engine::verilog::digital::logic_t::indeterminate_state; }
            if(n->num_of_analog_node == 0)
            {
                return n->node_information.dn.state;
            }
            double const v{n->node_information.an.voltage.real()};
            if(v >= m.Hl) { return ::phy_engine::verilog::digital::logic_t::true_state; }
            if(v <= m.Ll) { return ::phy_engine::verilog::digital::logic_t::false_state; }
            return ::phy_engine::verilog::digital::logic_t::indeterminate_state;
        };

        if(method == ::phy_engine::model::digital_update_method_t::before_all_clk) { ++m.tick; }

        // Sample input ports into internal signal table.
        for(::std::size_t i{}; i < cm.ports.size() && i < m.pins.size(); ++i)
        {
            auto const& p{cm.ports.index_unchecked(i)};
            if(p.dir == ::phy_engine::verilog::digital::port_dir::input || p.dir == ::phy_engine::verilog::digital::port_dir::inout ||
               p.dir == ::phy_engine::verilog::digital::port_dir::unknown)
            {
                auto const* node{m.pins.index_unchecked(i).nodes};
                if(node == nullptr) { continue; }
                if(p.signal < m.top_instance.state.values.size())
                {
                    m.top_instance.state.values.index_unchecked(p.signal) = read_dn(const_cast<::phy_engine::model::node_t*>(node));
                }
            }
        }

        bool const process_sequential{method == ::phy_engine::model::digital_update_method_t::before_all_clk};
        ::phy_engine::verilog::digital::simulate(m.top_instance, m.tick, process_sequential);

        // Drive output ports.
        for(::std::size_t i{}; i < cm.ports.size() && i < m.pins.size(); ++i)
        {
            auto const& p{cm.ports.index_unchecked(i)};
            if(p.dir != ::phy_engine::verilog::digital::port_dir::output && p.dir != ::phy_engine::verilog::digital::port_dir::inout) { continue; }

            auto* node{m.pins.index_unchecked(i).nodes};
            if(node == nullptr || p.signal >= m.top_instance.state.values.size()) { continue; }

            auto const v{m.top_instance.state.values.index_unchecked(p.signal)};

            if(node->num_of_analog_node == 0)
            {
                if(node->node_information.dn.state != v)
                {
                    node->node_information.dn.state = v;
                    table.tables.insert(node);
                }
            }
        }

        // Emit at most one analog drive per call (API limitation): rotate across analog output ports.
        if(cm.ports.empty()) { return {}; }

        ::std::size_t const nports{cm.ports.size()};
        for(::std::size_t k{}; k < nports; ++k)
        {
            ::std::size_t const i{(m.analog_emit_cursor + k) % nports};
            auto const& p{cm.ports.index_unchecked(i)};
            if(p.dir != ::phy_engine::verilog::digital::port_dir::output && p.dir != ::phy_engine::verilog::digital::port_dir::inout) { continue; }

            auto* node{m.pins.index_unchecked(i).nodes};
            if(node == nullptr || node->num_of_analog_node == 0 || p.signal >= m.top_instance.state.values.size()) { continue; }

            auto const v{m.top_instance.state.values.index_unchecked(p.signal)};
            if(v == ::phy_engine::verilog::digital::logic_t::high_impedence_state) { continue; }

            m.analog_emit_cursor = (i + 1) % nports;

            switch(v)
            {
                case ::phy_engine::verilog::digital::logic_t::false_state: return {m.Ll, node};
                case ::phy_engine::verilog::digital::logic_t::true_state: return {m.Hl, node};
                default: return {m.Ll, node};
            }
        }

        return {};
    }

    static_assert(::phy_engine::model::defines::can_update_digital_clk<VERILOG_MODULE>);

    // Build a VERILOG_MODULE from source text; returns an empty module if compilation or top lookup fails.
    inline VERILOG_MODULE make_verilog_module(::fast_io::u8string_view source,
                                              ::fast_io::u8string_view top_name) noexcept
    {
        VERILOG_MODULE m{};
        m.source = ::fast_io::u8string{source};
        m.top = ::fast_io::u8string{top_name};

        auto cr{::phy_engine::verilog::digital::compile(source)};
        if(cr.modules.empty()) { return m; }

        m.design = ::std::make_shared<::phy_engine::verilog::digital::compiled_design>(::phy_engine::verilog::digital::build_design(::std::move(cr)));
        if(m.design->modules.empty()) { return m; }

        auto const* chosen{::phy_engine::verilog::digital::find_module(*m.design, top_name)};
        if(chosen == nullptr) { chosen = __builtin_addressof(m.design->modules.front_unchecked()); }

        m.top_instance = ::phy_engine::verilog::digital::elaborate(*m.design, *chosen);

        m.pin_name_storage.clear();
        m.pins.clear();
        m.pin_name_storage.reserve(m.top_instance.mod->ports.size());
        m.pins.reserve(m.top_instance.mod->ports.size());

        for(auto const& p: m.top_instance.mod->ports)
        {
            m.pin_name_storage.push_back(p.name);
            auto const& name{m.pin_name_storage.back_unchecked()};
            m.pins.push_back({::fast_io::u8string_view{name.data(), name.size()}, nullptr, nullptr});
        }

        return m;
    }

}  // namespace phy_engine::model
