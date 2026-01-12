#pragma once

#include <fast_io/fast_io_dsal/string.h>
#include <fast_io/fast_io_dsal/string_view.h>
#include <fast_io/fast_io_dsal/vector.h>

#include "../../model_refs/base.h"
#include "../../pin/pin_view.h"
#include "../../../circuits/digital/update_table.h"

namespace phy_engine::model
{
    // A dynamic-pin, no-op digital model used as a placeholder for synthesized Verilog modules.
    // It exists only to expose a port list (as pins) so external wiring can attach to nodes.
    struct VERILOG_PORTS
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"VERILOG_PORTS"};
        inline static constexpr ::phy_engine::model::digital_update_method_t digital_update_method{::phy_engine::model::digital_update_method_t::update_table};
        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::digital};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"VERILOG_PORTS"};

        ::fast_io::vector<::fast_io::u8string> pin_name_storage{};
        ::fast_io::vector<::phy_engine::model::pin> pins{};

        VERILOG_PORTS() = default;

        explicit VERILOG_PORTS(::fast_io::vector<::fast_io::u8string>&& names)
        {
            set_pins(::std::move(names));
        }

        void set_pins(::fast_io::vector<::fast_io::u8string>&& names)
        {
            pin_name_storage = ::std::move(names);
            pins.clear();
            pins.reserve(pin_name_storage.size());
            for(auto const& nm: pin_name_storage)
            {
                pins.push_back({::fast_io::u8string_view{nm.data(), nm.size()}, nullptr, nullptr});
            }
        }
    };

    static_assert(::phy_engine::model::model<VERILOG_PORTS>);

    inline constexpr bool set_attribute_define(::phy_engine::model::model_reserve_type_t<VERILOG_PORTS>,
                                               VERILOG_PORTS& /*m*/,
                                               ::std::size_t /*index*/,
                                               ::phy_engine::model::variant /*vi*/) noexcept
    {
        return false;
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<VERILOG_PORTS>);

    inline constexpr ::phy_engine::model::variant get_attribute_define(::phy_engine::model::model_reserve_type_t<VERILOG_PORTS>,
                                                                       VERILOG_PORTS const& /*m*/,
                                                                       ::std::size_t /*index*/) noexcept
    {
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<VERILOG_PORTS>);

    inline constexpr ::fast_io::u8string_view get_attribute_name_define(::phy_engine::model::model_reserve_type_t<VERILOG_PORTS>, ::std::size_t /*index*/) noexcept
    {
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<VERILOG_PORTS>);

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<VERILOG_PORTS>, VERILOG_PORTS& m) noexcept
    {
        return {m.pins.data(), m.pins.size()};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<VERILOG_PORTS>);

    inline constexpr ::phy_engine::digital::need_operate_analog_node_t update_digital_clk_define(::phy_engine::model::model_reserve_type_t<VERILOG_PORTS>,
                                                                                                 VERILOG_PORTS& /*m*/,
                                                                                                 ::phy_engine::digital::digital_node_update_table& /*table*/,
                                                                                                 double /*tr_duration*/,
                                                                                                 ::phy_engine::model::digital_update_method_t /*method*/) noexcept
    {
        return {};
    }

    static_assert(::phy_engine::model::defines::can_update_digital_clk<VERILOG_PORTS>);
}  // namespace phy_engine::model
