#pragma once
#include "../constexpr_hash_table.h"

namespace phy_engine
{
    namespace verilog
    {
        inline constexpr void pretreatment_define_f(::phy_engine::verilog::Verilog_module& vmod, char8_t const* begin) noexcept {}

        constexpr ::phy_engine::verilog::constexpr_hash::pretreatment pretreatment_define{u8"define", __builtin_addressof(pretreatment_define_f)};

    }  // namespace verilog
}  // namespace phy_engine
