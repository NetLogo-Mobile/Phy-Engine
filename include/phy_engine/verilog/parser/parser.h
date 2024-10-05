#pragma once
#include "../ast/module.h"

namespace phy_engine::verilog
{
    template <bool check_bound = false>  // only need to check when loading file using normal allocator not mmap
    inline constexpr ::phy_engine::verilog::Verilog_module parser_file(char8_t const* begin, char8_t const* end) noexcept
    {
        if constexpr(check_bound)
        {
            if(end <= begin) [[unlikely]] { return {}; }
        }
        return {};
    }
}  // namespace phy_engine::verilog
