#pragma once
#include <fast_io/fast_io.h>
#include <fast_io/fast_io_dsal/vector.h>
#include "syntax.h"
#include "../parser/error.h"

namespace phy_engine::verilog
{
    struct Verilog_module
    {
        ::fast_io::vector<::phy_engine::verilog::syntax_t> syntaxes{};
        ::fast_io::vector<::phy_engine::verilog::error_t> errors{};
    };
}

namespace fast_io::freestanding
{
    template <>
    struct is_trivially_relocatable<::phy_engine::verilog::Verilog_module>
    {
        inline static constexpr bool value = true;
    };

    template <>
    struct is_zero_default_constructible<::phy_engine::verilog::Verilog_module>
    {
        inline static constexpr bool value = true;
    };

}  // namespace fast_io::freestanding

