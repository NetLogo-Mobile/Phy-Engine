#pragma once
#include <fast_io/fast_io.h>
#include <fast_io/fast_io_dsal/vector.h>
#include "syntax.h"

namespace phy_engine::verilog
{
    struct Verilog_module
    {
        ::fast_io::vector<::phy_engine::verilog::syntax_t> syntaxes{};
    };
}
