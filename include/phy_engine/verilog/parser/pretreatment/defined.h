#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include <fast_io/fast_io_dsal/string.h>
#include <absl/container/flat_hash_map.h>
#include "../../basic/absl_hash_def.h"

namespace phy_engine::verilog
{
    struct defined_table_t
    {
        ::absl::flat_hash_map<::fast_io::u8string, ::fast_io::u8string> tables{};
    };

}  // namespace phy_engine::verilog
