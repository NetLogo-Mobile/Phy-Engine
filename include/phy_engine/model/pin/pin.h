#pragma once
#include <fast_io/fast_io_dsal/string_view.h>

namespace phy_engine::model
{
    struct pin
    {
        ::fast_io::u8string_view name{};
    };
}  // namespace phy_engine::model
