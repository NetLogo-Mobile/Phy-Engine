#pragma once
#include <fast_io/fast_io_dsal/string_view.h>

#include "../model_refs/variant.h"

namespace phy_engine::model
{

    struct property
    {
        ::fast_io::u8string_view name{};
        ::phy_engine::model::variant var{};
    };
}  // namespace phy_engine::model
