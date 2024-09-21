#pragma once
#include "AC.h"
#include "DC.h"
#include "OP.h"
#include "TR.h"

namespace phy_engine::analyzer
{
    union analyzer_storage_t
    {
        ::phy_engine::analyzer::DC dc;
    };
}  // namespace phy_engine::analyzer
