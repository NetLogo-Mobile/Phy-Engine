#pragma once
#include "AC.h"
#include "DC.h"
#include "OP.h"
#include "TR.h"

namespace phy_engine::analyzer
{
    union analyzer_storage_t
    {
        ::phy_engine::analyzer::AC ac;
        ::phy_engine::analyzer::DC dc;
        ::phy_engine::analyzer::OP op;
    };
}  // namespace phy_engine::analyzer
