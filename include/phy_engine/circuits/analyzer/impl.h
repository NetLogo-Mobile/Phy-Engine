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
        ::phy_engine::analyzer::TR tr;
    };
}  // namespace phy_engine::analyzer
