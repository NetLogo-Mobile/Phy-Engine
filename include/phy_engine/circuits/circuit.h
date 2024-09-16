#pragma once
#include <cstdint>
#include <utility>

#include "environment/environment.h"
#include "../netlist/netlist.h"
#include "MNA/mna.h"
#include "analyze.h"

namespace phy_engine
{

    enum class CKT_mode_type : ::std::uint_fast8_t
    {
        DC,
        AC,
        TR,
        OP,
        TrOP,
    };

    struct circult
    {
        CKT_mode_type ckt_mode{};  // CKTmode
        ::phy_engine::environment env{};
        ::phy_engine::netlist::netlist nl{};

        ::phy_engine::MNA::MNA mna{};

        constexpr void prepare(::phy_engine::analyze_type at) noexcept
        {
            mna.resize(nl.nodes.size(), mna.branch_size);

            for(auto& i: nl.models)
            {
                for(auto c{i.begin}; c != i.curr; ++c)
                {
                    if(c->type != ::phy_engine::model::model_type::normal) [[unlikely]] { continue; }

                    c->ptr->init_model();
                    c->has_init = true;
                }
            }
        }
    };

}  // namespace phy_engine
