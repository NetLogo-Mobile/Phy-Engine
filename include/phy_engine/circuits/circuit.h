#pragma once
#include <cstdint>
#include <utility>

#include "environment/environment.h"
#include "../netlist/netlist.h"

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

        constexpr void prepare() noexcept
        {
            for(auto& i: nl.models)
            {
                for(auto c{i.begin}; c != i.curr; ++c)
                {
                    if(c->type != ::phy_engine::model::model_type::normal) [[unlikely]] { continue; }

                    c->ptr->init_model();
                    c->has_init = true;
                    
                }
            }
            auto const num_terml{nl.m_numTermls};
        }
    };

}  // namespace phy_engine
