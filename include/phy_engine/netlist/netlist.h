#pragma once
#include <cstddef>
#include <type_traits>

#include <fast_io/fast_io_core.h>
#include <fast_io/fast_io_dsal/deque.h>
#include "../model/model_refs/base.h"

namespace phy_engine::netlist
{

    struct netlist
    {
        using Alloc = ::fast_io::native_global_allocator;
        ::fast_io::deque<::phy_engine::model::model_base> netlist_memory{};
        ::std::size_t m_numNodes{};
        ::std::size_t m_numBranches{};
        ::std::size_t m_numTermls{};
        bool m_hasGround{};
    };

}  // namespace phy_engine::model
