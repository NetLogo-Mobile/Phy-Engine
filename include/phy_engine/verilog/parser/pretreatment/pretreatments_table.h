#pragma once
#include "constexpr_hash_table.h"

#include "pretreatments/define.h"

namespace phy_engine
{
    namespace verilog
    {
        namespace details
        {
            inline constexpr ::phy_engine::verilog::constexpr_hash::pretreatment const* pretreatment_unsort[]{
                __builtin_addressof(::phy_engine::verilog::pretreatment_define)};
        };

        inline constexpr auto pretreatments{::phy_engine::verilog::constexpr_hash::pretreatment_sort(details::pretreatment_unsort)};
#if 0
        inline constexpr ::std::size_t max_pretreatments_size{::phy_engine::verilog::constexpr_hash::calculate_max_pretreatment_size(pretreatments)};
#endif
        inline constexpr auto hash_table_size{::phy_engine::verilog::constexpr_hash::calculate_hash_table_size(pretreatments)};
        inline constexpr auto hash_table{
            ::phy_engine::verilog::constexpr_hash::generate_hash_table<hash_table_size.hash_table_size, hash_table_size.extra_size>(pretreatments)};
        inline constexpr [[maybe_unused]] auto sizeof_hash_table{sizeof(hash_table)};
        inline constexpr auto hash_table_view{
            ::phy_engine::verilog::constexpr_hash::generate_hash_table_view(hash_table)};

    }  // namespace verilog
}  // namespace phy_engine
