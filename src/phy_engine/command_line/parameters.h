#pragma once

#include <algorithm>

#include "../../phy_engine_utils/fast_io/fast_io_freestanding.h"

#include "../../phy_engine_utils/command_line/impl.h"

#include "inc.h"

namespace phy_engine {

namespace details {
inline constexpr ::phy_engine::command_line::parameter const* parameter_unsort[]{
#include "parainc.h"
};
}  // namespace details

inline constexpr auto parameters{::phy_engine::command_line::parameter_sort(details::parameter_unsort)};

inline constexpr ::std::size_t parameter_lookup_table_size{::phy_engine::command_line::calculate_all_parameters_size(parameters)};
inline constexpr auto parameter_lookup_table{::phy_engine::command_line::expand_all_parameters_and_check<parameter_lookup_table_size>(parameters)};

inline constexpr auto hash_table_size{::phy_engine::command_line::calculate_hash_table_size(parameter_lookup_table)};
inline constexpr auto hash_table{::phy_engine::command_line::generate_hash_table<hash_table_size.hash_table_size, hash_table_size.extra_size>(parameter_lookup_table)};
}