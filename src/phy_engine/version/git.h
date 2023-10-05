#pragma once
#include <string_view>

namespace phy_engine::git {
inline constexpr ::std::u8string_view fetch_head{
#include "../../../custom/git_commit_hash.h"
};
}

