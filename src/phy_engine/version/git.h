#pragma once
#include <string_view>

namespace phy_engine::git {
inline constexpr ::std::u8string_view commit_hash{
#include "../../../custom/git_commit_hash.h"
};
}

