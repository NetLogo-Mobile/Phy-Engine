#pragma once
#include <cstddef>
#include <initializer_list>

#include "file_format/impl.h"
#include "refs/impl.h"

namespace phy_engine::file_format {
inline constexpr ::std::size_t calculate_ff_eles_size() noexcept {
	::std::initializer_list<::phy_engine::file_format::file_format> file_formats{
#include "ffinc.h"
	};

}
}