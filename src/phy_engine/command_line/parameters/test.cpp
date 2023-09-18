#ifdef _DEBUG
#include "test.h"

bool(::phy_engine::parameter::details::test_callback)(int argc, char8_t** argv, int pos, ::std::u8string_view var) noexcept {
	return true;
}
#endif  // _DEBUG
