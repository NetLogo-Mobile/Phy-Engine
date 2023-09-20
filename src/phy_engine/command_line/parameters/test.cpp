#ifdef _DEBUG
#include "test.h"

::phy_engine::command_line::parameter_return_type(::phy_engine::parameter::details::test_callback)(::std::size_t, ::fast_io::vector<::phy_engine::command_line::parameter_parsing_results>&) noexcept {
	return ::phy_engine::command_line::parameter_return_type::def;
}
#endif  // _DEBUG
