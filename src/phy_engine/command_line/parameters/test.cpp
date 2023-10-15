#ifdef _DEBUG
#include <string>
#include <string_view>

#include "../../../phy_engine_utils/fast_io/fast_io.h"
#include "../../../phy_engine_utils/fast_io/fast_io_device.h"

#include "test.h"
#include "../../../phy_engine_netlist/netlist.h"
#include "../../../phy_engine_netlist/operation.h"
#include "../../../phy_engine_model/model_refs/operation.h"
#include "../../../phy_engine_model/model_refs/base.h"

::phy_engine::command_line::parameter_return_type(::phy_engine::parameter::details::test_callback)(::std::size_t, ::fast_io::vector<::phy_engine::command_line::parameter_parsing_results>&) noexcept {

	return ::phy_engine::command_line::parameter_return_type::def;
}
#endif  // _DEBUG
