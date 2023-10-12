#ifdef _DEBUG
#include <string>
#include <string_view>
#include "test.h"

#include "../../../phy_engine_netlist/netlist.h"
#include "../../../phy_engine_model/model_refs/operation.h"
namespace phy_engine::model {

struct test_model {
	static constexpr ::std::u8string_view model_name{u8"test_model"};
	static constexpr ::phy_engine::model::model_type type{::phy_engine::model::model_type::invalid};
	static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::non_linear};
	static constexpr ::std::u8string_view identification_name{u8"test"};
	static constexpr ::phy_engine::model::pin_view pins{};
};
}  // namespace model


::phy_engine::command_line::parameter_return_type(::phy_engine::parameter::details::test_callback)(::std::size_t, ::fast_io::vector<::phy_engine::command_line::parameter_parsing_results>&) noexcept {
	::phy_engine::model::netlist nl{};
	::phy_engine::model::test_model a{};
	init_model(a);
	init_model(::phy_engine::model::test_model{});
	add_model(nl, a);
	add_model(nl, ::phy_engine::model::test_model{});
	add_model(nl, ::phy_engine::model::test_model{});
	::phy_engine::model::netlist nl2{nl};
	::phy_engine::model::netlist nl3{::std::move(nl)};
	return ::phy_engine::command_line::parameter_return_type::def;
}
#endif  // _DEBUG
