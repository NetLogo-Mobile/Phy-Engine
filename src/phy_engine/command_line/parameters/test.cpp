#ifdef _DEBUG
#include <string>
#include <string_view>
#include "test.h"

#include "../../../phy_engine_netlist/netlist.h"
#include "../../../phy_engine_netlist/operation.h"
#include "../../../phy_engine_model/model_refs/operation.h"
#include "../../../phy_engine_model/model_refs/base.h"

struct test_model {
	static constexpr ::std::u8string_view model_name{u8"test_model"};
	static constexpr ::phy_engine::model::model_type type{::phy_engine::model::model_type::invalid};
	static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::non_linear};
	static constexpr ::std::u8string_view identification_name{u8"test"};
	static constexpr ::phy_engine::model::pin_view pins{};
};

template <typename mod>
struct test_str {
	using mod_type = ::std::remove_cvref_t<mod>;
	//using mod_type = mod;
	mod_type d{};
};

template <typename mod>
inline constexpr auto in_test_str(mod&& m) noexcept {
	return test_str<mod>{::std::forward<mod>(m)};
};

inline constexpr bool iterate_dc_define(::phy_engine::model::model_reserve_type_t<test_model>, test_model& t_m) noexcept {
	int a;
	auto i = in_test_str(a);
	//::phy_engine::model::model_reserve_type_t<int&> k; // static_assert
	return true;
}

::phy_engine::command_line::parameter_return_type(::phy_engine::parameter::details::test_callback)(::std::size_t, ::fast_io::vector<::phy_engine::command_line::parameter_parsing_results>&) noexcept {
#if 1
	::phy_engine::model::netlist nl{};
	test_model a{};
	::phy_engine::model::init_model(a);
	::phy_engine::model::init_model(test_model{});
	constexpr auto i{::phy_engine::model::defines::can_iterate_dc<test_model>};
	add_model(nl, a);
	add_model(nl, test_model{});
	add_model(nl, test_model{});
	::phy_engine::model::netlist nl2{nl};
	::phy_engine::model::netlist nl3{::std::move(nl)};
#endif  // 0
	return ::phy_engine::command_line::parameter_return_type::def;
}
#endif  // _DEBUG
