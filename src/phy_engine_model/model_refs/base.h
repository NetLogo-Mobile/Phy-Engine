#pragma once

#include <type_traits>
#include <concepts>

#include "../../phy_engine_utils/fast_io/fast_io_core.h"

#include "concept.h"
#include "operation.h"

namespace phy_engine::model {
namespace details {

struct module_base_impl {
	virtual constexpr ~module_base_impl() noexcept = default;
	virtual constexpr module_base_impl *clone() const noexcept = 0;

	virtual constexpr bool init_model() noexcept = 0;
	virtual constexpr bool prepare_ac() noexcept = 0;
	virtual constexpr bool prepare_dc() noexcept = 0;
	virtual constexpr bool prepare_tr() noexcept = 0;
	virtual constexpr bool prepare_op() noexcept = 0;
	virtual constexpr bool prepare_trop() noexcept = 0;
	virtual constexpr bool iterate_ac(double omega) noexcept = 0;
	virtual constexpr bool iterate_dc() noexcept = 0;
	virtual constexpr bool iterate_tr(double tTime) noexcept = 0;
	virtual constexpr bool iterate_op() noexcept = 0;
	virtual constexpr bool iterate_trop() noexcept = 0;
	virtual constexpr bool save_op() noexcept = 0;
	virtual constexpr bool load_temperature(double temp) noexcept = 0;
	virtual constexpr bool step_changed_tr(double tTemp, double nstep) noexcept = 0;
	virtual constexpr bool adapt_step(double& step) noexcept = 0;
	virtual constexpr bool check_convergence() noexcept = 0;
	virtual constexpr ::phy_engine::model::pin_view get_pins() noexcept = 0;
};
template <::phy_engine::model::model mod>
struct model_derv_impl : module_base_impl {
	mod m{};

	constexpr model_derv_impl(mod &&input_m) noexcept : m{::std::forward<mod>(input_m)} {}

	virtual constexpr module_base_impl *clone() const noexcept override {};

	virtual constexpr bool init_model() noexcept override {}
	virtual constexpr bool prepare_ac() noexcept override {}
	virtual constexpr bool prepare_dc() noexcept override {}
	virtual constexpr bool prepare_tr() noexcept override {}
	virtual constexpr bool prepare_op() noexcept override {}
	virtual constexpr bool prepare_trop() noexcept override {}
	virtual constexpr bool iterate_ac(double omega) noexcept override {}
	virtual constexpr bool iterate_dc() noexcept override {}
	virtual constexpr bool iterate_tr(double tTime) noexcept override {}
	virtual constexpr bool iterate_op() noexcept override {}
	virtual constexpr bool iterate_trop() noexcept override {}
	virtual constexpr bool save_op() noexcept override {}
	virtual constexpr bool load_temperature(double temp) noexcept override {}
	virtual constexpr bool step_changed_tr(double tTemp, double nstep) noexcept override {}
	virtual constexpr bool adapt_step(double &step) noexcept override {}
	virtual constexpr bool check_convergence() noexcept override {}
	virtual constexpr ::phy_engine::model::pin_view get_pins() noexcept override {}
};
}

struct module_base {

};
 
} 
