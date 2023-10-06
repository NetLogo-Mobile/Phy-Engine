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

	virtual constexpr bool init_model() noexcept override {
		return ::phy_engine::model::init_model<mod>(m);
	}
	virtual constexpr bool prepare_ac() noexcept override { 
		return ::phy_engine::model::prepare_ac<mod>(m); 
	}
	virtual constexpr bool prepare_dc() noexcept override {
		return ::phy_engine::model::prepare_dc<mod>(m);
	}
	virtual constexpr bool prepare_tr() noexcept override {
		return ::phy_engine::model::prepare_tr<mod>(m);
	}
	virtual constexpr bool prepare_op() noexcept override {
		return ::phy_engine::model::prepare_op<mod>(m);
	}
	virtual constexpr bool prepare_trop() noexcept override {
		return ::phy_engine::model::prepare_trop<mod>(m);
	}
	virtual constexpr bool iterate_ac(double omega) noexcept override {
		return ::phy_engine::model::iterate_ac<mod>(m, omega);
	}
	virtual constexpr bool iterate_dc() noexcept override {
		return ::phy_engine::model::iterate_dc<mod>(m);
	}
	virtual constexpr bool iterate_tr(double tTime) noexcept override {
		return ::phy_engine::model::iterate_tr<mod>(m, tTime);
	}
	virtual constexpr bool iterate_op() noexcept override {
		return ::phy_engine::model::iterate_op<mod>(m);
	}
	virtual constexpr bool iterate_trop() noexcept override {
		return ::phy_engine::model::iterate_trop<mod>(m);
	}
	virtual constexpr bool save_op() noexcept override {
		return ::phy_engine::model::save_op<mod>(m);
	}
	virtual constexpr bool load_temperature(double temp) noexcept override {
		return ::phy_engine::model::load_temperature<mod>(m, temp);
	}
	virtual constexpr bool step_changed_tr(double tTemp, double nstep) noexcept override {
		return ::phy_engine::model::step_changed_tr<mod>(m, tTemp, nstep);
	}
	virtual constexpr bool adapt_step(double &step) noexcept override {
		return ::phy_engine::model::adapt_step<mod>(m, step);
	}
	virtual constexpr bool check_convergence() noexcept override {
		return ::phy_engine::model::check_convergence<mod>(m);
	}
	virtual constexpr ::phy_engine::model::pin_view get_pins() noexcept override {
		return ::phy_engine::model::get_pins<mod>(m);
	}
};
}

struct module_base {

};
 
} 
