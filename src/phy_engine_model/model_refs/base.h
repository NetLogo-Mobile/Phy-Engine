#pragma once

#include <type_traits>
#include <concepts>

#include "../../phy_engine_utils/fast_io/fast_io_core.h"
#include "../../phy_engine_utils/fast_io/fast_io_dsal/vector.h"

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
	virtual constexpr bool adapt_step(double &step) noexcept = 0;
	virtual constexpr bool check_convergence() noexcept = 0;
	virtual constexpr ::phy_engine::model::pin_view get_pins() noexcept = 0;
	virtual constexpr ::std::u8string_view get_model_name() noexcept = 0;
	virtual constexpr ::std::u8string_view get_identification_name() noexcept = 0;
};
template <::phy_engine::model::model mod>
struct model_derv_impl : module_base_impl {
	mod m{};

	constexpr model_derv_impl(mod &&input_m) noexcept : m{::std::forward<mod>(input_m)} {}

	virtual constexpr module_base_impl *clone() const noexcept override {
		using Alloc = ::fast_io::native_global_allocator;
#if (__cpp_if_consteval >= 202106L || __cpp_lib_is_constant_evaluated >= 201811L) && __cpp_constexpr_dynamic_alloc >= 201907L
#if __cpp_if_consteval >= 202106L
		if consteval
#else
		if (__builtin_is_constant_evaluated())
#endif
		{
			return new model_derv_impl<mod>{*this};
		} else
#endif
		{
			module_base_impl *ptr{reinterpret_cast<module_base_impl *>(Alloc::allocate(sizeof(model_derv_impl<mod>)))};
			new (ptr) model_derv_impl<mod>{*this};
			return ptr;
		}
	};

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
	virtual constexpr ::std::u8string_view get_model_name() noexcept override {
		return mod::model_name;
	}
	virtual constexpr ::std::u8string_view get_identification_name() noexcept override {
		return mod::identification_name;
	}
};
}  // namespace details

struct module_base {
	using Alloc = ::fast_io::native_global_allocator;

	::phy_engine::model::model_type type{};
	details::module_base_impl *ptr{};

	::std::size_t identification{}; // intertype independence
	::std::u8string name{};
	::std::u8string describe{};
	::fast_io::vector<::std::size_t> nodes{};
	::fast_io::vector<::std::size_t> branchs{};


	constexpr module_base() noexcept = default;

	template <::phy_engine::model::model T>
	constexpr module_base(T &&tt) noexcept {
		type = T::type;
#if (__cpp_if_consteval >= 202106L || __cpp_lib_is_constant_evaluated >= 201811L) && __cpp_constexpr_dynamic_alloc >= 201907L
#if __cpp_if_consteval >= 202106L
		if consteval
#else
		if (__builtin_is_constant_evaluated())
#endif
		{
			ptr = new details::model_derv_impl<T>{tt};
		} else
#endif
		{
			ptr = reinterpret_cast<details::model_derv_impl<T> *>(Alloc::allocate(sizeof(details::model_derv_impl<T>)));
			new (ptr) details::model_derv_impl<T>{tt};
		}
	};

	constexpr module_base(module_base const &other) noexcept {
		type = other.type;
		if (other.ptr) [[likely]]
			ptr = other.ptr->clone();
		identification = other.identification;
		name = other.name;
		describe = other.describe;
		nodes = other.nodes;
		branchs = other.branchs;
	}

	constexpr module_base& operator=(module_base const &other) noexcept {
		if (__builtin_addressof(other) == this) {
			return *this;
		}
		type = other.type;
#if (__cpp_if_consteval >= 202106L || __cpp_lib_is_constant_evaluated >= 201811L) && __cpp_constexpr_dynamic_alloc >= 201907L
#if __cpp_if_consteval >= 202106L
		if consteval
#else
		if (__builtin_is_constant_evaluated())
#endif
		{
			delete ptr;
		} else
#endif
		{
			ptr->~module_base_impl();
			Alloc::deallocate(ptr);
		}
		if (other.ptr) [[likely]]
			ptr = other.ptr->clone();
		else
			ptr = nullptr;
		identification = other.identification;
		name = other.name;
		describe = other.describe;
		nodes = other.nodes;
		branchs = other.branchs;
		return *this;
	}

	constexpr module_base(module_base &&other) noexcept {
		type = other.type;
		other.type = ::phy_engine::model::model_type{};
		ptr = other.ptr;
		other.ptr = nullptr;
		identification = other.identification;
		other.identification = ::std::size_t{};
		name = ::std::move(other.name);
		describe = ::std::move(other.describe);
		nodes = ::std::move(other.nodes);
		branchs = ::std::move(other.branchs);
	}

	constexpr module_base operator=(module_base &&other) noexcept {
		if (__builtin_addressof(other) == this) {
			return *this;
		}
		type = other.type;
		other.type = ::phy_engine::model::model_type{};
#if (__cpp_if_consteval >= 202106L || __cpp_lib_is_constant_evaluated >= 201811L) && __cpp_constexpr_dynamic_alloc >= 201907L
#if __cpp_if_consteval >= 202106L
		if consteval
#else
		if (__builtin_is_constant_evaluated())
#endif
		{
			delete ptr;
		} else
#endif
		{
			ptr->~module_base_impl();
			Alloc::deallocate(ptr);
		}
		ptr = other.ptr;
		other.ptr = nullptr;
		identification = other.identification;
		other.identification = ::std::size_t{};
		name = ::std::move(other.name);
		describe = ::std::move(other.describe);
		nodes = ::std::move(other.nodes);
		branchs = ::std::move(other.branchs);
	}

	constexpr ~module_base() {
		clear();
	}

	constexpr void clear() noexcept {
		type = ::phy_engine::model::model_type{};
#if (__cpp_if_consteval >= 202106L || __cpp_lib_is_constant_evaluated >= 201811L) && __cpp_constexpr_dynamic_alloc >= 201907L
#if __cpp_if_consteval >= 202106L
		if consteval
#else
		if (__builtin_is_constant_evaluated())
#endif
		{
			delete ptr;
		} else
#endif
		{
			ptr->~module_base_impl();
			Alloc::deallocate(ptr);
		}
		ptr = nullptr;
		identification = ::std::size_t{};
		name.clear();
		describe.clear();
		nodes.clear();
		branchs.clear();
	}
};

}  // namespace phy_engine::model
