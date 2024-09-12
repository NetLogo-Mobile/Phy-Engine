﻿#pragma once

#include <type_traits>
#include <concepts>

#include <fast_io/fast_io_core.h>
#include <fast_io/fast_io_dsal/vector.h>
#include <fast_io/fast_io_dsal/string_view.h>
#include <fast_io/fast_io_dsal/string.h>

#include "concept.h"
#include "operation.h"

namespace phy_engine::model
{
    namespace details
    {

        struct model_base_impl
        {
            virtual constexpr ~model_base_impl() noexcept = default;
            virtual constexpr model_base_impl* clone() const noexcept = 0;

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

            virtual constexpr bool set_attribute(::std::size_t index, ::phy_engine::model::variant vi) noexcept = 0;
            virtual constexpr ::phy_engine::model::variant get_attribute(::std::size_t index) noexcept = 0;

            virtual constexpr ::phy_engine::model::pin_view generate_pin_view() noexcept = 0;
            virtual constexpr ::fast_io::u8string_view get_model_name() noexcept = 0;
            virtual constexpr ::fast_io::u8string_view get_identification_name() noexcept = 0;
        };

        template <::phy_engine::model::model mod>
        struct model_derv_impl : model_base_impl
        {
            using rcvmod_type = ::std::remove_cvref_t<mod>;

            rcvmod_type m;

            constexpr model_derv_impl(mod&& input_m) noexcept : m{::std::forward<mod>(input_m)} {}

            virtual constexpr model_base_impl* clone() const noexcept override
            {
                using Alloc = ::fast_io::native_global_allocator;
#if (__cpp_if_consteval >= 202106L || __cpp_lib_is_constant_evaluated >= 201811L) && __cpp_constexpr_dynamic_alloc >= 201907L
    #if __cpp_if_consteval >= 202106L
                if consteval
    #else
                if(__builtin_is_constant_evaluated())
    #endif
                {
                    return new model_derv_impl<mod>{*this};
                }
                else
#endif
                {
                    model_base_impl* ptr{reinterpret_cast<model_base_impl*>(Alloc::allocate(sizeof(model_derv_impl<mod>)))};
                    new(ptr) model_derv_impl<mod>{*this};
                    return ptr;
                }
            };

            virtual constexpr bool init_model() noexcept override
            {
                if constexpr(::phy_engine::model::defines::can_init<mod>) { return init_define(::phy_engine::model::model_reserve_type<rcvmod_type>, m); }
                else { return true; }
            }

            virtual constexpr bool prepare_ac() noexcept override
            {
                if constexpr(::phy_engine::model::defines::can_prepare_ac<mod>)
                {
                    return prepare_ac_define(::phy_engine::model::model_reserve_type<rcvmod_type>, m);
                }
                else { return true; }
            }

            virtual constexpr bool prepare_dc() noexcept override
            {
                if constexpr(::phy_engine::model::defines::can_prepare_dc<mod>)
                {
                    return prepare_dc_define(::phy_engine::model::model_reserve_type<rcvmod_type>, m);
                }
                else { return true; }
            }

            virtual constexpr bool prepare_tr() noexcept override
            {
                if constexpr(::phy_engine::model::defines::can_prepare_tr<mod>)
                {
                    return prepare_tr_define(::phy_engine::model::model_reserve_type<rcvmod_type>, m);
                }
                else { return true; }
            }

            virtual constexpr bool prepare_op() noexcept override
            {
                if constexpr(::phy_engine::model::defines::can_prepare_op<mod>)
                {
                    return prepare_op_define(::phy_engine::model::model_reserve_type<rcvmod_type>, m);
                }
                else if constexpr(::phy_engine::model::defines::can_prepare_dc<mod>)
                {
                    return prepare_dc_define(::phy_engine::model::model_reserve_type<rcvmod_type>, m);
                }
                else { return true; }
            }

            virtual constexpr bool prepare_trop() noexcept override
            {
                if constexpr(::phy_engine::model::defines::can_prepare_trop<mod>)
                {
                    return prepare_trop_define(::phy_engine::model::model_reserve_type<rcvmod_type>, m);
                }
                else if constexpr(::phy_engine::model::defines::can_prepare_tr<mod>)
                {
                    return prepare_tr_define(::phy_engine::model::model_reserve_type<rcvmod_type>, m);
                }
                else { return true; }
            }

            virtual constexpr bool iterate_ac(double omega) noexcept override
            {
                if constexpr(::phy_engine::model::defines::can_iterate_ac<mod>)
                {
                    return iterate_ac_define(::phy_engine::model::model_reserve_type<rcvmod_type>, m, omega);
                }
                else if constexpr(::phy_engine::model::defines::can_iterate_dc<mod>)
                {
                    return iterate_dc_define(::phy_engine::model::model_reserve_type<rcvmod_type>, m);
                }
                else { return false; }
            }

            virtual constexpr bool iterate_dc() noexcept override
            {
                if constexpr(::phy_engine::model::defines::can_iterate_dc<mod>)
                {
                    return iterate_dc_define(::phy_engine::model::model_reserve_type<rcvmod_type>, m);
                }
                else { return false; }
            }

            virtual constexpr bool iterate_tr(double tTime) noexcept override
            {
                if constexpr(::phy_engine::model::defines::can_iterate_tr<mod>)
                {
                    return iterate_tr_define(::phy_engine::model::model_reserve_type<rcvmod_type>, m, tTime);
                }
                else if constexpr(::phy_engine::model::defines::can_iterate_dc<mod>)
                {
                    return iterate_dc_define(::phy_engine::model::model_reserve_type<rcvmod_type>, m);
                }
                else { return false; }
            }

            virtual constexpr bool iterate_op() noexcept override
            {
                if constexpr(::phy_engine::model::defines::can_iterate_op<mod>)
                {
                    return iterate_op_define(::phy_engine::model::model_reserve_type<rcvmod_type>, m);
                }
                else if constexpr(::phy_engine::model::defines::can_iterate_dc<mod>)
                {
                    return iterate_dc_define(::phy_engine::model::model_reserve_type<rcvmod_type>, m);
                }
                else { return false; }
            }

            virtual constexpr bool iterate_trop() noexcept override
            {
                if constexpr(::phy_engine::model::defines::can_iterate_trop<mod>)
                {
                    return iterate_trop_define(::phy_engine::model::model_reserve_type<rcvmod_type>, m);
                }
                else if constexpr(::phy_engine::model::defines::can_iterate_tr<mod>)
                {
                    return iterate_tr_define(::phy_engine::model::model_reserve_type<rcvmod_type>, m, 0.0);
                }
                else if constexpr(::phy_engine::model::defines::can_iterate_dc<mod>)
                {
                    return iterate_dc_define(::phy_engine::model::model_reserve_type<rcvmod_type>, m);
                }
                else { return false; }
            }

            virtual constexpr bool save_op() noexcept override
            {
                // not a non-linear device and no need to store operating point
                if constexpr(rcvmod_type::device_type == ::phy_engine::model::model_device_type::non_linear)
                {
                    if constexpr(::phy_engine::model::defines::can_save_op<mod>)
                    {
                        return save_op_define(::phy_engine::model::model_reserve_type<rcvmod_type>, m);
                    }
                    else { return true; }
                }
                else { return true; }
            }

            virtual constexpr bool load_temperature(double temp) noexcept override
            {
                if constexpr(::phy_engine::model::defines::can_load_temperature<mod>)
                {
                    return load_temperature_define(::phy_engine::model::model_reserve_type<rcvmod_type>, m, temp);
                }
                else { return true; }
            }

            virtual constexpr bool step_changed_tr(double tTemp, double nstep) noexcept override
            {
                if constexpr(::phy_engine::model::defines::can_step_changed_tr<mod>)
                {
                    return step_changed_tr_define(::phy_engine::model::model_reserve_type<rcvmod_type>, m, tTemp, nstep);
                }
                else { return true; }
            }

            virtual constexpr bool adapt_step(double& step) noexcept override
            {
                if constexpr(::phy_engine::model::defines::can_adapt_step<mod>)
                {
                    return adapt_step_define(::phy_engine::model::model_reserve_type<rcvmod_type>, m, step);
                }
                else { return true; }
            }

            virtual constexpr bool check_convergence() noexcept override
            {
                // no model-specific checks for convergence
                if constexpr(::phy_engine::model::defines::can_check_convergence<mod>)
                {
                    return check_convergence_define(::phy_engine::model::model_reserve_type<rcvmod_type>, m);
                }
                else { return true; }
            }

            virtual constexpr bool set_attribute(::std::size_t index, ::phy_engine::model::variant vi) noexcept override
            {
                // no model-specific checks for convergence
                if constexpr(::phy_engine::model::defines::has_set_attribute<mod>)
                {
                    return set_attribute_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, m, index, vi);
                }
                else { return false; }
            }

            virtual constexpr ::phy_engine::model::variant get_attribute(::std::size_t index) noexcept override
            {
                // no model-specific checks for convergence
                if constexpr(::phy_engine::model::defines::has_get_attribute<mod>)
                {
                    return get_attribute_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, m, index);
                }
                else { return {}; }
            }

            virtual constexpr ::phy_engine::model::pin_view generate_pin_view() noexcept override
            {
                // no model-specific checks for convergence
                if constexpr(::phy_engine::model::defines::can_generate_pin_view<mod>)
                {
                    return generate_pin_view_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, m);
                }
                else { return {}; }
            }

            virtual constexpr ::fast_io::u8string_view get_model_name() noexcept override { return rcvmod_type::model_name; }

            virtual constexpr ::fast_io::u8string_view get_identification_name() noexcept override { return rcvmod_type::identification_name; }
        };
    }  // namespace details

    struct model_base
    {
        using Alloc = ::fast_io::native_global_allocator;

        ::phy_engine::model::model_type type{};
        details::model_base_impl* ptr{};

        ::std::size_t identification{};  // intertype independence
        ::fast_io::u8string name{};
        ::fast_io::u8string describe{};

        constexpr model_base() noexcept = default;

        template <::phy_engine::model::model mod>
        constexpr model_base(mod&& m) noexcept
        {
            type = ::phy_engine::model::model_type::normal;
#if (__cpp_if_consteval >= 202106L || __cpp_lib_is_constant_evaluated >= 201811L) && __cpp_constexpr_dynamic_alloc >= 201907L
    #if __cpp_if_consteval >= 202106L
            if consteval
    #else
            if(__builtin_is_constant_evaluated())
    #endif
            {
                ptr = new details::model_derv_impl<mod>{::std::forward<mod>(m)};
            }
            else
#endif
            {
                ptr = reinterpret_cast<details::model_derv_impl<mod>*>(Alloc::allocate(sizeof(details::model_derv_impl<mod>)));
                new(ptr) details::model_derv_impl<mod>{::std::forward<mod>(m)};
            }
        };

        constexpr model_base(model_base const& other) noexcept
        {
            type = other.type;
            if(other.ptr) [[likely]] { ptr = other.ptr->clone(); }
            identification = other.identification;
            name = other.name;
            describe = other.describe;
        }

        constexpr model_base& operator= (model_base const& other) noexcept
        {
            if(__builtin_addressof(other) == this) { return *this; }
            type = other.type;
            if(ptr != nullptr)
            {
#if (__cpp_if_consteval >= 202106L || __cpp_lib_is_constant_evaluated >= 201811L) && __cpp_constexpr_dynamic_alloc >= 201907L
    #if __cpp_if_consteval >= 202106L
                if consteval
    #else
                if(__builtin_is_constant_evaluated())
    #endif
                {
                    delete ptr;
                }
                else
#endif
                {
                    ptr->~model_base_impl();  // nullptr will crash
                    Alloc::deallocate(ptr);
                }
            }
            if(other.ptr) [[likely]] { ptr = other.ptr->clone(); }
            else { ptr = nullptr; }
            identification = other.identification;
            name = other.name;
            describe = other.describe;
            return *this;
        }

        constexpr model_base(model_base&& other) noexcept
        {
            type = other.type;
            other.type = ::phy_engine::model::model_type{};
            ptr = other.ptr;
            other.ptr = nullptr;
            identification = other.identification;
            other.identification = 0;
            name = ::std::move(other.name);
            describe = ::std::move(other.describe);
        }

        constexpr model_base operator= (model_base&& other) noexcept
        {
            if(__builtin_addressof(other) == this) { return *this; }
            type = other.type;
            other.type = ::phy_engine::model::model_type{};
            if(ptr != nullptr)
            {
#if (__cpp_if_consteval >= 202106L || __cpp_lib_is_constant_evaluated >= 201811L) && __cpp_constexpr_dynamic_alloc >= 201907L
    #if __cpp_if_consteval >= 202106L
                if consteval
    #else
                if(__builtin_is_constant_evaluated())
    #endif
                {
                    delete ptr;
                }
                else
#endif
                {
                    ptr->~model_base_impl();  // nullptr will crash
                    Alloc::deallocate(ptr);
                }
            }
            ptr = other.ptr;
            other.ptr = nullptr;
            identification = other.identification;
            other.identification = 0;
            name = ::std::move(other.name);
            describe = ::std::move(other.describe);
        }

        constexpr ~model_base() { clear(); }

        // member function
        constexpr void remove_from_node() noexcept
        {
            if(type == ::phy_engine::model::model_type::normal) [[likely]]
            {
                auto pin_view{ptr->generate_pin_view()};
                for(auto curr{pin_view.pins}; curr != pin_view.pins + pin_view.size; ++curr)
                {
                    auto node{curr->nodes};
                    if(node) [[likely]]
                    {
                        node->pins.erase(curr);
                        node = nullptr;
                    }
                }
            }
        }

        constexpr void clear() noexcept
        {
            remove_from_node();
            type = ::phy_engine::model::model_type::null;
            if(ptr != nullptr)
            {
#if (__cpp_if_consteval >= 202106L || __cpp_lib_is_constant_evaluated >= 201811L) && __cpp_constexpr_dynamic_alloc >= 201907L
    #if __cpp_if_consteval >= 202106L
                if consteval
    #else
                if(__builtin_is_constant_evaluated())
    #endif
                {
                    delete ptr;
                }
                else
#endif
                {
                    ptr->~model_base_impl();  // nullptr will crash
                    Alloc::deallocate(ptr);
                }
            }
            ptr = nullptr;
            identification = 0;
            name.clear();
            describe.clear();
        }
    };

    struct module_template
    {
        // inline static constexpr ::fast_io::u8string_view model_name{u8"Module Template"};
        // inline static constexpr ::fast_io::u8string_view model_description{u8"Describtion."};
        // inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::linear};
        // inline static constexpr ::fast_io::u8string_view identification_name{u8"Mt"};
        // inline static constexpr ::phy_engine::model::pin_view pins{};
    };

}  // namespace phy_engine::model

