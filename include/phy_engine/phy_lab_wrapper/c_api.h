#pragma once

#include "physicslab.h"

#include <cstring>
#include <string>

// Minimal C API for embedding / wasm bindings.
// All returned strings must be freed with `plw_string_free`.

namespace phy_engine::phy_lab_wrapper::c_api::detail
{
inline thread_local std::string last_error;

inline void set_error(std::string msg) { last_error = std::move(msg); }

inline char* dup_cstr(std::string const& s)
{
    auto* out = new char[s.size() + 1];
    std::memcpy(out, s.data(), s.size());
    out[s.size()] = '\0';
    return out;
}
}  // namespace phy_engine::phy_lab_wrapper::c_api::detail

extern "C" {

using plw_experiment_t = void*;

inline char const* plw_last_error()
{
    return phy_engine::phy_lab_wrapper::c_api::detail::last_error.c_str();
}

inline void plw_string_free(char* s) { delete[] s; }

inline plw_experiment_t plw_experiment_create(int type_value)
{
    using namespace phy_engine::phy_lab_wrapper;
    try
    {
        auto type = static_cast<experiment_type>(type_value);
        return new experiment(experiment::create(type));
    }
    catch (std::exception const& e)
    {
        c_api::detail::set_error(e.what());
        return nullptr;
    }
    catch (...)
    {
        c_api::detail::set_error("unknown error");
        return nullptr;
    }
}

inline plw_experiment_t plw_experiment_load_from_string(char const* sav_json)
{
    using namespace phy_engine::phy_lab_wrapper;
    try
    {
        if (sav_json == nullptr)
        {
            throw std::invalid_argument("sav_json is null");
        }
        return new experiment(experiment::load_from_string(sav_json));
    }
    catch (std::exception const& e)
    {
        c_api::detail::set_error(e.what());
        return nullptr;
    }
    catch (...)
    {
        c_api::detail::set_error("unknown error");
        return nullptr;
    }
}

inline void plw_experiment_destroy(plw_experiment_t handle)
{
    auto* ex = static_cast<phy_engine::phy_lab_wrapper::experiment*>(handle);
    delete ex;
}

inline char* plw_experiment_dump(plw_experiment_t handle, int indent)
{
    using namespace phy_engine::phy_lab_wrapper;
    try
    {
        auto* ex = static_cast<experiment*>(handle);
        if (ex == nullptr)
        {
            throw std::invalid_argument("experiment handle is null");
        }
        return c_api::detail::dup_cstr(ex->dump(indent));
    }
    catch (std::exception const& e)
    {
        c_api::detail::set_error(e.what());
        return nullptr;
    }
    catch (...)
    {
        c_api::detail::set_error("unknown error");
        return nullptr;
    }
}

inline char* plw_experiment_add_circuit_element(plw_experiment_t handle,
                                                char const* model_id,
                                                double x,
                                                double y,
                                                double z,
                                                int element_xyz_coords)
{
    using namespace phy_engine::phy_lab_wrapper;
    try
    {
        auto* ex = static_cast<experiment*>(handle);
        if (ex == nullptr)
        {
            throw std::invalid_argument("experiment handle is null");
        }
        if (model_id == nullptr)
        {
            throw std::invalid_argument("model_id is null");
        }
        auto id = ex->add_circuit_element(model_id, position{x, y, z}, element_xyz_coords != 0);
        return c_api::detail::dup_cstr(id);
    }
    catch (std::exception const& e)
    {
        c_api::detail::set_error(e.what());
        return nullptr;
    }
    catch (...)
    {
        c_api::detail::set_error("unknown error");
        return nullptr;
    }
}

inline int plw_experiment_connect(plw_experiment_t handle,
                                 char const* src_id,
                                 int src_pin,
                                 char const* dst_id,
                                 int dst_pin,
                                 int color_value)
{
    using namespace phy_engine::phy_lab_wrapper;
    try
    {
        auto* ex = static_cast<experiment*>(handle);
        if (ex == nullptr)
        {
            throw std::invalid_argument("experiment handle is null");
        }
        if (src_id == nullptr || dst_id == nullptr)
        {
            throw std::invalid_argument("src_id/dst_id is null");
        }
        ex->connect(src_id, src_pin, dst_id, dst_pin, static_cast<wire_color>(color_value));
        return 0;
    }
    catch (std::exception const& e)
    {
        c_api::detail::set_error(e.what());
        return 1;
    }
    catch (...)
    {
        c_api::detail::set_error("unknown error");
        return 1;
    }
}

}  // extern "C"

