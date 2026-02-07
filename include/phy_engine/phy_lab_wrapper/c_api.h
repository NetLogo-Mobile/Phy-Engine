#pragma once

#include "physicslab.h"

#include <cstring>
#include <new>
#include <string>

// Minimal C API for embedding / wasm bindings.
// All returned strings must be freed with `plw_string_free`.

namespace phy_engine::phy_lab_wrapper::c_api::detail
{
inline thread_local std::string last_error;

inline void set_error(std::string msg) { last_error = std::move(msg); }
inline void clear_error() noexcept { last_error.clear(); }

inline char* dup_cstr(std::string const& s)
{
    auto* out = new (std::nothrow) char[s.size() + 1];
    if(out == nullptr) { return nullptr; }
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
    c_api::detail::clear_error();
    auto type = static_cast<experiment_type>(type_value);
    auto* ex = new (std::nothrow) experiment(experiment::create(type));
    if(ex == nullptr)
    {
        c_api::detail::set_error("out of memory");
        return nullptr;
    }
    return ex;
}

inline plw_experiment_t plw_experiment_load_from_string(char const* sav_json)
{
    using namespace phy_engine::phy_lab_wrapper;
    c_api::detail::clear_error();
    if(sav_json == nullptr)
    {
        c_api::detail::set_error("sav_json is null");
        return nullptr;
    }
    auto r = experiment::load_from_string_ec(sav_json);
    if(!r)
    {
        c_api::detail::set_error(r.st.message);
        return nullptr;
    }
    auto* ex = new (std::nothrow) experiment(std::move(*r.value));
    if(ex == nullptr)
    {
        c_api::detail::set_error("out of memory");
        return nullptr;
    }
    return ex;
}

inline void plw_experiment_destroy(plw_experiment_t handle)
{
    auto* ex = static_cast<phy_engine::phy_lab_wrapper::experiment*>(handle);
    delete ex;
}

inline char* plw_experiment_dump(plw_experiment_t handle, int indent)
{
    using namespace phy_engine::phy_lab_wrapper;
    c_api::detail::clear_error();
    auto* ex = static_cast<experiment*>(handle);
    if(ex == nullptr)
    {
        c_api::detail::set_error("experiment handle is null");
        return nullptr;
    }
    auto r = ex->dump_ec(indent);
    if(!r)
    {
        c_api::detail::set_error(r.st.message);
        return nullptr;
    }
    auto* out = c_api::detail::dup_cstr(*r.value);
    if(out == nullptr)
    {
        c_api::detail::set_error("out of memory");
        return nullptr;
    }
    return out;
}

inline char* plw_experiment_add_circuit_element(plw_experiment_t handle,
                                                char const* model_id,
                                                double x,
                                                double y,
                                                double z,
                                                int element_xyz_coords)
{
    using namespace phy_engine::phy_lab_wrapper;
    c_api::detail::clear_error();
    auto* ex = static_cast<experiment*>(handle);
    if(ex == nullptr)
    {
        c_api::detail::set_error("experiment handle is null");
        return nullptr;
    }
    if(model_id == nullptr)
    {
        c_api::detail::set_error("model_id is null");
        return nullptr;
    }
    auto id_r = ex->add_circuit_element_ec(model_id, position{x, y, z}, element_xyz_coords != 0);
    if(!id_r)
    {
        c_api::detail::set_error(id_r.st.message);
        return nullptr;
    }
    auto* out = c_api::detail::dup_cstr(*id_r.value);
    if(out == nullptr)
    {
        c_api::detail::set_error("out of memory");
        return nullptr;
    }
    return out;
}

inline int plw_experiment_connect(plw_experiment_t handle,
                                 char const* src_id,
                                 int src_pin,
                                 char const* dst_id,
                                 int dst_pin,
                                 int color_value)
{
    using namespace phy_engine::phy_lab_wrapper;
    c_api::detail::clear_error();
    auto* ex = static_cast<experiment*>(handle);
    if(ex == nullptr)
    {
        c_api::detail::set_error("experiment handle is null");
        return 1;
    }
    if(src_id == nullptr || dst_id == nullptr)
    {
        c_api::detail::set_error("src_id/dst_id is null");
        return 1;
    }
    auto st = ex->connect_ec(src_id, src_pin, dst_id, dst_pin, static_cast<wire_color>(color_value));
    if(!st)
    {
        c_api::detail::set_error(st.message);
        return 1;
    }
    return 0;
}

}  // extern "C"
