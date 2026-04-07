#include <phy_engine/phy_engine.h>

#include <atomic>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <new>
#include <numbers>
#include <string>
#include <unordered_map>
#include <vector>

#include <phy_engine/model/models/linear/transformer.h>
#include <phy_engine/model/models/linear/transformer_center_tap.h>
#include <phy_engine/model/models/linear/coupled_inductors.h>
#include <phy_engine/model/models/linear/op_amp.h>

#include <phy_engine/model/models/controller/relay.h>
#include <phy_engine/model/models/controller/comparator.h>

#include <phy_engine/model/models/generator/sawtooth.h>
#include <phy_engine/model/models/generator/square.h>
#include <phy_engine/model/models/generator/pulse.h>
#include <phy_engine/model/models/generator/triangle.h>

#include <phy_engine/model/models/non-linear/BJT_NPN.h>
#include <phy_engine/model/models/non-linear/BJT_PNP.h>
#include <phy_engine/model/models/non-linear/nmosfet.h>
#include <phy_engine/model/models/non-linear/pmosfet.h>
#include <phy_engine/model/models/non-linear/full_bridge_rectifier.h>
#include <phy_engine/model/models/non-linear/bsim3v32.h>

#include <phy_engine/model/models/digital/logical/yes.h>
#include <phy_engine/model/models/digital/logical/tri_state.h>
#include <phy_engine/model/models/digital/verilog_ports.h>

#include <phy_engine/phy_lab_wrapper/auto_layout/auto_layout.h>
#include <phy_engine/phy_lab_wrapper/error.h>
#include <phy_engine/phy_lab_wrapper/pe_sim.h>
#include <phy_engine/phy_lab_wrapper/pe_to_pl.h>
#include <phy_engine/phy_lab_wrapper/physicslab.h>
#include <phy_engine/verilog/digital/pe_synth.h>

namespace
{
constexpr std::uint32_t VERILOG_SYNTH_FLAG_ALLOW_INOUT = 1u << 0;
constexpr std::uint32_t VERILOG_SYNTH_FLAG_ALLOW_MULTI_DRIVER = 1u << 1;
constexpr std::uint32_t VERILOG_SYNTH_FLAG_ASSUME_BINARY_INPUTS = 1u << 2;
constexpr std::uint32_t VERILOG_SYNTH_FLAG_OPT_WIRES = 1u << 3;
constexpr std::uint32_t VERILOG_SYNTH_FLAG_OPT_MUL2 = 1u << 4;
constexpr std::uint32_t VERILOG_SYNTH_FLAG_OPT_ADDERS = 1u << 5;

static ::std::atomic_uint8_t g_verilog_synth_opt_level{0};
static ::std::atomic_uint32_t g_verilog_synth_flags{VERILOG_SYNTH_FLAG_ALLOW_INOUT | VERILOG_SYNTH_FLAG_ALLOW_MULTI_DRIVER |
                                                    VERILOG_SYNTH_FLAG_OPT_WIRES | VERILOG_SYNTH_FLAG_OPT_MUL2 | VERILOG_SYNTH_FLAG_OPT_ADDERS};
static ::std::atomic_size_t g_verilog_synth_loop_unroll_limit{64};

[[nodiscard]] inline ::phy_engine::verilog::digital::pe_synth_options verilog_synth_options_snapshot() noexcept
{
    ::phy_engine::verilog::digital::pe_synth_options opt{};
    auto const lvl = g_verilog_synth_opt_level.load(::std::memory_order_relaxed);
    auto const flags = g_verilog_synth_flags.load(::std::memory_order_relaxed);
    opt.allow_inout = (flags & VERILOG_SYNTH_FLAG_ALLOW_INOUT) != 0;
    opt.allow_multi_driver = (flags & VERILOG_SYNTH_FLAG_ALLOW_MULTI_DRIVER) != 0;
    opt.assume_binary_inputs = (flags & VERILOG_SYNTH_FLAG_ASSUME_BINARY_INPUTS) != 0;
    opt.opt_level = lvl;
    opt.optimize_wires = (flags & VERILOG_SYNTH_FLAG_OPT_WIRES) != 0;
    opt.optimize_mul2 = (flags & VERILOG_SYNTH_FLAG_OPT_MUL2) != 0;
    opt.optimize_adders = (flags & VERILOG_SYNTH_FLAG_OPT_ADDERS) != 0;
    opt.loop_unroll_limit = g_verilog_synth_loop_unroll_limit.load(::std::memory_order_relaxed);
    return opt;
}
}  // namespace

extern "C" void verilog_synth_set_opt_level(::std::uint8_t level)
{
    g_verilog_synth_opt_level.store(level, ::std::memory_order_relaxed);
}

extern "C" ::std::uint8_t verilog_synth_get_opt_level()
{
    return g_verilog_synth_opt_level.load(::std::memory_order_relaxed);
}

extern "C" void verilog_synth_set_assume_binary_inputs(bool v)
{
    auto flags = g_verilog_synth_flags.load(::std::memory_order_relaxed);
    if(v) { flags |= VERILOG_SYNTH_FLAG_ASSUME_BINARY_INPUTS; }
    else { flags &= ~VERILOG_SYNTH_FLAG_ASSUME_BINARY_INPUTS; }
    g_verilog_synth_flags.store(flags, ::std::memory_order_relaxed);
}

extern "C" bool verilog_synth_get_assume_binary_inputs()
{
    return (g_verilog_synth_flags.load(::std::memory_order_relaxed) & VERILOG_SYNTH_FLAG_ASSUME_BINARY_INPUTS) != 0;
}

extern "C" void verilog_synth_set_allow_inout(bool v)
{
    auto flags = g_verilog_synth_flags.load(::std::memory_order_relaxed);
    if(v) { flags |= VERILOG_SYNTH_FLAG_ALLOW_INOUT; }
    else { flags &= ~VERILOG_SYNTH_FLAG_ALLOW_INOUT; }
    g_verilog_synth_flags.store(flags, ::std::memory_order_relaxed);
}

extern "C" bool verilog_synth_get_allow_inout()
{
    return (g_verilog_synth_flags.load(::std::memory_order_relaxed) & VERILOG_SYNTH_FLAG_ALLOW_INOUT) != 0;
}

extern "C" void verilog_synth_set_allow_multi_driver(bool v)
{
    auto flags = g_verilog_synth_flags.load(::std::memory_order_relaxed);
    if(v) { flags |= VERILOG_SYNTH_FLAG_ALLOW_MULTI_DRIVER; }
    else { flags &= ~VERILOG_SYNTH_FLAG_ALLOW_MULTI_DRIVER; }
    g_verilog_synth_flags.store(flags, ::std::memory_order_relaxed);
}

extern "C" bool verilog_synth_get_allow_multi_driver()
{
    return (g_verilog_synth_flags.load(::std::memory_order_relaxed) & VERILOG_SYNTH_FLAG_ALLOW_MULTI_DRIVER) != 0;
}

extern "C" void verilog_synth_set_optimize_wires(bool v)
{
    auto flags = g_verilog_synth_flags.load(::std::memory_order_relaxed);
    if(v) { flags |= VERILOG_SYNTH_FLAG_OPT_WIRES; }
    else { flags &= ~VERILOG_SYNTH_FLAG_OPT_WIRES; }
    g_verilog_synth_flags.store(flags, ::std::memory_order_relaxed);
}

extern "C" bool verilog_synth_get_optimize_wires()
{
    return (g_verilog_synth_flags.load(::std::memory_order_relaxed) & VERILOG_SYNTH_FLAG_OPT_WIRES) != 0;
}

extern "C" void verilog_synth_set_optimize_mul2(bool v)
{
    auto flags = g_verilog_synth_flags.load(::std::memory_order_relaxed);
    if(v) { flags |= VERILOG_SYNTH_FLAG_OPT_MUL2; }
    else { flags &= ~VERILOG_SYNTH_FLAG_OPT_MUL2; }
    g_verilog_synth_flags.store(flags, ::std::memory_order_relaxed);
}

extern "C" bool verilog_synth_get_optimize_mul2()
{
    return (g_verilog_synth_flags.load(::std::memory_order_relaxed) & VERILOG_SYNTH_FLAG_OPT_MUL2) != 0;
}

extern "C" void verilog_synth_set_optimize_adders(bool v)
{
    auto flags = g_verilog_synth_flags.load(::std::memory_order_relaxed);
    if(v) { flags |= VERILOG_SYNTH_FLAG_OPT_ADDERS; }
    else { flags &= ~VERILOG_SYNTH_FLAG_OPT_ADDERS; }
    g_verilog_synth_flags.store(flags, ::std::memory_order_relaxed);
}

extern "C" bool verilog_synth_get_optimize_adders()
{
    return (g_verilog_synth_flags.load(::std::memory_order_relaxed) & VERILOG_SYNTH_FLAG_OPT_ADDERS) != 0;
}

extern "C" void verilog_synth_set_loop_unroll_limit(::std::size_t n)
{
    g_verilog_synth_loop_unroll_limit.store(n, ::std::memory_order_relaxed);
}

extern "C" ::std::size_t verilog_synth_get_loop_unroll_limit()
{
    return g_verilog_synth_loop_unroll_limit.load(::std::memory_order_relaxed);
}

namespace
{
using verilog_logic_t = ::phy_engine::verilog::digital::logic_t;
using verilog_port_dir_t = ::phy_engine::verilog::digital::port_dir;

[[nodiscard]] inline ::std::string to_std_string(::fast_io::u8string_view sv)
{
    return ::std::string(reinterpret_cast<char const*>(sv.data()), sv.size());
}

[[nodiscard]] inline ::std::string to_std_string(::fast_io::u8string const& s)
{
    return ::std::string(reinterpret_cast<char const*>(s.data()), s.size());
}

[[nodiscard]] inline ::std::string to_std_string(char const* s, ::std::size_t size)
{
    if(s == nullptr || size == 0) { return {}; }
    return ::std::string(s, size);
}

inline void set_phy_engine_error(::std::string msg)
{
    ::phy_engine::phy_lab_wrapper::detail::set_last_error(::std::move(msg));
}

[[nodiscard]] inline char* dup_c_string(::std::string const& s)
{
    auto* out = new (::std::nothrow) char[s.size() + 1];
    if(out == nullptr) { return nullptr; }
    if(!s.empty()) { ::std::memcpy(out, s.data(), s.size()); }
    out[s.size()] = '\0';
    return out;
}

[[nodiscard]] inline ::std::uint8_t logic_to_u8(verilog_logic_t v) noexcept
{
    switch(v)
    {
        case ::phy_engine::model::digital_node_statement_t::false_state: return 0;
        case ::phy_engine::model::digital_node_statement_t::true_state: return 1;
        case ::phy_engine::model::digital_node_statement_t::indeterminate_state: return 2;
        case ::phy_engine::model::digital_node_statement_t::high_impedence_state: return 3;
        default: return 2;
    }
}

[[nodiscard]] inline bool u8_to_logic(::std::uint8_t state, verilog_logic_t& out) noexcept
{
    switch(state)
    {
        case 0: out = ::phy_engine::model::digital_node_statement_t::false_state; return true;
        case 1: out = ::phy_engine::model::digital_node_statement_t::true_state; return true;
        case 2: out = ::phy_engine::model::digital_node_statement_t::indeterminate_state; return true;
        case 3: out = ::phy_engine::model::digital_node_statement_t::high_impedence_state; return true;
        default: return false;
    }
}

[[nodiscard]] inline ::std::uint8_t port_dir_to_u8(verilog_port_dir_t dir) noexcept
{
    switch(dir)
    {
        case verilog_port_dir_t::input: return 1;
        case verilog_port_dir_t::output: return 2;
        case verilog_port_dir_t::inout: return 3;
        default: return 0;
    }
}

[[nodiscard]] inline int copy_bytes_to_c_buffer(char const* data, ::std::size_t size, char* out, ::std::size_t out_size, char const* what)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    if(out == nullptr)
    {
        set_phy_engine_error(::std::string(what) + ": output buffer is null");
        return 1;
    }
    if(out_size < size + 1)
    {
        set_phy_engine_error(::std::string(what) + ": output buffer too small");
        return 2;
    }
    if(size != 0) { ::std::memcpy(out, data, size); }
    out[size] = '\0';
    return 0;
}

[[nodiscard]] inline int copy_u8sv_to_c_buffer(::fast_io::u8string_view sv, char* out, ::std::size_t out_size, char const* what)
{
    return copy_bytes_to_c_buffer(reinterpret_cast<char const*>(sv.data()), sv.size(), out, out_size, what);
}

[[nodiscard]] inline bool read_text_file(::std::filesystem::path const& path, ::fast_io::u8string& out_text) noexcept
{
    try
    {
        ::std::ifstream ifs(path, ::std::ios::binary);
        if(!ifs.is_open()) { return false; }
        ifs.seekg(0, ::std::ios::end);
        auto const end_pos = ifs.tellg();
        if(end_pos < 0) { return false; }
        auto const size = static_cast<::std::size_t>(end_pos);
        ifs.seekg(0, ::std::ios::beg);
        ::std::string buf(size, '\0');
        if(size != 0) { ifs.read(buf.data(), static_cast<::std::streamsize>(size)); }
        out_text.assign(::fast_io::u8string_view{reinterpret_cast<char8_t const*>(buf.data()), buf.size()});
        return true;
    }
    catch(...)
    {
        return false;
    }
}

struct verilog_include_context
{
    ::std::vector<::std::filesystem::path> roots{};
};

[[nodiscard]] inline bool verilog_include_resolver(void* user, ::fast_io::u8string_view path, ::fast_io::u8string& out_text) noexcept
{
    auto const requested = ::std::filesystem::path(to_std_string(path));
    if(requested.is_absolute()) { return read_text_file(requested, out_text); }

    auto* ctx = static_cast<verilog_include_context*>(user);
    if(ctx == nullptr) { return false; }
    for(auto const& root: ctx->roots)
    {
        if(read_text_file(root / requested, out_text)) { return true; }
    }
    return false;
}

struct verilog_runtime_handle
{
    ::fast_io::u8string source{};
    ::fast_io::u8string preprocessed{};
    ::std::shared_ptr<::phy_engine::verilog::digital::compiled_design> design{};
    ::std::size_t top_module_index{};
    ::phy_engine::verilog::digital::instance_state top_instance{};
    ::std::uint64_t tick{};
};

[[nodiscard]] inline bool try_find_module_index(::phy_engine::verilog::digital::compiled_design const& d,
                                                ::fast_io::u8string_view name,
                                                ::std::size_t& out_index) noexcept
{
    auto const it = d.module_index.find(::fast_io::u8string{name});
    if(it == d.module_index.end()) { return false; }
    out_index = it->second;
    return true;
}

[[nodiscard]] inline ::phy_engine::verilog::digital::compiled_module const* runtime_top_module(verilog_runtime_handle const& rt) noexcept
{
    if(rt.design == nullptr || rt.top_module_index >= rt.design->modules.size()) { return nullptr; }
    return __builtin_addressof(rt.design->modules.index_unchecked(rt.top_module_index));
}

[[nodiscard]] inline verilog_logic_t const* runtime_port_value_ptr(verilog_runtime_handle const& rt, ::std::size_t port_index) noexcept
{
    auto const* mod = runtime_top_module(rt);
    if(mod == nullptr || port_index >= mod->ports.size()) { return nullptr; }
    auto const sig = mod->ports.index_unchecked(port_index).signal;
    if(sig >= rt.top_instance.state.values.size()) { return nullptr; }
    return __builtin_addressof(rt.top_instance.state.values.index_unchecked(sig));
}

[[nodiscard]] inline verilog_logic_t* runtime_port_value_ptr(verilog_runtime_handle& rt, ::std::size_t port_index) noexcept
{
    auto const* mod = runtime_top_module(rt);
    if(mod == nullptr || port_index >= mod->ports.size()) { return nullptr; }
    auto const sig = mod->ports.index_unchecked(port_index).signal;
    if(sig >= rt.top_instance.state.values.size()) { return nullptr; }
    return __builtin_addressof(rt.top_instance.state.values.index_unchecked(sig));
}

[[nodiscard]] inline verilog_logic_t const* runtime_signal_value_ptr(verilog_runtime_handle const& rt, ::std::size_t signal_index) noexcept
{
    if(signal_index >= rt.top_instance.state.values.size()) { return nullptr; }
    return __builtin_addressof(rt.top_instance.state.values.index_unchecked(signal_index));
}

[[nodiscard]] inline verilog_logic_t* runtime_signal_value_ptr(verilog_runtime_handle& rt, ::std::size_t signal_index) noexcept
{
    if(signal_index >= rt.top_instance.state.values.size()) { return nullptr; }
    return __builtin_addressof(rt.top_instance.state.values.index_unchecked(signal_index));
}

[[nodiscard]] inline bool runtime_reset_impl(verilog_runtime_handle& rt) noexcept
{
    auto const* mod = runtime_top_module(rt);
    if(mod == nullptr || rt.design == nullptr) { return false; }
    rt.top_instance = ::phy_engine::verilog::digital::elaborate(*rt.design, *mod);
    rt.tick = 0;
    return rt.top_instance.mod != nullptr;
}
}  // namespace

extern "C" char const* phy_engine_last_error()
{
    return ::phy_engine::phy_lab_wrapper::detail::last_error_c_str();
}

extern "C" void phy_engine_clear_error()
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
}

extern "C" void phy_engine_string_free(char* s)
{
    delete[] s;
}

extern "C" void* verilog_runtime_create(char const* src,
                                        ::std::size_t src_size,
                                        char const* top,
                                        ::std::size_t top_size,
                                        char const* const* include_dirs,
                                        ::std::size_t const* include_dir_sizes,
                                        ::std::size_t include_dir_count)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    if(src == nullptr)
    {
        set_phy_engine_error("verilog_runtime_create: src is null");
        return nullptr;
    }
    try
    {
        auto* rt = new (::std::nothrow) verilog_runtime_handle{};
        if(rt == nullptr)
        {
            set_phy_engine_error("verilog_runtime_create: allocation failed");
            return nullptr;
        }

        rt->source.assign(::fast_io::u8string_view{reinterpret_cast<char8_t const*>(src), src_size});

        ::phy_engine::verilog::digital::compile_options opt{};
        verilog_include_context ictx{};
        if(include_dirs != nullptr && include_dir_sizes != nullptr)
        {
            ictx.roots.reserve(include_dir_count);
            for(::std::size_t i{}; i < include_dir_count; ++i)
            {
                if(include_dirs[i] == nullptr) { continue; }
                ictx.roots.emplace_back(::std::string(include_dirs[i], include_dir_sizes[i]));
            }
            if(!ictx.roots.empty())
            {
                opt.preprocess.user = __builtin_addressof(ictx);
                opt.preprocess.include_resolver = verilog_include_resolver;
            }
        }

        auto const src_view = ::fast_io::u8string_view{rt->source.data(), rt->source.size()};
        auto pp = ::phy_engine::verilog::digital::preprocess(src_view, opt.preprocess);
        rt->preprocessed = pp.output;

        auto cr = ::phy_engine::verilog::digital::compile(src_view, opt);
        if(!cr.errors.empty() || cr.modules.empty())
        {
            auto formatted = ::phy_engine::verilog::digital::format_compile_errors(cr, src_view);
            if(formatted.empty())
            {
                set_phy_engine_error("verilog_runtime_create: Verilog compile failed");
            }
            else
            {
                set_phy_engine_error(to_std_string(formatted));
            }
            delete rt;
            return nullptr;
        }

        rt->design =
            ::std::make_shared<::phy_engine::verilog::digital::compiled_design>(::phy_engine::verilog::digital::build_design(::std::move(cr)));
        if(rt->design == nullptr || rt->design->modules.empty())
        {
            set_phy_engine_error("verilog_runtime_create: no compiled modules");
            delete rt;
            return nullptr;
        }

        ::std::size_t top_index{};
        if(top != nullptr && top_size != 0)
        {
            auto const top_view = ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(top), top_size};
            if(!try_find_module_index(*rt->design, top_view, top_index))
            {
                set_phy_engine_error("verilog_runtime_create: top module not found: " + ::std::string(top, top_size));
                delete rt;
                return nullptr;
            }
        }
        else
        {
            static constexpr char8_t fallback_top_name[] = u8"top";
            if(!try_find_module_index(*rt->design,
                                      ::fast_io::u8string_view{fallback_top_name, sizeof(fallback_top_name) - 1},
                                      top_index))
            {
                top_index = 0;
            }
        }

        rt->top_module_index = top_index;
        if(!runtime_reset_impl(*rt))
        {
            set_phy_engine_error("verilog_runtime_create: failed to elaborate top module");
            delete rt;
            return nullptr;
        }

        return rt;
    }
    catch(::std::exception const& e)
    {
        set_phy_engine_error(::std::string("verilog_runtime_create: exception: ") + e.what());
        return nullptr;
    }
    catch(...)
    {
        set_phy_engine_error("verilog_runtime_create: unknown exception");
        return nullptr;
    }
}

extern "C" void verilog_runtime_destroy(void* runtime_ptr)
{
    delete static_cast<verilog_runtime_handle*>(runtime_ptr);
}

extern "C" ::std::uint64_t verilog_runtime_get_tick(void* runtime_ptr)
{
    auto* rt = static_cast<verilog_runtime_handle*>(runtime_ptr);
    return rt == nullptr ? 0 : rt->tick;
}

extern "C" int verilog_runtime_reset(void* runtime_ptr)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    auto* rt = static_cast<verilog_runtime_handle*>(runtime_ptr);
    if(rt == nullptr)
    {
        set_phy_engine_error("verilog_runtime_reset: runtime is null");
        return 1;
    }
    if(!runtime_reset_impl(*rt))
    {
        set_phy_engine_error("verilog_runtime_reset: failed to re-elaborate top module");
        return 2;
    }
    return 0;
}

extern "C" int verilog_runtime_step(void* runtime_ptr, ::std::uint64_t tick, ::std::uint8_t process_sequential)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    auto* rt = static_cast<verilog_runtime_handle*>(runtime_ptr);
    if(rt == nullptr)
    {
        set_phy_engine_error("verilog_runtime_step: runtime is null");
        return 1;
    }
    if(rt->top_instance.mod == nullptr)
    {
        set_phy_engine_error("verilog_runtime_step: top instance is null");
        return 2;
    }
    rt->tick = tick;
    ::phy_engine::verilog::digital::simulate(rt->top_instance, tick, process_sequential != 0);
    return 0;
}

extern "C" int verilog_runtime_tick(void* runtime_ptr)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    auto* rt = static_cast<verilog_runtime_handle*>(runtime_ptr);
    if(rt == nullptr)
    {
        set_phy_engine_error("verilog_runtime_tick: runtime is null");
        return 1;
    }
    auto const next_tick = rt->tick + 1;
    ::phy_engine::verilog::digital::simulate(rt->top_instance, next_tick, true);
    rt->tick = next_tick;
    return 0;
}

extern "C" ::std::size_t verilog_runtime_module_count(void* runtime_ptr)
{
    auto* rt = static_cast<verilog_runtime_handle*>(runtime_ptr);
    return (rt == nullptr || rt->design == nullptr) ? 0 : rt->design->modules.size();
}

extern "C" ::std::size_t verilog_runtime_port_count(void* runtime_ptr)
{
    auto* rt = static_cast<verilog_runtime_handle*>(runtime_ptr);
    auto const* mod = (rt == nullptr) ? nullptr : runtime_top_module(*rt);
    return mod == nullptr ? 0 : mod->ports.size();
}

extern "C" ::std::size_t verilog_runtime_signal_count(void* runtime_ptr)
{
    auto* rt = static_cast<verilog_runtime_handle*>(runtime_ptr);
    auto const* mod = (rt == nullptr) ? nullptr : runtime_top_module(*rt);
    return mod == nullptr ? 0 : mod->signal_names.size();
}

extern "C" ::std::size_t verilog_runtime_preprocessed_size(void* runtime_ptr)
{
    auto* rt = static_cast<verilog_runtime_handle*>(runtime_ptr);
    return rt == nullptr ? 0 : rt->preprocessed.size();
}

extern "C" int verilog_runtime_copy_preprocessed(void* runtime_ptr, char* out, ::std::size_t out_size)
{
    auto* rt = static_cast<verilog_runtime_handle*>(runtime_ptr);
    if(rt == nullptr)
    {
        set_phy_engine_error("verilog_runtime_copy_preprocessed: runtime is null");
        return 1;
    }
    return copy_u8sv_to_c_buffer(::fast_io::u8string_view{rt->preprocessed.data(), rt->preprocessed.size()},
                                 out,
                                 out_size,
                                 "verilog_runtime_copy_preprocessed");
}

extern "C" ::std::size_t verilog_runtime_top_module_name_size(void* runtime_ptr)
{
    auto* rt = static_cast<verilog_runtime_handle*>(runtime_ptr);
    auto const* mod = (rt == nullptr) ? nullptr : runtime_top_module(*rt);
    return mod == nullptr ? 0 : mod->name.size();
}

extern "C" int verilog_runtime_copy_top_module_name(void* runtime_ptr, char* out, ::std::size_t out_size)
{
    auto* rt = static_cast<verilog_runtime_handle*>(runtime_ptr);
    auto const* mod = (rt == nullptr) ? nullptr : runtime_top_module(*rt);
    if(mod == nullptr)
    {
        set_phy_engine_error("verilog_runtime_copy_top_module_name: top module is null");
        return 1;
    }
    return copy_u8sv_to_c_buffer(::fast_io::u8string_view{mod->name.data(), mod->name.size()}, out, out_size, "verilog_runtime_copy_top_module_name");
}

extern "C" ::std::size_t verilog_runtime_module_name_size(void* runtime_ptr, ::std::size_t module_index)
{
    auto* rt = static_cast<verilog_runtime_handle*>(runtime_ptr);
    if(rt == nullptr || rt->design == nullptr || module_index >= rt->design->modules.size()) { return 0; }
    return rt->design->modules.index_unchecked(module_index).name.size();
}

extern "C" int verilog_runtime_copy_module_name(void* runtime_ptr, ::std::size_t module_index, char* out, ::std::size_t out_size)
{
    auto* rt = static_cast<verilog_runtime_handle*>(runtime_ptr);
    if(rt == nullptr || rt->design == nullptr || module_index >= rt->design->modules.size())
    {
        set_phy_engine_error("verilog_runtime_copy_module_name: invalid module index");
        return 1;
    }
    auto const& mod = rt->design->modules.index_unchecked(module_index);
    return copy_u8sv_to_c_buffer(::fast_io::u8string_view{mod.name.data(), mod.name.size()}, out, out_size, "verilog_runtime_copy_module_name");
}

extern "C" ::std::size_t verilog_runtime_port_name_size(void* runtime_ptr, ::std::size_t port_index)
{
    auto* rt = static_cast<verilog_runtime_handle*>(runtime_ptr);
    auto const* mod = (rt == nullptr) ? nullptr : runtime_top_module(*rt);
    if(mod == nullptr || port_index >= mod->ports.size()) { return 0; }
    return mod->ports.index_unchecked(port_index).name.size();
}

extern "C" int verilog_runtime_copy_port_name(void* runtime_ptr, ::std::size_t port_index, char* out, ::std::size_t out_size)
{
    auto* rt = static_cast<verilog_runtime_handle*>(runtime_ptr);
    auto const* mod = (rt == nullptr) ? nullptr : runtime_top_module(*rt);
    if(mod == nullptr || port_index >= mod->ports.size())
    {
        set_phy_engine_error("verilog_runtime_copy_port_name: invalid port index");
        return 1;
    }
    auto const& port = mod->ports.index_unchecked(port_index);
    return copy_u8sv_to_c_buffer(::fast_io::u8string_view{port.name.data(), port.name.size()}, out, out_size, "verilog_runtime_copy_port_name");
}

extern "C" ::std::uint8_t verilog_runtime_port_dir(void* runtime_ptr, ::std::size_t port_index)
{
    auto* rt = static_cast<verilog_runtime_handle*>(runtime_ptr);
    auto const* mod = (rt == nullptr) ? nullptr : runtime_top_module(*rt);
    if(mod == nullptr || port_index >= mod->ports.size()) { return 0; }
    return port_dir_to_u8(mod->ports.index_unchecked(port_index).dir);
}

extern "C" ::std::uint8_t verilog_runtime_get_port_value(void* runtime_ptr, ::std::size_t port_index)
{
    auto* rt = static_cast<verilog_runtime_handle*>(runtime_ptr);
    auto const* v = (rt == nullptr) ? nullptr : runtime_port_value_ptr(*rt, port_index);
    return v == nullptr ? 2 : logic_to_u8(*v);
}

extern "C" int verilog_runtime_set_port_value(void* runtime_ptr, ::std::size_t port_index, ::std::uint8_t state)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    auto* rt = static_cast<verilog_runtime_handle*>(runtime_ptr);
    auto const* mod = (rt == nullptr) ? nullptr : runtime_top_module(*rt);
    if(mod == nullptr || port_index >= mod->ports.size())
    {
        set_phy_engine_error("verilog_runtime_set_port_value: invalid port index");
        return 1;
    }
    auto const dir = mod->ports.index_unchecked(port_index).dir;
    if(dir == verilog_port_dir_t::output)
    {
        set_phy_engine_error("verilog_runtime_set_port_value: output port is read-only");
        return 2;
    }
    auto* dst = runtime_port_value_ptr(*rt, port_index);
    if(dst == nullptr)
    {
        set_phy_engine_error("verilog_runtime_set_port_value: unresolved port signal");
        return 3;
    }
    verilog_logic_t lv{};
    if(!u8_to_logic(state, lv))
    {
        set_phy_engine_error("verilog_runtime_set_port_value: invalid digital state");
        return 4;
    }
    *dst = lv;
    return 0;
}

extern "C" ::std::size_t verilog_runtime_signal_name_size(void* runtime_ptr, ::std::size_t signal_index)
{
    auto* rt = static_cast<verilog_runtime_handle*>(runtime_ptr);
    auto const* mod = (rt == nullptr) ? nullptr : runtime_top_module(*rt);
    if(mod == nullptr || signal_index >= mod->signal_names.size()) { return 0; }
    return mod->signal_names.index_unchecked(signal_index).size();
}

extern "C" int verilog_runtime_copy_signal_name(void* runtime_ptr, ::std::size_t signal_index, char* out, ::std::size_t out_size)
{
    auto* rt = static_cast<verilog_runtime_handle*>(runtime_ptr);
    auto const* mod = (rt == nullptr) ? nullptr : runtime_top_module(*rt);
    if(mod == nullptr || signal_index >= mod->signal_names.size())
    {
        set_phy_engine_error("verilog_runtime_copy_signal_name: invalid signal index");
        return 1;
    }
    auto const& signal_name = mod->signal_names.index_unchecked(signal_index);
    return copy_u8sv_to_c_buffer(::fast_io::u8string_view{signal_name.data(), signal_name.size()}, out, out_size, "verilog_runtime_copy_signal_name");
}

extern "C" ::std::uint8_t verilog_runtime_get_signal_value(void* runtime_ptr, ::std::size_t signal_index)
{
    auto* rt = static_cast<verilog_runtime_handle*>(runtime_ptr);
    auto const* v = (rt == nullptr) ? nullptr : runtime_signal_value_ptr(*rt, signal_index);
    return v == nullptr ? 2 : logic_to_u8(*v);
}

extern "C" int verilog_runtime_set_signal_value(void* runtime_ptr, ::std::size_t signal_index, ::std::uint8_t state)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    auto* rt = static_cast<verilog_runtime_handle*>(runtime_ptr);
    auto* dst = (rt == nullptr) ? nullptr : runtime_signal_value_ptr(*rt, signal_index);
    if(dst == nullptr)
    {
        set_phy_engine_error("verilog_runtime_set_signal_value: invalid signal index");
        return 1;
    }
    verilog_logic_t lv{};
    if(!u8_to_logic(state, lv))
    {
        set_phy_engine_error("verilog_runtime_set_signal_value: invalid digital state");
        return 2;
    }
    *dst = lv;
    return 0;
}

extern "C" void* pl_experiment_create(int type_value)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    try
    {
        auto* ex = new (::std::nothrow)
            ::phy_engine::phy_lab_wrapper::experiment(::phy_engine::phy_lab_wrapper::experiment::create(
                static_cast<::phy_engine::phy_lab_wrapper::experiment_type>(type_value)));
        if(ex == nullptr)
        {
            set_phy_engine_error("pl_experiment_create: allocation failed");
            return nullptr;
        }
        return ex;
    }
    catch(::std::exception const& e)
    {
        set_phy_engine_error(::std::string("pl_experiment_create: exception: ") + e.what());
        return nullptr;
    }
    catch(...)
    {
        set_phy_engine_error("pl_experiment_create: unknown exception");
        return nullptr;
    }
}

extern "C" void* pl_experiment_load_from_string(char const* sav_json, ::std::size_t sav_json_size)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    if(sav_json == nullptr)
    {
        set_phy_engine_error("pl_experiment_load_from_string: sav_json is null");
        return nullptr;
    }
    auto r = ::phy_engine::phy_lab_wrapper::experiment::load_from_string_ec(::std::string_view{sav_json, sav_json_size});
    if(!r)
    {
        set_phy_engine_error(r.st.message);
        return nullptr;
    }
    auto* ex = new (::std::nothrow) ::phy_engine::phy_lab_wrapper::experiment(::std::move(*r.value));
    if(ex == nullptr)
    {
        set_phy_engine_error("pl_experiment_load_from_string: allocation failed");
        return nullptr;
    }
    return ex;
}

extern "C" void* pl_experiment_load_from_file(char const* path, ::std::size_t path_size)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    if(path == nullptr)
    {
        set_phy_engine_error("pl_experiment_load_from_file: path is null");
        return nullptr;
    }
    auto r = ::phy_engine::phy_lab_wrapper::experiment::load_ec(::std::filesystem::path(to_std_string(path, path_size)));
    if(!r)
    {
        set_phy_engine_error(r.st.message);
        return nullptr;
    }
    auto* ex = new (::std::nothrow) ::phy_engine::phy_lab_wrapper::experiment(::std::move(*r.value));
    if(ex == nullptr)
    {
        set_phy_engine_error("pl_experiment_load_from_file: allocation failed");
        return nullptr;
    }
    return ex;
}

extern "C" void pl_experiment_destroy(void* experiment_ptr)
{
    delete static_cast<::phy_engine::phy_lab_wrapper::experiment*>(experiment_ptr);
}

extern "C" char* pl_experiment_dump(void* experiment_ptr, int indent)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    auto* ex = static_cast<::phy_engine::phy_lab_wrapper::experiment*>(experiment_ptr);
    if(ex == nullptr)
    {
        set_phy_engine_error("pl_experiment_dump: experiment is null");
        return nullptr;
    }
    auto r = ex->dump_ec(indent);
    if(!r)
    {
        set_phy_engine_error(r.st.message);
        return nullptr;
    }
    auto* out = dup_c_string(*r.value);
    if(out == nullptr)
    {
        set_phy_engine_error("pl_experiment_dump: allocation failed");
        return nullptr;
    }
    return out;
}

extern "C" int pl_experiment_save(void* experiment_ptr, char const* path, ::std::size_t path_size, int indent)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    auto* ex = static_cast<::phy_engine::phy_lab_wrapper::experiment*>(experiment_ptr);
    if(ex == nullptr || path == nullptr)
    {
        set_phy_engine_error("pl_experiment_save: experiment/path is null");
        return 1;
    }
    auto st = ex->save_ec(::std::filesystem::path(to_std_string(path, path_size)), indent);
    if(!st)
    {
        set_phy_engine_error(st.message);
        return 2;
    }
    return 0;
}

extern "C" char* pl_experiment_add_circuit_element(void* experiment_ptr,
                                                   char const* model_id,
                                                   ::std::size_t model_id_size,
                                                   double x,
                                                   double y,
                                                   double z,
                                                   ::std::uint8_t element_xyz_coords,
                                                   ::std::uint8_t is_big_element,
                                                   ::std::uint8_t participate_in_layout)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    auto* ex = static_cast<::phy_engine::phy_lab_wrapper::experiment*>(experiment_ptr);
    if(ex == nullptr || model_id == nullptr)
    {
        set_phy_engine_error("pl_experiment_add_circuit_element: experiment/model_id is null");
        return nullptr;
    }
    auto r = ex->add_circuit_element_ec(to_std_string(model_id, model_id_size),
                                        ::phy_engine::phy_lab_wrapper::position{x, y, z},
                                        element_xyz_coords != 0,
                                        is_big_element != 0,
                                        participate_in_layout != 0);
    if(!r)
    {
        set_phy_engine_error(r.st.message);
        return nullptr;
    }
    auto* out = dup_c_string(*r.value);
    if(out == nullptr)
    {
        set_phy_engine_error("pl_experiment_add_circuit_element: allocation failed");
        return nullptr;
    }
    return out;
}

extern "C" int pl_experiment_connect(void* experiment_ptr,
                                     char const* src_id,
                                     ::std::size_t src_id_size,
                                     int src_pin,
                                     char const* dst_id,
                                     ::std::size_t dst_id_size,
                                     int dst_pin,
                                     int color_value)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    auto* ex = static_cast<::phy_engine::phy_lab_wrapper::experiment*>(experiment_ptr);
    if(ex == nullptr || src_id == nullptr || dst_id == nullptr)
    {
        set_phy_engine_error("pl_experiment_connect: experiment/src_id/dst_id is null");
        return 1;
    }
    auto st = ex->connect_ec(to_std_string(src_id, src_id_size),
                             src_pin,
                             to_std_string(dst_id, dst_id_size),
                             dst_pin,
                             static_cast<::phy_engine::phy_lab_wrapper::wire_color>(color_value));
    if(!st)
    {
        set_phy_engine_error(st.message);
        return 2;
    }
    return 0;
}

extern "C" int pl_experiment_clear_wires(void* experiment_ptr)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    auto* ex = static_cast<::phy_engine::phy_lab_wrapper::experiment*>(experiment_ptr);
    if(ex == nullptr)
    {
        set_phy_engine_error("pl_experiment_clear_wires: experiment is null");
        return 1;
    }
    auto st = ex->clear_wires_ec();
    if(!st)
    {
        set_phy_engine_error(st.message);
        return 2;
    }
    return 0;
}

extern "C" int pl_experiment_set_xyz_precision(void* experiment_ptr, int decimals)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    auto* ex = static_cast<::phy_engine::phy_lab_wrapper::experiment*>(experiment_ptr);
    if(ex == nullptr)
    {
        set_phy_engine_error("pl_experiment_set_xyz_precision: experiment is null");
        return 1;
    }
    auto st = ex->set_xyz_precision_ec(decimals);
    if(!st)
    {
        set_phy_engine_error(st.message);
        return 2;
    }
    return 0;
}

extern "C" int pl_experiment_set_element_xyz(void* experiment_ptr,
                                             ::std::uint8_t enabled,
                                             double origin_x,
                                             double origin_y,
                                             double origin_z)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    auto* ex = static_cast<::phy_engine::phy_lab_wrapper::experiment*>(experiment_ptr);
    if(ex == nullptr)
    {
        set_phy_engine_error("pl_experiment_set_element_xyz: experiment is null");
        return 1;
    }
    ex->set_element_xyz(enabled != 0, ::phy_engine::phy_lab_wrapper::position{origin_x, origin_y, origin_z});
    return 0;
}

extern "C" int pl_experiment_set_camera(void* experiment_ptr,
                                        double vision_center_x,
                                        double vision_center_y,
                                        double vision_center_z,
                                        double target_rotation_x,
                                        double target_rotation_y,
                                        double target_rotation_z)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    auto* ex = static_cast<::phy_engine::phy_lab_wrapper::experiment*>(experiment_ptr);
    if(ex == nullptr)
    {
        set_phy_engine_error("pl_experiment_set_camera: experiment is null");
        return 1;
    }
    ex->set_camera(::phy_engine::phy_lab_wrapper::position{vision_center_x, vision_center_y, vision_center_z},
                   ::phy_engine::phy_lab_wrapper::position{target_rotation_x, target_rotation_y, target_rotation_z});
    return 0;
}

extern "C" int pl_experiment_set_element_property_number(void* experiment_ptr,
                                                         char const* element_id,
                                                         ::std::size_t element_id_size,
                                                         char const* key,
                                                         ::std::size_t key_size,
                                                         double value)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    auto* ex = static_cast<::phy_engine::phy_lab_wrapper::experiment*>(experiment_ptr);
    if(ex == nullptr || element_id == nullptr || key == nullptr)
    {
        set_phy_engine_error("pl_experiment_set_element_property_number: experiment/element_id/key is null");
        return 1;
    }
    auto* el = ex->find_element(to_std_string(element_id, element_id_size));
    if(el == nullptr)
    {
        set_phy_engine_error("pl_experiment_set_element_property_number: unknown element identifier");
        return 2;
    }
    auto& data = el->data();
    if(!data.contains("Properties") || !data["Properties"].is_object()) { data["Properties"] = ::phy_engine::phy_lab_wrapper::json::object(); }
    auto const key_s = to_std_string(key, key_size);
    data["Properties"][key_s] = value;
    if(key_s == "锁定") { data["IsLocked"] = (value != 0.0); }
    return 0;
}

extern "C" int pl_experiment_set_element_label(void* experiment_ptr,
                                               char const* element_id,
                                               ::std::size_t element_id_size,
                                               char const* label,
                                               ::std::size_t label_size)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    auto* ex = static_cast<::phy_engine::phy_lab_wrapper::experiment*>(experiment_ptr);
    if(ex == nullptr || element_id == nullptr)
    {
        set_phy_engine_error("pl_experiment_set_element_label: experiment/element_id is null");
        return 1;
    }
    auto* el = ex->find_element(to_std_string(element_id, element_id_size));
    if(el == nullptr)
    {
        set_phy_engine_error("pl_experiment_set_element_label: unknown element identifier");
        return 2;
    }
    if(label == nullptr || label_size == 0) { el->data()["Label"] = nullptr; }
    else { el->data()["Label"] = to_std_string(label, label_size); }
    return 0;
}

extern "C" int pl_experiment_set_element_position(void* experiment_ptr,
                                                  char const* element_id,
                                                  ::std::size_t element_id_size,
                                                  double x,
                                                  double y,
                                                  double z,
                                                  ::std::uint8_t element_xyz_coords)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    auto* ex = static_cast<::phy_engine::phy_lab_wrapper::experiment*>(experiment_ptr);
    if(ex == nullptr || element_id == nullptr)
    {
        set_phy_engine_error("pl_experiment_set_element_position: experiment/element_id is null");
        return 1;
    }
    auto* el = ex->find_element(to_std_string(element_id, element_id_size));
    if(el == nullptr)
    {
        set_phy_engine_error("pl_experiment_set_element_position: unknown element identifier");
        return 2;
    }
    el->set_element_position(::phy_engine::phy_lab_wrapper::position{x, y, z}, element_xyz_coords != 0);
    return 0;
}

extern "C" int pl_experiment_merge(void* dst_experiment_ptr,
                                   void* src_experiment_ptr,
                                   double offset_x,
                                   double offset_y,
                                   double offset_z)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    auto* dst = static_cast<::phy_engine::phy_lab_wrapper::experiment*>(dst_experiment_ptr);
    auto* src = static_cast<::phy_engine::phy_lab_wrapper::experiment*>(src_experiment_ptr);
    if(dst == nullptr || src == nullptr)
    {
        set_phy_engine_error("pl_experiment_merge: dst/src experiment is null");
        return 1;
    }
    auto st = dst->merge_ec(*src, ::phy_engine::phy_lab_wrapper::position{offset_x, offset_y, offset_z});
    if(!st)
    {
        set_phy_engine_error(st.message);
        return 2;
    }
    return 0;
}

extern "C" void* pl_pe_circuit_build(void* experiment_ptr)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    auto* ex = static_cast<::phy_engine::phy_lab_wrapper::experiment*>(experiment_ptr);
    if(ex == nullptr)
    {
        set_phy_engine_error("pl_pe_circuit_build: experiment is null");
        return nullptr;
    }
    auto r = ::phy_engine::phy_lab_wrapper::pe::circuit::build_from_ec(*ex);
    if(!r)
    {
        set_phy_engine_error(r.st.message);
        return nullptr;
    }
    auto* out = new (::std::nothrow) ::phy_engine::phy_lab_wrapper::pe::circuit(::std::move(*r.value));
    if(out == nullptr)
    {
        set_phy_engine_error("pl_pe_circuit_build: allocation failed");
        return nullptr;
    }
    return out;
}

extern "C" void pl_pe_circuit_destroy(void* pe_circuit_ptr)
{
    delete static_cast<::phy_engine::phy_lab_wrapper::pe::circuit*>(pe_circuit_ptr);
}

extern "C" ::std::size_t pl_pe_circuit_comp_size(void* pe_circuit_ptr)
{
    auto* c = static_cast<::phy_engine::phy_lab_wrapper::pe::circuit*>(pe_circuit_ptr);
    return c == nullptr ? 0 : c->comp_size();
}

extern "C" int pl_pe_circuit_set_analyze_type(void* pe_circuit_ptr, ::std::uint32_t analyze_type_value)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    auto* c = static_cast<::phy_engine::phy_lab_wrapper::pe::circuit*>(pe_circuit_ptr);
    if(c == nullptr)
    {
        set_phy_engine_error("pl_pe_circuit_set_analyze_type: circuit is null");
        return 1;
    }
    auto st = c->set_analyze_type_ec(static_cast<phy_engine_analyze_type>(analyze_type_value));
    if(!st)
    {
        set_phy_engine_error(st.message);
        return 2;
    }
    return 0;
}

extern "C" int pl_pe_circuit_set_tr(void* pe_circuit_ptr, double t_step, double t_stop)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    auto* c = static_cast<::phy_engine::phy_lab_wrapper::pe::circuit*>(pe_circuit_ptr);
    if(c == nullptr)
    {
        set_phy_engine_error("pl_pe_circuit_set_tr: circuit is null");
        return 1;
    }
    auto st = c->set_tr_ec(t_step, t_stop);
    if(!st)
    {
        set_phy_engine_error(st.message);
        return 2;
    }
    return 0;
}

extern "C" int pl_pe_circuit_set_ac_omega(void* pe_circuit_ptr, double omega)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    auto* c = static_cast<::phy_engine::phy_lab_wrapper::pe::circuit*>(pe_circuit_ptr);
    if(c == nullptr)
    {
        set_phy_engine_error("pl_pe_circuit_set_ac_omega: circuit is null");
        return 1;
    }
    auto st = c->set_ac_omega_ec(omega);
    if(!st)
    {
        set_phy_engine_error(st.message);
        return 2;
    }
    return 0;
}

extern "C" int pl_pe_circuit_analyze(void* pe_circuit_ptr)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    auto* c = static_cast<::phy_engine::phy_lab_wrapper::pe::circuit*>(pe_circuit_ptr);
    if(c == nullptr)
    {
        set_phy_engine_error("pl_pe_circuit_analyze: circuit is null");
        return 1;
    }
    auto st = c->analyze_ec();
    if(!st)
    {
        set_phy_engine_error(st.message);
        return 2;
    }
    return 0;
}

extern "C" int pl_pe_circuit_digital_clk(void* pe_circuit_ptr)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    auto* c = static_cast<::phy_engine::phy_lab_wrapper::pe::circuit*>(pe_circuit_ptr);
    if(c == nullptr)
    {
        set_phy_engine_error("pl_pe_circuit_digital_clk: circuit is null");
        return 1;
    }
    auto st = c->digital_clk_ec();
    if(!st)
    {
        set_phy_engine_error(st.message);
        return 2;
    }
    return 0;
}

extern "C" int pl_pe_circuit_sync_inputs_from_pl(void* pe_circuit_ptr, void* experiment_ptr)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    auto* c = static_cast<::phy_engine::phy_lab_wrapper::pe::circuit*>(pe_circuit_ptr);
    auto* ex = static_cast<::phy_engine::phy_lab_wrapper::experiment*>(experiment_ptr);
    if(c == nullptr || ex == nullptr)
    {
        set_phy_engine_error("pl_pe_circuit_sync_inputs_from_pl: circuit/experiment is null");
        return 1;
    }
    auto st = c->sync_inputs_from_pl_ec(*ex);
    if(!st)
    {
        set_phy_engine_error(st.message);
        return 2;
    }
    return 0;
}

extern "C" int pl_pe_circuit_write_back_to_pl(void* pe_circuit_ptr, void* experiment_ptr)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    auto* c = static_cast<::phy_engine::phy_lab_wrapper::pe::circuit*>(pe_circuit_ptr);
    auto* ex = static_cast<::phy_engine::phy_lab_wrapper::experiment*>(experiment_ptr);
    if(c == nullptr || ex == nullptr)
    {
        set_phy_engine_error("pl_pe_circuit_write_back_to_pl: circuit/experiment is null");
        return 1;
    }
    auto s = c->sample_now_digital_state_ec();
    if(!s)
    {
        set_phy_engine_error(s.st.message);
        return 2;
    }
    auto st = c->write_back_to_pl_ec(*ex, *s.value);
    if(!st)
    {
        set_phy_engine_error(st.message);
        return 3;
    }
    return 0;
}

extern "C" int pl_pe_circuit_write_back_to_pl_ex(void* pe_circuit_ptr,
                                                 void* experiment_ptr,
                                                 double logic_output_low,
                                                 double logic_output_high,
                                                 double logic_output_x,
                                                 double logic_output_z)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    auto* c = static_cast<::phy_engine::phy_lab_wrapper::pe::circuit*>(pe_circuit_ptr);
    auto* ex = static_cast<::phy_engine::phy_lab_wrapper::experiment*>(experiment_ptr);
    if(c == nullptr || ex == nullptr)
    {
        set_phy_engine_error("pl_pe_circuit_write_back_to_pl_ex: circuit/experiment is null");
        return 1;
    }
    auto s = c->sample_now_digital_state_ec();
    if(!s)
    {
        set_phy_engine_error(s.st.message);
        return 2;
    }
    ::phy_engine::phy_lab_wrapper::pe::write_back_options opt{};
    opt.logic_output_low = logic_output_low;
    opt.logic_output_high = logic_output_high;
    opt.logic_output_x = logic_output_x;
    opt.logic_output_z = logic_output_z;
    auto st = c->write_back_to_pl_ec(*ex, *s.value, opt);
    if(!st)
    {
        set_phy_engine_error(st.message);
        return 3;
    }
    return 0;
}

extern "C" int pl_pe_circuit_sample_layout(void* pe_circuit_ptr,
                                           ::std::size_t* voltage_ord,
                                           ::std::size_t* current_ord,
                                           ::std::size_t* digital_ord)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    auto* c = static_cast<::phy_engine::phy_lab_wrapper::pe::circuit*>(pe_circuit_ptr);
    if(c == nullptr || voltage_ord == nullptr || current_ord == nullptr || digital_ord == nullptr)
    {
        set_phy_engine_error("pl_pe_circuit_sample_layout: invalid arguments");
        return 1;
    }
    auto s = c->sample_now_ec();
    if(!s)
    {
        set_phy_engine_error(s.st.message);
        return 2;
    }
    for(::std::size_t i{}; i < s.value->pin_voltage_ord.size(); ++i)
    {
        voltage_ord[i] = s.value->pin_voltage_ord[i];
        current_ord[i] = s.value->branch_current_ord[i];
        digital_ord[i] = s.value->pin_digital_ord[i];
    }
    return 0;
}

extern "C" int pl_pe_circuit_sample_u8(void* pe_circuit_ptr,
                                       double* voltage,
                                       ::std::size_t* voltage_ord,
                                       double* current,
                                       ::std::size_t* current_ord,
                                       ::std::uint8_t* digital,
                                       ::std::size_t* digital_ord)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    auto* c = static_cast<::phy_engine::phy_lab_wrapper::pe::circuit*>(pe_circuit_ptr);
    if(c == nullptr || voltage == nullptr || voltage_ord == nullptr || current == nullptr || current_ord == nullptr || digital == nullptr ||
       digital_ord == nullptr)
    {
        set_phy_engine_error("pl_pe_circuit_sample_u8: invalid arguments");
        return 1;
    }
    auto s = c->sample_now_ec();
    if(!s)
    {
        set_phy_engine_error(s.st.message);
        return 2;
    }
    for(::std::size_t i{}; i < s.value->pin_voltage_ord.size(); ++i)
    {
        voltage_ord[i] = s.value->pin_voltage_ord[i];
        current_ord[i] = s.value->branch_current_ord[i];
        digital_ord[i] = s.value->pin_digital_ord[i];
    }
    for(::std::size_t i{}; i < s.value->pin_voltage.size(); ++i) { voltage[i] = s.value->pin_voltage[i]; }
    for(::std::size_t i{}; i < s.value->branch_current.size(); ++i) { current[i] = s.value->branch_current[i]; }
    for(::std::size_t i{}; i < s.value->pin_digital.size(); ++i) { digital[i] = s.value->pin_digital[i]; }
    return 0;
}

extern "C" int pl_pe_circuit_sample_digital_state_u8(void* pe_circuit_ptr,
                                                     double* voltage,
                                                     ::std::size_t* voltage_ord,
                                                     double* current,
                                                     ::std::size_t* current_ord,
                                                     ::std::uint8_t* digital,
                                                     ::std::size_t* digital_ord)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    auto* c = static_cast<::phy_engine::phy_lab_wrapper::pe::circuit*>(pe_circuit_ptr);
    if(c == nullptr || voltage == nullptr || voltage_ord == nullptr || current == nullptr || current_ord == nullptr || digital == nullptr ||
       digital_ord == nullptr)
    {
        set_phy_engine_error("pl_pe_circuit_sample_digital_state_u8: invalid arguments");
        return 1;
    }
    auto s = c->sample_now_digital_state_ec();
    if(!s)
    {
        set_phy_engine_error(s.st.message);
        return 2;
    }
    for(::std::size_t i{}; i < s.value->pin_voltage_ord.size(); ++i)
    {
        voltage_ord[i] = s.value->pin_voltage_ord[i];
        current_ord[i] = s.value->branch_current_ord[i];
        digital_ord[i] = s.value->pin_digital_ord[i];
    }
    for(::std::size_t i{}; i < s.value->pin_voltage.size(); ++i) { voltage[i] = s.value->pin_voltage[i]; }
    for(::std::size_t i{}; i < s.value->branch_current.size(); ++i) { current[i] = s.value->branch_current[i]; }
    for(::std::size_t i{}; i < s.value->pin_digital.size(); ++i) { digital[i] = s.value->pin_digital[i]; }
    return 0;
}

extern "C" void* pe_to_pl_convert(void* circuit_ptr,
                                  double fixed_x,
                                  double fixed_y,
                                  double fixed_z,
                                  ::std::uint8_t element_xyz_coords,
                                  ::std::uint8_t keep_pl_macros,
                                  ::std::uint8_t include_linear,
                                  ::std::uint8_t include_ground,
                                  ::std::uint8_t generate_wires,
                                  ::std::uint8_t keep_unknown_as_placeholders,
                                  ::std::uint8_t drop_dangling_logic_inputs)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    auto* c = static_cast<::phy_engine::circult*>(circuit_ptr);
    if(c == nullptr)
    {
        set_phy_engine_error("pe_to_pl_convert: circuit is null");
        return nullptr;
    }

    ::phy_engine::phy_lab_wrapper::pe_to_pl::options opt{};
    opt.fixed_pos = ::phy_engine::phy_lab_wrapper::position{fixed_x, fixed_y, fixed_z};
    opt.element_xyz_coords = element_xyz_coords != 0;
    opt.keep_pl_macros = keep_pl_macros != 0;
    opt.include_linear = include_linear != 0;
    opt.include_ground = include_ground != 0;
    opt.generate_wires = generate_wires != 0;
    opt.keep_unknown_as_placeholders = keep_unknown_as_placeholders != 0;
    opt.drop_dangling_logic_inputs = drop_dangling_logic_inputs != 0;

    auto r = ::phy_engine::phy_lab_wrapper::pe_to_pl::convert_ec(c->get_netlist(), opt);
    if(!r)
    {
        set_phy_engine_error(r.st.message);
        return nullptr;
    }
    auto* ex = new (::std::nothrow) ::phy_engine::phy_lab_wrapper::experiment(::std::move(r.value->ex));
    if(ex == nullptr)
    {
        set_phy_engine_error("pe_to_pl_convert: allocation failed");
        return nullptr;
    }
    return ex;
}

extern "C" int pl_experiment_auto_layout(void* experiment_ptr,
                                         double corner0_x,
                                         double corner0_y,
                                         double corner0_z,
                                         double corner1_x,
                                         double corner1_y,
                                         double corner1_z,
                                         double z_fixed,
                                         int backend_value,
                                         int mode_value,
                                         double step_x,
                                         double step_y,
                                         double margin_x,
                                         double margin_y,
                                         ::std::size_t* out_grid_w,
                                         ::std::size_t* out_grid_h,
                                         ::std::size_t* out_fixed_obstacles,
                                         ::std::size_t* out_placed,
                                         ::std::size_t* out_skipped)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    auto* ex = static_cast<::phy_engine::phy_lab_wrapper::experiment*>(experiment_ptr);
    if(ex == nullptr)
    {
        set_phy_engine_error("pl_experiment_auto_layout: experiment is null");
        return 1;
    }

    ::phy_engine::phy_lab_wrapper::auto_layout::options opt{};
    opt.backend_kind = static_cast<::phy_engine::phy_lab_wrapper::auto_layout::backend>(backend_value);
    opt.layout_mode = static_cast<::phy_engine::phy_lab_wrapper::auto_layout::mode>(mode_value);
    opt.step_x = step_x;
    opt.step_y = step_y;
    opt.margin_x = margin_x;
    opt.margin_y = margin_y;

    auto r = ::phy_engine::phy_lab_wrapper::auto_layout::layout_ec(
        *ex,
        ::phy_engine::phy_lab_wrapper::position{corner0_x, corner0_y, corner0_z},
        ::phy_engine::phy_lab_wrapper::position{corner1_x, corner1_y, corner1_z},
        z_fixed,
        opt);
    if(!r)
    {
        set_phy_engine_error(r.st.message);
        return 2;
    }
    if(out_grid_w != nullptr) { *out_grid_w = r.value->grid_w; }
    if(out_grid_h != nullptr) { *out_grid_h = r.value->grid_h; }
    if(out_fixed_obstacles != nullptr) { *out_fixed_obstacles = r.value->fixed_obstacles; }
    if(out_placed != nullptr) { *out_placed = r.value->placed; }
    if(out_skipped != nullptr) { *out_skipped = r.value->skipped; }
    return 0;
}

// Union-Find find function
static int uf_find(int x, int* parent, int* visited)
{
    if(parent[x] != x) { parent[x] = uf_find(parent[x], parent, visited); }
    visited[x] = 1;
    return parent[x];
}

// Build netlist from wires
void build_netlist_from_wires(phy_engine::netlist::netlist& nl,
                              int* elements,
                              int ele_size,
                              int* wires,
                              int wire_count,
                              ::phy_engine::netlist::model_pos* model_pos_arr)
{
    if(elements == nullptr || wires == nullptr || model_pos_arr == nullptr) { return; }
    if(ele_size <= 0 || wire_count <= 0) { return; }

    // Compute per-element pin counts from the actual model pin views. This avoids relying on a fixed MAX_PINS.
    ::std::size_t* base_offset = static_cast<::std::size_t*>(::std::calloc(static_cast<::std::size_t>(ele_size) + 1, sizeof(::std::size_t)));
    ::std::size_t* pin_count = static_cast<::std::size_t*>(::std::calloc(static_cast<::std::size_t>(ele_size), sizeof(::std::size_t)));
    if(base_offset == nullptr || pin_count == nullptr)
    {
        ::std::free(base_offset);
        ::std::free(pin_count);
        return;
    }

    int comp_id_for_pin_scan = 0;
    for(int ele_id = 0; ele_id < ele_size; ++ele_id)
    {
        if(!elements[ele_id])
        {
            pin_count[ele_id] = 0;
            continue;
        }
        auto model = get_model(nl, model_pos_arr[comp_id_for_pin_scan]);
        if(model == nullptr || model->ptr == nullptr)
        {
            pin_count[ele_id] = 0;
            ++comp_id_for_pin_scan;
            continue;
        }
        auto const pv = model->ptr->generate_pin_view();
        pin_count[ele_id] = pv.size;
        ++comp_id_for_pin_scan;
    }

    for(int ele_id = 0; ele_id < ele_size; ++ele_id) { base_offset[ele_id + 1] = base_offset[ele_id] + pin_count[ele_id]; }

    int const total_nodes = static_cast<int>(base_offset[ele_size]);

    // Extended array: add a ground node slot
    int const GROUND_NODE_ID = total_nodes;  // Ground node index

    int* parent = static_cast<int*>(malloc((static_cast<::std::size_t>(total_nodes) + 1) * sizeof(int)));  // +1 for ground node
    int* visited = static_cast<int*>(calloc(static_cast<::std::size_t>(total_nodes) + 1, sizeof(int)));
    if(parent == nullptr || visited == nullptr)
    {
        ::std::free(base_offset);
        ::std::free(pin_count);
        ::std::free(parent);
        ::std::free(visited);
        return;
    }

    // Initialize Union-Find
    for(int i = 0; i <= total_nodes; i++) { parent[i] = i; }

    // Process all wire connections
    for(int i = 0; i < wire_count; i++)
    {
        int ele1 = wires[i * 4];
        int pin1 = wires[i * 4 + 1];
        int ele2 = wires[i * 4 + 2];
        int pin2 = wires[i * 4 + 3];

        if(ele1 < 0 || ele2 < 0 || ele1 >= ele_size || ele2 >= ele_size) { continue; }

        // Check if it's a ground element
        int node1, node2;

        if(!elements[ele1])
        {
            node1 = GROUND_NODE_ID;  // Connect to ground node
        }
        else
        {
            if(pin1 < 0 || static_cast<::std::size_t>(pin1) >= pin_count[ele1]) { continue; }
            node1 = static_cast<int>(base_offset[ele1] + static_cast<::std::size_t>(pin1));
        }

        if(!elements[ele2])
        {
            node2 = GROUND_NODE_ID;  // Connect to ground node
        }
        else
        {
            if(pin2 < 0 || static_cast<::std::size_t>(pin2) >= pin_count[ele2]) { continue; }
            node2 = static_cast<int>(base_offset[ele2] + static_cast<::std::size_t>(pin2));
        }

        int root1 = uf_find(node1, parent, visited);
        int root2 = uf_find(node2, parent, visited);
        if(root1 != root2)
        {
            // Ensure ground node is always root
            if(root1 == GROUND_NODE_ID)
            {
                parent[root2] = root1;  // Other nodes connect to ground
            }
            else if(root2 == GROUND_NODE_ID)
            {
                parent[root1] = root2;  // Other nodes connect to ground
            }
            else
            {
                parent[root2] = root1;  // Regular node merge
            }
        }
    }

    // Create node mapping table
    ::std::unordered_map<int, ::phy_engine::model::node_t*> node_map;

    // Create node_t for each connected component (excluding ground node)
    for(int i = 0; i < total_nodes; i++)
    {  // Note: excluding GROUND_NODE_ID
        if(parent[i] == i && visited[i])
        {
            // Check if connected to ground node
            int root = uf_find(i, parent, visited);
            if(root == GROUND_NODE_ID)
            {
                // Nodes connected to ground use ground node directly
                node_map[i] = &get_ground_node(nl);
            }
            else
            {
                // Create new regular node
                auto& node = create_node(nl);
                node_map[i] = &node;
            }
        }
    }

    // Add ground node to mapping table
    node_map[GROUND_NODE_ID] = &get_ground_node(nl);

    // Connect all pins to corresponding nodes
    int comp_id = 0;
    for(int ele_id = 0; ele_id < ele_size; ++ele_id)
    {
        if(!elements[ele_id]) { continue; }
        auto model = get_model(nl, model_pos_arr[comp_id]);
        if(model == nullptr || model->ptr == nullptr)
        {
            ++comp_id;
            continue;
        }

        auto pin_view = model->ptr->generate_pin_view();
        for(::std::size_t pin_id{}; pin_id < pin_view.size; pin_id++)
        {
            int node_id;

            node_id = static_cast<int>(base_offset[ele_id] + pin_id);

            // Only process actually used pins
            if(visited[node_id])
            {
                int root = uf_find(node_id, parent, visited);
                auto& node = *node_map[root];
                add_to_node(nl, model_pos_arr[comp_id], pin_id, node);
            }
        }
        ++comp_id;
    }

    ::std::free(base_offset);
    ::std::free(pin_count);
    free(parent);
    free(visited);
}

::phy_engine::netlist::add_model_retstr add_model_via_code(phy_engine::netlist::netlist& nl, int element_code, double** curr_prop_ptr)
{
    // element_code is the component code
    if(curr_prop_ptr == nullptr || *curr_prop_ptr == nullptr) { return {}; }
    switch(element_code)
    {
        // For each case, the syntax should be
        // add_model(nl, ::phy_engine::model::[model_name]{.[attribute1] = *((*curr_prop_ptr)++), .[attribute2] = *((*curr_prop_ptr)++), ...});
        // The rest will auto-increment
        case 1:
            // Resistor
            return add_model(nl, ::phy_engine::model::resistance{.r = *((*curr_prop_ptr)++)});
        case 2:
            // Capacitor
            return add_model(nl, ::phy_engine::model::capacitor{.m_kZimag = *((*curr_prop_ptr)++)});
        case 3:
            // Inductor
            return add_model(nl, ::phy_engine::model::inductor{.m_kZimag = *((*curr_prop_ptr)++)});
        case 4:
            // VDC
            return add_model(nl, ::phy_engine::model::VDC{.V = *((*curr_prop_ptr)++)});
        case 5:
        {
            // VAC: Vp, freq(Hz), phase(deg)
            double const vp = *((*curr_prop_ptr)++);
            double const freq = *((*curr_prop_ptr)++);
            double const phase_deg = *((*curr_prop_ptr)++);
            return add_model(nl,
                             ::phy_engine::model::VAC{
                                 .m_Vp = vp,
                                 .m_omega = freq * (2.0 * ::std::numbers::pi),
                                 .m_phase = phase_deg * (::std::numbers::pi / 180.0),
                             });
        }
        case 6:
            // IDC
            return add_model(nl, ::phy_engine::model::IDC{.I = *((*curr_prop_ptr)++)});
        case 7:
        {
            // IAC: Ip, freq(Hz), phase(deg)
            double const ip = *((*curr_prop_ptr)++);
            double const freq = *((*curr_prop_ptr)++);
            double const phase_deg = *((*curr_prop_ptr)++);
            return add_model(nl,
                             ::phy_engine::model::IAC{
                                 .m_Ip = ip,
                                 .m_omega = freq * (2.0 * ::std::numbers::pi),
                                 .m_phase = phase_deg * (::std::numbers::pi / 180.0),
                             });
        }
        case 8:
            // VCCS
            return add_model(nl, ::phy_engine::model::VCCS{.m_g = *((*curr_prop_ptr)++)});
        case 9:
            // VCVS
            return add_model(nl, ::phy_engine::model::VCVS{.m_mu = *((*curr_prop_ptr)++)});
        case 10:
            // CCCS
            return add_model(nl, ::phy_engine::model::CCCS{.m_alpha = *((*curr_prop_ptr)++)});
        case 11:
            // CCVS
            return add_model(nl, ::phy_engine::model::CCVS{.m_r = *((*curr_prop_ptr)++)});
        case 12:
        {
            // single pole switch: 0/1
            double const v = *((*curr_prop_ptr)++);
            return add_model(nl, ::phy_engine::model::single_pole_switch{.cut_through = (v != 0.0)});
        }
        case 13:
        {
            // PN junction: Is, N, Isr, Nr, Temp, Ibv, Bv, Bv_set(0/1), Area
            ::phy_engine::model::PN_junction pn{};
            pn.Is = *((*curr_prop_ptr)++);
            pn.N = *((*curr_prop_ptr)++);
            pn.Isr = *((*curr_prop_ptr)++);
            pn.Nr = *((*curr_prop_ptr)++);
            pn.Temp = *((*curr_prop_ptr)++);
            pn.Ibv = *((*curr_prop_ptr)++);
            pn.Bv = *((*curr_prop_ptr)++);
            pn.Bv_set = (*((*curr_prop_ptr)++) != 0.0);
            pn.Area = *((*curr_prop_ptr)++);
            return add_model(nl, ::std::move(pn));
        }
        case 14:
        {
            // Transformer: n
            return add_model(nl, ::phy_engine::model::transformer{.n = *((*curr_prop_ptr)++)});
        }
        case 15:
        {
            // Coupled inductors: L1, L2, k
            ::phy_engine::model::coupled_inductors kl{};
            kl.L1 = *((*curr_prop_ptr)++);
            kl.L2 = *((*curr_prop_ptr)++);
            kl.k = *((*curr_prop_ptr)++);
            return add_model(nl, ::std::move(kl));
        }
        case 16:
        {
            // Transformer center tap: n_total
            return add_model(nl, ::phy_engine::model::transformer_center_tap{.n_total = *((*curr_prop_ptr)++)});
        }
        case 17:
        {
            // OpAmp: mu
            return add_model(nl, ::phy_engine::model::op_amp{.mu = *((*curr_prop_ptr)++)});
        }
        case 18:
        {
            // Relay: Von, Voff
            ::phy_engine::model::relay r{};
            r.Von = *((*curr_prop_ptr)++);
            r.Voff = *((*curr_prop_ptr)++);
            return add_model(nl, ::std::move(r));
        }
        case 19:
        {
            // Comparator: Ll, Hl
            ::phy_engine::model::comparator c{};
            c.Ll = *((*curr_prop_ptr)++);
            c.Hl = *((*curr_prop_ptr)++);
            return add_model(nl, ::std::move(c));
        }
        case 20:
        {
            // Sawtooth generator: Vh, Vl, freq(Hz), phase(rad)
            ::phy_engine::model::sawtooth_gen g{};
            g.Vh = *((*curr_prop_ptr)++);
            g.Vl = *((*curr_prop_ptr)++);
            g.freq = *((*curr_prop_ptr)++);
            g.phase = *((*curr_prop_ptr)++);
            return add_model(nl, ::std::move(g));
        }
        case 21:
        {
            // Square generator: Vh, Vl, freq(Hz), duty, phase(rad)
            ::phy_engine::model::square_gen g{};
            g.Vh = *((*curr_prop_ptr)++);
            g.Vl = *((*curr_prop_ptr)++);
            g.freq = *((*curr_prop_ptr)++);
            g.duty = *((*curr_prop_ptr)++);
            g.phase = *((*curr_prop_ptr)++);
            return add_model(nl, ::std::move(g));
        }
        case 22:
        {
            // Pulse generator: Vh, Vl, freq(Hz), duty, phase(rad), tr, tf
            ::phy_engine::model::pulse_gen g{};
            g.Vh = *((*curr_prop_ptr)++);
            g.Vl = *((*curr_prop_ptr)++);
            g.freq = *((*curr_prop_ptr)++);
            g.duty = *((*curr_prop_ptr)++);
            g.phase = *((*curr_prop_ptr)++);
            g.tr = *((*curr_prop_ptr)++);
            g.tf = *((*curr_prop_ptr)++);
            return add_model(nl, ::std::move(g));
        }
        case 23:
        {
            // Triangle generator: Vh, Vl, freq(Hz), phase(rad)
            ::phy_engine::model::triangle_gen g{};
            g.Vh = *((*curr_prop_ptr)++);
            g.Vl = *((*curr_prop_ptr)++);
            g.freq = *((*curr_prop_ptr)++);
            g.phase = *((*curr_prop_ptr)++);
            return add_model(nl, ::std::move(g));
        }
        case 50:
        {
            // NPN BJT: Is, N, BetaF, Temp, Area
            ::phy_engine::model::BJT_NPN q{};
            q.Is = *((*curr_prop_ptr)++);
            q.N = *((*curr_prop_ptr)++);
            q.BetaF = *((*curr_prop_ptr)++);
            q.Temp = *((*curr_prop_ptr)++);
            q.Area = *((*curr_prop_ptr)++);
            return add_model(nl, ::std::move(q));
        }
        case 51:
        {
            // PNP BJT: Is, N, BetaF, Temp, Area
            ::phy_engine::model::BJT_PNP q{};
            q.Is = *((*curr_prop_ptr)++);
            q.N = *((*curr_prop_ptr)++);
            q.BetaF = *((*curr_prop_ptr)++);
            q.Temp = *((*curr_prop_ptr)++);
            q.Area = *((*curr_prop_ptr)++);
            return add_model(nl, ::std::move(q));
        }
        case 52:
        {
            // NMOSFET: Kp, lambda, Vth
            ::phy_engine::model::nmosfet m{};
            m.Kp = *((*curr_prop_ptr)++);
            m.lambda = *((*curr_prop_ptr)++);
            m.Vth = *((*curr_prop_ptr)++);
            return add_model(nl, ::std::move(m));
        }
        case 53:
        {
            // PMOSFET: Kp, lambda, Vth
            ::phy_engine::model::pmosfet m{};
            m.Kp = *((*curr_prop_ptr)++);
            m.lambda = *((*curr_prop_ptr)++);
            m.Vth = *((*curr_prop_ptr)++);
            return add_model(nl, ::std::move(m));
        }
        case 54:
        {
            // Full bridge rectifier (uses internal default diode params)
            return add_model(nl, ::phy_engine::model::full_bridge_rectifier{});
        }
        case 55:
        {
            // BSIM3v3.2 NMOS (compat skeleton): W,L,Kp,lambda,Vth0,gamma,phi,Cgs,Cgd,Cgb,diode_Is,diode_N,Temp
            ::phy_engine::model::bsim3v32_nmos m{};
            m.W = *((*curr_prop_ptr)++);
            m.L = *((*curr_prop_ptr)++);
            m.Kp = *((*curr_prop_ptr)++);
            m.lambda = *((*curr_prop_ptr)++);
            m.Vth0 = *((*curr_prop_ptr)++);
            m.gamma = *((*curr_prop_ptr)++);
            m.phi = *((*curr_prop_ptr)++);
            m.Cgs = *((*curr_prop_ptr)++);
            m.Cgd = *((*curr_prop_ptr)++);
            m.Cgb = *((*curr_prop_ptr)++);
            m.diode_Is = *((*curr_prop_ptr)++);
            m.diode_N = *((*curr_prop_ptr)++);
            m.Temp = *((*curr_prop_ptr)++);
            return add_model(nl, ::std::move(m));
        }
        case 56:
        {
            // BSIM3v3.2 PMOS (compat skeleton): W,L,Kp,lambda,Vth0,gamma,phi,Cgs,Cgd,Cgb,diode_Is,diode_N,Temp
            ::phy_engine::model::bsim3v32_pmos m{};
            m.W = *((*curr_prop_ptr)++);
            m.L = *((*curr_prop_ptr)++);
            m.Kp = *((*curr_prop_ptr)++);
            m.lambda = *((*curr_prop_ptr)++);
            m.Vth0 = *((*curr_prop_ptr)++);
            m.gamma = *((*curr_prop_ptr)++);
            m.phi = *((*curr_prop_ptr)++);
            m.Cgs = *((*curr_prop_ptr)++);
            m.Cgd = *((*curr_prop_ptr)++);
            m.Cgb = *((*curr_prop_ptr)++);
            m.diode_Is = *((*curr_prop_ptr)++);
            m.diode_N = *((*curr_prop_ptr)++);
            m.Temp = *((*curr_prop_ptr)++);
            return add_model(nl, ::std::move(m));
        }
        case 200:
        {
            // Digital INPUT: state (0=L,1=H,2=X,3=Z)
            int const st = static_cast<int>(*(((*curr_prop_ptr)++)));
            ::phy_engine::model::digital_node_statement_t dv{::phy_engine::model::digital_node_statement_t::X};
            switch(st)
            {
                case 0: dv = ::phy_engine::model::digital_node_statement_t::L; break;
                case 1: dv = ::phy_engine::model::digital_node_statement_t::H; break;
                case 2: dv = ::phy_engine::model::digital_node_statement_t::X; break;
                case 3: dv = ::phy_engine::model::digital_node_statement_t::Z; break;
                default: dv = ::phy_engine::model::digital_node_statement_t::X; break;
            }
            return add_model(nl, ::phy_engine::model::INPUT{.outputA = dv});
        }
        case 201:
            // Digital OUTPUT
            return add_model(nl, ::phy_engine::model::OUTPUT{});
        case 202:
            // Digital OR gate
            return add_model(nl, ::phy_engine::model::OR{});
        case 203:
            // Digital YES (buffer)
            return add_model(nl, ::phy_engine::model::YES{});
        case 204:
            // Digital AND gate
            return add_model(nl, ::phy_engine::model::AND{});
        case 205:
            // Digital NOT gate
            return add_model(nl, ::phy_engine::model::NOT{});
        case 206:
            // Digital XOR gate
            return add_model(nl, ::phy_engine::model::XOR{});
        case 207:
            // Digital XNOR gate
            return add_model(nl, ::phy_engine::model::XNOR{});
        case 208:
            // Digital NAND gate
            return add_model(nl, ::phy_engine::model::NAND{});
        case 209:
            // Digital NOR gate
            return add_model(nl, ::phy_engine::model::NOR{});
        case 210:
            // Digital TRI (tri-state buffer)
            return add_model(nl, ::phy_engine::model::TRI{});
        case 211:
            // Digital IMP (implication)
            return add_model(nl, ::phy_engine::model::IMP{});
        case 212:
            // Digital NIMP (non-implication)
            return add_model(nl, ::phy_engine::model::NIMP{});
        case 220:
            // Digital HALF_ADDER
            return add_model(nl, ::phy_engine::model::HALF_ADDER{});
        case 221:
            // Digital FULL_ADDER
            return add_model(nl, ::phy_engine::model::FULL_ADDER{});
        case 222:
            // Digital HALF_SUBTRACTOR
            return add_model(nl, ::phy_engine::model::HALF_SUB{});
        case 223:
            // Digital FULL_SUBTRACTOR
            return add_model(nl, ::phy_engine::model::FULL_SUB{});
        case 224:
            // Digital MUL2
            return add_model(nl, ::phy_engine::model::MUL2{});
        case 225:
            // Digital DFF
            return add_model(nl, ::phy_engine::model::DFF{});
        case 226:
            // Digital TFF
            return add_model(nl, ::phy_engine::model::TFF{});
        case 227:
            // Digital T_BAR_FF
            return add_model(nl, ::phy_engine::model::T_BAR_FF{});
        case 228:
            // Digital JKFF
            return add_model(nl, ::phy_engine::model::JKFF{});
        case 229:
        {
            // Digital COUNTER4: init_value (0..15)
            double const v = *((*curr_prop_ptr)++);
            auto const init = (v < 0.0) ? 0u : (v > 15.0 ? 15u : static_cast<unsigned>(v));
            return add_model(nl, ::phy_engine::model::COUNTER4{.value = static_cast<::std::uint8_t>(init)});
        }
        case 230:
        {
            // Digital RANDOM_GENERATOR4: init_state (0..15)
            double const v = *((*curr_prop_ptr)++);
            auto const init = (v < 0.0) ? 0u : (v > 15.0 ? 15u : static_cast<unsigned>(v));
            return add_model(nl, ::phy_engine::model::RANDOM_GENERATOR4{.state = static_cast<::std::uint8_t>(init)});
        }
        case 231:
        {
            // Digital EIGHT_BIT_INPUT: value (0..255)
            double const v = *((*curr_prop_ptr)++);
            auto const init = (v < 0.0) ? 0u : (v > 255.0 ? 255u : static_cast<unsigned>(v));
            return add_model(nl, ::phy_engine::model::EIGHT_BIT_INPUT{.value = static_cast<::std::uint8_t>(init)});
        }
        case 232:
            // Digital EIGHT_BIT_DISPLAY
            return add_model(nl, ::phy_engine::model::EIGHT_BIT_DISPLAY{});
        case 233:
        {
            // Digital SCHMITT_TRIGGER: Vth_low,Vth_high,inverted(0/1),Ll,Hl
            ::phy_engine::model::SCHMITT_TRIGGER s{};
            s.Vth_low = *((*curr_prop_ptr)++);
            s.Vth_high = *((*curr_prop_ptr)++);
            s.inverted = (*((*curr_prop_ptr)++) != 0.0);
            s.Ll = *((*curr_prop_ptr)++);
            s.Hl = *((*curr_prop_ptr)++);
            return add_model(nl, ::std::move(s));
        }
        default:
            // ...to be continued later
            return {};
    }
}

namespace
{
    // Extended element code for Verilog module creation through create_circuit_ex().
    inline constexpr int PE_ELEMENT_VERILOG_MODULE = 300;
    inline constexpr int PE_ELEMENT_VERILOG_NETLIST = 301;

    struct verilog_text_tables
    {
        char const* const* texts{};
        ::std::size_t const* sizes{};
        ::std::size_t text_count{};
        ::std::size_t const* element_src_index{};  // length ele_size; out-of-range => no verilog
        ::std::size_t const* element_top_index{};  // length ele_size; out-of-range => default "top"
        ::std::size_t ele_size{};
    };

    inline ::fast_io::u8string_view get_u8_text(verilog_text_tables const& vt, ::std::size_t idx) noexcept
    {
        if(vt.texts == nullptr || vt.sizes == nullptr) { return {}; }
        if(idx >= vt.text_count) { return {}; }
        auto const* p = vt.texts[idx];
        auto const n = vt.sizes[idx];
        if(p == nullptr || n == 0) { return {}; }
        return ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(p), n};
    }

    inline ::fast_io::u8string_view get_verilog_src(verilog_text_tables const& vt, ::std::size_t ele_id) noexcept
    {
        if(vt.element_src_index == nullptr || ele_id >= vt.ele_size) { return {}; }
        return get_u8_text(vt, vt.element_src_index[ele_id]);
    }

    inline ::fast_io::u8string_view get_verilog_top(verilog_text_tables const& vt, ::std::size_t ele_id) noexcept
    {
        if(vt.element_top_index == nullptr || ele_id >= vt.ele_size) { return {}; }
        return get_u8_text(vt, vt.element_top_index[ele_id]);
    }

    inline ::phy_engine::netlist::add_model_retstr add_model_via_code_ex(::phy_engine::netlist::netlist& nl,
                                                                         int element_code,
                                                                         double** curr_prop_ptr,
                                                                         verilog_text_tables const* vt,
                                                                         ::std::size_t ele_id) noexcept
    {
        if(element_code == PE_ELEMENT_VERILOG_MODULE)
        {
            if(vt == nullptr) { return {}; }
            auto const src = get_verilog_src(*vt, ele_id);
            if(src.empty()) { return {}; }
            auto top = get_verilog_top(*vt, ele_id);
            if(top.empty())
            {
                static constexpr char8_t fallback_top[] = u8"top";
                top = ::fast_io::u8string_view{fallback_top, sizeof(fallback_top) - 1};
            }
            return add_model(nl, ::phy_engine::model::make_verilog_module(src, top));
        }

        // Otherwise, fall back to the numeric-property based creator.
        return add_model_via_code(nl, element_code, curr_prop_ptr);
    }
}  // namespace

extern "C" void destroy_circuit(void* circuit_ptr, ::std::size_t* vec_pos, ::std::size_t* chunk_pos);

extern "C" int circuit_set_analyze_type(void* circuit_ptr, ::std::uint32_t analyze_type_value)
{
    if(circuit_ptr == nullptr) { return 1; }
    auto* c = static_cast<::phy_engine::circult*>(circuit_ptr);
    c->set_analyze_type(static_cast<::phy_engine::analyze_type>(analyze_type_value));
    return 0;
}

extern "C" int circuit_set_tr(void* circuit_ptr, double t_step, double t_stop)
{
    if(circuit_ptr == nullptr) { return 1; }
    auto* c = static_cast<::phy_engine::circult*>(circuit_ptr);
    c->get_analyze_setting().tr.t_step = t_step;
    c->get_analyze_setting().tr.t_stop = t_stop;
    return 0;
}

extern "C" int circuit_set_ac_omega(void* circuit_ptr, double omega)
{
    if(circuit_ptr == nullptr) { return 1; }
    auto* c = static_cast<::phy_engine::circult*>(circuit_ptr);
    c->get_analyze_setting().ac.sweep = ::phy_engine::analyzer::AC::sweep_type::single;
    c->get_analyze_setting().ac.omega = omega;
    return 0;
}

extern "C" int circuit_set_temperature(void* circuit_ptr, double temp_c)
{
    if(circuit_ptr == nullptr) { return 1; }
    auto* c = static_cast<::phy_engine::circult*>(circuit_ptr);
    c->get_environment().temperature = temp_c;
    return 0;
}

extern "C" int circuit_set_tnom(void* circuit_ptr, double tnom_c)
{
    if(circuit_ptr == nullptr) { return 1; }
    auto* c = static_cast<::phy_engine::circult*>(circuit_ptr);
    c->get_environment().norm_temperature = tnom_c;
    return 0;
}

void set_property(phy_engine::model::model_base* model, ::std::size_t index, double property);

namespace
{
    inline constexpr bool ascii_ieq(char a, char b) noexcept
    {
        auto const ua = static_cast<unsigned char>(a);
        auto const ub = static_cast<unsigned char>(b);
        auto const la = (ua >= 'A' && ua <= 'Z') ? static_cast<unsigned char>(ua + ('a' - 'A')) : ua;
        auto const lb = (ub >= 'A' && ub <= 'Z') ? static_cast<unsigned char>(ub + ('a' - 'A')) : ub;
        return la == lb;
    }

    inline bool ascii_ieq_span(::fast_io::u8string_view a, char const* b, ::std::size_t bsz) noexcept
    {
        if(b == nullptr) { return false; }
        if(a.size() != bsz) { return false; }
        auto const* ap = reinterpret_cast<char const*>(a.data());
        for(::std::size_t i{}; i < bsz; ++i)
        {
            if(!ascii_ieq(ap[i], b[i])) { return false; }
        }
        return true;
    }

    inline bool
        find_attribute_index_by_name(::phy_engine::model::model_base* model, char const* name, ::std::size_t name_size, ::std::size_t& out_index) noexcept
    {
        if(model == nullptr || model->ptr == nullptr || name == nullptr || name_size == 0) { return false; }

        // Scan attribute indices with a conservative budget. Most models have small sparse index sets.
        constexpr ::std::size_t kMaxScan{2048};
        constexpr ::std::size_t kMaxConsecutiveEmpty{128};
        ::std::size_t empty_run{};
        bool seen_any{};
        for(::std::size_t idx{}; idx < kMaxScan; ++idx)
        {
            ::fast_io::u8string_view const n = model->ptr->get_attribute_name(idx);
            if(n.empty())
            {
                if(seen_any && ++empty_run >= kMaxConsecutiveEmpty) { break; }
                continue;
            }
            seen_any = true;
            empty_run = 0;
            if(ascii_ieq_span(n, name, name_size))
            {
                out_index = idx;
                return true;
            }
        }
        return false;
    }
}  // namespace

extern "C" int
    circuit_set_model_double_by_name(void* circuit_ptr, ::std::size_t vec_pos, ::std::size_t chunk_pos, char const* name, ::std::size_t name_size, double value)
{
    if(circuit_ptr == nullptr || name == nullptr || name_size == 0) { return 1; }
    auto* c = static_cast<::phy_engine::circult*>(circuit_ptr);
    auto& nl{c->get_netlist()};

    ::phy_engine::model::model_base* model = get_model(nl, ::phy_engine::netlist::model_pos{vec_pos, chunk_pos});
    if(model == nullptr || model->ptr == nullptr) { return 2; }

    ::std::size_t idx{};
    if(!find_attribute_index_by_name(model, name, name_size, idx)) { return 3; }
    set_property(model, idx, value);
    return 0;
}

extern "C" int circuit_analyze(void* circuit_ptr)
{
    if(circuit_ptr == nullptr) { return 1; }
    auto* c = static_cast<::phy_engine::circult*>(circuit_ptr);
    return c->analyze() ? 0 : 1;
}

extern "C" int circuit_digital_clk(void* circuit_ptr)
{
    if(circuit_ptr == nullptr) { return 1; }
    auto* c = static_cast<::phy_engine::circult*>(circuit_ptr);
    c->digital_clk();
    return 0;
}

extern "C" int circuit_sample_layout(void* circuit_ptr,
                                     ::std::size_t* vec_pos,
                                     ::std::size_t* chunk_pos,
                                     ::std::size_t comp_size,
                                     ::std::size_t* voltage_ord,
                                     ::std::size_t* current_ord,
                                     ::std::size_t* digital_ord)
{
    if(circuit_ptr == nullptr || vec_pos == nullptr || chunk_pos == nullptr || voltage_ord == nullptr || current_ord == nullptr || digital_ord == nullptr)
    {
        return 1;
    }

    auto* c = static_cast<::phy_engine::circult*>(circuit_ptr);
    auto& nl{c->get_netlist()};

    voltage_ord[0] = current_ord[0] = digital_ord[0] = 0;
    for(::std::size_t i{}; i < comp_size; ++i)
    {
        auto* model = get_model(nl, ::phy_engine::netlist::model_pos{vec_pos[i], chunk_pos[i]});
        if(model == nullptr || model->ptr == nullptr)
        {
            voltage_ord[i + 1] = voltage_ord[i];
            current_ord[i + 1] = current_ord[i];
            digital_ord[i + 1] = digital_ord[i];
            continue;
        }

        auto const model_pin_view{model->ptr->generate_pin_view()};
        auto const model_branch_view{model->ptr->generate_branch_view()};
        voltage_ord[i + 1] = voltage_ord[i] + model_pin_view.size;
        current_ord[i + 1] = current_ord[i] + model_branch_view.size;
        digital_ord[i + 1] = digital_ord[i] + model_pin_view.size;
    }

    return 0;
}

extern "C" int circuit_sample(void* circuit_ptr,
                              ::std::size_t* vec_pos,
                              ::std::size_t* chunk_pos,
                              ::std::size_t comp_size,
                              double* voltage,
                              ::std::size_t* voltage_ord,
                              double* current,
                              ::std::size_t* current_ord,
                              bool* digital,
                              ::std::size_t* digital_ord)
{
    if(circuit_ptr == nullptr || vec_pos == nullptr || chunk_pos == nullptr || voltage == nullptr || voltage_ord == nullptr || current == nullptr ||
       current_ord == nullptr || digital == nullptr || digital_ord == nullptr)
    {
        return 1;
    }

    auto* c = static_cast<::phy_engine::circult*>(circuit_ptr);
    auto& nl{c->get_netlist()};

    if(circuit_sample_layout(circuit_ptr, vec_pos, chunk_pos, comp_size, voltage_ord, current_ord, digital_ord) != 0) { return 1; }
    for(::std::size_t i{}; i < comp_size; ++i)
    {
        phy_engine::model::model_base* model = get_model(nl, ::phy_engine::netlist::model_pos{vec_pos[i], chunk_pos[i]});
        if(model == nullptr || model->ptr == nullptr)
        {
            continue;
        }

        auto const model_pin_view{model->ptr->generate_pin_view()};
        auto const model_branch_view{model->ptr->generate_branch_view()};

        for(::std::size_t j{}; j < model_pin_view.size; ++j)
        {
            auto const* node{model_pin_view.pins[j].nodes};
            voltage[j + voltage_ord[i]] = (node != nullptr) ? node->node_information.an.voltage.real() : 0.0;
        }
        for(::std::size_t j{}; j < model_branch_view.size; ++j) { current[j + current_ord[i]] = model_branch_view.branches[j].current.real(); }

        for(::std::size_t j{}; j < model_pin_view.size; ++j)
        {
            auto const* node{model_pin_view.pins[j].nodes};
            bool v{};
            if(node != nullptr && node->num_of_analog_node == 0)
            {
                v = (node->node_information.dn.state == ::phy_engine::model::digital_node_statement_t::true_state);
            }
            digital[j + digital_ord[i]] = v;
        }
    }

    return 0;
}

extern "C" int circuit_sample_u8(void* circuit_ptr,
                                 ::std::size_t* vec_pos,
                                 ::std::size_t* chunk_pos,
                                 ::std::size_t comp_size,
                                 double* voltage,
                                 ::std::size_t* voltage_ord,
                                 double* current,
                                 ::std::size_t* current_ord,
                                 ::std::uint8_t* digital,
                                 ::std::size_t* digital_ord)
{
    if(circuit_ptr == nullptr || vec_pos == nullptr || chunk_pos == nullptr || voltage == nullptr || voltage_ord == nullptr || current == nullptr ||
       current_ord == nullptr || digital == nullptr || digital_ord == nullptr)
    {
        return 1;
    }

    auto* c = static_cast<::phy_engine::circult*>(circuit_ptr);
    auto& nl{c->get_netlist()};

    if(circuit_sample_layout(circuit_ptr, vec_pos, chunk_pos, comp_size, voltage_ord, current_ord, digital_ord) != 0) { return 1; }
    for(::std::size_t i{}; i < comp_size; ++i)
    {
        phy_engine::model::model_base* model = get_model(nl, ::phy_engine::netlist::model_pos{vec_pos[i], chunk_pos[i]});
        if(model == nullptr || model->ptr == nullptr)
        {
            continue;
        }

        auto const model_pin_view{model->ptr->generate_pin_view()};
        auto const model_branch_view{model->ptr->generate_branch_view()};

        for(::std::size_t j{}; j < model_pin_view.size; ++j)
        {
            auto const* node{model_pin_view.pins[j].nodes};
            voltage[j + voltage_ord[i]] = (node != nullptr) ? node->node_information.an.voltage.real() : 0.0;
        }
        for(::std::size_t j{}; j < model_branch_view.size; ++j) { current[j + current_ord[i]] = model_branch_view.branches[j].current.real(); }

        for(::std::size_t j{}; j < model_pin_view.size; ++j)
        {
            auto const* node{model_pin_view.pins[j].nodes};
            ::std::uint8_t v{};
            if(node != nullptr && node->num_of_analog_node == 0)
            {
                v = (node->node_information.dn.state == ::phy_engine::model::digital_node_statement_t::true_state) ? 1 : 0;
            }
            digital[j + digital_ord[i]] = v;
        }
    }

    return 0;
}

extern "C" int circuit_sample_digital_state_u8(void* circuit_ptr,
                                               ::std::size_t* vec_pos,
                                               ::std::size_t* chunk_pos,
                                               ::std::size_t comp_size,
                                               double* voltage,
                                               ::std::size_t* voltage_ord,
                                               double* current,
                                               ::std::size_t* current_ord,
                                               ::std::uint8_t* digital,
                                               ::std::size_t* digital_ord)
{
    if(circuit_ptr == nullptr || vec_pos == nullptr || chunk_pos == nullptr || voltage == nullptr || voltage_ord == nullptr || current == nullptr ||
       current_ord == nullptr || digital == nullptr || digital_ord == nullptr)
    {
        return 1;
    }

    auto* c = static_cast<::phy_engine::circult*>(circuit_ptr);
    auto& nl{c->get_netlist()};

    if(circuit_sample_layout(circuit_ptr, vec_pos, chunk_pos, comp_size, voltage_ord, current_ord, digital_ord) != 0) { return 1; }
    for(::std::size_t i{}; i < comp_size; ++i)
    {
        phy_engine::model::model_base* model = get_model(nl, ::phy_engine::netlist::model_pos{vec_pos[i], chunk_pos[i]});
        if(model == nullptr || model->ptr == nullptr) { continue; }

        auto const model_pin_view{model->ptr->generate_pin_view()};
        auto const model_branch_view{model->ptr->generate_branch_view()};

        for(::std::size_t j{}; j < model_pin_view.size; ++j)
        {
            auto const* node{model_pin_view.pins[j].nodes};
            voltage[j + voltage_ord[i]] = (node != nullptr) ? node->node_information.an.voltage.real() : 0.0;
        }

        for(::std::size_t j{}; j < model_branch_view.size; ++j) { current[j + current_ord[i]] = model_branch_view.branches[j].current.real(); }

        for(::std::size_t j{}; j < model_pin_view.size; ++j)
        {
            auto const* node{model_pin_view.pins[j].nodes};
            ::std::uint8_t v{static_cast<::std::uint8_t>(::phy_engine::model::digital_node_statement_t::X)};
            if(node != nullptr && node->num_of_analog_node == 0)
            {
                v = static_cast<::std::uint8_t>(node->node_information.dn.state);
            }
            digital[j + digital_ord[i]] = v;
        }
    }

    return 0;
}

extern "C" int circuit_set_model_digital(void* circuit_ptr, ::std::size_t vec_pos, ::std::size_t chunk_pos, ::std::size_t attribute_index, ::std::uint8_t state)
{
    if(circuit_ptr == nullptr) { return 1; }
    auto* c = static_cast<::phy_engine::circult*>(circuit_ptr);
    auto& nl{c->get_netlist()};

    auto* model = get_model(nl, ::phy_engine::netlist::model_pos{vec_pos, chunk_pos});
    if(model == nullptr || model->ptr == nullptr) { return 2; }

    ::phy_engine::model::digital_node_statement_t dv{::phy_engine::model::digital_node_statement_t::X};
    switch(state)
    {
        case 0: dv = ::phy_engine::model::digital_node_statement_t::L; break;
        case 1: dv = ::phy_engine::model::digital_node_statement_t::H; break;
        case 2: dv = ::phy_engine::model::digital_node_statement_t::X; break;
        case 3: dv = ::phy_engine::model::digital_node_statement_t::Z; break;
        default: dv = ::phy_engine::model::digital_node_statement_t::X; break;
    }

    ::phy_engine::model::variant vi{};
    vi.digital = dv;
    vi.type = ::phy_engine::model::variant_type::digital;
    return model->ptr->set_attribute(attribute_index, vi) ? 0 : 3;
}

extern "C" void* create_circuit(int* elements,
                                ::std::size_t ele_size,
                                int* wires,
                                ::std::size_t wires_size,
                                double* properties,
                                ::std::size_t** vec_pos,
                                ::std::size_t** chunk_pos,
                                ::std::size_t* comp_size)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    // TODO In future versions, perhaps 0 (ground element) should not be allowed in elements
    if(vec_pos == nullptr || chunk_pos == nullptr || comp_size == nullptr)
    {
        set_phy_engine_error("create_circuit: output pointers are null");
        return nullptr;
    }
    *vec_pos = nullptr;
    *chunk_pos = nullptr;
    *comp_size = 0;
    if(elements == nullptr || properties == nullptr)
    {
        set_phy_engine_error("create_circuit: elements/properties are null");
        return nullptr;
    }
    *vec_pos = static_cast<::std::size_t*>(malloc(ele_size * sizeof(::std::size_t)));
    *chunk_pos = static_cast<::std::size_t*>(malloc(ele_size * sizeof(::std::size_t)));
    if(*vec_pos == nullptr || *chunk_pos == nullptr)
    {
        set_phy_engine_error("create_circuit: failed to allocate component position arrays");
        ::std::free(*vec_pos);
        ::std::free(*chunk_pos);
        *vec_pos = nullptr;
        *chunk_pos = nullptr;
        return nullptr;
    }
    ::phy_engine::circult* c = static_cast<::phy_engine::circult*>(std::malloc(sizeof(::phy_engine::circult)));
    if(c == nullptr) [[unlikely]] { ::fast_io::fast_terminate(); }
    ::std::construct_at(c);

    c->set_analyze_type(::phy_engine::analyze_type::TR);
    auto& setting{c->get_analyze_setting()};

    // Step size setting
    setting.tr.t_step = 1e-6;
    setting.tr.t_stop = 1e-6;

    auto& nl{c->get_netlist()};

    phy_engine::netlist::model_pos* model_pos_arr = static_cast<phy_engine::netlist::model_pos*>(malloc(ele_size * sizeof(phy_engine::netlist::model_pos)));
    if(model_pos_arr == nullptr)
    {
        set_phy_engine_error("create_circuit: failed to allocate model_pos array");
        destroy_circuit(c, *vec_pos, *chunk_pos);
        *vec_pos = nullptr;
        *chunk_pos = nullptr;
        return nullptr;
    }

    double* curr_prop = properties;  // current 'properties', for we don't know the length of properties
    ::std::size_t curr_i{};  // curr_i distinguishes from i, indicating which non-ground element // TODO If ground elements are to be prohibited in elements,
                             // curr_i needs to be removed
    for(::std::size_t i{}; i < ele_size; ++i)
    {
        // vec_pos and chunk_pos maintain order, this is important
        if(elements[i])
        {
            auto [ele, ele_pos]{add_model_via_code(nl, elements[i], &curr_prop)};
            if(ele == nullptr)
            {
                set_phy_engine_error("create_circuit: failed to create element at index " + ::std::to_string(i) + " with code " +
                                     ::std::to_string(elements[i]));
                ::std::free(model_pos_arr);
                destroy_circuit(c, *vec_pos, *chunk_pos);
                *vec_pos = nullptr;
                *chunk_pos = nullptr;
                return nullptr;
            }
            (*vec_pos)[curr_i] = ele_pos.vec_pos;
            (*chunk_pos)[curr_i] = ele_pos.chunk_pos;
            model_pos_arr[curr_i] = ele_pos;
            ++curr_i;
        }

        /* TEMP
        if (elements[i]) { // Non-ground element
            auto [ele, ele_pos]{add_model_via_code(nl, elements[i], &curr_prop)};
            (*vec_pos)[i] = ele_pos.vec_pos;
            (*chunk_pos)[i] = ele_pos.chunk_pos;
            model_pos_arr[i] = ele_pos;
        } else { // Ground element
            (*vec_pos)[i] = 0;
            (*chunk_pos)[i] = 0;
            model_pos_arr[i] = {};
        }
        */
    }
    *comp_size = curr_i;

    // Default: wires element values won't exceed ele_size, and wires_size is a multiple of 4

    // Call build_netlist_from_wires here, use model_pos_arr to get model_pos and build components
    // TODO comp_size should be the count excluding ground elements
    // TODO Need to deeply consider whether ground elements should be included in pos
    // TODO Check build_netlist_from_wires again
    int const wire_count = static_cast<int>(wires_size / 4);
    if(wires != nullptr && wire_count > 0) { build_netlist_from_wires(nl, elements, static_cast<int>(ele_size), wires, wire_count, model_pos_arr); }

    ::std::free(model_pos_arr);

    return c;
}

extern "C" void* create_circuit_ex(int* elements,
                                   ::std::size_t ele_size,
                                   int* wires,
                                   ::std::size_t wires_size,
                                   double* properties,
                                   char const* const* texts,
                                   ::std::size_t const* text_sizes,
                                   ::std::size_t text_count,
                                   ::std::size_t const* element_src_index,
                                   ::std::size_t const* element_top_index,
                                   ::std::size_t** vec_pos,
                                   ::std::size_t** chunk_pos,
                                   ::std::size_t* comp_size)
{
    ::phy_engine::phy_lab_wrapper::detail::clear_last_error();
    if(vec_pos == nullptr || chunk_pos == nullptr || comp_size == nullptr)
    {
        set_phy_engine_error("create_circuit_ex: output pointers are null");
        return nullptr;
    }
    *vec_pos = nullptr;
    *chunk_pos = nullptr;
    *comp_size = 0;
    if(elements == nullptr || properties == nullptr)
    {
        set_phy_engine_error("create_circuit_ex: elements/properties are null");
        return nullptr;
    }

    verilog_text_tables vt{
        .texts = texts,
        .sizes = text_sizes,
        .text_count = text_count,
        .element_src_index = element_src_index,
        .element_top_index = element_top_index,
        .ele_size = ele_size,
    };

    *vec_pos = static_cast<::std::size_t*>(malloc(ele_size * sizeof(::std::size_t)));
    *chunk_pos = static_cast<::std::size_t*>(malloc(ele_size * sizeof(::std::size_t)));
    if(*vec_pos == nullptr || *chunk_pos == nullptr)
    {
        set_phy_engine_error("create_circuit_ex: failed to allocate component position arrays");
        ::std::free(*vec_pos);
        ::std::free(*chunk_pos);
        *vec_pos = nullptr;
        *chunk_pos = nullptr;
        return nullptr;
    }

    ::phy_engine::circult* c = static_cast<::phy_engine::circult*>(std::malloc(sizeof(::phy_engine::circult)));
    if(c == nullptr) [[unlikely]] { ::fast_io::fast_terminate(); }
    ::std::construct_at(c);

    c->set_analyze_type(::phy_engine::analyze_type::TR);
    auto& setting{c->get_analyze_setting()};
    setting.tr.t_step = 1e-9;
    setting.tr.t_stop = 1e-9;

    auto& nl{c->get_netlist()};

    struct verilog_netlist_job
    {
        ::phy_engine::netlist::model_pos stub_pos{};
        ::std::shared_ptr<::phy_engine::verilog::digital::compiled_design> design{};
        ::phy_engine::verilog::digital::instance_state top{};
    };

    ::std::vector<verilog_netlist_job> verilog_jobs{};

    phy_engine::netlist::model_pos* model_pos_arr = static_cast<phy_engine::netlist::model_pos*>(malloc(ele_size * sizeof(phy_engine::netlist::model_pos)));
    if(model_pos_arr == nullptr)
    {
        set_phy_engine_error("create_circuit_ex: failed to allocate model_pos array");
        destroy_circuit(c, *vec_pos, *chunk_pos);
        *vec_pos = nullptr;
        *chunk_pos = nullptr;
        return nullptr;
    }

    double* curr_prop = properties;
    ::std::size_t curr_i{};
    for(::std::size_t i{}; i < ele_size; ++i)
    {
        if(elements[i])
        {
            ::phy_engine::netlist::add_model_retstr ret{};

            if(elements[i] == PE_ELEMENT_VERILOG_NETLIST)
            {
                auto const src = get_verilog_src(vt, i);
                if(src.empty())
                {
                    set_phy_engine_error("create_circuit_ex: missing Verilog source for element index " + ::std::to_string(i));
                    ::std::free(model_pos_arr);
                    destroy_circuit(c, *vec_pos, *chunk_pos);
                    *vec_pos = nullptr;
                    *chunk_pos = nullptr;
                    return nullptr;
                }

                auto top = get_verilog_top(vt, i);
                if(top.empty())
                {
                    static constexpr char8_t fallback_top[] = u8"top";
                    top = ::fast_io::u8string_view{fallback_top, sizeof(fallback_top) - 1};
                }

                ::phy_engine::verilog::digital::compile_options opt{};
                auto cr = ::phy_engine::verilog::digital::compile(src, opt);
                if(!cr.errors.empty() || cr.modules.empty())
                {
                    auto formatted = ::phy_engine::verilog::digital::format_compile_errors(cr, src);
                    if(formatted.empty())
                    {
                        set_phy_engine_error("create_circuit_ex: Verilog compile failed for element index " + ::std::to_string(i));
                    }
                    else
                    {
                        set_phy_engine_error("create_circuit_ex: Verilog compile failed for element index " + ::std::to_string(i) + "\n" +
                                             to_std_string(formatted));
                    }
                    ::std::free(model_pos_arr);
                    destroy_circuit(c, *vec_pos, *chunk_pos);
                    *vec_pos = nullptr;
                    *chunk_pos = nullptr;
                    return nullptr;
                }

                auto design =
                    ::std::make_shared<::phy_engine::verilog::digital::compiled_design>(::phy_engine::verilog::digital::build_design(::std::move(cr)));
                if(design->modules.empty())
                {
                    set_phy_engine_error("create_circuit_ex: compiled design is empty for element index " + ::std::to_string(i));
                    ::std::free(model_pos_arr);
                    destroy_circuit(c, *vec_pos, *chunk_pos);
                    *vec_pos = nullptr;
                    *chunk_pos = nullptr;
                    return nullptr;
                }

                auto const* chosen = ::phy_engine::verilog::digital::find_module(*design, top);
                if(chosen == nullptr) { chosen = __builtin_addressof(design->modules.front_unchecked()); }

                auto top_inst = ::phy_engine::verilog::digital::elaborate(*design, *chosen);

                ::fast_io::vector<::fast_io::u8string> pin_names{};
                if(top_inst.mod != nullptr)
                {
                    pin_names.reserve(top_inst.mod->ports.size());
                    for(auto const& p: top_inst.mod->ports) { pin_names.push_back(p.name); }
                }

                ret = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::VERILOG_PORTS{::std::move(pin_names)});
                if(ret.mod == nullptr)
                {
                    ::std::free(model_pos_arr);
                    destroy_circuit(c, *vec_pos, *chunk_pos);
                    *vec_pos = nullptr;
                    *chunk_pos = nullptr;
                    return nullptr;
                }

                verilog_jobs.push_back(verilog_netlist_job{
                    .stub_pos = ret.mod_pos,
                    .design = ::std::move(design),
                    .top = ::std::move(top_inst),
                });
            }
            else
            {
                ret = add_model_via_code_ex(nl, elements[i], &curr_prop, &vt, i);
            }

            auto [ele, ele_pos]{ret};
            if(ele == nullptr)
            {
                set_phy_engine_error("create_circuit_ex: failed to create element at index " + ::std::to_string(i) + " with code " +
                                     ::std::to_string(elements[i]));
                ::std::free(model_pos_arr);
                destroy_circuit(c, *vec_pos, *chunk_pos);
                *vec_pos = nullptr;
                *chunk_pos = nullptr;
                return nullptr;
            }
            (*vec_pos)[curr_i] = ele_pos.vec_pos;
            (*chunk_pos)[curr_i] = ele_pos.chunk_pos;
            model_pos_arr[curr_i] = ele_pos;
            ++curr_i;
        }
    }
    *comp_size = curr_i;

    int const wire_count = static_cast<int>(wires_size / 4);
    if(wires != nullptr && wire_count > 0) { build_netlist_from_wires(nl, elements, static_cast<int>(ele_size), wires, wire_count, model_pos_arr); }

    // Expand synthesized Verilog modules into PE digital primitives and wire them to the already-connected port stub pins.
	    for(auto& job: verilog_jobs)
	    {
        auto* stub = ::phy_engine::netlist::get_model(nl, job.stub_pos);
        if(stub == nullptr || stub->ptr == nullptr)
        {
            set_phy_engine_error("create_circuit_ex: synthesized Verilog port stub lookup failed");
            ::std::free(model_pos_arr);
            destroy_circuit(c, *vec_pos, *chunk_pos);
            *vec_pos = nullptr;
            *chunk_pos = nullptr;
            return nullptr;
        }

        auto pv = stub->ptr->generate_pin_view();
        ::std::vector<::phy_engine::model::node_t*> port_nodes{};
        port_nodes.reserve(pv.size);
        for(::std::size_t pi{}; pi < pv.size; ++pi)
        {
            auto* n = pv.pins[pi].nodes;
            if(n == nullptr)
            {
                auto& created = ::phy_engine::netlist::create_node(nl);
                if(!::phy_engine::netlist::add_to_node(nl, *stub, pi, created))
                {
                    set_phy_engine_error("create_circuit_ex: failed to wire synthesized Verilog port stub");
                    ::std::free(model_pos_arr);
                    destroy_circuit(c, *vec_pos, *chunk_pos);
                    *vec_pos = nullptr;
                    *chunk_pos = nullptr;
                    return nullptr;
                }
                n = __builtin_addressof(created);
            }
            port_nodes.push_back(n);
        }

	        ::phy_engine::verilog::digital::pe_synth_error syn_err{};
	        auto const syn_opt = verilog_synth_options_snapshot();
	        if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, job.top, port_nodes, &syn_err, syn_opt))
	        {
	            if(syn_err.message.empty())
	            {
	                set_phy_engine_error("create_circuit_ex: Verilog synthesis to PE netlist failed");
	            }
	            else
	            {
	                set_phy_engine_error("create_circuit_ex: Verilog synthesis to PE netlist failed\n" + to_std_string(syn_err.message));
	            }
	            ::std::free(model_pos_arr);
	            destroy_circuit(c, *vec_pos, *chunk_pos);
            *vec_pos = nullptr;
            *chunk_pos = nullptr;
            return nullptr;
        }
    }

    ::std::free(model_pos_arr);
    return c;
}

extern "C" void destroy_circuit(void* circuit_ptr, ::std::size_t* vec_pos, ::std::size_t* chunk_pos)
{  // Does pos need to be void* here?
    if(circuit_ptr)
    {
        ::phy_engine::circult* c = static_cast<::phy_engine::circult*>(circuit_ptr);
        ::std::destroy_at(c);
        ::std::free(circuit_ptr);
        circuit_ptr = NULL;
    }
    if(vec_pos)
    {
        ::std::free(vec_pos);
        vec_pos = NULL;
    }
    if(chunk_pos)
    {
        ::std::free(chunk_pos);
        chunk_pos = NULL;
    }
}

void set_property(phy_engine::model::model_base* model, ::std::size_t index, double property)
{
    if(model == nullptr || model->ptr == nullptr) { return; }

    // First try numeric attribute.
    if(model->ptr->set_attribute(index, {.d{property}, .type{::phy_engine::model::variant_type::d}})) { return; }

    // Then try boolean (treat non-zero as true).
    if(model->ptr->set_attribute(index, {.boolean{property != 0.0}, .type{::phy_engine::model::variant_type::boolean}})) { return; }

    // Finally try digital (0 -> 0, 1 -> 1, others -> X).
    ::phy_engine::model::digital_node_statement_t dv{::phy_engine::model::digital_node_statement_t::indeterminate_state};
    if(property == 0.0) { dv = ::phy_engine::model::digital_node_statement_t::false_state; }
    else if(property == 1.0) { dv = ::phy_engine::model::digital_node_statement_t::true_state; }
    (void)model->ptr->set_attribute(index, {.digital{dv}, .type{::phy_engine::model::variant_type::digital}});
}

extern "C" int analyze_circuit(void* circuit_ptr,
                               ::std::size_t* vec_pos,
                               ::std::size_t* chunk_pos,
                               ::std::size_t comp_size,
                               int* changed_ele,
                               ::std::size_t* changed_ind,
                               double* changed_prop,
                               ::std::size_t prop_size,
                               double* voltage,
                               ::std::size_t* voltage_ord,
                               double* current,
                               ::std::size_t* current_ord,
                               bool* digital,
                               ::std::size_t* digital_ord)
{
    // No need to check prop-related, as it may indeed be empty
    // TODO Need to get runtime?
    if(circuit_ptr && vec_pos && chunk_pos && voltage && voltage_ord && current && current_ord && digital && digital_ord)
    {
        ::phy_engine::circult* c = static_cast<::phy_engine::circult*>(circuit_ptr);
        auto& nl{c->get_netlist()};
        // Modify properties as needed
        for(::std::size_t i{}; i < prop_size; ++i)
        {
            phy_engine::model::model_base* model = get_model(nl, ::phy_engine::netlist::model_pos{vec_pos[changed_ele[i]], chunk_pos[changed_ele[i]]});
            set_property(model, changed_ind[i], changed_prop[i]);
        }

        // Analyze
        if(!c->analyze()) { return 1; }

        // Read (doesn't trigger digital_clk; if needed, caller triggers separately via circuit_digital_clk())
        return circuit_sample(circuit_ptr, vec_pos, chunk_pos, comp_size, voltage, voltage_ord, current, current_ord, digital, digital_ord);
    }
    return 0;
}
