#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <type_traits>

#include <fast_io/fast_io_dsal/string.h>
#include <fast_io/fast_io_dsal/string_view.h>
#include <fast_io/fast_io_dsal/vector.h>

#include <phy_engine/netlist/operation.h>
#include <phy_engine/model/model_refs/base.h>
#include <phy_engine/verilog/digital/digital.h>

#include "codec.h"

namespace phy_engine::pe_nl_fileformat
{
    struct model_codec_entry
    {
        ::fast_io::u8string_view model_name{};

        ::phy_engine::netlist::add_model_retstr (*add_model)(::phy_engine::netlist::netlist& nl) noexcept {};
        status (*save_state)(::phy_engine::model::model_base const& m, std::string& out) {};
        status (*load_state)(::phy_engine::model::model_base& m, std::string_view in) {};
        // For applying checkpoint/runtime-only blobs onto an existing circuit without breaking connectivity.
        // Default behavior is to call load_state, but models with dynamic pin storage (e.g. VERILOG_MODULE)
        // must override this to avoid reallocating pins.
        status (*load_checkpoint_state)(::phy_engine::model::model_base& m, std::string_view in) {};
    };

    class model_registry
    {
    public:
        void add(model_codec_entry e) { entries_.push_back(e); }

        [[nodiscard]] model_codec_entry const* find(::fast_io::u8string_view model_name) const noexcept
        {
            for(auto const& e: entries_)
            {
                if(e.model_name == model_name) { return __builtin_addressof(e); }
            }
            return nullptr;
        }

    private:
        ::fast_io::vector<model_codec_entry> entries_{};
    };

    namespace details
    {
        template <typename Mod>
        inline ::phy_engine::model::details::model_derv_impl<Mod>* get_impl(::phy_engine::model::model_base& mb) noexcept
        {
            if(mb.ptr == nullptr) { return nullptr; }
            return dynamic_cast<::phy_engine::model::details::model_derv_impl<Mod>*>(mb.ptr);
        }

        template <typename Mod>
        inline ::phy_engine::model::details::model_derv_impl<Mod> const* get_impl(::phy_engine::model::model_base const& mb) noexcept
        {
            if(mb.ptr == nullptr) { return nullptr; }
            return dynamic_cast<::phy_engine::model::details::model_derv_impl<Mod> const*>(mb.ptr);
        }

        template <typename Mod>
        inline status save_trivial_raw(::phy_engine::model::model_base const& mb, std::string& out)
        {
            static_assert(std::is_trivially_copyable_v<Mod>);
            auto const* impl = get_impl<Mod>(mb);
            if(impl == nullptr) { return {errc::corrupt, "model type mismatch (save_trivial_raw)"}; }

            out.clear();
            // [sizeof][alignof][bytes]
            details::append_uleb128(out, sizeof(Mod));
            details::append_uleb128(out, alignof(Mod));
            details::append_bytes(out, __builtin_addressof(impl->m), sizeof(Mod));
            return {};
        }

        template <typename Mod>
        inline status load_trivial_raw(::phy_engine::model::model_base& mb, std::string_view in)
        {
            static_assert(std::is_trivially_copyable_v<Mod>);
            auto* impl = get_impl<Mod>(mb);
            if(impl == nullptr) { return {errc::corrupt, "model type mismatch (load_trivial_raw)"}; }

            std::size_t off{};
            std::uint64_t sz{};
            std::uint64_t al{};
            auto st = details::read_uleb128(in, off, sz);
            if(!st) { return st; }
            st = details::read_uleb128(in, off, al);
            if(!st) { return st; }
            if(sz != sizeof(Mod) || al != alignof(Mod)) { return {errc::unsupported, "model raw state ABI mismatch"}; }
            if(in.size() - off != sizeof(Mod)) { return {errc::corrupt, "model raw state length mismatch"}; }
            std::memcpy(__builtin_addressof(impl->m), in.data() + off, sizeof(Mod));
            return {};
        }

        // --- BSIM3v32 specialized (exclude internal node_t btree_set) ---
        template <typename Mod>
        inline status save_bsim3v32(::phy_engine::model::model_base const& mb, std::string& out)
        {
            auto const* impl = get_impl<Mod>(mb);
            if(impl == nullptr) { return {errc::corrupt, "model type mismatch (save_bsim3v32)"}; }

            out.clear();
            constexpr std::size_t prefix_size = offsetof(Mod, internal_nodes);
            details::append_uleb128(out, sizeof(Mod));
            details::append_uleb128(out, alignof(Mod));
            details::append_uleb128(out, prefix_size);
            details::append_bytes(out, __builtin_addressof(impl->m), prefix_size);

            // Internal node voltages (always 6 slots; only prefix of them may be used).
            details::append_uleb128(out, 6);
            for(std::size_t i{}; i < 6; ++i)
            {
                auto const v = impl->m.internal_nodes[i].node_information.an.voltage;
                details::append_f64(out, v.real());
                details::append_f64(out, v.imag());
            }
            return {};
        }

        template <typename Mod>
        inline status load_bsim3v32(::phy_engine::model::model_base& mb, std::string_view in)
        {
            auto* impl = get_impl<Mod>(mb);
            if(impl == nullptr) { return {errc::corrupt, "model type mismatch (load_bsim3v32)"}; }

            std::size_t off{};
            std::uint64_t sz{};
            std::uint64_t al{};
            std::uint64_t prefix_sz{};
            auto st = details::read_uleb128(in, off, sz);
            if(!st) { return st; }
            st = details::read_uleb128(in, off, al);
            if(!st) { return st; }
            st = details::read_uleb128(in, off, prefix_sz);
            if(!st) { return st; }
            if(sz != sizeof(Mod) || al != alignof(Mod)) { return {errc::unsupported, "bsim3v32 ABI mismatch"}; }
            if(prefix_sz != offsetof(Mod, internal_nodes)) { return {errc::unsupported, "bsim3v32 prefix ABI mismatch"}; }
            if(in.size() - off < prefix_sz) { return {errc::corrupt, "unexpected EOF while reading bsim3v32 prefix"}; }

            std::memcpy(__builtin_addressof(impl->m), in.data() + off, static_cast<std::size_t>(prefix_sz));
            off += static_cast<std::size_t>(prefix_sz);

            std::uint64_t n_nodes{};
            st = details::read_uleb128(in, off, n_nodes);
            if(!st) { return st; }
            if(n_nodes != 6) { return {errc::corrupt, "unexpected internal node count for bsim3v32"}; }
            for(std::size_t i{}; i < 6; ++i)
            {
                double re{};
                double im{};
                st = details::read_f64(in, off, re);
                if(!st) { return st; }
                st = details::read_f64(in, off, im);
                if(!st) { return st; }
                auto& n = impl->m.internal_nodes[i];
                n.pins.clear();
                n.num_of_analog_node = 1;
                n.node_index = SIZE_MAX;
                n.node_information.an.voltage = {re, im};
            }
            if(off != in.size()) { return {errc::corrupt, "trailing bytes in bsim3v32 state"}; }
            return {};
        }

        // --- VERILOG_MODULE specialized ---
        inline void ensure_verilog_pins(::phy_engine::model::VERILOG_MODULE& vm) noexcept
        {
            vm.pin_name_storage.clear();
            vm.pins.clear();
            if(vm.top_instance.mod == nullptr) { return; }
            auto const& cm = *vm.top_instance.mod;
            vm.pin_name_storage.reserve(cm.ports.size());
            vm.pins.reserve(cm.ports.size());
            for(std::size_t i{}; i < cm.ports.size(); ++i)
            {
                auto const& p = cm.ports.index_unchecked(i);
                vm.pin_name_storage.push_back(p.name);
                auto& stored = vm.pin_name_storage.back_unchecked();
                ::phy_engine::model::pin pin{};
                pin.name = ::fast_io::u8string_view{stored.data(), stored.size()};
                pin.nodes = nullptr;
                pin.model = nullptr;
                vm.pins.push_back(pin);
            }
        }

        inline status rebuild_verilog_design(::phy_engine::model::VERILOG_MODULE& vm)
        {
            using namespace ::phy_engine::verilog::digital;
            auto cr = compile(::fast_io::u8string_view{vm.source.data(), vm.source.size()});
            if(!cr.errors.empty())
            {
                auto const fe = format_compile_errors(cr, ::fast_io::u8string_view{vm.source.data(), vm.source.size()});
                auto const msg = details::u8sv_to_bytes(::fast_io::u8string_view{fe.data(), fe.size()});
                return {errc::corrupt, "verilog compile failed: " + msg};
            }

            auto design = std::make_shared<compiled_design>(build_design(std::move(cr)));
            auto const* top_mod = find_module(*design, vm.top);
            if(top_mod == nullptr) { return {errc::not_found, "verilog top module not found"}; }

            vm.design = std::move(design);
            vm.top_instance = elaborate(*vm.design, *top_mod);
            ensure_verilog_pins(vm);
            return {};
        }

        inline void append_logic_vec(std::string& out, ::fast_io::vector<::phy_engine::verilog::digital::logic_t> const& v)
        {
            details::append_uleb128(out, v.size());
            for(auto const x: v) { details::append_u8(out, static_cast<std::uint8_t>(x)); }
        }

        inline status read_logic_vec(std::string_view in, std::size_t& off, ::fast_io::vector<::phy_engine::verilog::digital::logic_t>& v)
        {
            std::uint64_t n{};
            auto st = details::read_uleb128(in, off, n);
            if(!st) { return st; }
            v.resize(static_cast<std::size_t>(n));
            for(std::size_t i{}; i < static_cast<std::size_t>(n); ++i)
            {
                if(off >= in.size()) { return {errc::corrupt, "unexpected EOF while reading logic vector"}; }
                v.index_unchecked(i) = static_cast<::phy_engine::verilog::digital::logic_t>(static_cast<std::uint8_t>(in[off++]));
            }
            return {};
        }

        inline void append_scheduled_event(std::string& out, ::phy_engine::verilog::digital::scheduled_event const& ev)
        {
            details::append_trivial(out, ev.due_tick);
            details::append_u8(out, ev.nonblocking ? 1u : 0u);
            details::append_trivial(out, static_cast<std::uint64_t>(ev.lhs_signal));
            details::append_trivial(out, static_cast<std::uint64_t>(ev.expr_root));
            details::append_u8(out, ev.is_vector ? 1u : 0u);
            details::append_trivial_vector(out, ev.lhs_signals);
            details::append_trivial_vector(out, ev.expr_roots);
        }

        inline status read_scheduled_event(std::string_view in, std::size_t& off, ::phy_engine::verilog::digital::scheduled_event& ev)
        {
            auto st = details::read_trivial(in, off, ev.due_tick);
            if(!st) { return st; }
            if(off + 1 > in.size()) { return {errc::corrupt, "unexpected EOF reading scheduled_event flags"}; }
            ev.nonblocking = static_cast<std::uint8_t>(in[off++]) != 0;
            std::uint64_t lhs{};
            std::uint64_t expr{};
            st = details::read_trivial(in, off, lhs);
            if(!st) { return st; }
            st = details::read_trivial(in, off, expr);
            if(!st) { return st; }
            ev.lhs_signal = static_cast<std::size_t>(lhs);
            ev.expr_root = static_cast<std::size_t>(expr);
            if(off + 1 > in.size()) { return {errc::corrupt, "unexpected EOF reading scheduled_event is_vector"}; }
            ev.is_vector = static_cast<std::uint8_t>(in[off++]) != 0;
            st = details::read_trivial_vector(in, off, ev.lhs_signals);
            if(!st) { return st; }
            st = details::read_trivial_vector(in, off, ev.expr_roots);
            if(!st) { return st; }
            return {};
        }

        inline void append_module_state(std::string& out, ::phy_engine::verilog::digital::module_state const& st)
        {
            append_logic_vec(out, st.values);
            append_logic_vec(out, st.prev_values);
            append_logic_vec(out, st.comb_prev_values);

            details::append_uleb128(out, st.events.size());
            for(auto const& e: st.events) { append_scheduled_event(out, e); }

            details::append_uleb128(out, st.nba_queue.size());
            for(auto const& kv: st.nba_queue)
            {
                details::append_trivial(out, static_cast<std::uint64_t>(kv.first));
                details::append_u8(out, static_cast<std::uint8_t>(kv.second));
            }

            append_logic_vec(out, st.next_net_values);
            details::append_trivial_vector(out, st.change_mark);
            details::append_trivial_vector(out, st.changed_signals);
            details::append_trivial(out, st.change_token);

            append_logic_vec(out, st.expr_eval_cache);
            details::append_trivial_vector(out, st.expr_eval_mark);
            details::append_trivial(out, st.expr_eval_token);
        }

        inline status read_module_state(std::string_view in, std::size_t& off, ::phy_engine::verilog::digital::module_state& st)
        {
            auto rc = read_logic_vec(in, off, st.values);
            if(!rc) { return rc; }
            rc = read_logic_vec(in, off, st.prev_values);
            if(!rc) { return rc; }
            rc = read_logic_vec(in, off, st.comb_prev_values);
            if(!rc) { return rc; }

            std::uint64_t n_events{};
            rc = details::read_uleb128(in, off, n_events);
            if(!rc) { return rc; }
            st.events.resize(static_cast<std::size_t>(n_events));
            for(std::size_t i{}; i < static_cast<std::size_t>(n_events); ++i)
            {
                rc = read_scheduled_event(in, off, st.events.index_unchecked(i));
                if(!rc) { return rc; }
            }

            std::uint64_t n_nba{};
            rc = details::read_uleb128(in, off, n_nba);
            if(!rc) { return rc; }
            st.nba_queue.resize(static_cast<std::size_t>(n_nba));
            for(std::size_t i{}; i < static_cast<std::size_t>(n_nba); ++i)
            {
                std::uint64_t sig{};
                rc = details::read_trivial(in, off, sig);
                if(!rc) { return rc; }
                if(off >= in.size()) { return {errc::corrupt, "unexpected EOF reading nba_queue value"}; }
                auto const v = static_cast<std::uint8_t>(in[off++]);
                st.nba_queue.index_unchecked(i) = {static_cast<std::size_t>(sig), static_cast<::phy_engine::verilog::digital::logic_t>(v)};
            }

            rc = read_logic_vec(in, off, st.next_net_values);
            if(!rc) { return rc; }
            rc = details::read_trivial_vector(in, off, st.change_mark);
            if(!rc) { return rc; }
            rc = details::read_trivial_vector(in, off, st.changed_signals);
            if(!rc) { return rc; }
            rc = details::read_trivial(in, off, st.change_token);
            if(!rc) { return rc; }

            rc = read_logic_vec(in, off, st.expr_eval_cache);
            if(!rc) { return rc; }
            rc = details::read_trivial_vector(in, off, st.expr_eval_mark);
            if(!rc) { return rc; }
            rc = details::read_trivial(in, off, st.expr_eval_token);
            if(!rc) { return rc; }
            return {};
        }

        inline void append_instance_tree(std::string& out, ::phy_engine::verilog::digital::instance_state const& inst)
        {
            details::append_string(out, details::u8sv_to_bytes(::fast_io::u8string_view{inst.instance_name.data(), inst.instance_name.size()}));
            details::append_string(out,
                                   inst.mod ? details::u8sv_to_bytes(::fast_io::u8string_view{inst.mod->name.data(), inst.mod->name.size()}) : std::string{});
            append_module_state(out, inst.state);

            details::append_uleb128(out, inst.children.size());
            for(std::size_t i{}; i < inst.children.size(); ++i)
            {
                auto const& ch = inst.children.index_unchecked(i);
                details::append_u8(out, ch ? 1u : 0u);
                if(ch) { append_instance_tree(out, *ch); }
            }
        }

        inline status read_instance_tree(std::string_view in, std::size_t& off, ::phy_engine::verilog::digital::instance_state& inst)
        {
            std::string inst_name{};
            std::string mod_name{};
            auto st = details::read_string(in, off, inst_name);
            if(!st) { return st; }
            st = details::read_string(in, off, mod_name);
            if(!st) { return st; }

            // Validate names against already-elaborated instance.
            if(inst_name != details::u8sv_to_bytes(::fast_io::u8string_view{inst.instance_name.data(), inst.instance_name.size()}))
            {
                return {errc::corrupt, "verilog instance_name mismatch"};
            }
            if(inst.mod)
            {
                auto const expected_mod = details::u8sv_to_bytes(::fast_io::u8string_view{inst.mod->name.data(), inst.mod->name.size()});
                if(!mod_name.empty() && mod_name != expected_mod) { return {errc::corrupt, "verilog module_name mismatch"}; }
            }

            st = read_module_state(in, off, inst.state);
            if(!st) { return st; }

            std::uint64_t n_children{};
            st = details::read_uleb128(in, off, n_children);
            if(!st) { return st; }
            if(n_children != inst.children.size()) { return {errc::corrupt, "verilog instance child_count mismatch"}; }

            for(std::size_t i{}; i < inst.children.size(); ++i)
            {
                if(off >= in.size()) { return {errc::corrupt, "unexpected EOF reading verilog child present flag"}; }
                bool const present = static_cast<std::uint8_t>(in[off++]) != 0;
                auto const& ch = inst.children.index_unchecked(i);
                if(static_cast<bool>(ch) != present) { return {errc::corrupt, "verilog child presence mismatch"}; }
                if(ch)
                {
                    st = read_instance_tree(in, off, *ch);
                    if(!st) { return st; }
                }
            }
            return {};
        }

        inline status save_verilog_module(::phy_engine::model::model_base const& mb, std::string& out)
        {
            auto const* impl = get_impl<::phy_engine::model::VERILOG_MODULE>(mb);
            if(impl == nullptr) { return {errc::corrupt, "model type mismatch (save_verilog_module)"}; }
            auto const& vm = impl->m;

            out.clear();
            details::append_f64(out, vm.Ll);
            details::append_f64(out, vm.Hl);
            details::append_string(out, details::u8sv_to_bytes(::fast_io::u8string_view{vm.source.data(), vm.source.size()}));
            details::append_string(out, details::u8sv_to_bytes(::fast_io::u8string_view{vm.top.data(), vm.top.size()}));
            details::append_trivial(out, vm.tick);
            details::append_trivial(out, static_cast<std::uint64_t>(vm.analog_emit_cursor));
            append_instance_tree(out, vm.top_instance);
            return {};
        }

        inline status load_verilog_module(::phy_engine::model::model_base& mb, std::string_view in)
        {
            auto* impl = get_impl<::phy_engine::model::VERILOG_MODULE>(mb);
            if(impl == nullptr) { return {errc::corrupt, "model type mismatch (load_verilog_module)"}; }

            std::size_t off{};
            auto& vm = impl->m;

            auto st = details::read_f64(in, off, vm.Ll);
            if(!st) { return st; }
            st = details::read_f64(in, off, vm.Hl);
            if(!st) { return st; }

            std::string src{};
            std::string top{};
            st = details::read_string(in, off, src);
            if(!st) { return st; }
            st = details::read_string(in, off, top);
            if(!st) { return st; }
            vm.source = details::bytes_to_u8string(src);
            vm.top = details::bytes_to_u8string(top);

            st = details::read_trivial(in, off, vm.tick);
            if(!st) { return st; }
            std::uint64_t aec{};
            st = details::read_trivial(in, off, aec);
            if(!st) { return st; }
            vm.analog_emit_cursor = static_cast<std::size_t>(aec);

            st = rebuild_verilog_design(vm);
            if(!st) { return st; }

            st = read_instance_tree(in, off, vm.top_instance);
            if(!st) { return st; }

            if(off != in.size()) { return {errc::corrupt, "trailing bytes in verilog state"}; }
            return {};
        }

        inline status load_verilog_checkpoint(::phy_engine::model::model_base& mb, std::string_view in)
        {
            auto* impl = get_impl<::phy_engine::model::VERILOG_MODULE>(mb);
            if(impl == nullptr) { return {errc::corrupt, "model type mismatch (load_verilog_checkpoint)"}; }

            std::size_t off{};
            auto& vm = impl->m;

            double Ll{};
            double Hl{};
            auto st = details::read_f64(in, off, Ll);
            if(!st) { return st; }
            st = details::read_f64(in, off, Hl);
            if(!st) { return st; }

            // Keep existing design/pins. Validate source/top match if present.
            std::string src{};
            std::string top{};
            st = details::read_string(in, off, src);
            if(!st) { return st; }
            st = details::read_string(in, off, top);
            if(!st) { return st; }

            if(!src.empty())
            {
                auto const cur = details::u8sv_to_bytes(::fast_io::u8string_view{vm.source.data(), vm.source.size()});
                if(cur != src) { return {errc::unsupported, "checkpoint verilog source mismatch"}; }
            }
            if(!top.empty())
            {
                auto const cur = details::u8sv_to_bytes(::fast_io::u8string_view{vm.top.data(), vm.top.size()});
                if(cur != top) { return {errc::unsupported, "checkpoint verilog top mismatch"}; }
            }

            // Only accept Ll/Hl if they match current attributes (they should).
            (void)Ll;
            (void)Hl;

            st = details::read_trivial(in, off, vm.tick);
            if(!st) { return st; }
            std::uint64_t aec{};
            st = details::read_trivial(in, off, aec);
            if(!st) { return st; }
            vm.analog_emit_cursor = static_cast<std::size_t>(aec);

            if(vm.top_instance.mod == nullptr || vm.design == nullptr) { return {errc::unsupported, "checkpoint requires existing verilog design"}; }

            st = read_instance_tree(in, off, vm.top_instance);
            if(!st) { return st; }
            if(off != in.size()) { return {errc::corrupt, "trailing bytes in verilog checkpoint state"}; }
            return {};
        }

        template <typename Mod>
        inline status save_state_dispatch(::phy_engine::model::model_base const& mb, std::string& out)
        {
            if constexpr(std::is_same_v<Mod, ::phy_engine::model::VERILOG_MODULE>) { return save_verilog_module(mb, out); }
            else if constexpr(std::is_same_v<Mod, ::phy_engine::model::bsim3v32_nmos> || std::is_same_v<Mod, ::phy_engine::model::bsim3v32_pmos>)
            {
                return save_bsim3v32<Mod>(mb, out);
            }
            else if constexpr(std::is_trivially_copyable_v<Mod>) { return save_trivial_raw<Mod>(mb, out); }
            else
            {
                (void)mb;
                out.clear();
                return {errc::unsupported, "model is not trivially-serializable; no codec provided"};
            }
        }

        template <typename Mod>
        inline status load_state_dispatch(::phy_engine::model::model_base& mb, std::string_view in)
        {
            if constexpr(std::is_same_v<Mod, ::phy_engine::model::VERILOG_MODULE>) { return load_verilog_module(mb, in); }
            else if constexpr(std::is_same_v<Mod, ::phy_engine::model::bsim3v32_nmos> || std::is_same_v<Mod, ::phy_engine::model::bsim3v32_pmos>)
            {
                return load_bsim3v32<Mod>(mb, in);
            }
            else if constexpr(std::is_trivially_copyable_v<Mod>) { return load_trivial_raw<Mod>(mb, in); }
            else
            {
                (void)mb;
                (void)in;
                return {errc::unsupported, "model is not trivially-deserializable; no codec provided"};
            }
        }

        template <typename Mod>
        inline status load_checkpoint_state_dispatch(::phy_engine::model::model_base& mb, std::string_view in)
        {
            if constexpr(std::is_same_v<Mod, ::phy_engine::model::VERILOG_MODULE>) { return load_verilog_checkpoint(mb, in); }
            else { return load_state_dispatch<Mod>(mb, in); }
        }

        template <typename Mod>
        inline ::phy_engine::netlist::add_model_retstr add_to_netlist(::phy_engine::netlist::netlist& nl) noexcept
        {
            return ::phy_engine::netlist::add_model(nl, Mod{});
        }

        template <typename Mod>
        inline model_codec_entry make_entry() noexcept
        {
            model_codec_entry e{};
            e.model_name = Mod::model_name;
            e.add_model = &add_to_netlist<Mod>;
            e.save_state = [](::phy_engine::model::model_base const& m, std::string& out) { return save_state_dispatch<Mod>(m, out); };
            e.load_state = [](::phy_engine::model::model_base& m, std::string_view in) { return load_state_dispatch<Mod>(m, in); };
            e.load_checkpoint_state = [](::phy_engine::model::model_base& m, std::string_view in) { return load_checkpoint_state_dispatch<Mod>(m, in); };
            return e;
        }
    }  // namespace details
}  // namespace phy_engine::pe_nl_fileformat
