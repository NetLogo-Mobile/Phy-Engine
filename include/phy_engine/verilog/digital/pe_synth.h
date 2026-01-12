#pragma once

#include <cstddef>
#include <cstring>
#include <unordered_map>
#include <utility>
#include <vector>

#include <fast_io/fast_io_dsal/string.h>

#include "../../netlist/operation.h"

#include "../../model/models/digital/combinational/d_ff.h"
#include "../../model/models/digital/combinational/d_ff_arstn.h"
#include "../../model/models/digital/combinational/d_latch.h"
#include "../../model/models/digital/logical/and.h"
#include "../../model/models/digital/logical/case_eq.h"
#include "../../model/models/digital/logical/input.h"
#include "../../model/models/digital/logical/is_unknown.h"
#include "../../model/models/digital/logical/not.h"
#include "../../model/models/digital/logical/or.h"
#include "../../model/models/digital/logical/resolve2.h"
#include "../../model/models/digital/logical/tick_delay.h"
#include "../../model/models/digital/logical/tri_state.h"
#include "../../model/models/digital/logical/xnor.h"
#include "../../model/models/digital/logical/xor.h"
#include "../../model/models/digital/logical/yes.h"

#include "digital.h"

namespace phy_engine::verilog::digital
{
    struct pe_synth_options
    {
        bool allow_inout{false};
        bool allow_multi_driver{false};
        bool support_always_comb{true};
        bool support_always_ff{true};
        ::std::size_t loop_unroll_limit{64};  // bounded unrolling for dynamic for/while in procedural blocks
    };

    struct pe_synth_error
    {
        ::fast_io::u8string message{};
    };

    namespace details
    {
        struct synth_context;

        struct instance_builder
        {
            synth_context& ctx;
            ::phy_engine::verilog::digital::instance_state const& inst;
            instance_builder const* parent{};

            ::std::vector<::phy_engine::model::node_t*> sig_nodes{};
            ::std::unordered_map<::std::size_t, ::phy_engine::model::node_t*> expr_cache{};

            [[nodiscard]] ::phy_engine::model::node_t* signal(::std::size_t sig) const noexcept
            {
                if(sig >= sig_nodes.size()) { return nullptr; }
                return sig_nodes[sig];
            }

            [[nodiscard]] ::phy_engine::model::node_t* expr(::std::size_t root) noexcept;
            [[nodiscard]] ::phy_engine::model::node_t* expr_in_env(::std::size_t root,
                                                                   ::std::vector<::phy_engine::model::node_t*> const& env,
                                                                   ::std::vector<bool> const* use_env) noexcept;
        };

        struct synth_context
        {
            ::phy_engine::netlist::netlist& nl;
            pe_synth_options opt{};
            pe_synth_error* err{};
            bool failed{};

            // driver count per node (best-effort; does not include any drivers created before synthesis)
            ::std::unordered_map<::phy_engine::model::node_t*, ::std::size_t> driver_count{};
            ::std::unordered_map<int, ::phy_engine::model::node_t*> const_nodes{};
            ::std::unordered_map<::std::uint64_t, ::phy_engine::model::node_t*> delay_cache{};

            [[nodiscard]] bool try_get_const(::phy_engine::model::node_t* n, ::phy_engine::verilog::digital::logic_t& out) const noexcept
            {
                if(n == nullptr) { return false; }
                for(auto const& kv: const_nodes)
                {
                    if(kv.second == n)
                    {
                        out = static_cast<::phy_engine::verilog::digital::logic_t>(kv.first);
                        return true;
                    }
                }
                return false;
            }

            void set_error(char const* msg) noexcept
            {
                if(failed) { return; }
                failed = true;
                if(err)
                {
                    auto const n = ::std::strlen(msg);
                    err->message = ::fast_io::u8string{
                        ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(msg), n}
                    };
                }
            }

            void set_error(::fast_io::u8string msg) noexcept
            {
                if(failed) { return; }
                failed = true;
                if(err) { err->message = ::std::move(msg); }
            }

            [[nodiscard]] bool ok() const noexcept { return !failed; }

            [[nodiscard]] ::phy_engine::model::node_t* make_node() noexcept
            {
                auto& n = ::phy_engine::netlist::create_node(nl);
                return __builtin_addressof(n);
            }

            [[nodiscard]] bool connect_pin(::phy_engine::model::model_base* m, ::std::size_t pin, ::phy_engine::model::node_t* node) noexcept
            {
                if(!ok()) { return false; }
                if(m == nullptr || node == nullptr)
                {
                    set_error("pe_synth: null model/node in connect");
                    return false;
                }
                if(!::phy_engine::netlist::add_to_node(nl, *m, pin, *node))
                {
                    set_error("pe_synth: add_to_node failed");
                    return false;
                }
                return true;
            }

            [[nodiscard]] bool connect_driver(::phy_engine::model::model_base* m, ::std::size_t pin, ::phy_engine::model::node_t* node) noexcept
            {
                if(!connect_pin(m, pin, node)) { return false; }
                if(node == nullptr) { return false; }

                auto& c = driver_count[node];
                ++c;
                if(c > 1 && !opt.allow_multi_driver)
                {
                    set_error("pe_synth: multiple drivers on one net (not supported)");
                    return false;
                }
                return true;
            }

            [[nodiscard]] ::phy_engine::model::node_t* const_node(::phy_engine::verilog::digital::logic_t v) noexcept
            {
                if(!ok()) { return nullptr; }
                int const key = static_cast<int>(v);
                if(auto it = const_nodes.find(key); it != const_nodes.end()) { return it->second; }

                auto* node = make_node();
                auto [m, pos]{::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = v})};
                (void)pos;
                if(m == nullptr)
                {
                    set_error("pe_synth: failed to create const INPUT");
                    return nullptr;
                }
                if(!connect_driver(m, 0, node)) { return nullptr; }
                const_nodes.emplace(key, node);
                return node;
            }

            [[nodiscard]] ::phy_engine::model::node_t* gate_not(::phy_engine::model::node_t* in) noexcept
            {
                ::phy_engine::verilog::digital::logic_t iv{};
                if(try_get_const(in, iv)) { return const_node(::phy_engine::verilog::digital::logic_not(iv)); }

                auto [m, pos]{::phy_engine::netlist::add_model(nl, ::phy_engine::model::NOT{})};
                (void)pos;
                if(m == nullptr)
                {
                    set_error("pe_synth: failed to create NOT");
                    return nullptr;
                }
                auto* out = make_node();
                if(!connect_pin(m, 0, in)) { return nullptr; }
                if(!connect_driver(m, 1, out)) { return nullptr; }
                return out;
            }

            [[nodiscard]] ::phy_engine::model::node_t* gate_and(::phy_engine::model::node_t* a, ::phy_engine::model::node_t* b) noexcept
            {
                ::phy_engine::verilog::digital::logic_t av{};
                ::phy_engine::verilog::digital::logic_t bv{};
                bool const aconst{try_get_const(a, av)};
                bool const bconst{try_get_const(b, bv)};
                if(aconst && bconst) { return const_node(::phy_engine::verilog::digital::logic_and(av, bv)); }
                if(aconst)
                {
                    av = ::phy_engine::verilog::digital::normalize_z_to_x(av);
                    if(av == ::phy_engine::verilog::digital::logic_t::false_state)
                    {
                        return const_node(::phy_engine::verilog::digital::logic_t::false_state);
                    }
                    if(av == ::phy_engine::verilog::digital::logic_t::true_state) { return b; }
                    if(bconst) { return const_node(::phy_engine::verilog::digital::logic_and(av, bv)); }
                }
                if(bconst)
                {
                    bv = ::phy_engine::verilog::digital::normalize_z_to_x(bv);
                    if(bv == ::phy_engine::verilog::digital::logic_t::false_state)
                    {
                        return const_node(::phy_engine::verilog::digital::logic_t::false_state);
                    }
                    if(bv == ::phy_engine::verilog::digital::logic_t::true_state) { return a; }
                    if(aconst) { return const_node(::phy_engine::verilog::digital::logic_and(av, bv)); }
                }

                auto [m, pos]{::phy_engine::netlist::add_model(nl, ::phy_engine::model::AND{})};
                (void)pos;
                if(m == nullptr)
                {
                    set_error("pe_synth: failed to create AND");
                    return nullptr;
                }
                auto* out = make_node();
                if(!connect_pin(m, 0, a)) { return nullptr; }
                if(!connect_pin(m, 1, b)) { return nullptr; }
                if(!connect_driver(m, 2, out)) { return nullptr; }
                return out;
            }

            [[nodiscard]] ::phy_engine::model::node_t* gate_or(::phy_engine::model::node_t* a, ::phy_engine::model::node_t* b) noexcept
            {
                ::phy_engine::verilog::digital::logic_t av{};
                ::phy_engine::verilog::digital::logic_t bv{};
                bool const aconst{try_get_const(a, av)};
                bool const bconst{try_get_const(b, bv)};
                if(aconst && bconst) { return const_node(::phy_engine::verilog::digital::logic_or(av, bv)); }
                if(aconst)
                {
                    av = ::phy_engine::verilog::digital::normalize_z_to_x(av);
                    if(av == ::phy_engine::verilog::digital::logic_t::true_state) { return const_node(::phy_engine::verilog::digital::logic_t::true_state); }
                    if(av == ::phy_engine::verilog::digital::logic_t::false_state) { return b; }
                    if(bconst) { return const_node(::phy_engine::verilog::digital::logic_or(av, bv)); }
                }
                if(bconst)
                {
                    bv = ::phy_engine::verilog::digital::normalize_z_to_x(bv);
                    if(bv == ::phy_engine::verilog::digital::logic_t::true_state) { return const_node(::phy_engine::verilog::digital::logic_t::true_state); }
                    if(bv == ::phy_engine::verilog::digital::logic_t::false_state) { return a; }
                    if(aconst) { return const_node(::phy_engine::verilog::digital::logic_or(av, bv)); }
                }

                auto [m, pos]{::phy_engine::netlist::add_model(nl, ::phy_engine::model::OR{})};
                (void)pos;
                if(m == nullptr)
                {
                    set_error("pe_synth: failed to create OR");
                    return nullptr;
                }
                auto* out = make_node();
                if(!connect_pin(m, 0, a)) { return nullptr; }
                if(!connect_pin(m, 1, b)) { return nullptr; }
                if(!connect_driver(m, 2, out)) { return nullptr; }
                return out;
            }

            [[nodiscard]] ::phy_engine::model::node_t* gate_xor(::phy_engine::model::node_t* a, ::phy_engine::model::node_t* b) noexcept
            {
                ::phy_engine::verilog::digital::logic_t av{};
                ::phy_engine::verilog::digital::logic_t bv{};
                bool const aconst{try_get_const(a, av)};
                bool const bconst{try_get_const(b, bv)};
                if(aconst && bconst) { return const_node(::phy_engine::verilog::digital::logic_xor(av, bv)); }
                if(aconst)
                {
                    av = ::phy_engine::verilog::digital::normalize_z_to_x(av);
                    if(av == ::phy_engine::verilog::digital::logic_t::false_state) { return b; }
                    if(av == ::phy_engine::verilog::digital::logic_t::true_state) { return gate_not(b); }
                    if(bconst) { return const_node(::phy_engine::verilog::digital::logic_xor(av, bv)); }
                }
                if(bconst)
                {
                    bv = ::phy_engine::verilog::digital::normalize_z_to_x(bv);
                    if(bv == ::phy_engine::verilog::digital::logic_t::false_state) { return a; }
                    if(bv == ::phy_engine::verilog::digital::logic_t::true_state) { return gate_not(a); }
                    if(aconst) { return const_node(::phy_engine::verilog::digital::logic_xor(av, bv)); }
                }

                auto [m, pos]{::phy_engine::netlist::add_model(nl, ::phy_engine::model::XOR{})};
                (void)pos;
                if(m == nullptr)
                {
                    set_error("pe_synth: failed to create XOR");
                    return nullptr;
                }
                auto* out = make_node();
                if(!connect_pin(m, 0, a)) { return nullptr; }
                if(!connect_pin(m, 1, b)) { return nullptr; }
                if(!connect_driver(m, 2, out)) { return nullptr; }
                return out;
            }

            [[nodiscard]] ::phy_engine::model::node_t* gate_xnor(::phy_engine::model::node_t* a, ::phy_engine::model::node_t* b) noexcept
            {
                ::phy_engine::verilog::digital::logic_t av{};
                ::phy_engine::verilog::digital::logic_t bv{};
                bool const aconst{try_get_const(a, av)};
                bool const bconst{try_get_const(b, bv)};
                if(aconst && bconst)
                {
                    return const_node(::phy_engine::verilog::digital::logic_not(::phy_engine::verilog::digital::logic_xor(av, bv)));
                }
                if(aconst)
                {
                    av = ::phy_engine::verilog::digital::normalize_z_to_x(av);
                    if(av == ::phy_engine::verilog::digital::logic_t::false_state) { return gate_not(b); }
                    if(av == ::phy_engine::verilog::digital::logic_t::true_state) { return b; }
                    if(bconst)
                    {
                        return const_node(::phy_engine::verilog::digital::logic_not(::phy_engine::verilog::digital::logic_xor(av, bv)));
                    }
                }
                if(bconst)
                {
                    bv = ::phy_engine::verilog::digital::normalize_z_to_x(bv);
                    if(bv == ::phy_engine::verilog::digital::logic_t::false_state) { return gate_not(a); }
                    if(bv == ::phy_engine::verilog::digital::logic_t::true_state) { return a; }
                    if(aconst)
                    {
                        return const_node(::phy_engine::verilog::digital::logic_not(::phy_engine::verilog::digital::logic_xor(av, bv)));
                    }
                }

                auto [m, pos]{::phy_engine::netlist::add_model(nl, ::phy_engine::model::XNOR{})};
                (void)pos;
                if(m == nullptr)
                {
                    set_error("pe_synth: failed to create XNOR");
                    return nullptr;
                }
                auto* out = make_node();
                if(!connect_pin(m, 0, a)) { return nullptr; }
                if(!connect_pin(m, 1, b)) { return nullptr; }
                if(!connect_driver(m, 2, out)) { return nullptr; }
                return out;
            }

            [[nodiscard]] ::phy_engine::model::node_t* gate_is_unknown(::phy_engine::model::node_t* in) noexcept
            {
                ::phy_engine::verilog::digital::logic_t iv{};
                if(try_get_const(in, iv))
                {
                    bool const u = ::phy_engine::verilog::digital::is_unknown(iv);
                    return const_node(u ? ::phy_engine::verilog::digital::logic_t::true_state : ::phy_engine::verilog::digital::logic_t::false_state);
                }

                auto [m, pos]{::phy_engine::netlist::add_model(nl, ::phy_engine::model::IS_UNKNOWN{})};
                (void)pos;
                if(m == nullptr)
                {
                    set_error("pe_synth: failed to create IS_UNKNOWN");
                    return nullptr;
                }
                auto* out = make_node();
                if(!connect_pin(m, 0, in)) { return nullptr; }
                if(!connect_driver(m, 1, out)) { return nullptr; }
                return out;
            }

            [[nodiscard]] ::phy_engine::model::node_t* gate_case_eq(::phy_engine::model::node_t* a, ::phy_engine::model::node_t* b) noexcept
            {
                if(a == b) { return const_node(::phy_engine::verilog::digital::logic_t::true_state); }

                ::phy_engine::verilog::digital::logic_t av{};
                ::phy_engine::verilog::digital::logic_t bv{};
                if(try_get_const(a, av) && try_get_const(b, bv))
                {
                    bool const eq = (av == bv);
                    return const_node(eq ? ::phy_engine::verilog::digital::logic_t::true_state : ::phy_engine::verilog::digital::logic_t::false_state);
                }

                auto [m, pos]{::phy_engine::netlist::add_model(nl, ::phy_engine::model::CASE_EQ{})};
                (void)pos;
                if(m == nullptr)
                {
                    set_error("pe_synth: failed to create CASE_EQ");
                    return nullptr;
                }
                auto* out = make_node();
                if(!connect_pin(m, 0, a)) { return nullptr; }
                if(!connect_pin(m, 1, b)) { return nullptr; }
                if(!connect_driver(m, 2, out)) { return nullptr; }
                return out;
            }

            [[nodiscard]] ::phy_engine::model::node_t* tick_delay(::phy_engine::model::node_t* in, ::std::uint64_t ticks) noexcept
            {
                if(!ok()) { return nullptr; }
                if(in == nullptr) { return nullptr; }
                if(ticks == 0) { return in; }

                // key = (ticks<<32) ^ (ptr>>4) (best-effort)
                auto const key = (static_cast<::std::uint64_t>(ticks) << 32) ^
                                 (static_cast<::std::uint64_t>(reinterpret_cast<::std::uintptr_t>(in)) >> 4);
                if(auto it = delay_cache.find(key); it != delay_cache.end()) { return it->second; }

                auto [m, pos]{::phy_engine::netlist::add_model(nl, ::phy_engine::model::TICK_DELAY{static_cast<::std::size_t>(ticks)})};
                (void)pos;
                if(m == nullptr)
                {
                    set_error("pe_synth: failed to create TICK_DELAY");
                    return nullptr;
                }
                auto* out = make_node();
                if(!connect_pin(m, 0, in)) { return nullptr; }
                if(!connect_driver(m, 1, out)) { return nullptr; }
                delay_cache.emplace(key, out);
                return out;
            }

            [[nodiscard]] ::phy_engine::model::node_t*
                mux2(::phy_engine::model::node_t* sel, ::phy_engine::model::node_t* d0, ::phy_engine::model::node_t* d1) noexcept
            {
                if(d0 == d1) { return d0; }
                // y = (sel & d1) | (~sel & d0)
                auto* n_sel = sel;
                auto* n_d0 = d0;
                auto* n_d1 = d1;
                if(n_sel == nullptr || n_d0 == nullptr || n_d1 == nullptr)
                {
                    set_error("pe_synth: mux2 null input");
                    return nullptr;
                }

                auto* n_not = gate_not(n_sel);
                auto* n_a = gate_and(n_d1, n_sel);
                auto* n_b = gate_and(n_d0, n_not);
                return gate_or(n_a, n_b);
            }

            [[nodiscard]] bool synth_instance(::phy_engine::verilog::digital::instance_state const& inst,
                                              instance_builder const* parent,
                                              ::std::vector<::phy_engine::model::node_t*> const& top_ports) noexcept;

            [[nodiscard]] bool resolve_multi_driver_digital_nets() noexcept;
        };

        inline ::phy_engine::model::node_t* instance_builder::expr(::std::size_t root) noexcept
        {
            if(!ctx.ok()) { return nullptr; }
            if(auto it = expr_cache.find(root); it != expr_cache.end()) { return it->second; }

            auto const* m = inst.mod;
            if(m == nullptr || root >= m->expr_nodes.size())
            {
                auto* res = ctx.const_node(::phy_engine::verilog::digital::logic_t::indeterminate_state);
                expr_cache.emplace(root, res);
                return res;
            }

            auto const& n = m->expr_nodes.index_unchecked(root);
            using ek = ::phy_engine::verilog::digital::expr_kind;
            ::phy_engine::model::node_t* res{};

            switch(n.kind)
            {
                case ek::literal: res = ctx.const_node(n.literal); break;
                case ek::signal: res = signal(n.signal); break;
                case ek::is_unknown:
                {
                    res = ctx.gate_is_unknown(expr(n.a));
                    break;
                }
                case ek::unary_not:
                {
                    res = ctx.gate_not(expr(n.a));
                    break;
                }
                case ek::binary_and:
                {
                    res = ctx.gate_and(expr(n.a), expr(n.b));
                    break;
                }
                case ek::binary_or:
                {
                    res = ctx.gate_or(expr(n.a), expr(n.b));
                    break;
                }
                case ek::binary_xor:
                {
                    res = ctx.gate_xor(expr(n.a), expr(n.b));
                    break;
                }
                case ek::binary_eq:
                {
                    res = ctx.gate_xnor(expr(n.a), expr(n.b));
                    break;
                }
                case ek::binary_neq:
                {
                    res = ctx.gate_xor(expr(n.a), expr(n.b));
                    break;
                }
                case ek::binary_case_eq:
                {
                    res = ctx.gate_case_eq(expr(n.a), expr(n.b));
                    break;
                }
                default:
                {
                    res = ctx.const_node(::phy_engine::verilog::digital::logic_t::indeterminate_state);
                    break;
                }
            }

            expr_cache.emplace(root, res);
            return res;
        }

        inline ::phy_engine::model::node_t* instance_builder::expr_in_env(::std::size_t root,
                                                                          ::std::vector<::phy_engine::model::node_t*> const& env,
                                                                          ::std::vector<bool> const* use_env) noexcept
        {
            if(!ctx.ok()) { return nullptr; }

            auto const* m = inst.mod;
            if(m == nullptr || root >= m->expr_nodes.size())
            {
                return ctx.const_node(::phy_engine::verilog::digital::logic_t::indeterminate_state);
            }

            ::std::unordered_map<::std::size_t, ::phy_engine::model::node_t*> cache{};

            auto rec = [&](auto&& self, ::std::size_t r) noexcept -> ::phy_engine::model::node_t*
            {
                if(!ctx.ok()) { return nullptr; }
                if(auto it = cache.find(r); it != cache.end()) { return it->second; }
                if(m == nullptr || r >= m->expr_nodes.size())
                {
                    auto* res = ctx.const_node(::phy_engine::verilog::digital::logic_t::indeterminate_state);
                    cache.emplace(r, res);
                    return res;
                }

                auto const& n = m->expr_nodes.index_unchecked(r);
                using ek = ::phy_engine::verilog::digital::expr_kind;
                ::phy_engine::model::node_t* res{};

                switch(n.kind)
                {
                    case ek::literal: res = ctx.const_node(n.literal); break;
                    case ek::signal:
                    {
                        bool subst{false};
                        if(n.signal < env.size())
                        {
                            if(use_env == nullptr) { subst = true; }
                            else if(n.signal < use_env->size() && (*use_env)[n.signal]) { subst = true; }
                        }
                        if(subst)
                        {
                            res = env[n.signal];
                            if(res == nullptr) { res = signal(n.signal); }
                        }
                        else
                        {
                            res = signal(n.signal);
                        }
                        break;
                    }
                    case ek::is_unknown:
                    {
                        res = ctx.gate_is_unknown(self(self, n.a));
                        break;
                    }
                    case ek::unary_not:
                    {
                        res = ctx.gate_not(self(self, n.a));
                        break;
                    }
                    case ek::binary_and:
                    {
                        res = ctx.gate_and(self(self, n.a), self(self, n.b));
                        break;
                    }
                    case ek::binary_or:
                    {
                        res = ctx.gate_or(self(self, n.a), self(self, n.b));
                        break;
                    }
                    case ek::binary_xor:
                    {
                        res = ctx.gate_xor(self(self, n.a), self(self, n.b));
                        break;
                    }
                    case ek::binary_eq:
                    {
                        res = ctx.gate_xnor(self(self, n.a), self(self, n.b));
                        break;
                    }
                    case ek::binary_neq:
                    {
                        res = ctx.gate_xor(self(self, n.a), self(self, n.b));
                        break;
                    }
                    case ek::binary_case_eq:
                    {
                        res = ctx.gate_case_eq(self(self, n.a), self(self, n.b));
                        break;
                    }
                    default:
                    {
                        res = ctx.const_node(::phy_engine::verilog::digital::logic_t::indeterminate_state);
                        break;
                    }
                }

                cache.emplace(r, res);
                return res;
            };

            return rec(rec, root);
        }

        inline bool eval_const_expr_to_logic(instance_builder& b, ::std::size_t root, ::phy_engine::verilog::digital::logic_t& out) noexcept
        {
            if(!b.ctx.ok()) { return false; }
            if(b.inst.mod == nullptr) { return false; }
            auto const& m = *b.inst.mod;
            if(root >= m.expr_nodes.size()) { return false; }

            auto const& n = m.expr_nodes.index_unchecked(root);
            using ek = ::phy_engine::verilog::digital::expr_kind;
            switch(n.kind)
            {
                case ek::literal: out = n.literal; return true;
                case ek::signal:
                {
                    bool const is_const = (n.signal < m.signal_is_const.size()) ? m.signal_is_const.index_unchecked(n.signal) : false;
                    if(!is_const) { return false; }
                    if(n.signal >= b.inst.state.values.size()) { return false; }
                    out = b.inst.state.values.index_unchecked(n.signal);
                    return true;
                }
                case ek::unary_not:
                {
                    ::phy_engine::verilog::digital::logic_t a{};
                    if(!eval_const_expr_to_logic(b, n.a, a)) { return false; }
                    out = ::phy_engine::verilog::digital::logic_not(a);
                    return true;
                }
                default: return false;
            }
        }

        inline void collect_assigned_signals(::fast_io::vector<stmt_node> const& arena, ::std::size_t stmt_idx, ::std::vector<bool>& out) noexcept
        {
            if(stmt_idx >= arena.size()) { return; }
            auto const& n = arena.index_unchecked(stmt_idx);
            switch(n.k)
            {
                case stmt_node::kind::blocking_assign:
                case stmt_node::kind::nonblocking_assign:
                {
                    if(n.lhs_signal < out.size()) { out[n.lhs_signal] = true; }
                    return;
                }
                case stmt_node::kind::block:
                {
                    for(auto const s: n.stmts) { collect_assigned_signals(arena, s, out); }
                    return;
                }
                case stmt_node::kind::if_stmt:
                {
                    for(auto const s: n.stmts) { collect_assigned_signals(arena, s, out); }
                    for(auto const s: n.else_stmts) { collect_assigned_signals(arena, s, out); }
                    return;
                }
                case stmt_node::kind::case_stmt:
                {
                    for(auto const& ci: n.case_items)
                    {
                        for(auto const s: ci.stmts) { collect_assigned_signals(arena, s, out); }
                    }
                    return;
                }
                case stmt_node::kind::for_stmt:
                {
                    if(n.init_stmt != SIZE_MAX) { collect_assigned_signals(arena, n.init_stmt, out); }
                    if(n.step_stmt != SIZE_MAX) { collect_assigned_signals(arena, n.step_stmt, out); }
                    if(n.body_stmt != SIZE_MAX) { collect_assigned_signals(arena, n.body_stmt, out); }
                    return;
                }
                case stmt_node::kind::while_stmt:
                {
                    if(n.body_stmt != SIZE_MAX) { collect_assigned_signals(arena, n.body_stmt, out); }
                    return;
                }
                default: return;
            }
        }

        inline bool find_async_reset_if_stmt(::fast_io::vector<stmt_node> const& arena,
                                             ::fast_io::vector<::std::size_t> const& roots,
                                             ::std::size_t& out_if_stmt) noexcept
        {
            if(roots.size() != 1) { return false; }
            auto const root = roots.front_unchecked();
            if(root >= arena.size()) { return false; }
            auto const& n = arena.index_unchecked(root);
            if(n.k == stmt_node::kind::if_stmt)
            {
                out_if_stmt = root;
                return true;
            }
            if(n.k != stmt_node::kind::block) { return false; }

            ::std::size_t picked{SIZE_MAX};
            for(auto const s: n.stmts)
            {
                if(s >= arena.size()) { continue; }
                auto const& sn = arena.index_unchecked(s);
                if(sn.k == stmt_node::kind::empty) { continue; }
                if(picked != SIZE_MAX) { return false; }
                picked = s;
            }
            if(picked == SIZE_MAX) { return false; }
            if(picked >= arena.size()) { return false; }
            if(arena.index_unchecked(picked).k != stmt_node::kind::if_stmt) { return false; }
            out_if_stmt = picked;
            return true;
        }

        inline bool collect_async_reset_values(instance_builder& b,
                                               ::fast_io::vector<stmt_node> const& arena,
                                               ::std::size_t stmt_idx,
                                               ::std::vector<::phy_engine::verilog::digital::logic_t>& reset_values,
                                               ::std::vector<bool>& has_reset,
                                               ::std::vector<bool> const& targets) noexcept
        {
            if(!b.ctx.ok()) { return false; }
            if(stmt_idx >= arena.size())
            {
                b.ctx.set_error("pe_synth: stmt index out of range");
                return false;
            }

            auto const& n = arena.index_unchecked(stmt_idx);
            switch(n.k)
            {
                case stmt_node::kind::empty: return true;
                case stmt_node::kind::block:
                {
                    for(auto const s: n.stmts)
                    {
                        if(!collect_async_reset_values(b, arena, s, reset_values, has_reset, targets)) { return false; }
                    }
                    return true;
                }
                case stmt_node::kind::blocking_assign:
                case stmt_node::kind::nonblocking_assign:
                {
                    if(n.lhs_signal >= reset_values.size() || n.lhs_signal >= has_reset.size()) { return true; }
                    if(n.lhs_signal < targets.size() && targets[n.lhs_signal])
                    {
                        ::phy_engine::verilog::digital::logic_t v{};
                        if(!eval_const_expr_to_logic(b, n.expr_root, v))
                        {
                            b.ctx.set_error("pe_synth: async reset assignment must be constant");
                            return false;
                        }
                        reset_values[n.lhs_signal] = v;
                        has_reset[n.lhs_signal] = true;
                    }
                    return true;
                }
                default:
                {
                    b.ctx.set_error("pe_synth: unsupported statement in async reset branch");
                    return false;
                }
            }
        }

        inline bool try_collect_async_reset_values(instance_builder& b,
                                                   ::fast_io::vector<stmt_node> const& arena,
                                                   ::std::size_t stmt_idx,
                                                   ::std::vector<::phy_engine::verilog::digital::logic_t>& reset_values,
                                                   ::std::vector<bool>& has_reset,
                                                   ::std::vector<bool> const& targets) noexcept
        {
            if(!b.ctx.ok()) { return false; }
            if(stmt_idx >= arena.size()) { return false; }

            auto const& n = arena.index_unchecked(stmt_idx);
            switch(n.k)
            {
                case stmt_node::kind::empty: return true;
                case stmt_node::kind::block:
                {
                    for(auto const s: n.stmts)
                    {
                        if(!try_collect_async_reset_values(b, arena, s, reset_values, has_reset, targets)) { return false; }
                    }
                    return true;
                }
                case stmt_node::kind::blocking_assign:
                case stmt_node::kind::nonblocking_assign:
                {
                    if(n.lhs_signal >= reset_values.size() || n.lhs_signal >= has_reset.size()) { return true; }
                    if(n.lhs_signal < targets.size() && targets[n.lhs_signal])
                    {
                        ::phy_engine::verilog::digital::logic_t v{};
                        if(!eval_const_expr_to_logic(b, n.expr_root, v)) { return false; }
                        reset_values[n.lhs_signal] = v;
                        has_reset[n.lhs_signal] = true;
                    }
                    return true;
                }
                default: return false;
            }
        }

        inline bool synth_stmt_ff(instance_builder& b,
                                  ::fast_io::vector<stmt_node> const& arena,
                                  ::std::size_t stmt_idx,
                                  ::std::vector<::phy_engine::model::node_t*>& cur,
                                  ::std::vector<::phy_engine::model::node_t*>& next,
                                  ::std::vector<bool> const& targets) noexcept
        {
            if(!b.ctx.ok()) { return false; }
            if(stmt_idx >= arena.size())
            {
                b.ctx.set_error("pe_synth: stmt index out of range");
                return false;
            }
            auto const& n = arena.index_unchecked(stmt_idx);
            switch(n.k)
            {
                case stmt_node::kind::empty: return true;
                case stmt_node::kind::block:
                {
                    for(auto const s: n.stmts)
                    {
                        if(!synth_stmt_ff(b, arena, s, cur, next, targets)) { return false; }
                    }
                    return true;
                }
                case stmt_node::kind::blocking_assign:
                case stmt_node::kind::nonblocking_assign:
                {
                    if(n.lhs_signal >= next.size()) { return true; }
                    auto* rhs = b.expr_in_env(n.expr_root, cur, nullptr);
                    if(n.delay_ticks != 0) { rhs = b.ctx.tick_delay(rhs, n.delay_ticks); }
                    if(n.k == stmt_node::kind::blocking_assign)
                    {
                        if(n.lhs_signal < cur.size()) { cur[n.lhs_signal] = rhs; }
                    }
                    if(n.lhs_signal < targets.size() && targets[n.lhs_signal]) { next[n.lhs_signal] = rhs; }
                    return true;
                }
                case stmt_node::kind::if_stmt:
                {
                    auto* raw_cond = b.expr_in_env(n.expr_root, cur, nullptr);
                    auto* cond = b.ctx.gate_case_eq(raw_cond, b.ctx.const_node(::phy_engine::verilog::digital::logic_t::true_state));

                    auto then_cur = cur;
                    auto then_next = next;
                    for(auto const s: n.stmts)
                    {
                        if(!synth_stmt_ff(b, arena, s, then_cur, then_next, targets)) { return false; }
                    }

                    auto else_cur = cur;
                    auto else_next = next;
                    for(auto const s: n.else_stmts)
                    {
                        if(!synth_stmt_ff(b, arena, s, else_cur, else_next, targets)) { return false; }
                    }

                    for(::std::size_t sig{}; sig < cur.size() && sig < then_cur.size() && sig < else_cur.size(); ++sig)
                    {
                        cur[sig] = b.ctx.mux2(cond, else_cur[sig], then_cur[sig]);
                    }
                    for(::std::size_t sig{}; sig < targets.size() && sig < next.size(); ++sig)
                    {
                        if(!targets[sig]) { continue; }
                        next[sig] = b.ctx.mux2(cond, else_next[sig], then_next[sig]);
                    }
                    return true;
                }
                case stmt_node::kind::case_stmt:
                {
                    auto const base_cur = cur;
                    auto const base_next = next;

                    ::std::size_t const w{n.case_expr_roots.empty() ? 1u : n.case_expr_roots.size()};
                    ::std::vector<::phy_engine::model::node_t*> key_bits{};
                    key_bits.resize(w);
                    if(n.case_expr_roots.empty())
                    {
                        key_bits[0] = b.ctx.const_node(::phy_engine::verilog::digital::logic_t::indeterminate_state);
                    }
                    else
                    {
                        for(::std::size_t i{}; i < w; ++i) { key_bits[i] = b.expr_in_env(n.case_expr_roots.index_unchecked(i), base_cur, nullptr); }
                    }

                    auto* z = b.ctx.const_node(::phy_engine::verilog::digital::logic_t::high_impedence_state);

                    auto bit_match = [&](::phy_engine::model::node_t* a, ::phy_engine::model::node_t* bb) noexcept
                        -> ::phy_engine::model::node_t*
                    {
                        auto* eq = b.ctx.gate_case_eq(a, bb);
                        switch(n.ck)
                        {
                            case stmt_node::case_kind::normal: return eq;
                            case stmt_node::case_kind::casez:
                            {
                                auto* az = b.ctx.gate_case_eq(a, z);
                                auto* bz = b.ctx.gate_case_eq(bb, z);
                                return b.ctx.gate_or(eq, b.ctx.gate_or(az, bz));
                            }
                            case stmt_node::case_kind::casex:
                            {
                                auto* ua = b.ctx.gate_is_unknown(a);
                                auto* ub = b.ctx.gate_is_unknown(bb);
                                return b.ctx.gate_or(eq, b.ctx.gate_or(ua, ub));
                            }
                            default: return eq;
                        }
                    };

                    auto item_match = [&](case_item const& ci) noexcept -> ::phy_engine::model::node_t*
                    {
                        if(ci.match_roots.size() != w) { return b.ctx.const_node(::phy_engine::verilog::digital::logic_t::false_state); }
                        auto* m = b.ctx.const_node(::phy_engine::verilog::digital::logic_t::true_state);
                        for(::std::size_t i{}; i < w; ++i)
                        {
                            auto* mb = b.expr_in_env(ci.match_roots.index_unchecked(i), base_cur, nullptr);
                            m = b.ctx.gate_and(m, bit_match(key_bits[i], mb));
                        }
                        return m;
                    };

                    case_item const* def{};
                    ::std::vector<case_item const*> items{};
                    items.reserve(n.case_items.size());
                    for(auto const& ci: n.case_items)
                    {
                        if(ci.is_default) { def = __builtin_addressof(ci); }
                        else { items.push_back(__builtin_addressof(ci)); }
                    }

                    auto agg_cur = base_cur;
                    auto agg_next = base_next;
                    if(def != nullptr)
                    {
                        agg_cur = base_cur;
                        agg_next = base_next;
                        for(auto const s: def->stmts)
                        {
                            if(!synth_stmt_ff(b, arena, s, agg_cur, agg_next, targets)) { return false; }
                        }
                    }

                    for(::std::size_t rev{}; rev < items.size(); ++rev)
                    {
                        auto const& ci = *items[items.size() - 1 - rev];
                        auto* match = item_match(ci);

                        auto item_cur = base_cur;
                        auto item_next = base_next;
                        for(auto const s: ci.stmts)
                        {
                            if(!synth_stmt_ff(b, arena, s, item_cur, item_next, targets)) { return false; }
                        }

                        for(::std::size_t sig{}; sig < agg_cur.size() && sig < item_cur.size(); ++sig)
                        {
                            agg_cur[sig] = b.ctx.mux2(match, agg_cur[sig], item_cur[sig]);
                        }
                        for(::std::size_t sig{}; sig < targets.size() && sig < agg_next.size(); ++sig)
                        {
                            if(!targets[sig]) { continue; }
                            agg_next[sig] = b.ctx.mux2(match, agg_next[sig], item_next[sig]);
                        }
                    }

                    cur = ::std::move(agg_cur);
                    next = ::std::move(agg_next);
                    return true;
                }
                case stmt_node::kind::for_stmt:
                {
                    auto const max_iter{b.ctx.opt.loop_unroll_limit};
                    if(max_iter == 0)
                    {
                        b.ctx.set_error("pe_synth: loop_unroll_limit is 0 (loops disabled)");
                        return false;
                    }
                    if(n.init_stmt != SIZE_MAX)
                    {
                        if(!synth_stmt_ff(b, arena, n.init_stmt, cur, next, targets)) { return false; }
                    }

                    for(::std::size_t iter{}; iter < max_iter; ++iter)
                    {
                        auto* raw_cond = b.expr_in_env(n.expr_root, cur, nullptr);
                        auto* cond = b.ctx.gate_case_eq(raw_cond, b.ctx.const_node(::phy_engine::verilog::digital::logic_t::true_state));

                        auto then_cur = cur;
                        auto then_next = next;
                        if(n.body_stmt != SIZE_MAX)
                        {
                            if(!synth_stmt_ff(b, arena, n.body_stmt, then_cur, then_next, targets)) { return false; }
                        }
                        if(n.step_stmt != SIZE_MAX)
                        {
                            if(!synth_stmt_ff(b, arena, n.step_stmt, then_cur, then_next, targets)) { return false; }
                        }

                        auto else_cur = cur;
                        auto else_next = next;

                        for(::std::size_t sig{}; sig < cur.size() && sig < then_cur.size() && sig < else_cur.size(); ++sig)
                        {
                            cur[sig] = b.ctx.mux2(cond, else_cur[sig], then_cur[sig]);
                        }
                        for(::std::size_t sig{}; sig < targets.size() && sig < next.size(); ++sig)
                        {
                            if(!targets[sig]) { continue; }
                            next[sig] = b.ctx.mux2(cond, else_next[sig], then_next[sig]);
                        }
                    }

                    return true;
                }
                case stmt_node::kind::while_stmt:
                {
                    auto const max_iter{b.ctx.opt.loop_unroll_limit};
                    if(max_iter == 0)
                    {
                        b.ctx.set_error("pe_synth: loop_unroll_limit is 0 (loops disabled)");
                        return false;
                    }

                    for(::std::size_t iter{}; iter < max_iter; ++iter)
                    {
                        auto* raw_cond = b.expr_in_env(n.expr_root, cur, nullptr);

                        auto* cond = b.ctx.gate_case_eq(raw_cond, b.ctx.const_node(::phy_engine::verilog::digital::logic_t::true_state));

                        auto then_cur = cur;
                        auto then_next = next;
                        if(n.body_stmt != SIZE_MAX)
                        {
                            if(!synth_stmt_ff(b, arena, n.body_stmt, then_cur, then_next, targets)) { return false; }
                        }

                        auto else_cur = cur;
                        auto else_next = next;

                        for(::std::size_t sig{}; sig < cur.size() && sig < then_cur.size() && sig < else_cur.size(); ++sig)
                        {
                            cur[sig] = b.ctx.mux2(cond, else_cur[sig], then_cur[sig]);
                        }
                        for(::std::size_t sig{}; sig < targets.size() && sig < next.size(); ++sig)
                        {
                            if(!targets[sig]) { continue; }
                            next[sig] = b.ctx.mux2(cond, else_next[sig], then_next[sig]);
                        }
                    }
                    return true;
                }
                default:
                {
                    b.ctx.set_error("pe_synth: unsupported statement in always_ff");
                    return false;
                }
            }
        }

        inline bool synth_stmt_comb(instance_builder& b,
                                    ::fast_io::vector<stmt_node> const& arena,
                                    ::std::size_t stmt_idx,
                                    ::std::vector<::phy_engine::model::node_t*>& value,
                                    ::std::vector<::phy_engine::model::node_t*>& assigned_cond,
                                    ::std::vector<bool> const& targets) noexcept
        {
            if(!b.ctx.ok()) { return false; }
            if(stmt_idx >= arena.size())
            {
                b.ctx.set_error("pe_synth: stmt index out of range");
                return false;
            }
            auto const& n = arena.index_unchecked(stmt_idx);
            switch(n.k)
            {
                case stmt_node::kind::empty: return true;
                case stmt_node::kind::block:
                {
                    for(auto const s: n.stmts)
                    {
                        if(!synth_stmt_comb(b, arena, s, value, assigned_cond, targets)) { return false; }
                    }
                    return true;
                }
                case stmt_node::kind::blocking_assign:
                case stmt_node::kind::nonblocking_assign:
                {
                    if(n.lhs_signal >= value.size() || n.lhs_signal >= assigned_cond.size()) { return true; }
                    if(n.lhs_signal < targets.size() && targets[n.lhs_signal])
                    {
                        auto* rhs = b.expr_in_env(n.expr_root, value, nullptr);
                        if(n.delay_ticks != 0) { rhs = b.ctx.tick_delay(rhs, n.delay_ticks); }
                        value[n.lhs_signal] = rhs;
                        assigned_cond[n.lhs_signal] = b.ctx.const_node(::phy_engine::verilog::digital::logic_t::true_state);
                    }
                    return true;
                }
                case stmt_node::kind::if_stmt:
                {
                    auto* raw_cond = b.expr_in_env(n.expr_root, value, nullptr);
                    auto* cond = b.ctx.gate_case_eq(raw_cond, b.ctx.const_node(::phy_engine::verilog::digital::logic_t::true_state));

                    auto then_value = value;
                    auto then_assigned = assigned_cond;
                    for(auto const s: n.stmts)
                    {
                        if(!synth_stmt_comb(b, arena, s, then_value, then_assigned, targets)) { return false; }
                    }

                    auto else_value = value;
                    auto else_assigned = assigned_cond;
                    for(auto const s: n.else_stmts)
                    {
                        if(!synth_stmt_comb(b, arena, s, else_value, else_assigned, targets)) { return false; }
                    }

                    for(::std::size_t sig{}; sig < targets.size() && sig < value.size(); ++sig)
                    {
                        if(!targets[sig]) { continue; }
                        value[sig] = b.ctx.mux2(cond, else_value[sig], then_value[sig]);
                        assigned_cond[sig] = b.ctx.mux2(cond, else_assigned[sig], then_assigned[sig]);
                    }
                    return true;
                }
                case stmt_node::kind::case_stmt:
                {
                    auto const base_value = value;
                    auto const base_assigned = assigned_cond;

                    ::std::size_t const w{n.case_expr_roots.empty() ? 1u : n.case_expr_roots.size()};
                    ::std::vector<::phy_engine::model::node_t*> key_bits{};
                    key_bits.resize(w);
                    if(n.case_expr_roots.empty())
                    {
                        key_bits[0] = b.ctx.const_node(::phy_engine::verilog::digital::logic_t::indeterminate_state);
                    }
                    else
                    {
                        for(::std::size_t i{}; i < w; ++i)
                        {
                            key_bits[i] = b.expr_in_env(n.case_expr_roots.index_unchecked(i), base_value, nullptr);
                        }
                    }

                    auto* z = b.ctx.const_node(::phy_engine::verilog::digital::logic_t::high_impedence_state);

                    auto bit_match = [&](::phy_engine::model::node_t* a, ::phy_engine::model::node_t* bb) noexcept
                        -> ::phy_engine::model::node_t*
                    {
                        auto* eq = b.ctx.gate_case_eq(a, bb);
                        switch(n.ck)
                        {
                            case stmt_node::case_kind::normal: return eq;
                            case stmt_node::case_kind::casez:
                            {
                                auto* az = b.ctx.gate_case_eq(a, z);
                                auto* bz = b.ctx.gate_case_eq(bb, z);
                                return b.ctx.gate_or(eq, b.ctx.gate_or(az, bz));
                            }
                            case stmt_node::case_kind::casex:
                            {
                                auto* ua = b.ctx.gate_is_unknown(a);
                                auto* ub = b.ctx.gate_is_unknown(bb);
                                return b.ctx.gate_or(eq, b.ctx.gate_or(ua, ub));
                            }
                            default: return eq;
                        }
                    };

                    auto item_match = [&](case_item const& ci) noexcept -> ::phy_engine::model::node_t*
                    {
                        if(ci.match_roots.size() != w) { return b.ctx.const_node(::phy_engine::verilog::digital::logic_t::false_state); }
                        auto* m = b.ctx.const_node(::phy_engine::verilog::digital::logic_t::true_state);
                        for(::std::size_t i{}; i < w; ++i)
                        {
                            auto* mb = b.expr_in_env(ci.match_roots.index_unchecked(i), base_value, nullptr);
                            m = b.ctx.gate_and(m, bit_match(key_bits[i], mb));
                        }
                        return m;
                    };

                    case_item const* def{};
                    ::std::vector<case_item const*> items{};
                    items.reserve(n.case_items.size());
                    for(auto const& ci: n.case_items)
                    {
                        if(ci.is_default) { def = __builtin_addressof(ci); }
                        else { items.push_back(__builtin_addressof(ci)); }
                    }

                    auto agg_value = base_value;
                    auto agg_assigned = base_assigned;
                    if(def != nullptr)
                    {
                        for(auto const s: def->stmts)
                        {
                            if(!synth_stmt_comb(b, arena, s, agg_value, agg_assigned, targets)) { return false; }
                        }
                    }

                    for(::std::size_t rev{}; rev < items.size(); ++rev)
                    {
                        auto const& ci = *items[items.size() - 1 - rev];
                        auto* match = item_match(ci);

                        auto item_value = base_value;
                        auto item_assigned = base_assigned;
                        for(auto const s: ci.stmts)
                        {
                            if(!synth_stmt_comb(b, arena, s, item_value, item_assigned, targets)) { return false; }
                        }

                        for(::std::size_t sig{}; sig < targets.size() && sig < agg_value.size(); ++sig)
                        {
                            if(!targets[sig]) { continue; }
                            agg_value[sig] = b.ctx.mux2(match, agg_value[sig], item_value[sig]);
                            agg_assigned[sig] = b.ctx.mux2(match, agg_assigned[sig], item_assigned[sig]);
                        }
                    }

                    value = ::std::move(agg_value);
                    assigned_cond = ::std::move(agg_assigned);
                    return true;
                }
                case stmt_node::kind::for_stmt:
                {
                    auto const max_iter{b.ctx.opt.loop_unroll_limit};
                    if(max_iter == 0)
                    {
                        b.ctx.set_error("pe_synth: loop_unroll_limit is 0 (loops disabled)");
                        return false;
                    }
                    if(n.init_stmt != SIZE_MAX)
                    {
                        if(!synth_stmt_comb(b, arena, n.init_stmt, value, assigned_cond, targets)) { return false; }
                    }

                    for(::std::size_t iter{}; iter < max_iter; ++iter)
                    {
                        auto* raw_cond = b.expr_in_env(n.expr_root, value, nullptr);
                        auto* cond = b.ctx.gate_case_eq(raw_cond, b.ctx.const_node(::phy_engine::verilog::digital::logic_t::true_state));

                        auto then_value = value;
                        auto then_assigned = assigned_cond;
                        if(n.body_stmt != SIZE_MAX)
                        {
                            if(!synth_stmt_comb(b, arena, n.body_stmt, then_value, then_assigned, targets)) { return false; }
                        }
                        if(n.step_stmt != SIZE_MAX)
                        {
                            if(!synth_stmt_comb(b, arena, n.step_stmt, then_value, then_assigned, targets)) { return false; }
                        }

                        auto else_value = value;
                        auto else_assigned = assigned_cond;

                        for(::std::size_t sig{}; sig < targets.size() && sig < value.size(); ++sig)
                        {
                            if(!targets[sig]) { continue; }
                            value[sig] = b.ctx.mux2(cond, else_value[sig], then_value[sig]);
                            assigned_cond[sig] = b.ctx.mux2(cond, else_assigned[sig], then_assigned[sig]);
                        }
                    }
                    return true;
                }
                case stmt_node::kind::while_stmt:
                {
                    auto const max_iter{b.ctx.opt.loop_unroll_limit};
                    if(max_iter == 0)
                    {
                        b.ctx.set_error("pe_synth: loop_unroll_limit is 0 (loops disabled)");
                        return false;
                    }

                    for(::std::size_t iter{}; iter < max_iter; ++iter)
                    {
                        auto* raw_cond = b.expr_in_env(n.expr_root, value, nullptr);
                        auto* cond = b.ctx.gate_case_eq(raw_cond, b.ctx.const_node(::phy_engine::verilog::digital::logic_t::true_state));

                        auto then_value = value;
                        auto then_assigned = assigned_cond;
                        if(n.body_stmt != SIZE_MAX)
                        {
                            if(!synth_stmt_comb(b, arena, n.body_stmt, then_value, then_assigned, targets)) { return false; }
                        }

                        auto else_value = value;
                        auto else_assigned = assigned_cond;

                        for(::std::size_t sig{}; sig < targets.size() && sig < value.size(); ++sig)
                        {
                            if(!targets[sig]) { continue; }
                            value[sig] = b.ctx.mux2(cond, else_value[sig], then_value[sig]);
                            assigned_cond[sig] = b.ctx.mux2(cond, else_assigned[sig], then_assigned[sig]);
                        }
                    }
                    return true;
                }
                default:
                {
                    b.ctx.set_error("pe_synth: unsupported statement in always_comb");
                    return false;
                }
            }
        }

        inline bool synth_context::synth_instance(::phy_engine::verilog::digital::instance_state const& inst,
                                                  instance_builder const* parent,
                                                  ::std::vector<::phy_engine::model::node_t*> const& top_ports) noexcept
        {
            if(!ok()) { return false; }
            if(inst.mod == nullptr)
            {
                set_error("pe_synth: instance has no module");
                return false;
            }
            auto const& m = *inst.mod;

            instance_builder b{*this, inst, parent};
            b.sig_nodes.assign(m.signal_names.size(), nullptr);

            // Bind port signals.
            for(::std::size_t pi{}; pi < m.ports.size(); ++pi)
            {
                auto const& p = m.ports.index_unchecked(pi);
                if(p.signal >= b.sig_nodes.size()) { continue; }

                ::phy_engine::model::node_t* node{};
                if(parent == nullptr)
                {
                    if(pi >= top_ports.size())
                    {
                        set_error("pe_synth: top port node list size mismatch");
                        return false;
                    }
                    node = top_ports[pi];
                }
                else
                {
                    if(pi >= inst.bindings.size())
                    {
                        set_error("pe_synth: binding size mismatch");
                        return false;
                    }

                    auto const& bind = inst.bindings.index_unchecked(pi);
                    switch(bind.k)
                    {
                        case port_binding::kind::unconnected: node = nullptr; break;
                        case port_binding::kind::parent_signal:
                        {
                            node = parent->signal(bind.parent_signal);
                            break;
                        }
                        case port_binding::kind::literal:
                        {
                            if(p.dir == port_dir::output || p.dir == port_dir::inout)
                            {
                                set_error("pe_synth: output port bound to literal (unsupported)");
                                return false;
                            }
                            node = const_node(bind.literal);
                            break;
                        }
                        case port_binding::kind::parent_expr_root:
                        {
                            if(p.dir == port_dir::output || p.dir == port_dir::inout)
                            {
                                set_error("pe_synth: output port bound to expression (unsupported)");
                                return false;
                            }
                            auto* parent_mut = const_cast<instance_builder*>(parent);
                            node = parent_mut->expr(bind.parent_expr_root);
                            break;
                        }
                        default: node = nullptr; break;
                    }

                    if(p.dir == port_dir::inout && !opt.allow_inout)
                    {
                        set_error("pe_synth: inout ports not supported");
                        return false;
                    }
                }

                if(node == nullptr) { node = make_node(); }
                b.sig_nodes[p.signal] = node;
            }

            // Bind const signals and allocate remaining internal nets.
            for(::std::size_t si{}; si < b.sig_nodes.size(); ++si)
            {
                if(b.sig_nodes[si] != nullptr) { continue; }
                bool const is_const = (si < m.signal_is_const.size()) ? m.signal_is_const.index_unchecked(si) : false;
                if(is_const)
                {
                    auto v = ::phy_engine::verilog::digital::logic_t::indeterminate_state;
                    if(si < inst.state.values.size()) { v = inst.state.values.index_unchecked(si); }
                    b.sig_nodes[si] = const_node(v);
                    continue;
                }
                b.sig_nodes[si] = make_node();
            }

            // Continuous assigns.
            for(auto const& a: m.assigns)
            {
                if(!ok()) { return false; }
                if(a.lhs_signal >= b.sig_nodes.size()) { continue; }
                auto* lhs = b.sig_nodes[a.lhs_signal];
                auto* rhs = b.expr(a.expr_root);
                if(lhs == nullptr || rhs == nullptr) { continue; }

                if(a.guard_root != SIZE_MAX)
                {
                    auto* en = b.expr(a.guard_root);
                    auto [tri, pos]{::phy_engine::netlist::add_model(nl, ::phy_engine::model::TRI{})};
                    (void)pos;
                    if(tri == nullptr)
                    {
                        set_error("pe_synth: failed to create TRI");
                        return false;
                    }
                    if(!connect_pin(tri, 0, rhs)) { return false; }
                    if(!connect_pin(tri, 1, en)) { return false; }
                    if(!connect_driver(tri, 2, lhs)) { return false; }
                }
                else
                {
                    auto [buf, pos]{::phy_engine::netlist::add_model(nl, ::phy_engine::model::YES{})};
                    (void)pos;
                    if(buf == nullptr)
                    {
                        set_error("pe_synth: failed to create YES");
                        return false;
                    }
                    if(!connect_pin(buf, 0, rhs)) { return false; }
                    if(!connect_driver(buf, 1, lhs)) { return false; }
                }
            }

            // always_comb blocks (restricted subset).
            if(opt.support_always_comb)
            {
                for(auto const& comb: m.always_combs)
                {
                    (void)comb;
                    ::std::vector<bool> targets(m.signal_names.size(), false);
                    for(auto const root: comb.roots) { collect_assigned_signals(comb.stmt_nodes, root, targets); }

                    ::std::vector<::phy_engine::model::node_t*> values = b.sig_nodes;
                    ::std::vector<::phy_engine::model::node_t*> assigned_cond(m.signal_names.size(),
                                                                              const_node(::phy_engine::verilog::digital::logic_t::false_state));
                    for(auto const root: comb.roots)
                    {
                        if(!synth_stmt_comb(b, comb.stmt_nodes, root, values, assigned_cond, targets)) { return false; }
                    }

                    for(::std::size_t sig{}; sig < targets.size(); ++sig)
                    {
                        if(!targets[sig]) { continue; }
                        auto* lhs = b.sig_nodes[sig];
                        auto* rhs = values[sig];
                        if(lhs == nullptr || rhs == nullptr) { continue; }

                        ::phy_engine::verilog::digital::logic_t av{};
                        bool const always_assigned = try_get_const(assigned_cond[sig], av) &&
                                                     (av == ::phy_engine::verilog::digital::logic_t::true_state);

                        if(always_assigned)
                        {
                            auto [buf, pos]{::phy_engine::netlist::add_model(nl, ::phy_engine::model::YES{})};
                            (void)pos;
                            if(buf == nullptr)
                            {
                                set_error("pe_synth: failed to create YES");
                                return false;
                            }
                            if(!connect_pin(buf, 0, rhs)) { return false; }
                            if(!connect_driver(buf, 1, lhs)) { return false; }
                        }
                        else
                        {
                            auto [lat, pos]{::phy_engine::netlist::add_model(nl, ::phy_engine::model::DLATCH{})};
                            (void)pos;
                            if(lat == nullptr)
                            {
                                set_error("pe_synth: failed to create DLATCH");
                                return false;
                            }
                            if(!connect_pin(lat, 0, rhs)) { return false; }
                            if(!connect_pin(lat, 1, assigned_cond[sig])) { return false; }
                            if(!connect_driver(lat, 2, lhs)) { return false; }
                        }
                    }
                }
            }
            else if(!m.always_combs.empty())
            {
                set_error("pe_synth: always_comb not supported by options");
                return false;
            }

            // always_ff blocks (restricted subset).
            if(opt.support_always_ff)
            {
                for(auto const& ff: m.always_ffs)
                {
                    ::std::vector<bool> targets(m.signal_names.size(), false);
                    for(auto const root: ff.roots) { collect_assigned_signals(ff.stmt_nodes, root, targets); }

                    // Validate targets are regs.
                    for(::std::size_t sig{}; sig < targets.size(); ++sig)
                    {
                        if(!targets[sig]) { continue; }
                        bool const is_reg = (sig < m.signal_is_reg.size()) ? m.signal_is_reg.index_unchecked(sig) : false;
                        if(!is_reg)
                        {
                            set_error("pe_synth: always_ff assigns to non-reg (unsupported)");
                            return false;
                        }
                    }

                    ::phy_engine::model::node_t* clk{};
                    ::phy_engine::model::node_t* arst_n{};
                    ::std::vector<::phy_engine::verilog::digital::logic_t> reset_values(m.signal_names.size(),
                                                                                        ::phy_engine::verilog::digital::logic_t::indeterminate_state);
                    ::std::vector<bool> has_reset(m.signal_names.size(), false);

                    if(ff.events.size() == 1)
                    {
                        auto const ev = ff.events.front_unchecked();
                        if(ev.signal >= b.sig_nodes.size())
                        {
                            set_error("pe_synth: always_ff clock signal out of range");
                            return false;
                        }
                        clk = b.sig_nodes[ev.signal];
                        if(clk == nullptr)
                        {
                            set_error("pe_synth: null clk node");
                            return false;
                        }
                        // Treat level event as posedge for this synthesis subset.
                        if(ev.k == sensitivity_event::kind::negedge) { clk = gate_not(clk); }
                    }
                    else if(ff.events.size() >= 2)
                    {
                        ::std::size_t if_stmt{};
                        if(!find_async_reset_if_stmt(ff.stmt_nodes, ff.roots, if_stmt))
                        {
                            set_error("pe_synth: async-reset always_ff requires a single top-level if");
                            return false;
                        }
                        auto const& ifn = ff.stmt_nodes.index_unchecked(if_stmt);

                        // Identify clock: first edge event, or fall back to first level event.
                        ::std::size_t clk_idx{SIZE_MAX};
                        for(::std::size_t i{}; i < ff.events.size(); ++i)
                        {
                            auto const ev = ff.events.index_unchecked(i);
                            if(ev.k == sensitivity_event::kind::posedge || ev.k == sensitivity_event::kind::negedge)
                            {
                                clk_idx = i;
                                break;
                            }
                        }
                        if(clk_idx == SIZE_MAX) { clk_idx = 0; }

                        auto const clk_ev = ff.events.index_unchecked(clk_idx);
                        if(clk_ev.signal >= b.sig_nodes.size())
                        {
                            set_error("pe_synth: always_ff clock signal out of range");
                            return false;
                        }
                        clk = b.sig_nodes[clk_ev.signal];
                        if(clk == nullptr)
                        {
                            set_error("pe_synth: null clk node");
                            return false;
                        }
                        if(clk_ev.k == sensitivity_event::kind::negedge) { clk = gate_not(clk); }

                        // Reset condition is an arbitrary (supported) boolean expression.
                        auto* raw_cond = b.expr_in_env(ifn.expr_root, b.sig_nodes, nullptr);
                        auto* cond_true = gate_case_eq(raw_cond, const_node(::phy_engine::verilog::digital::logic_t::true_state));

                        // Decide whether reset is in then-branch or else-branch by checking which side contains only constant assignments.
                        auto then_reset_values = reset_values;
                        auto else_reset_values = reset_values;
                        auto then_has_reset = has_reset;
                        auto else_has_reset = has_reset;

                        bool then_ok{true};
                        for(auto const s: ifn.stmts)
                        {
                            if(!try_collect_async_reset_values(b, ff.stmt_nodes, s, then_reset_values, then_has_reset, targets))
                            {
                                then_ok = false;
                                break;
                            }
                        }
                        bool else_ok{true};
                        for(auto const s: ifn.else_stmts)
                        {
                            if(!try_collect_async_reset_values(b, ff.stmt_nodes, s, else_reset_values, else_has_reset, targets))
                            {
                                else_ok = false;
                                break;
                            }
                        }

                        bool reset_is_then{};
                        if(then_ok && !else_ok) { reset_is_then = true; }
                        else if(!then_ok && else_ok) { reset_is_then = false; }
                        else if(then_ok && else_ok)
                        {
                            bool same{true};
                            for(::std::size_t sig{}; sig < targets.size() && sig < then_has_reset.size() && sig < else_has_reset.size(); ++sig)
                            {
                                if(!targets[sig]) { continue; }
                                if(then_has_reset[sig] != else_has_reset[sig]) { same = false; break; }
                                if(then_has_reset[sig] && then_reset_values[sig] != else_reset_values[sig]) { same = false; break; }
                            }
                            if(!same)
                            {
                                set_error("pe_synth: ambiguous async reset branch (both sides constant but differ)");
                                return false;
                            }
                            reset_is_then = true;
                        }
                        else
                        {
                            set_error("pe_synth: async reset requires one branch to be constant-only");
                            return false;
                        }

                        auto* rst_active = reset_is_then ? cond_true : gate_not(cond_true);
                        arst_n = gate_not(rst_active);  // DFF_ARSTN expects active-low

                        reset_values = reset_is_then ? std::move(then_reset_values) : std::move(else_reset_values);
                        has_reset = reset_is_then ? std::move(then_has_reset) : std::move(else_has_reset);
                    }
                    else
                    {
                        set_error("pe_synth: always_ff requires at least 1 event");
                        return false;
                    }

                    auto cur = b.sig_nodes;
                    auto next = b.sig_nodes;
                    for(auto const root: ff.roots)
                    {
                        if(!synth_stmt_ff(b, ff.stmt_nodes, root, cur, next, targets)) { return false; }
                    }

                    for(::std::size_t sig{}; sig < targets.size(); ++sig)
                    {
                        if(!targets[sig]) { continue; }
                        auto* q = b.sig_nodes[sig];
                        auto* d = next[sig];
                        if(q == nullptr || d == nullptr) { continue; }

                        if(arst_n != nullptr && has_reset[sig])
                        {
                            auto [dff, pos]{::phy_engine::netlist::add_model(nl, ::phy_engine::model::DFF_ARSTN{.reset_value = reset_values[sig]})};
                            (void)pos;
                            if(dff == nullptr)
                            {
                                set_error("pe_synth: failed to create DFF_ARSTN");
                                return false;
                            }
                            if(!connect_pin(dff, 0, d)) { return false; }
                            if(!connect_pin(dff, 1, clk)) { return false; }
                            if(!connect_pin(dff, 2, arst_n)) { return false; }
                            if(!connect_driver(dff, 3, q)) { return false; }
                        }
                        else
                        {
                            auto [dff, pos]{::phy_engine::netlist::add_model(nl, ::phy_engine::model::DFF{})};
                            (void)pos;
                            if(dff == nullptr)
                            {
                                set_error("pe_synth: failed to create DFF");
                                return false;
                            }
                            if(!connect_pin(dff, 0, d)) { return false; }
                            if(!connect_pin(dff, 1, clk)) { return false; }
                            if(!connect_driver(dff, 2, q)) { return false; }
                        }
                    }
                }
            }
            else if(!m.always_ffs.empty())
            {
                set_error("pe_synth: always_ff not supported by options");
                return false;
            }

            // Children.
            for(auto const& child: inst.children)
            {
                if(child)
                {
                    if(!synth_instance(*child, __builtin_addressof(b), {})) { return false; }
                }
            }
            return ok();
        }

        inline bool is_digital_output_pin(::phy_engine::model::model_base const& m, ::std::size_t pin_index) noexcept
        {
            if(m.ptr == nullptr) { return false; }
            if(m.ptr->get_device_type() != ::phy_engine::model::model_device_type::digital) { return false; }

            auto const id = m.ptr->get_identification_name();

            // Single-output primitives.
            if(id == u8"INPUT") { return pin_index == 0; }
            if(id == u8"YES") { return pin_index == 1; }
            if(id == u8"NOT") { return pin_index == 1; }
            if(id == u8"IS_UNKNOWN") { return pin_index == 1; }
            if(id == u8"SCHMITT_TRIGGER") { return pin_index == 1; }

            if(id == u8"AND" || id == u8"OR" || id == u8"XOR" || id == u8"XNOR" || id == u8"NAND" || id == u8"NOR" || id == u8"IMP" || id == u8"NIMP" ||
               id == u8"TRI" || id == u8"CASE_EQ")
            {
                return pin_index == 2;
            }

            // FFs/counters.
            if(id == u8"DFF") { return pin_index == 2; }
            if(id == u8"DFF_ARSTN") { return pin_index == 3; }
            if(id == u8"TFF") { return pin_index == 2; }
            if(id == u8"T_BAR_FF") { return pin_index == 2; }
            if(id == u8"JKFF") { return pin_index == 3; }
            if(id == u8"COUNTER4" || id == u8"RANDOM_GENERATOR4") { return pin_index < 4; }

            // Multi-output combinational blocks.
            if(id == u8"HA" || id == u8"HS") { return pin_index == 2 || pin_index == 3; }
            if(id == u8"FA" || id == u8"FS") { return pin_index == 3 || pin_index == 4; }
            if(id == u8"M2") { return pin_index >= 4 && pin_index < 8; }

            // IO blocks.
            if(id == u8"EIGHT_BIT_INPUT") { return pin_index < 8; }

            // Resolver itself.
            if(id == u8"RESOLVE2") { return pin_index == 2; }

            // Known sink/stub blocks.
            if(id == u8"OUTPUT" || id == u8"EIGHT_BIT_DISPLAY" || id == u8"VERILOG_PORTS") { return false; }

            return false;
        }

        inline bool synth_context::resolve_multi_driver_digital_nets() noexcept
        {
            if(!ok()) { return false; }
            if(!opt.allow_multi_driver) { return true; }

            struct pin_meta
            {
                ::phy_engine::model::model_base* model{};
                ::std::size_t pin_index{};
            };

            ::std::unordered_map<::phy_engine::model::pin*, pin_meta> pin_to_meta{};

            // Scan models once to build pin->(model,index) metadata.
            for(auto& mb: nl.models)
            {
                for(auto* m = mb.begin; m != mb.curr; ++m)
                {
                    if(m->type != ::phy_engine::model::model_type::normal) { continue; }
                    if(m->ptr == nullptr) { continue; }

                    auto pv = m->ptr->generate_pin_view();
                    for(::std::size_t pi{}; pi < pv.size; ++pi)
                    {
                        auto* p = __builtin_addressof(pv.pins[pi]);
                        pin_to_meta.emplace(p, pin_meta{m, pi});
                    }
                }
            }

            struct driver_pin
            {
                ::phy_engine::model::pin* pin{};
                ::phy_engine::model::model_base* model{};
                ::std::size_t pin_index{};
            };

            // Rewrite each digital node to have at most one driver by inserting a RESOLVE2 chain.
            for(auto& nb: nl.nodes)
            {
                for(auto* node = nb.begin; node != nb.curr; ++node)
                {
                    if(node->num_of_analog_node != 0) { continue; }
                    if(node->pins.size() < 2) { continue; }

                    ::std::vector<driver_pin> drivers{};
                    drivers.reserve(node->pins.size());

                    for(auto* p: node->pins)
                    {
                        auto it = pin_to_meta.find(p);
                        if(it == pin_to_meta.end()) { continue; }
                        auto* model = it->second.model;
                        if(model == nullptr) { continue; }
                        if(!is_digital_output_pin(*model, it->second.pin_index)) { continue; }
                        drivers.push_back(driver_pin{p, model, it->second.pin_index});
                    }

                    if(drivers.size() <= 1) { continue; }

                    ::std::vector<::phy_engine::model::node_t*> drv_nodes{};
                    drv_nodes.reserve(drivers.size());

                    // Detach each driver from the shared node and reattach it to its own intermediate node.
                    for(auto const& d: drivers)
                    {
                        auto* p = d.pin;
                        if(p == nullptr) { continue; }

                        p->nodes = nullptr;
                        node->pins.erase(p);

                        auto* dn = make_node();
                        p->nodes = dn;
                        dn->pins.insert(p);
                        drv_nodes.push_back(dn);
                    }

                    if(drv_nodes.size() <= 1) { continue; }

                    auto* resolved = drv_nodes.front();
                    for(::std::size_t i{1}; i < drv_nodes.size(); ++i)
                    {
                        auto [res, pos]{::phy_engine::netlist::add_model(nl, ::phy_engine::model::RESOLVE2{})};
                        (void)pos;
                        if(res == nullptr)
                        {
                            set_error("pe_synth: failed to create RESOLVE2");
                            return false;
                        }

                        if(!connect_pin(res, 0, resolved)) { return false; }
                        if(!connect_pin(res, 1, drv_nodes[i])) { return false; }

                        auto* out = (i + 1 == drv_nodes.size()) ? node : make_node();
                        if(!connect_pin(res, 2, out)) { return false; }
                        resolved = out;
                    }
                }
            }

            return ok();
        }
    }  // namespace details

    inline bool synthesize_to_pe_netlist(::phy_engine::netlist::netlist& nl,
                                         ::phy_engine::verilog::digital::instance_state const& top,
                                         ::std::vector<::phy_engine::model::node_t*> const& top_port_nodes,
                                         pe_synth_error* err = nullptr,
                                         pe_synth_options const& opt = {}) noexcept
    {
        details::synth_context ctx{nl, opt, err};
        if(!ctx.synth_instance(top, nullptr, top_port_nodes)) { return false; }
        if(!ctx.resolve_multi_driver_digital_nets()) { return false; }
        return ctx.ok();
    }
}  // namespace phy_engine::verilog::digital
