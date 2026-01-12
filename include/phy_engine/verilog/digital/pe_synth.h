#pragma once

#include <cstddef>
#include <cstring>
#include <unordered_map>
#include <utility>
#include <vector>

#include <fast_io/fast_io_dsal/string.h>

#include "../../netlist/operation.h"

#include "../../model/models/digital/combinational/d_ff.h"
#include "../../model/models/digital/logical/and.h"
#include "../../model/models/digital/logical/case_eq.h"
#include "../../model/models/digital/logical/input.h"
#include "../../model/models/digital/logical/is_unknown.h"
#include "../../model/models/digital/logical/not.h"
#include "../../model/models/digital/logical/or.h"
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

            void set_error(char const* msg) noexcept
            {
                if(failed) { return; }
                failed = true;
                if(err)
                {
                    auto const n = ::std::strlen(msg);
                    err->message = ::fast_io::u8string{::fast_io::u8string_view{reinterpret_cast<char8_t const*>(msg), n}};
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
                auto [m, pos]{::phy_engine::netlist::add_model(nl, ::phy_engine::model::NOT{})};
                (void)pos;
                if(m == nullptr) { set_error("pe_synth: failed to create NOT"); return nullptr; }
                auto* out = make_node();
                if(!connect_pin(m, 0, in)) { return nullptr; }
                if(!connect_driver(m, 1, out)) { return nullptr; }
                return out;
            }

            [[nodiscard]] ::phy_engine::model::node_t* gate_and(::phy_engine::model::node_t* a, ::phy_engine::model::node_t* b) noexcept
            {
                auto [m, pos]{::phy_engine::netlist::add_model(nl, ::phy_engine::model::AND{})};
                (void)pos;
                if(m == nullptr) { set_error("pe_synth: failed to create AND"); return nullptr; }
                auto* out = make_node();
                if(!connect_pin(m, 0, a)) { return nullptr; }
                if(!connect_pin(m, 1, b)) { return nullptr; }
                if(!connect_driver(m, 2, out)) { return nullptr; }
                return out;
            }

            [[nodiscard]] ::phy_engine::model::node_t* gate_or(::phy_engine::model::node_t* a, ::phy_engine::model::node_t* b) noexcept
            {
                auto [m, pos]{::phy_engine::netlist::add_model(nl, ::phy_engine::model::OR{})};
                (void)pos;
                if(m == nullptr) { set_error("pe_synth: failed to create OR"); return nullptr; }
                auto* out = make_node();
                if(!connect_pin(m, 0, a)) { return nullptr; }
                if(!connect_pin(m, 1, b)) { return nullptr; }
                if(!connect_driver(m, 2, out)) { return nullptr; }
                return out;
            }

            [[nodiscard]] ::phy_engine::model::node_t* gate_xor(::phy_engine::model::node_t* a, ::phy_engine::model::node_t* b) noexcept
            {
                auto [m, pos]{::phy_engine::netlist::add_model(nl, ::phy_engine::model::XOR{})};
                (void)pos;
                if(m == nullptr) { set_error("pe_synth: failed to create XOR"); return nullptr; }
                auto* out = make_node();
                if(!connect_pin(m, 0, a)) { return nullptr; }
                if(!connect_pin(m, 1, b)) { return nullptr; }
                if(!connect_driver(m, 2, out)) { return nullptr; }
                return out;
            }

            [[nodiscard]] ::phy_engine::model::node_t* gate_xnor(::phy_engine::model::node_t* a, ::phy_engine::model::node_t* b) noexcept
            {
                auto [m, pos]{::phy_engine::netlist::add_model(nl, ::phy_engine::model::XNOR{})};
                (void)pos;
                if(m == nullptr) { set_error("pe_synth: failed to create XNOR"); return nullptr; }
                auto* out = make_node();
                if(!connect_pin(m, 0, a)) { return nullptr; }
                if(!connect_pin(m, 1, b)) { return nullptr; }
                if(!connect_driver(m, 2, out)) { return nullptr; }
                return out;
            }

            [[nodiscard]] ::phy_engine::model::node_t* gate_is_unknown(::phy_engine::model::node_t* in) noexcept
            {
                auto [m, pos]{::phy_engine::netlist::add_model(nl, ::phy_engine::model::IS_UNKNOWN{})};
                (void)pos;
                if(m == nullptr) { set_error("pe_synth: failed to create IS_UNKNOWN"); return nullptr; }
                auto* out = make_node();
                if(!connect_pin(m, 0, in)) { return nullptr; }
                if(!connect_driver(m, 1, out)) { return nullptr; }
                return out;
            }

            [[nodiscard]] ::phy_engine::model::node_t* gate_case_eq(::phy_engine::model::node_t* a, ::phy_engine::model::node_t* b) noexcept
            {
                auto [m, pos]{::phy_engine::netlist::add_model(nl, ::phy_engine::model::CASE_EQ{})};
                (void)pos;
                if(m == nullptr) { set_error("pe_synth: failed to create CASE_EQ"); return nullptr; }
                auto* out = make_node();
                if(!connect_pin(m, 0, a)) { return nullptr; }
                if(!connect_pin(m, 1, b)) { return nullptr; }
                if(!connect_driver(m, 2, out)) { return nullptr; }
                return out;
            }

            [[nodiscard]] ::phy_engine::model::node_t* mux2(::phy_engine::model::node_t* sel,
                                                            ::phy_engine::model::node_t* d0,
                                                            ::phy_engine::model::node_t* d1) noexcept
            {
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

        inline void collect_assigned_signals(::fast_io::vector<stmt_node> const& arena,
                                             ::std::size_t stmt_idx,
                                             ::std::vector<bool>& out) noexcept
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
                default: return;
            }
        }

        inline bool synth_stmt_ff(instance_builder& b,
                                  ::fast_io::vector<stmt_node> const& arena,
                                  ::std::size_t stmt_idx,
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
                        if(!synth_stmt_ff(b, arena, s, next, targets)) { return false; }
                    }
                    return true;
                }
                case stmt_node::kind::blocking_assign:
                case stmt_node::kind::nonblocking_assign:
                {
                    if(n.lhs_signal >= next.size()) { return true; }
                    if(n.lhs_signal < targets.size() && targets[n.lhs_signal])
                    {
                        next[n.lhs_signal] = b.expr(n.expr_root);
                    }
                    return true;
                }
                case stmt_node::kind::if_stmt:
                {
                    auto* cond = b.expr(n.expr_root);

                    auto then_next = next;
                    for(auto const s: n.stmts)
                    {
                        if(!synth_stmt_ff(b, arena, s, then_next, targets)) { return false; }
                    }

                    auto else_next = next;
                    for(auto const s: n.else_stmts)
                    {
                        if(!synth_stmt_ff(b, arena, s, else_next, targets)) { return false; }
                    }

                    for(::std::size_t sig{}; sig < targets.size() && sig < next.size(); ++sig)
                    {
                        if(!targets[sig]) { continue; }
                        next[sig] = b.ctx.mux2(cond, else_next[sig], then_next[sig]);
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
                                    ::std::vector<bool>& assigned,
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
                        if(!synth_stmt_comb(b, arena, s, value, assigned, targets)) { return false; }
                    }
                    return true;
                }
                case stmt_node::kind::blocking_assign:
                case stmt_node::kind::nonblocking_assign:
                {
                    if(n.lhs_signal >= value.size() || n.lhs_signal >= assigned.size()) { return true; }
                    if(n.lhs_signal < targets.size() && targets[n.lhs_signal])
                    {
                        value[n.lhs_signal] = b.expr(n.expr_root);
                        assigned[n.lhs_signal] = true;
                    }
                    return true;
                }
                case stmt_node::kind::if_stmt:
                {
                    auto* cond = b.expr(n.expr_root);

                    auto then_value = value;
                    auto then_assigned = assigned;
                    for(auto const s: n.stmts)
                    {
                        if(!synth_stmt_comb(b, arena, s, then_value, then_assigned, targets)) { return false; }
                    }

                    auto else_value = value;
                    auto else_assigned = assigned;
                    for(auto const s: n.else_stmts)
                    {
                        if(!synth_stmt_comb(b, arena, s, else_value, else_assigned, targets)) { return false; }
                    }

                    for(::std::size_t sig{}; sig < targets.size() && sig < value.size(); ++sig)
                    {
                        if(!targets[sig]) { continue; }
                        if(then_assigned[sig] != else_assigned[sig])
                        {
                            b.ctx.set_error("pe_synth: latch inferred in always_comb (not supported)");
                            return false;
                        }
                        if(!then_assigned[sig]) { continue; }
                        value[sig] = b.ctx.mux2(cond, else_value[sig], then_value[sig]);
                        assigned[sig] = true;
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

                if(node == nullptr)
                {
                    node = make_node();
                }
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
                    if(tri == nullptr) { set_error("pe_synth: failed to create TRI"); return false; }
                    if(!connect_pin(tri, 0, rhs)) { return false; }
                    if(!connect_pin(tri, 1, en)) { return false; }
                    if(!connect_driver(tri, 2, lhs)) { return false; }
                }
                else
                {
                    auto [buf, pos]{::phy_engine::netlist::add_model(nl, ::phy_engine::model::YES{})};
                    (void)pos;
                    if(buf == nullptr) { set_error("pe_synth: failed to create YES"); return false; }
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

                    ::std::vector<::phy_engine::model::node_t*> values(m.signal_names.size(), nullptr);
                    ::std::vector<bool> assigned(m.signal_names.size(), false);
                    for(auto const root: comb.roots)
                    {
                        if(!synth_stmt_comb(b, comb.stmt_nodes, root, values, assigned, targets)) { return false; }
                    }

                    for(::std::size_t sig{}; sig < targets.size(); ++sig)
                    {
                        if(!targets[sig]) { continue; }
                        if(!assigned[sig])
                        {
                            set_error("pe_synth: always_comb did not assign all targets");
                            return false;
                        }
                        auto* lhs = b.sig_nodes[sig];
                        auto* rhs = values[sig];
                        if(lhs == nullptr || rhs == nullptr) { continue; }
                        auto [buf, pos]{::phy_engine::netlist::add_model(nl, ::phy_engine::model::YES{})};
                        (void)pos;
                        if(buf == nullptr) { set_error("pe_synth: failed to create YES"); return false; }
                        if(!connect_pin(buf, 0, rhs)) { return false; }
                        if(!connect_driver(buf, 1, lhs)) { return false; }
                    }
                }
            }
            else if(!m.always_combs.empty())
            {
                set_error("pe_synth: always_comb not supported by options");
                return false;
            }

            // always_ff blocks (restricted subset, no async reset yet).
            if(opt.support_always_ff)
            {
                for(auto const& ff: m.always_ffs)
                {
                    if(ff.events.size() != 1)
                    {
                        set_error("pe_synth: always_ff with multiple events (async reset) not supported yet");
                        return false;
                    }
                    auto const ev = ff.events.front_unchecked();
                    if(ev.k == sensitivity_event::kind::level)
                    {
                        set_error("pe_synth: level-sensitive always_ff not supported");
                        return false;
                    }
                    if(ev.signal >= b.sig_nodes.size())
                    {
                        set_error("pe_synth: always_ff clock signal out of range");
                        return false;
                    }
                    auto* clk = b.sig_nodes[ev.signal];
                    if(clk == nullptr) { set_error("pe_synth: null clk node"); return false; }
                    if(ev.k == sensitivity_event::kind::negedge)
                    {
                        clk = gate_not(clk);
                    }

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

                    auto next = b.sig_nodes;
                    for(auto const root: ff.roots)
                    {
                        if(!synth_stmt_ff(b, ff.stmt_nodes, root, next, targets)) { return false; }
                    }

                    for(::std::size_t sig{}; sig < targets.size(); ++sig)
                    {
                        if(!targets[sig]) { continue; }
                        auto* q = b.sig_nodes[sig];
                        auto* d = next[sig];
                        if(q == nullptr || d == nullptr) { continue; }

                        auto [dff, pos]{::phy_engine::netlist::add_model(nl, ::phy_engine::model::DFF{})};
                        (void)pos;
                        if(dff == nullptr) { set_error("pe_synth: failed to create DFF"); return false; }
                        if(!connect_pin(dff, 0, d)) { return false; }
                        if(!connect_pin(dff, 1, clk)) { return false; }
                        if(!connect_driver(dff, 2, q)) { return false; }
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
                if(child) { if(!synth_instance(*child, __builtin_addressof(b), {})) { return false; } }
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
        return ctx.synth_instance(top, nullptr, top_port_nodes);
    }
}  // namespace phy_engine::verilog::digital
