#pragma once

#include <algorithm>
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include <fast_io/fast_io_dsal/string.h>

#include "../../netlist/operation.h"

#include "../../model/models/digital/combinational/d_ff.h"
#include "../../model/models/digital/combinational/d_ff_arstn.h"
#include "../../model/models/digital/combinational/d_latch.h"
#include "../../model/models/digital/combinational/full_adder.h"
#include "../../model/models/digital/combinational/half_adder.h"
#include "../../model/models/digital/combinational/mul2.h"
#include "../../model/models/digital/combinational/random_generator4.h"
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
        bool assume_binary_inputs{false};  // treat X/Z as absent: `is_unknown(...)` folds to 0, dropping X-propagation mux networks
        bool optimize_wires{false};   // best-effort: remove synthesized YES buffers (net aliasing), keeps top-level port nodes intact
        bool optimize_mul2{false};    // best-effort: replace 2-bit multiplier tiles with MUL2 models
        bool optimize_adders{false};  // best-effort: replace gate-level adders with HALF_ADDER/FULL_ADDER models
        ::std::size_t loop_unroll_limit{64};  // bounded unrolling for dynamic for/while in procedural blocks
    };

    struct pe_synth_error
    {
        ::fast_io::u8string message{};
    };

    namespace details
    {
        inline bool is_output_pin(::fast_io::u8string_view model_name, std::size_t pin_idx, std::size_t pin_count) noexcept;

        inline ::fast_io::u8string_view model_name_u8(::phy_engine::model::model_base const& mb) noexcept
        {
            return (mb.ptr == nullptr) ? ::fast_io::u8string_view{} : mb.ptr->get_model_name();
        }

        inline bool is_const_input_model(::phy_engine::model::model_base const& mb,
                                         ::phy_engine::model::digital_node_statement_t v) noexcept
        {
            if(mb.ptr == nullptr) { return false; }
            if(mb.name.size() != 0) { return false; }  // named INPUTs are external IO, not constants
            if(model_name_u8(mb) != u8"INPUT") { return false; }
            auto vi = mb.ptr->get_attribute(0);
            if(vi.type != ::phy_engine::model::variant_type::digital) { return false; }
            return vi.digital == v;
        }

        inline ::phy_engine::model::node_t*
            find_existing_const_node(::phy_engine::netlist::netlist& nl, ::phy_engine::model::digital_node_statement_t v) noexcept
        {
            for(auto& blk : nl.models)
            {
                for(auto* m = blk.begin; m != blk.curr; ++m)
                {
                    if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }
                    if(is_const_input_model(*m, v))
                    {
                        auto pv = m->ptr->generate_pin_view();
                        if(pv.size == 1 && pv.pins[0].nodes != nullptr) { return pv.pins[0].nodes; }
                    }
                }
            }
            return nullptr;
        }

        inline ::phy_engine::model::node_t*
            make_const_node(::phy_engine::netlist::netlist& nl, ::phy_engine::model::digital_node_statement_t v) noexcept
        {
            auto& n = ::phy_engine::netlist::create_node(nl);
            auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = v});
            (void)pos;
            if(m == nullptr) { return nullptr; }
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, n)) { return nullptr; }
            return __builtin_addressof(n);
        }

        inline ::phy_engine::model::node_t*
            find_or_make_const_node(::phy_engine::netlist::netlist& nl, ::phy_engine::model::digital_node_statement_t v) noexcept
        {
            if(auto* n = find_existing_const_node(nl, v); n != nullptr) { return n; }
            return make_const_node(nl, v);
        }

        inline void optimize_adders_in_pe_netlist(::phy_engine::netlist::netlist& nl) noexcept
        {
            struct model_pos
            {
                std::size_t vec_pos{};
                std::size_t chunk_pos{};
            };
            struct gate
            {
                enum class kind : std::uint8_t
                {
                    and_gate,
                    or_gate,
                    xor_gate,
                    not_gate,
                    xnor_gate,
                };
                kind k{};
                model_pos pos{};
                ::phy_engine::model::model_base* mb{};
                ::phy_engine::model::node_t* in0{};
                ::phy_engine::model::node_t* in1{};
                ::phy_engine::model::node_t* out{};
            };

            struct gate_key
            {
                gate::kind k{};
                ::phy_engine::model::node_t* a{};
                ::phy_engine::model::node_t* b{};
            };
            struct gate_key_hash
            {
                std::size_t operator()(gate_key const& x) const noexcept
                {
                    auto const mix = [](std::size_t h, std::size_t v) noexcept -> std::size_t
                    {
                        return (h ^ (v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2)));
                    };
                    std::size_t h{};
                    h = mix(h, static_cast<std::size_t>(x.k));
                    h = mix(h, reinterpret_cast<std::size_t>(x.a));
                    h = mix(h, reinterpret_cast<std::size_t>(x.b));
                    return h;
                }
            };
            struct gate_key_eq
            {
                bool operator()(gate_key const& x, gate_key const& y) const noexcept { return x.k == y.k && x.a == y.a && x.b == y.b; }
            };

            auto canon_pair = [](gate::kind k, ::phy_engine::model::node_t* a, ::phy_engine::model::node_t* b) noexcept -> gate_key {
                if(reinterpret_cast<std::uintptr_t>(a) > reinterpret_cast<std::uintptr_t>(b)) { ::std::swap(a, b); }
                return gate_key{k, a, b};
            };

            ::std::vector<gate> gates{};
            gates.reserve(4096);

            ::std::unordered_map<::phy_engine::model::node_t*, std::size_t> gate_by_out{};
            gate_by_out.reserve(4096);

            ::std::unordered_map<gate_key, ::phy_engine::model::node_t*, gate_key_hash, gate_key_eq> out_by_inputs{};
            out_by_inputs.reserve(8192);

            ::std::unordered_map<::phy_engine::model::pin const*, bool> pin_is_output{};
            pin_is_output.reserve(1 << 14);

            ::std::unordered_map<::phy_engine::model::node_t*, std::size_t> consumer_count{};
            ::std::unordered_map<::phy_engine::model::node_t*, std::size_t> driver_count{};
            consumer_count.reserve(1 << 14);
            driver_count.reserve(1 << 14);

            auto note_pin = [&](::phy_engine::model::pin const* p, bool is_out) noexcept {
                if(p == nullptr) { return; }
                pin_is_output.emplace(p, is_out);
                if(p->nodes == nullptr) { return; }
                if(is_out) { ++driver_count[p->nodes]; }
                else { ++consumer_count[p->nodes]; }
            };

            auto classify_gate = [&](::phy_engine::model::model_base& mb,
                                     std::size_t vec_pos,
                                     std::size_t chunk_pos) noexcept -> std::optional<gate> {
                if(mb.type != ::phy_engine::model::model_type::normal || mb.ptr == nullptr) { return std::nullopt; }
                auto const name = model_name_u8(mb);
                gate g{};
                g.pos = model_pos{vec_pos, chunk_pos};
                g.mb = __builtin_addressof(mb);

                auto pv = mb.ptr->generate_pin_view();
                if(name == u8"AND" || name == u8"OR" || name == u8"XOR" || name == u8"XNOR")
                {
                    if(pv.size != 3) { return std::nullopt; }
                    g.in0 = pv.pins[0].nodes;
                    g.in1 = pv.pins[1].nodes;
                    g.out = pv.pins[2].nodes;
                    if(name == u8"AND") { g.k = gate::kind::and_gate; }
                    else if(name == u8"OR") { g.k = gate::kind::or_gate; }
                    else if(name == u8"XOR") { g.k = gate::kind::xor_gate; }
                    else { g.k = gate::kind::xnor_gate; }

                    note_pin(__builtin_addressof(pv.pins[0]), false);
                    note_pin(__builtin_addressof(pv.pins[1]), false);
                    note_pin(__builtin_addressof(pv.pins[2]), true);

                    return g;
                }
                if(name == u8"NOT")
                {
                    if(pv.size != 2) { return std::nullopt; }
                    g.k = gate::kind::not_gate;
                    g.in0 = pv.pins[0].nodes;
                    g.out = pv.pins[1].nodes;

                    note_pin(__builtin_addressof(pv.pins[0]), false);
                    note_pin(__builtin_addressof(pv.pins[1]), true);

                    return g;
                }

                // generic pin accounting for non-gates (so fanout is correct)
                for(std::size_t i{}; i < pv.size; ++i)
                {
                    // Best-effort: for most models output pins are later; but unknown => treat as consumer.
                    // (This only affects whether we delete a gate; safe because false positives reduce optimization.)
                    note_pin(__builtin_addressof(pv.pins[i]), false);
                }
                return std::nullopt;
            };

            for(std::size_t chunk_pos{}; chunk_pos < nl.models.size(); ++chunk_pos)
            {
                auto& blk = nl.models.index_unchecked(chunk_pos);
                for(std::size_t vec_pos{}; blk.begin + vec_pos < blk.curr; ++vec_pos)
                {
                    auto& mb = blk.begin[vec_pos];
                    auto og = classify_gate(mb, vec_pos, chunk_pos);
                    if(!og) { continue; }
                    if(og->out == nullptr) { continue; }

                    auto const gate_index = gates.size();
                    gates.push_back(*og);
                    gate_by_out.emplace(og->out, gate_index);

                    if(og->k == gate::kind::and_gate || og->k == gate::kind::or_gate || og->k == gate::kind::xor_gate ||
                       og->k == gate::kind::xnor_gate)
                    {
                        if(og->in0 != nullptr && og->in1 != nullptr)
                        {
                            out_by_inputs.emplace(canon_pair(og->k, og->in0, og->in1), og->out);
                        }
                    }
                }
            }

            auto gate_out = [&](gate::kind k, ::phy_engine::model::node_t* a, ::phy_engine::model::node_t* b) noexcept -> ::phy_engine::model::node_t* {
                auto it = out_by_inputs.find(canon_pair(k, a, b));
                return it == out_by_inputs.end() ? nullptr : it->second;
            };

            auto gate_ptr_by_out = [&](::phy_engine::model::node_t* out) noexcept -> gate const* {
                auto it = gate_by_out.find(out);
                if(it == gate_by_out.end()) { return nullptr; }
                return __builtin_addressof(gates[it->second]);
            };

            auto can_delete_gate_output = [&](::phy_engine::model::node_t* out) noexcept -> bool {
                if(out == nullptr) { return false; }
                auto const dc = driver_count.find(out);
                if(dc == driver_count.end() || dc->second != 1) { return false; }
                return true;
            };

            struct action
            {
                bool is_full{};
                ::phy_engine::model::node_t* a{};
                ::phy_engine::model::node_t* b{};
                ::phy_engine::model::node_t* cin{};
                ::phy_engine::model::node_t* s{};
                ::phy_engine::model::node_t* cout{};
                ::std::vector<model_pos> del{};
            };

            ::std::vector<action> actions{};
            actions.reserve(4096);
            ::std::unordered_map<::phy_engine::model::model_base*, bool> used_models{};
            used_models.reserve(1 << 14);

            auto mark_used = [&](gate const* g) noexcept -> bool {
                if(g == nullptr || g->mb == nullptr) { return false; }
                if(used_models.contains(g->mb)) { return false; }
                used_models.emplace(g->mb, true);
                return true;
            };

            auto try_add_half_adder = [&](gate const& gx) noexcept {
                if(gx.k != gate::kind::xor_gate) { return; }
                if(gx.in0 == nullptr || gx.in1 == nullptr || gx.out == nullptr) { return; }
                auto* cnode = gate_out(gate::kind::and_gate, gx.in0, gx.in1);
                if(cnode == nullptr) { return; }
                auto const* gand = gate_ptr_by_out(cnode);
                if(gand == nullptr || gand->k != gate::kind::and_gate) { return; }

                if(!can_delete_gate_output(gx.out) || !can_delete_gate_output(cnode)) { return; }
                if(consumer_count[gx.out] == 0) { return; }

                if(!mark_used(__builtin_addressof(gx))) { return; }
                if(!mark_used(gand)) { return; }

                action a{};
                a.is_full = false;
                a.a = gx.in0;
                a.b = gx.in1;
                a.s = gx.out;
                a.cout = cnode;
                a.del = {gx.pos, gand->pos};
                actions.push_back(::std::move(a));
            };

            auto try_add_full_adder_cin1 = [&](gate const& gnot) noexcept {
                if(gnot.k != gate::kind::not_gate) { return; }
                if(gnot.in0 == nullptr || gnot.out == nullptr) { return; }

                // sum = NOT(axb), where axb = XOR(a,b)
                auto const* gxb = gate_ptr_by_out(gnot.in0);
                if(gxb == nullptr || gxb->k != gate::kind::xor_gate || gxb->in0 == nullptr || gxb->in1 == nullptr) { return; }

                // cout = OR(a,b)
                auto* cnode = gate_out(gate::kind::or_gate, gxb->in0, gxb->in1);
                if(cnode == nullptr) { return; }
                auto const* gor = gate_ptr_by_out(cnode);
                if(gor == nullptr || gor->k != gate::kind::or_gate) { return; }

                if(!can_delete_gate_output(gnot.out) || !can_delete_gate_output(cnode)) { return; }
                if(consumer_count[gnot.out] == 0) { return; }
                if(consumer_count[cnode] == 0) { return; }

                // We can delete XOR(a,b) only if it's only used by NOT(sum).
                bool const can_delete_xor_ab = (consumer_count[gnot.in0] == 1 && can_delete_gate_output(gnot.in0));
                if(!mark_used(__builtin_addressof(gnot))) { return; }
                if(!mark_used(gor)) { return; }
                if(can_delete_xor_ab && !mark_used(gxb)) { return; }

                auto* cin1 = find_or_make_const_node(nl, ::phy_engine::model::digital_node_statement_t::true_state);
                if(cin1 == nullptr) { return; }

                action a{};
                a.is_full = true;
                a.a = gxb->in0;
                a.b = gxb->in1;
                a.cin = cin1;
                a.s = gnot.out;
                a.cout = cnode;
                a.del = {gnot.pos, gor->pos};
                if(can_delete_xor_ab) { a.del.push_back(gxb->pos); }
                actions.push_back(::std::move(a));
            };

            auto try_add_full_adder_general = [&](gate const& g2) noexcept {
                if(g2.k != gate::kind::xor_gate) { return; }
                if(g2.in0 == nullptr || g2.in1 == nullptr || g2.out == nullptr) { return; }

                // sum = XOR(axb, cin), where axb = XOR(a,b)
                for(int pick = 0; pick < 2; ++pick)
                {
                    auto* axb = (pick == 0) ? g2.in0 : g2.in1;
                    auto* cin = (pick == 0) ? g2.in1 : g2.in0;
                    auto const* g1 = gate_ptr_by_out(axb);
                    if(g1 == nullptr || g1->k != gate::kind::xor_gate || g1->in0 == nullptr || g1->in1 == nullptr) { continue; }

                    auto* a = g1->in0;
                    auto* b = g1->in1;

                    auto* t1 = gate_out(gate::kind::and_gate, a, b);
                    auto* t2 = gate_out(gate::kind::and_gate, a, cin);
                    auto* t3 = gate_out(gate::kind::and_gate, b, cin);
                    if(t1 == nullptr || t2 == nullptr || t3 == nullptr) { continue; }

                    auto* or12 = gate_out(gate::kind::or_gate, t1, t2);
                    if(or12 == nullptr) { continue; }
                    auto* cout = gate_out(gate::kind::or_gate, or12, t3);
                    if(cout == nullptr) { continue; }

                    auto const* gt1 = gate_ptr_by_out(t1);
                    auto const* gt2 = gate_ptr_by_out(t2);
                    auto const* gt3 = gate_ptr_by_out(t3);
                    auto const* gor12 = gate_ptr_by_out(or12);
                    auto const* gcout = gate_ptr_by_out(cout);
                    if(gt1 == nullptr || gt2 == nullptr || gt3 == nullptr || gor12 == nullptr || gcout == nullptr) { continue; }
                    if(gt1->k != gate::kind::and_gate || gt2->k != gate::kind::and_gate || gt3->k != gate::kind::and_gate) { continue; }
                    if(gor12->k != gate::kind::or_gate || gcout->k != gate::kind::or_gate) { continue; }

                    if(!can_delete_gate_output(g2.out) || !can_delete_gate_output(cout)) { continue; }
                    if(consumer_count[g2.out] == 0) { continue; }
                    if(consumer_count[t1] != 1 || consumer_count[t2] != 1 || consumer_count[t3] != 1 || consumer_count[or12] != 1) { continue; }

                    bool const can_delete_xor_ab = (consumer_count[axb] == 1 && can_delete_gate_output(axb));

                    if(!mark_used(__builtin_addressof(g2))) { continue; }
                    if(!mark_used(gt1) || !mark_used(gt2) || !mark_used(gt3) || !mark_used(gor12) || !mark_used(gcout)) { continue; }
                    if(can_delete_xor_ab && !mark_used(g1)) { continue; }

                    action act{};
                    act.is_full = true;
                    act.a = a;
                    act.b = b;
                    act.cin = cin;
                    act.s = g2.out;
                    act.cout = cout;
                    act.del = {g2.pos, gt1->pos, gt2->pos, gt3->pos, gor12->pos, gcout->pos};
                    if(can_delete_xor_ab) { act.del.push_back(g1->pos); }
                    actions.push_back(::std::move(act));
                    return;
                }
            };

            // Pass 1: general full adders (non-constant carry).
            for(auto const& g : gates) { try_add_full_adder_general(g); }
            // Pass 2: cin=0 half adders (common in LSB).
            for(auto const& g : gates) { try_add_half_adder(g); }
            // Pass 3: cin=1 simplified form (two's complement +1).
            for(auto const& g : gates) { try_add_full_adder_cin1(g); }

            ::std::vector<model_pos> to_delete{};
            to_delete.reserve(actions.size() * 8);
            for(auto const& a : actions)
            {
                for(auto const& p : a.del) { to_delete.push_back(p); }
            }

            auto less_desc = [](model_pos const& x, model_pos const& y) noexcept {
                if(x.chunk_pos != y.chunk_pos) { return x.chunk_pos > y.chunk_pos; }
                return x.vec_pos > y.vec_pos;
            };
            ::std::sort(to_delete.begin(), to_delete.end(), less_desc);
            to_delete.erase(::std::unique(to_delete.begin(),
                                          to_delete.end(),
                                          [](model_pos const& x, model_pos const& y) noexcept {
                                              return x.chunk_pos == y.chunk_pos && x.vec_pos == y.vec_pos;
                                          }),
                            to_delete.end());

            for(auto const& p : to_delete) { (void)::phy_engine::netlist::delete_model(nl, p.vec_pos, p.chunk_pos); }

            for(auto const& a : actions)
            {
                if(a.is_full)
                {
                    auto [m, mp] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::FULL_ADDER{});
                    (void)mp;
                    if(m == nullptr) { continue; }
                    if(a.a) { (void)::phy_engine::netlist::add_to_node(nl, *m, 0, *a.a); }
                    if(a.b) { (void)::phy_engine::netlist::add_to_node(nl, *m, 1, *a.b); }
                    if(a.cin) { (void)::phy_engine::netlist::add_to_node(nl, *m, 2, *a.cin); }
                    if(a.s) { (void)::phy_engine::netlist::add_to_node(nl, *m, 3, *a.s); }
                    if(a.cout) { (void)::phy_engine::netlist::add_to_node(nl, *m, 4, *a.cout); }
                }
                else
                {
                    auto [m, mp] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::HALF_ADDER{});
                    (void)mp;
                    if(m == nullptr) { continue; }
                    if(a.a) { (void)::phy_engine::netlist::add_to_node(nl, *m, 0, *a.a); }
                    if(a.b) { (void)::phy_engine::netlist::add_to_node(nl, *m, 1, *a.b); }
                    if(a.s) { (void)::phy_engine::netlist::add_to_node(nl, *m, 2, *a.s); }
                    if(a.cout) { (void)::phy_engine::netlist::add_to_node(nl, *m, 3, *a.cout); }
                }
            }
        }

        inline void optimize_mul2_in_pe_netlist(::phy_engine::netlist::netlist& nl) noexcept
        {
            struct model_pos
            {
                std::size_t vec_pos{};
                std::size_t chunk_pos{};
            };
            struct gate
            {
                enum class kind : std::uint8_t
                {
                    and_gate,
                    xor_gate
                };
                kind k{};
                model_pos pos{};
                ::phy_engine::model::model_base* mb{};
                ::phy_engine::model::node_t* in0{};
                ::phy_engine::model::node_t* in1{};
                ::phy_engine::model::node_t* out{};
            };

            struct gate_key
            {
                gate::kind k{};
                ::phy_engine::model::node_t* a{};
                ::phy_engine::model::node_t* b{};
            };
            struct gate_key_hash
            {
                std::size_t operator()(gate_key const& x) const noexcept
                {
                    auto const mix = [](std::size_t h, std::size_t v) noexcept -> std::size_t
                    {
                        return (h ^ (v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2)));
                    };
                    std::size_t h{};
                    h = mix(h, static_cast<std::size_t>(x.k));
                    h = mix(h, reinterpret_cast<std::size_t>(x.a));
                    h = mix(h, reinterpret_cast<std::size_t>(x.b));
                    return h;
                }
            };
            struct gate_key_eq
            {
                bool operator()(gate_key const& x, gate_key const& y) const noexcept { return x.k == y.k && x.a == y.a && x.b == y.b; }
            };

            auto canon_pair = [](gate::kind k, ::phy_engine::model::node_t* a, ::phy_engine::model::node_t* b) noexcept -> gate_key
            {
                if(reinterpret_cast<std::uintptr_t>(a) > reinterpret_cast<std::uintptr_t>(b)) { ::std::swap(a, b); }
                return gate_key{k, a, b};
            };

            ::std::vector<gate> gates{};
            gates.reserve(4096);

            ::std::unordered_map<::phy_engine::model::node_t*, std::size_t> gate_by_out{};
            gate_by_out.reserve(4096);

            ::std::unordered_map<gate_key, ::phy_engine::model::node_t*, gate_key_hash, gate_key_eq> out_by_inputs{};
            out_by_inputs.reserve(8192);

            ::std::unordered_map<::phy_engine::model::node_t*, ::std::vector<std::size_t>> uses{};
            uses.reserve(8192);

            ::std::unordered_map<::phy_engine::model::node_t*, std::size_t> consumer_count{};
            ::std::unordered_map<::phy_engine::model::node_t*, std::size_t> driver_count{};
            consumer_count.reserve(1 << 14);
            driver_count.reserve(1 << 14);

            ::std::unordered_map<::phy_engine::model::node_t*, bool> driver_is_named_input{};
            ::std::unordered_map<::phy_engine::model::node_t*, bool> driver_is_non_input{};
            driver_is_named_input.reserve(1 << 14);
            driver_is_non_input.reserve(1 << 14);
            ::std::unordered_map<::phy_engine::model::node_t*, ::fast_io::u8string_view> named_input_name{};
            named_input_name.reserve(1 << 14);

            // Best-effort pin direction classification for the whole netlist.
            for(auto& blk : nl.models)
            {
                for(auto* mb = blk.begin; mb != blk.curr; ++mb)
                {
                    if(mb->type != ::phy_engine::model::model_type::normal || mb->ptr == nullptr) { continue; }
                    auto const name = model_name_u8(*mb);
                    auto pv = mb->ptr->generate_pin_view();
                    for(std::size_t i{}; i < pv.size; ++i)
                    {
                        auto* n = pv.pins[i].nodes;
                        if(n == nullptr) { continue; }
                        if(is_output_pin(name, i, pv.size)) { ++driver_count[n]; }
                        else { ++consumer_count[n]; }

                        if(is_output_pin(name, i, pv.size))
                        {
                            if(name == u8"INPUT" && !mb->name.empty())
                            {
                                driver_is_named_input[n] = true;
                                named_input_name.emplace(n, ::fast_io::u8string_view{mb->name.data(), mb->name.size()});
                            }
                            else if(name != u8"INPUT")
                            {
                                driver_is_non_input[n] = true;
                            }
                        }
                    }
                }
            }

            auto classify_gate = [&](::phy_engine::model::model_base& mb, std::size_t vec_pos, std::size_t chunk_pos) noexcept -> std::optional<gate>
            {
                if(mb.type != ::phy_engine::model::model_type::normal || mb.ptr == nullptr) { return std::nullopt; }
                auto const name = model_name_u8(mb);
                gate g{};
                if(name == u8"AND") { g.k = gate::kind::and_gate; }
                else if(name == u8"XOR") { g.k = gate::kind::xor_gate; }
                else { return std::nullopt; }
                auto pv = mb.ptr->generate_pin_view();
                if(pv.size != 3) { return std::nullopt; }
                g.pos = model_pos{vec_pos, chunk_pos};
                g.mb = __builtin_addressof(mb);
                g.in0 = pv.pins[0].nodes;
                g.in1 = pv.pins[1].nodes;
                g.out = pv.pins[2].nodes;
                if(g.in0 == nullptr || g.in1 == nullptr || g.out == nullptr) { return std::nullopt; }
                return g;
            };

            // Collect candidate gates and build indices.
            for(std::size_t chunk_pos{}; chunk_pos < nl.models.size(); ++chunk_pos)
            {
                auto& blk = nl.models.index_unchecked(chunk_pos);
                for(auto* mb = blk.begin; mb != blk.curr; ++mb)
                {
                    if(mb->type != ::phy_engine::model::model_type::normal || mb->ptr == nullptr) { continue; }
                    auto const vec_pos = static_cast<std::size_t>(mb - blk.begin);
                    auto og = classify_gate(*mb, vec_pos, chunk_pos);
                    if(!og) { continue; }
                    auto const idx = gates.size();
                    gates.push_back(*og);
                    gate_by_out.emplace(og->out, idx);
                    out_by_inputs.emplace(canon_pair(og->k, og->in0, og->in1), og->out);
                    uses[og->in0].push_back(idx);
                    uses[og->in1].push_back(idx);
                }
            }

            if(gates.empty()) { return; }

            ::std::vector<bool> dead{};
            dead.assign(gates.size(), false);

            auto gate_idx_for_out = [&](::phy_engine::model::node_t* out) noexcept -> std::optional<std::size_t>
            {
                auto it = gate_by_out.find(out);
                if(it == gate_by_out.end()) { return std::nullopt; }
                return it->second;
            };

            auto out_of_and = [&](::phy_engine::model::node_t* a, ::phy_engine::model::node_t* b) noexcept -> ::phy_engine::model::node_t*
            {
                auto it = out_by_inputs.find(canon_pair(gate::kind::and_gate, a, b));
                if(it == out_by_inputs.end()) { return nullptr; }
                return it->second;
            };

            auto is_in_pair = [&](::phy_engine::model::node_t* x, ::phy_engine::model::node_t* p0, ::phy_engine::model::node_t* p1) noexcept -> bool
            {
                return x == p0 || x == p1;
            };

            auto try_make_mul2 = [&](std::size_t p1_xor_idx) noexcept -> bool
            {
                if(p1_xor_idx >= gates.size() || dead[p1_xor_idx]) { return false; }
                auto const& g_p1 = gates[p1_xor_idx];
                if(g_p1.k != gate::kind::xor_gate) { return false; }

                auto* const t1_out = g_p1.in0;
                auto* const t2_out = g_p1.in1;
                auto const t1_idx_opt = gate_idx_for_out(t1_out);
                auto const t2_idx_opt = gate_idx_for_out(t2_out);
                if(!t1_idx_opt || !t2_idx_opt) { return false; }
                auto const t1_idx = *t1_idx_opt;
                auto const t2_idx = *t2_idx_opt;
                if(dead[t1_idx] || dead[t2_idx]) { return false; }
                auto const& g_t1 = gates[t1_idx];
                auto const& g_t2 = gates[t2_idx];
                if(g_t1.k != gate::kind::and_gate || g_t2.k != gate::kind::and_gate) { return false; }

                auto* const c1_out = out_of_and(t1_out, t2_out);
                if(c1_out == nullptr) { return false; }
                auto const c1_idx_opt = gate_idx_for_out(c1_out);
                if(!c1_idx_opt) { return false; }
                auto const c1_idx = *c1_idx_opt;
                if(dead[c1_idx] || gates[c1_idx].k != gate::kind::and_gate) { return false; }

                // Strict internal-node usage checks (skip if shared elsewhere).
                auto const dc = [&](::phy_engine::model::node_t* n) noexcept -> std::size_t
                {
                    auto it = driver_count.find(n);
                    return it == driver_count.end() ? 0u : it->second;
                };
                auto const cc = [&](::phy_engine::model::node_t* n) noexcept -> std::size_t
                {
                    auto it = consumer_count.find(n);
                    return it == consumer_count.end() ? 0u : it->second;
                };
                if(dc(t1_out) != 1u || cc(t1_out) != 2u) { return false; }
                if(dc(t2_out) != 1u || cc(t2_out) != 2u) { return false; }

                // Find p2 = XOR(t3, c1) and p3 = AND(t3, c1).
                auto uses_it = uses.find(c1_out);
                if(uses_it == uses.end()) { return false; }
                for(auto const p2_xor_idx : uses_it->second)
                {
                    if(p2_xor_idx >= gates.size() || dead[p2_xor_idx]) { continue; }
                    if(p2_xor_idx == p1_xor_idx) { continue; }
                    auto const& g_p2 = gates[p2_xor_idx];
                    if(g_p2.k != gate::kind::xor_gate) { continue; }

                    auto* const t3_out = (g_p2.in0 == c1_out) ? g_p2.in1 : (g_p2.in1 == c1_out ? g_p2.in0 : nullptr);
                    if(t3_out == nullptr) { continue; }

                    auto* const p3_out = out_of_and(c1_out, t3_out);
                    if(p3_out == nullptr) { continue; }
                    auto const p3_idx_opt = gate_idx_for_out(p3_out);
                    if(!p3_idx_opt) { continue; }
                    auto const p3_idx = *p3_idx_opt;
                    if(dead[p3_idx] || gates[p3_idx].k != gate::kind::and_gate) { continue; }

                    auto const t3_idx_opt = gate_idx_for_out(t3_out);
                    if(!t3_idx_opt) { continue; }
                    auto const t3_idx = *t3_idx_opt;
                    if(dead[t3_idx] || gates[t3_idx].k != gate::kind::and_gate) { continue; }
                    auto const& g_t3 = gates[t3_idx];

                    if(dc(c1_out) != 1u || cc(c1_out) != 2u) { continue; }
                    if(dc(t3_out) != 1u || cc(t3_out) != 2u) { continue; }

                    // t3 must be AND(one from g_t1 inputs, one from g_t2 inputs).
                    auto* const s10 = g_t1.in0;
                    auto* const s11 = g_t1.in1;
                    auto* const s20 = g_t2.in0;
                    auto* const s21 = g_t2.in1;
                    bool const t3_in0_in_s1 = is_in_pair(g_t3.in0, s10, s11);
                    bool const t3_in1_in_s1 = is_in_pair(g_t3.in1, s10, s11);
                    bool const t3_in0_in_s2 = is_in_pair(g_t3.in0, s20, s21);
                    bool const t3_in1_in_s2 = is_in_pair(g_t3.in1, s20, s21);
                    if(!((t3_in0_in_s1 && t3_in1_in_s2) || (t3_in0_in_s2 && t3_in1_in_s1))) { continue; }

                    auto* const b1 = t3_in0_in_s1 ? g_t3.in0 : g_t3.in1;
                    auto* const a1 = t3_in0_in_s2 ? g_t3.in0 : g_t3.in1;
                    auto* const a0 = (b1 == s10) ? s11 : s10;
                    auto* const b0 = (a1 == s20) ? s21 : s20;

                    // Deterministically assign operand A/B based on the driving named INPUT instance names.
                    // Multiplication is commutative, but downstream export (e.g. PhysicsLab Multiplier) has labeled A/B pins.
                    auto base_of_named_input = [&](::phy_engine::model::node_t* n) noexcept -> ::fast_io::u8string_view
                    {
                        auto it = named_input_name.find(n);
                        if(it == named_input_name.end()) { return {}; }
                        auto const full = it->second;
                        for(std::size_t i{}; i < full.size(); ++i)
                        {
                            if(full[i] == u8'[') { return ::fast_io::u8string_view{full.data(), i}; }
                        }
                        return full;
                    };

                    auto const base_a0 = base_of_named_input(a0);
                    auto const base_a1 = base_of_named_input(a1);
                    auto const base_b0 = base_of_named_input(b0);
                    auto const base_b1 = base_of_named_input(b1);

                    auto* aa0 = a0;
                    auto* aa1 = a1;
                    auto* bb0 = b0;
                    auto* bb1 = b1;

                    if(!base_a0.empty() && base_a0 == base_a1 && !base_b0.empty() && base_b0 == base_b1)
                    {
                        // Prefer explicit "a"/"b" naming; otherwise choose stable (lexicographically smaller) base as operand A.
                        auto desired_a = base_a0;
                        if(base_a0 != u8"a" && base_b0 == u8"a") { desired_a = base_b0; }
                        else if(base_a0 != u8"a" && base_b0 != u8"a")
                        {
                            desired_a = (base_a0 <= base_b0) ? base_a0 : base_b0;
                        }

                        if(base_a0 != desired_a)
                        {
                            ::std::swap(aa0, bb0);
                            ::std::swap(aa1, bb1);
                        }
                    }

                    // p0 = AND(a0, b0)
                    auto* const p0_out = out_of_and(aa0, bb0);
                    if(p0_out == nullptr) { continue; }
                    auto const p0_idx_opt = gate_idx_for_out(p0_out);
                    if(!p0_idx_opt) { continue; }
                    auto const p0_idx = *p0_idx_opt;
                    if(dead[p0_idx] || gates[p0_idx].k != gate::kind::and_gate) { continue; }

                    // Restrict to "tile inputs" driven by named top-level INPUT models to avoid false-positive matches.
                    auto is_primary_input_node = [&](::phy_engine::model::node_t* n) noexcept -> bool
                    {
                        if(n == nullptr) { return false; }
                        if(dc(n) != 1u) { return false; }
                        if(auto it = driver_is_non_input.find(n); it != driver_is_non_input.end() && it->second) { return false; }
                        auto it = driver_is_named_input.find(n);
                        return it != driver_is_named_input.end() && it->second;
                    };
                    if(!is_primary_input_node(a0) || !is_primary_input_node(a1) || !is_primary_input_node(b0) || !is_primary_input_node(b1)) { continue; }

                    // Sanity: output nodes should have a single driver.
                    if(dc(p0_out) != 1u || dc(g_p1.out) != 1u || dc(g_p2.out) != 1u || dc(p3_out) != 1u) { continue; }

                    // Distinct gates.
                    std::size_t ids[8]{p1_xor_idx, t1_idx, t2_idx, c1_idx, p2_xor_idx, p3_idx, t3_idx, p0_idx};
                    bool distinct{true};
                    for(int i = 0; i < 8 && distinct; ++i)
                    {
                        for(int j = i + 1; j < 8; ++j)
                        {
                            if(ids[i] == ids[j])
                            {
                                distinct = false;
                                break;
                            }
                        }
                    }
                    if(!distinct) { continue; }

                    {
                        auto [m, mp] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::MUL2{});
                        (void)mp;
                        if(m == nullptr) { return false; }

                        // Inputs: a0,a1,b0,b1; Outputs: p0,p1,p2,p3
                        if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *aa0)) { return false; }
                        if(!::phy_engine::netlist::add_to_node(nl, *m, 1, *aa1)) { return false; }
                        if(!::phy_engine::netlist::add_to_node(nl, *m, 2, *bb0)) { return false; }
                        if(!::phy_engine::netlist::add_to_node(nl, *m, 3, *bb1)) { return false; }

                        if(!::phy_engine::netlist::add_to_node(nl, *m, 4, *p0_out)) { return false; }
                        if(!::phy_engine::netlist::add_to_node(nl, *m, 5, *g_p1.out)) { return false; }
                        if(!::phy_engine::netlist::add_to_node(nl, *m, 6, *g_p2.out)) { return false; }
                        if(!::phy_engine::netlist::add_to_node(nl, *m, 7, *p3_out)) { return false; }
                    }

                    // Delete old gates (best-effort).
                    auto kill = [&](std::size_t i) noexcept
                    {
                        dead[i] = true;
                        (void)::phy_engine::netlist::delete_model(nl, gates[i].pos.vec_pos, gates[i].pos.chunk_pos);
                    };
                    for(auto const i : ids) { kill(i); }
                    return true;
                }

                return false;
            };

            for(std::size_t i{}; i < gates.size(); ++i) { (void)try_make_mul2(i); }
        }

        inline bool is_output_pin(::fast_io::u8string_view model_name, std::size_t pin_idx, std::size_t pin_count) noexcept
        {
            if(model_name == u8"INPUT") { return pin_idx == 0; }
            if(model_name == u8"OUTPUT") { return false; }
            if(model_name == u8"YES") { return pin_idx == 1; }
            if(model_name == u8"NOT") { return pin_idx == 1; }
            if(model_name == u8"AND" || model_name == u8"OR" || model_name == u8"XOR" || model_name == u8"XNOR" || model_name == u8"NAND" ||
               model_name == u8"NOR" || model_name == u8"IMP" || model_name == u8"NIMP" || model_name == u8"CASE_EQ" || model_name == u8"IS_UNKNOWN")
            {
                return pin_count != 0 && pin_idx + 1 == pin_count;
            }
            if(model_name == u8"HALF_ADDER") { return pin_idx == 2 || pin_idx == 3; }
            if(model_name == u8"FULL_ADDER") { return pin_idx == 3 || pin_idx == 4; }
            if(model_name == u8"HALF_SUB") { return pin_idx == 2 || pin_idx == 3; }
            if(model_name == u8"FULL_SUB") { return pin_idx == 3 || pin_idx == 4; }
            if(model_name == u8"MUL2") { return pin_idx >= 4; }
            if(model_name == u8"DFF") { return pin_idx == 2; }
            if(model_name == u8"DFF_ARSTN") { return pin_idx == 3; }
            if(model_name == u8"D_LATCH") { return pin_idx == 2; }
            if(model_name == u8"TICK_DELAY") { return pin_idx == 2; }
            if(model_name == u8"TRI") { return pin_idx == 2; }
            if(model_name == u8"RESOLVE2") { return pin_idx == 2; }
            return false;
        }

        inline void optimize_eliminate_yes_buffers(::phy_engine::netlist::netlist& nl,
                                                   ::std::vector<::phy_engine::model::node_t*> const& protected_nodes) noexcept
        {
            ::std::unordered_map<::phy_engine::model::pin const*, bool> pin_out{};
            pin_out.reserve(1 << 16);

            auto const is_protected = [&](::phy_engine::model::node_t* n) noexcept -> bool {
                for(auto* p : protected_nodes)
                {
                    if(p == n) { return true; }
                }
                return false;
            };

            // Build pin->is_output map for the whole netlist (best-effort).
            for(auto& blk : nl.models)
            {
                for(auto* m = blk.begin; m != blk.curr; ++m)
                {
                    if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }
                    auto const name = model_name_u8(*m);
                    auto pv = m->ptr->generate_pin_view();
                    for(std::size_t i{}; i < pv.size; ++i)
                    {
                        pin_out.emplace(__builtin_addressof(pv.pins[i]), is_output_pin(name, i, pv.size));
                    }
                }
            }

            ::std::vector<::phy_engine::netlist::model_pos> yes_models{};
            yes_models.reserve(1 << 14);

            for(std::size_t chunk_pos{}; chunk_pos < nl.models.size(); ++chunk_pos)
            {
                auto& blk = nl.models.index_unchecked(chunk_pos);
                for(std::size_t vec_pos{}; blk.begin + vec_pos < blk.curr; ++vec_pos)
                {
                    auto& mb = blk.begin[vec_pos];
                    if(mb.type != ::phy_engine::model::model_type::normal || mb.ptr == nullptr) { continue; }
                    if(model_name_u8(mb) != u8"YES") { continue; }
                    yes_models.push_back({vec_pos, chunk_pos});
                }
            }

            for(auto const mp : yes_models)
            {
                auto* mb = ::phy_engine::netlist::get_model(nl, mp);
                if(mb == nullptr || mb->type != ::phy_engine::model::model_type::normal || mb->ptr == nullptr) { continue; }
                if(model_name_u8(*mb) != u8"YES") { continue; }

                auto pv = mb->ptr->generate_pin_view();
                if(pv.size != 2) { continue; }
                auto* in_node = pv.pins[0].nodes;
                auto* out_node = pv.pins[1].nodes;
                auto const* out_pin = __builtin_addressof(pv.pins[1]);
                if(in_node == nullptr || out_node == nullptr) { continue; }
                if(in_node == out_node) { continue; }
                if(is_protected(out_node)) { continue; }

                // Ensure `out_node` has exactly one driver pin, and it's this YES output pin.
                std::size_t drivers{};
                bool ok_driver{true};
                for(auto const* p : out_node->pins)
                {
                    auto it = pin_out.find(p);
                    bool const is_out = (it != pin_out.end()) ? it->second : false;
                    if(is_out)
                    {
                        ++drivers;
                        if(p != out_pin) { ok_driver = false; }
                    }
                }
                if(!ok_driver || drivers != 1) { continue; }

                // Move all non-YES-output pins from out_node to in_node.
                ::std::vector<::phy_engine::model::pin*> pins_to_move{};
                pins_to_move.reserve(out_node->pins.size());
                for(auto* p : out_node->pins)
                {
                    if(p == out_pin) { continue; }
                    pins_to_move.push_back(p);
                }
                for(auto* p : pins_to_move)
                {
                    out_node->pins.erase(p);
                    p->nodes = in_node;
                    in_node->pins.insert(p);
                }

                (void)::phy_engine::netlist::delete_model(nl, mp);
            }
        }

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
                if(opt.assume_binary_inputs)
                {
                    // In many PE workflows we only care about 0/1 operation and want to avoid the large mux networks
                    // generated by X/Z propagation logic in the Verilog frontend.
                    return const_node(::phy_engine::verilog::digital::logic_t::false_state);
                }

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
                case stmt_node::kind::blocking_assign_vec:
                case stmt_node::kind::nonblocking_assign_vec:
                {
                    for(auto const sig: n.lhs_signals)
                    {
                        if(sig < out.size()) { out[sig] = true; }
                    }
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
                case stmt_node::kind::blocking_assign_vec:
                case stmt_node::kind::nonblocking_assign_vec:
                {
                    if(n.lhs_signals.size() != n.rhs_roots.size()) { return true; }
                    for(::std::size_t i{}; i < n.lhs_signals.size(); ++i)
                    {
                        auto const sig{n.lhs_signals.index_unchecked(i)};
                        if(sig >= reset_values.size() || sig >= has_reset.size()) { continue; }
                        if(sig < targets.size() && targets[sig])
                        {
                            ::phy_engine::verilog::digital::logic_t v{};
                            if(!eval_const_expr_to_logic(b, n.rhs_roots.index_unchecked(i), v))
                            {
                                b.ctx.set_error("pe_synth: async reset assignment must be constant");
                                return false;
                            }
                            reset_values[sig] = v;
                            has_reset[sig] = true;
                        }
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
                case stmt_node::kind::blocking_assign_vec:
                case stmt_node::kind::nonblocking_assign_vec:
                {
                    if(n.lhs_signals.size() != n.rhs_roots.size()) { return true; }
                    for(::std::size_t i{}; i < n.lhs_signals.size(); ++i)
                    {
                        auto const sig{n.lhs_signals.index_unchecked(i)};
                        if(sig >= reset_values.size() || sig >= has_reset.size()) { continue; }
                        if(sig < targets.size() && targets[sig])
                        {
                            ::phy_engine::verilog::digital::logic_t v{};
                            if(!eval_const_expr_to_logic(b, n.rhs_roots.index_unchecked(i), v)) { return false; }
                            reset_values[sig] = v;
                            has_reset[sig] = true;
                        }
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
                case stmt_node::kind::blocking_assign_vec:
                case stmt_node::kind::nonblocking_assign_vec:
                {
                    if(n.lhs_signals.size() != n.rhs_roots.size()) { return true; }
                    if(n.lhs_signals.empty()) { return true; }

                    ::std::vector<::phy_engine::model::node_t*> rhs_nodes{};
                    rhs_nodes.resize(n.rhs_roots.size());
                    for(::std::size_t i{}; i < n.rhs_roots.size(); ++i)
                    {
                        auto* rhs = b.expr_in_env(n.rhs_roots.index_unchecked(i), cur, nullptr);
                        if(n.delay_ticks != 0) { rhs = b.ctx.tick_delay(rhs, n.delay_ticks); }
                        rhs_nodes[i] = rhs;
                    }

                    if(n.k == stmt_node::kind::blocking_assign_vec)
                    {
                        for(::std::size_t i{}; i < n.lhs_signals.size(); ++i)
                        {
                            auto const sig{n.lhs_signals.index_unchecked(i)};
                            if(sig < cur.size()) { cur[sig] = rhs_nodes[i]; }
                        }
                    }

                    for(::std::size_t i{}; i < n.lhs_signals.size(); ++i)
                    {
                        auto const sig{n.lhs_signals.index_unchecked(i)};
                        if(sig < targets.size() && targets[sig] && sig < next.size()) { next[sig] = rhs_nodes[i]; }
                    }
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

                        ::phy_engine::verilog::digital::logic_t cv{};
                        if(b.ctx.try_get_const(cond, cv))
                        {
                            if(cv == ::phy_engine::verilog::digital::logic_t::false_state) { break; }
                            if(cv == ::phy_engine::verilog::digital::logic_t::true_state)
                            {
                                if(n.body_stmt != SIZE_MAX)
                                {
                                    if(!synth_stmt_ff(b, arena, n.body_stmt, cur, next, targets)) { return false; }
                                }
                                if(n.step_stmt != SIZE_MAX)
                                {
                                    if(!synth_stmt_ff(b, arena, n.step_stmt, cur, next, targets)) { return false; }
                                }
                                continue;
                            }
                        }

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

                        ::phy_engine::verilog::digital::logic_t cv{};
                        if(b.ctx.try_get_const(cond, cv))
                        {
                            if(cv == ::phy_engine::verilog::digital::logic_t::false_state) { break; }
                            if(cv == ::phy_engine::verilog::digital::logic_t::true_state)
                            {
                                if(n.body_stmt != SIZE_MAX)
                                {
                                    if(!synth_stmt_ff(b, arena, n.body_stmt, cur, next, targets)) { return false; }
                                }
                                continue;
                            }
                        }

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
                case stmt_node::kind::blocking_assign_vec:
                case stmt_node::kind::nonblocking_assign_vec:
                {
                    if(n.lhs_signals.size() != n.rhs_roots.size()) { return true; }
                    if(n.lhs_signals.empty()) { return true; }

                    // RHS must be sampled before any LHS update (vector atomicity).
                    ::std::vector<::phy_engine::model::node_t*> rhs_nodes{};
                    rhs_nodes.resize(n.rhs_roots.size());
                    for(::std::size_t i{}; i < n.rhs_roots.size(); ++i)
                    {
                        auto* rhs = b.expr_in_env(n.rhs_roots.index_unchecked(i), value, nullptr);
                        if(n.delay_ticks != 0) { rhs = b.ctx.tick_delay(rhs, n.delay_ticks); }
                        rhs_nodes[i] = rhs;
                    }

                    for(::std::size_t i{}; i < n.lhs_signals.size(); ++i)
                    {
                        auto const sig{n.lhs_signals.index_unchecked(i)};
                        if(sig >= value.size() || sig >= assigned_cond.size()) { continue; }
                        if(sig < targets.size() && targets[sig])
                        {
                            value[sig] = rhs_nodes[i];
                            assigned_cond[sig] = b.ctx.const_node(::phy_engine::verilog::digital::logic_t::true_state);
                        }
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

                        ::phy_engine::verilog::digital::logic_t cv{};
                        if(b.ctx.try_get_const(cond, cv))
                        {
                            if(cv == ::phy_engine::verilog::digital::logic_t::false_state) { break; }
                            if(cv == ::phy_engine::verilog::digital::logic_t::true_state)
                            {
                                if(n.body_stmt != SIZE_MAX)
                                {
                                    if(!synth_stmt_comb(b, arena, n.body_stmt, value, assigned_cond, targets)) { return false; }
                                }
                                if(n.step_stmt != SIZE_MAX)
                                {
                                    if(!synth_stmt_comb(b, arena, n.step_stmt, value, assigned_cond, targets)) { return false; }
                                }
                                continue;
                            }
                        }

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

                        ::phy_engine::verilog::digital::logic_t cv{};
                        if(b.ctx.try_get_const(cond, cv))
                        {
                            if(cv == ::phy_engine::verilog::digital::logic_t::false_state) { break; }
                            if(cv == ::phy_engine::verilog::digital::logic_t::true_state)
                            {
                                if(n.body_stmt != SIZE_MAX)
                                {
                                    if(!synth_stmt_comb(b, arena, n.body_stmt, value, assigned_cond, targets)) { return false; }
                                }
                                continue;
                            }
                        }

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

            // Child output/inout ports drive parent nets via `instance_state::output_drives`.
            // Note: `instance_state::bindings` only covers input (and inout-as-input) ports.
            if(parent != nullptr)
            {
                for(auto const& d : inst.output_drives)
                {
                    if(!ok()) { return false; }
                    if(d.parent_signal == SIZE_MAX) { continue; }

                    auto* dst = parent->signal(d.parent_signal);
                    if(dst == nullptr)
                    {
                        set_error("pe_synth: output_drives parent_signal out of range");
                        return false;
                    }

                    ::phy_engine::model::node_t* src{};
                    if(d.src_is_literal)
                    {
                        src = const_node(d.literal);
                    }
                    else
                    {
                        if(d.child_signal == SIZE_MAX) { continue; }
                        src = b.signal(d.child_signal);
                    }
                    if(src == nullptr) { continue; }

                    auto [buf, pos]{::phy_engine::netlist::add_model(nl, ::phy_engine::model::YES{})};
                    (void)pos;
                    if(buf == nullptr)
                    {
                        set_error("pe_synth: failed to create YES for output drive");
                        return false;
                    }
                    if(!connect_pin(buf, 0, src)) { return false; }
                    if(!connect_driver(buf, 1, dst)) { return false; }
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

            // System RNG bus: `$urandom` / `$random` are parsed as `$urandom[3:0]` and lowered to a PE RANDOM_GENERATOR4.
            if(auto itv = m.vectors.find(::fast_io::u8string{u8"$urandom"}); itv != m.vectors.end())
            {
                auto const& vd = itv->second;
                if(vd.bits.size() != 4)
                {
                    set_error("pe_synth: $urandom/$random internal bus width mismatch (expected [3:0])");
                    return false;
                }

                auto find_sig_node = [&](::fast_io::u8string_view nm) noexcept -> ::phy_engine::model::node_t*
                {
                    auto it_sig = m.signal_index.find(::fast_io::u8string{nm});
                    if(it_sig == m.signal_index.end()) { return nullptr; }
                    return b.signal(it_sig->second);
                };

                auto* clk = find_sig_node(u8"clk");
                if(clk == nullptr)
                {
                    set_error("pe_synth: $urandom/$random requires a 1-bit signal named 'clk'");
                    return false;
                }

                auto* rstn = find_sig_node(u8"rst_n");
                if(rstn == nullptr) { rstn = find_sig_node(u8"reset_n"); }
                if(rstn == nullptr) { rstn = const_node(::phy_engine::verilog::digital::logic_t::true_state); }

                auto [rng, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::RANDOM_GENERATOR4{});
                (void)pos;
                if(rng == nullptr)
                {
                    set_error("pe_synth: failed to create RANDOM_GENERATOR4 for $urandom/$random");
                    return false;
                }

                // q3..q0 (pins 0..3) -> $urandom[3:0]
                for(::std::size_t i{}; i < 4; ++i)
                {
                    auto const sig = vd.bits.index_unchecked(i);
                    auto* out = b.signal(sig);
                    if(out == nullptr) { continue; }
                    if(!connect_driver(rng, i, out)) { return false; }
                }

                if(!connect_pin(rng, 4, clk)) { return false; }
                if(!connect_pin(rng, 5, rstn)) { return false; }
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
        if(opt.optimize_wires) { details::optimize_eliminate_yes_buffers(nl, top_port_nodes); }
        if(opt.optimize_mul2) { details::optimize_mul2_in_pe_netlist(nl); }
        if(opt.optimize_adders) { details::optimize_adders_in_pe_netlist(nl); }
        return ctx.ok();
    }
}  // namespace phy_engine::verilog::digital
