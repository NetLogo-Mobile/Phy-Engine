#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <limits>
#include <optional>
#include <tuple>
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
#include "../../model/models/digital/logical/implication.h"
#include "../../model/models/digital/logical/input.h"
#include "../../model/models/digital/logical/is_unknown.h"
#include "../../model/models/digital/logical/nand.h"
#include "../../model/models/digital/logical/non_implication.h"
#include "../../model/models/digital/logical/not.h"
#include "../../model/models/digital/logical/nor.h"
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
        ::std::uint8_t opt_level{0};       // 0=O0, 1=O1, 2=O2, 3=O3 (gate-count driven post-synth optimizations)
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

        struct gate_opt_fanout
        {
            ::std::unordered_map<::phy_engine::model::pin const*, bool> pin_out{};
            ::std::unordered_map<::phy_engine::model::node_t*, ::std::size_t> consumer_count{};
            ::std::unordered_map<::phy_engine::model::node_t*, ::std::size_t> driver_count{};
        };

        [[nodiscard]] inline gate_opt_fanout build_gate_opt_fanout(::phy_engine::netlist::netlist& nl) noexcept
        {
            gate_opt_fanout info{};
            info.pin_out.reserve(1 << 16);
            info.consumer_count.reserve(1 << 14);
            info.driver_count.reserve(1 << 14);

            for(auto& blk : nl.models)
            {
                for(auto* m = blk.begin; m != blk.curr; ++m)
                {
                    if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }
                    auto const name = model_name_u8(*m);
                    auto pv = m->ptr->generate_pin_view();
                    for(std::size_t i{}; i < pv.size; ++i)
                    {
                        auto const* p = __builtin_addressof(pv.pins[i]);
                        bool const is_out = is_output_pin(name, i, pv.size);
                        info.pin_out.emplace(p, is_out);
                        auto* n = pv.pins[i].nodes;
                        if(n == nullptr) { continue; }
                        if(is_out) { ++info.driver_count[n]; }
                        else { ++info.consumer_count[n]; }
                    }
                }
            }

            return info;
        }

        [[nodiscard]] inline bool has_unique_driver_pin(::phy_engine::model::node_t* node,
                                                        ::phy_engine::model::pin const* expected_driver,
                                                        ::std::unordered_map<::phy_engine::model::pin const*, bool> const& pin_out) noexcept
        {
            if(node == nullptr || expected_driver == nullptr) { return false; }
            std::size_t drivers{};
            for(auto const* p : node->pins)
            {
                auto it = pin_out.find(p);
                bool const is_out = (it != pin_out.end()) ? it->second : false;
                if(!is_out) { continue; }
                ++drivers;
                if(p != expected_driver) { return false; }
            }
            return drivers == 1;
        }

        [[nodiscard]] inline bool optimize_fuse_inverters_in_pe_netlist(
            ::phy_engine::netlist::netlist& nl,
            ::std::vector<::phy_engine::model::node_t*> const& protected_nodes) noexcept
        {
            enum class bin_kind : std::uint8_t
            {
                and_gate,
                or_gate,
                xor_gate,
                xnor_gate,
                nand_gate,
                nor_gate,
                imp_gate,
                nimp_gate,
            };

            struct bin_gate
            {
                bin_kind k{};
                ::phy_engine::netlist::model_pos pos{};
                ::phy_engine::model::node_t* in0{};
                ::phy_engine::model::node_t* in1{};
                ::phy_engine::model::node_t* out{};
                ::phy_engine::model::pin const* out_pin{};
            };

            struct not_gate
            {
                ::phy_engine::netlist::model_pos pos{};
                ::phy_engine::model::node_t* in{};
                ::phy_engine::model::node_t* out{};
                ::phy_engine::model::pin const* out_pin{};
            };

            ::std::unordered_map<::phy_engine::model::node_t*, bool> protected_map{};
            protected_map.reserve(protected_nodes.size() * 2u + 1u);
            for(auto* n : protected_nodes) { protected_map.emplace(n, true); }
            auto const is_protected = [&](::phy_engine::model::node_t* n) noexcept -> bool { return protected_map.contains(n); };

            auto const fan = build_gate_opt_fanout(nl);

            ::std::unordered_map<::phy_engine::model::node_t*, bin_gate> gate_by_out{};
            gate_by_out.reserve(1 << 14);
            ::std::vector<not_gate> nots{};
            nots.reserve(1 << 14);

            auto classify_bin = [&](::phy_engine::model::model_base const& mb,
                                    ::phy_engine::netlist::model_pos pos) noexcept -> ::std::optional<bin_gate>
            {
                if(mb.type != ::phy_engine::model::model_type::normal || mb.ptr == nullptr) { return ::std::nullopt; }
                auto const name = model_name_u8(mb);
                auto pv = mb.ptr->generate_pin_view();
                if(pv.size != 3) { return ::std::nullopt; }

                bin_gate g{};
                g.pos = pos;
                g.in0 = pv.pins[0].nodes;
                g.in1 = pv.pins[1].nodes;
                g.out = pv.pins[2].nodes;
                g.out_pin = __builtin_addressof(pv.pins[2]);

                if(name == u8"AND") { g.k = bin_kind::and_gate; }
                else if(name == u8"OR") { g.k = bin_kind::or_gate; }
                else if(name == u8"XOR") { g.k = bin_kind::xor_gate; }
                else if(name == u8"XNOR") { g.k = bin_kind::xnor_gate; }
                else if(name == u8"NAND") { g.k = bin_kind::nand_gate; }
                else if(name == u8"NOR") { g.k = bin_kind::nor_gate; }
                else if(name == u8"IMP") { g.k = bin_kind::imp_gate; }
                else if(name == u8"NIMP") { g.k = bin_kind::nimp_gate; }
                else { return ::std::nullopt; }

                return g;
            };

            auto classify_not = [&](::phy_engine::model::model_base const& mb,
                                    ::phy_engine::netlist::model_pos pos) noexcept -> ::std::optional<not_gate>
            {
                if(mb.type != ::phy_engine::model::model_type::normal || mb.ptr == nullptr) { return ::std::nullopt; }
                if(model_name_u8(mb) != u8"NOT") { return ::std::nullopt; }
                auto pv = mb.ptr->generate_pin_view();
                if(pv.size != 2) { return ::std::nullopt; }
                not_gate ng{};
                ng.pos = pos;
                ng.in = pv.pins[0].nodes;
                ng.out = pv.pins[1].nodes;
                ng.out_pin = __builtin_addressof(pv.pins[1]);
                return ng;
            };

            for(std::size_t chunk_pos{}; chunk_pos < nl.models.size(); ++chunk_pos)
            {
                auto& blk = nl.models.index_unchecked(chunk_pos);
                for(std::size_t vec_pos{}; blk.begin + vec_pos < blk.curr; ++vec_pos)
                {
                    auto const& mb = blk.begin[vec_pos];
                    if(auto og = classify_bin(mb, {vec_pos, chunk_pos}); og && og->out != nullptr)
                    {
                        gate_by_out.emplace(og->out, *og);
                        continue;
                    }
                    if(auto on = classify_not(mb, {vec_pos, chunk_pos}); on) { nots.push_back(*on); }
                }
            }

            auto const complement = [](bin_kind k) noexcept -> ::std::optional<bin_kind> {
                switch(k)
                {
                    case bin_kind::and_gate: return bin_kind::nand_gate;
                    case bin_kind::nand_gate: return bin_kind::and_gate;
                    case bin_kind::or_gate: return bin_kind::nor_gate;
                    case bin_kind::nor_gate: return bin_kind::or_gate;
                    case bin_kind::xor_gate: return bin_kind::xnor_gate;
                    case bin_kind::xnor_gate: return bin_kind::xor_gate;
                    case bin_kind::imp_gate: return bin_kind::nimp_gate;
                    case bin_kind::nimp_gate: return bin_kind::imp_gate;
                    default: return ::std::nullopt;
                }
            };

            auto add_bin_gate = [&](bin_kind k,
                                    ::phy_engine::model::node_t* a,
                                    ::phy_engine::model::node_t* b,
                                    ::phy_engine::model::node_t* out) noexcept -> ::std::optional<::phy_engine::netlist::model_pos>
            {
                if(a == nullptr || b == nullptr || out == nullptr) { return ::std::nullopt; }
                ::phy_engine::netlist::add_model_retstr r{};
                switch(k)
                {
                    case bin_kind::and_gate: r = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::AND{}); break;
                    case bin_kind::or_gate: r = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OR{}); break;
                    case bin_kind::xor_gate: r = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::XOR{}); break;
                    case bin_kind::xnor_gate: r = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::XNOR{}); break;
                    case bin_kind::nand_gate: r = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::NAND{}); break;
                    case bin_kind::nor_gate: r = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::NOR{}); break;
                    case bin_kind::imp_gate: r = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::IMP{}); break;
                    case bin_kind::nimp_gate: r = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::NIMP{}); break;
                    default: return ::std::nullopt;
                }
                if(r.mod == nullptr) { return ::std::nullopt; }

                // Connect inputs first; output is connected after deleting the old driver to avoid multi-driver nets.
                if(!::phy_engine::netlist::add_to_node(nl, *r.mod, 0, *a) ||
                   !::phy_engine::netlist::add_to_node(nl, *r.mod, 1, *b))
                {
                    (void)::phy_engine::netlist::delete_model(nl, r.mod_pos);
                    return ::std::nullopt;
                }

                return r.mod_pos;
            };

            bool changed{};
            for(auto const& n : nots)
            {
                if(n.in == nullptr || n.out == nullptr || n.out_pin == nullptr) { continue; }
                if(is_protected(n.in)) { continue; }  // don't orphan a top-level port node

                auto it = gate_by_out.find(n.in);
                if(it == gate_by_out.end()) { continue; }
                auto const& g = it->second;
                if(g.in0 == nullptr || g.in1 == nullptr || g.out == nullptr || g.out_pin == nullptr) { continue; }
                if(g.out != n.in) { continue; }

                auto const nk = complement(g.k);
                if(!nk) { continue; }

                auto const dc_it = fan.driver_count.find(g.out);
                auto const cc_it = fan.consumer_count.find(g.out);
                if(dc_it == fan.driver_count.end() || dc_it->second != 1) { continue; }
                if(cc_it == fan.consumer_count.end() || cc_it->second != 1) { continue; }

                if(!has_unique_driver_pin(g.out, g.out_pin, fan.pin_out)) { continue; }
                if(!has_unique_driver_pin(n.out, n.out_pin, fan.pin_out)) { continue; }

                // Create complement gate, then remove old NOT+gate, then connect output.
                auto const new_pos = add_bin_gate(*nk, g.in0, g.in1, n.out);
                if(!new_pos) { continue; }

                (void)::phy_engine::netlist::delete_model(nl, n.pos);
                (void)::phy_engine::netlist::delete_model(nl, g.pos);

                auto* nm = ::phy_engine::netlist::get_model(nl, *new_pos);
                if(nm == nullptr || nm->type != ::phy_engine::model::model_type::normal || nm->ptr == nullptr)
                {
                    continue;
                }
                if(!::phy_engine::netlist::add_to_node(nl, *nm, 2, *n.out))
                {
                    (void)::phy_engine::netlist::delete_model(nl, *new_pos);
                    continue;
                }

                changed = true;
            }

            return changed;
        }

        [[nodiscard]] inline bool optimize_push_input_inverters_in_pe_netlist(
            ::phy_engine::netlist::netlist& nl,
            ::std::vector<::phy_engine::model::node_t*> const& protected_nodes) noexcept
        {
            enum class kind : std::uint8_t
            {
                and_gate,
                or_gate,
                xor_gate,
                xnor_gate,
            };
            struct bin_gate
            {
                kind k{};
                ::phy_engine::netlist::model_pos pos{};
                ::phy_engine::model::node_t* in0{};
                ::phy_engine::model::node_t* in1{};
                ::phy_engine::model::node_t* out{};
                ::phy_engine::model::pin const* out_pin{};
            };
            struct not_gate
            {
                ::phy_engine::netlist::model_pos pos{};
                ::phy_engine::model::node_t* in{};
                ::phy_engine::model::node_t* out{};
                ::phy_engine::model::pin const* out_pin{};
            };

            ::std::unordered_map<::phy_engine::model::node_t*, bool> protected_map{};
            protected_map.reserve(protected_nodes.size() * 2u + 1u);
            for(auto* n : protected_nodes) { protected_map.emplace(n, true); }
            auto const is_protected = [&](::phy_engine::model::node_t* n) noexcept -> bool { return protected_map.contains(n); };

            auto const fan = build_gate_opt_fanout(nl);

            ::std::unordered_map<::phy_engine::model::node_t*, not_gate> not_by_out{};
            not_by_out.reserve(1 << 14);
            ::std::vector<bin_gate> bins{};
            bins.reserve(1 << 14);

            auto classify_not = [&](::phy_engine::model::model_base const& mb,
                                    ::phy_engine::netlist::model_pos pos) noexcept -> ::std::optional<not_gate>
            {
                if(mb.type != ::phy_engine::model::model_type::normal || mb.ptr == nullptr) { return ::std::nullopt; }
                if(model_name_u8(mb) != u8"NOT") { return ::std::nullopt; }
                auto pv = mb.ptr->generate_pin_view();
                if(pv.size != 2) { return ::std::nullopt; }
                not_gate ng{};
                ng.pos = pos;
                ng.in = pv.pins[0].nodes;
                ng.out = pv.pins[1].nodes;
                ng.out_pin = __builtin_addressof(pv.pins[1]);
                return ng;
            };

            auto classify_bin = [&](::phy_engine::model::model_base const& mb,
                                    ::phy_engine::netlist::model_pos pos) noexcept -> ::std::optional<bin_gate>
            {
                if(mb.type != ::phy_engine::model::model_type::normal || mb.ptr == nullptr) { return ::std::nullopt; }
                auto const name = model_name_u8(mb);
                if(name != u8"AND" && name != u8"OR" && name != u8"XOR" && name != u8"XNOR") { return ::std::nullopt; }
                auto pv = mb.ptr->generate_pin_view();
                if(pv.size != 3) { return ::std::nullopt; }
                bin_gate g{};
                g.pos = pos;
                g.in0 = pv.pins[0].nodes;
                g.in1 = pv.pins[1].nodes;
                g.out = pv.pins[2].nodes;
                g.out_pin = __builtin_addressof(pv.pins[2]);
                if(name == u8"AND") { g.k = kind::and_gate; }
                else if(name == u8"OR") { g.k = kind::or_gate; }
                else if(name == u8"XOR") { g.k = kind::xor_gate; }
                else { g.k = kind::xnor_gate; }
                return g;
            };

            for(std::size_t chunk_pos{}; chunk_pos < nl.models.size(); ++chunk_pos)
            {
                auto& blk = nl.models.index_unchecked(chunk_pos);
                for(std::size_t vec_pos{}; blk.begin + vec_pos < blk.curr; ++vec_pos)
                {
                    auto const& mb = blk.begin[vec_pos];
                    if(auto on = classify_not(mb, {vec_pos, chunk_pos}); on && on->out != nullptr)
                    {
                        not_by_out.emplace(on->out, *on);
                        continue;
                    }
                    if(auto og = classify_bin(mb, {vec_pos, chunk_pos}); og && og->out != nullptr) { bins.push_back(*og); }
                }
            }

            auto add_model_inputs_only = [&](::phy_engine::model::model_base* m,
                                             ::phy_engine::model::node_t* a,
                                             ::phy_engine::model::node_t* b) noexcept -> bool
            {
                if(m == nullptr || a == nullptr || b == nullptr) { return false; }
                return ::phy_engine::netlist::add_to_node(nl, *m, 0, *a) && ::phy_engine::netlist::add_to_node(nl, *m, 1, *b);
            };

            auto add_bin = [&](::fast_io::u8string_view name,
                               ::phy_engine::model::node_t* a,
                               ::phy_engine::model::node_t* b) noexcept -> ::std::optional<::phy_engine::netlist::add_model_retstr>
            {
                if(a == nullptr || b == nullptr) { return ::std::nullopt; }
                ::phy_engine::netlist::add_model_retstr r{};
                if(name == u8"AND") { r = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::AND{}); }
                else if(name == u8"OR") { r = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OR{}); }
                else if(name == u8"XOR") { r = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::XOR{}); }
                else if(name == u8"XNOR") { r = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::XNOR{}); }
                else if(name == u8"NAND") { r = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::NAND{}); }
                else if(name == u8"NOR") { r = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::NOR{}); }
                else if(name == u8"IMP") { r = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::IMP{}); }
                else if(name == u8"NIMP") { r = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::NIMP{}); }
                else { return ::std::nullopt; }
                if(r.mod == nullptr) { return ::std::nullopt; }
                if(!add_model_inputs_only(r.mod, a, b))
                {
                    (void)::phy_engine::netlist::delete_model(nl, r.mod_pos);
                    return ::std::nullopt;
                }
                return r;
            };

            bool changed{};

            for(auto const& g : bins)
            {
                if(g.in0 == nullptr || g.in1 == nullptr || g.out == nullptr || g.out_pin == nullptr) { continue; }
                if(!has_unique_driver_pin(g.out, g.out_pin, fan.pin_out)) { continue; }

                // Ensure the gate still exists and matches our snapshot.
                {
                    auto* mb = ::phy_engine::netlist::get_model(nl, g.pos);
                    if(mb == nullptr || mb->type != ::phy_engine::model::model_type::normal || mb->ptr == nullptr) { continue; }
                    auto const nm = model_name_u8(*mb);
                    if(nm != u8"AND" && nm != u8"OR" && nm != u8"XOR" && nm != u8"XNOR") { continue; }
                }

                auto it0 = not_by_out.find(g.in0);
                auto it1 = not_by_out.find(g.in1);
                bool const inv0 = (it0 != not_by_out.end());
                bool const inv1 = (it1 != not_by_out.end());
                if(!inv0 && !inv1) { continue; }

                auto const ok_not = [&](not_gate const& n) noexcept -> bool
                {
                    if(n.in == nullptr || n.out == nullptr || n.out_pin == nullptr) { return false; }
                    if(is_protected(n.out)) { return false; }
                    auto const dc_it = fan.driver_count.find(n.out);
                    if(dc_it == fan.driver_count.end() || dc_it->second != 1) { return false; }
                    return has_unique_driver_pin(n.out, n.out_pin, fan.pin_out);
                };

                ::phy_engine::model::node_t* a = g.in0;
                ::phy_engine::model::node_t* b = g.in1;
                not_gate n0{};
                not_gate n1{};
                if(inv0) { n0 = it0->second; if(!ok_not(n0)) { continue; } a = n0.in; }
                if(inv1) { n1 = it1->second; if(!ok_not(n1)) { continue; } b = n1.in; }

                ::fast_io::u8string_view new_name{};
                ::phy_engine::model::node_t* na{};
                ::phy_engine::model::node_t* nb{};

                // Select a cheaper equivalent primitive.
                if(g.k == kind::and_gate)
                {
                    if(inv0 && inv1) { new_name = u8"NOR"; na = a; nb = b; }  // ~a & ~b = ~(a|b)
                    else if(inv0) { new_name = u8"NIMP"; na = g.in1; nb = a; } // b & ~a = NIMP(b,a)
                    else { new_name = u8"NIMP"; na = g.in0; nb = b; }         // a & ~b = NIMP(a,b)
                }
                else if(g.k == kind::or_gate)
                {
                    if(inv0 && inv1) { new_name = u8"NAND"; na = a; nb = b; }  // ~a | ~b = ~(a&b)
                    else if(inv0) { new_name = u8"IMP"; na = a; nb = g.in1; }  // ~a | b = IMP(a,b)
                    else { new_name = u8"IMP"; na = b; nb = g.in0; }           // a | ~b = IMP(b,a)
                }
                else if(g.k == kind::xor_gate)
                {
                    if(inv0 && inv1) { new_name = u8"XOR"; na = a; nb = b; }
                    else { new_name = u8"XNOR"; na = inv0 ? a : g.in0; nb = inv1 ? b : g.in1; }
                }
                else  // XNOR
                {
                    if(inv0 && inv1) { new_name = u8"XNOR"; na = a; nb = b; }
                    else { new_name = u8"XOR"; na = inv0 ? a : g.in0; nb = inv1 ? b : g.in1; }
                }

                if(new_name.empty()) { continue; }
                auto const r = add_bin(new_name, na, nb);
                if(!r) { continue; }

                // Remove old drivers before connecting the new output.
                (void)::phy_engine::netlist::delete_model(nl, g.pos);

                auto* nm = ::phy_engine::netlist::get_model(nl, r->mod_pos);
                if(nm == nullptr || nm->type != ::phy_engine::model::model_type::normal || nm->ptr == nullptr)
                {
                    continue;
                }
                if(!::phy_engine::netlist::add_to_node(nl, *nm, 2, *g.out))
                {
                    (void)::phy_engine::netlist::delete_model(nl, r->mod_pos);
                    continue;
                }

                // Best-effort: delete input NOTs if now unused.
                auto try_delete_not_if_unused = [&](not_gate const& n) noexcept
                {
                    if(n.out == nullptr || n.out_pin == nullptr) { return; }
                    if(is_protected(n.out)) { return; }
                    if(!has_unique_driver_pin(n.out, n.out_pin, fan.pin_out)) { return; }
                    bool has_consumer{};
                    for(auto const* p : n.out->pins)
                    {
                        if(p == n.out_pin) { continue; }
                        auto it = fan.pin_out.find(p);
                        bool const is_out = (it != fan.pin_out.end()) ? it->second : false;
                        if(!is_out) { has_consumer = true; break; }
                    }
                    if(!has_consumer) { (void)::phy_engine::netlist::delete_model(nl, n.pos); }
                };
                if(inv0) { try_delete_not_if_unused(n0); }
                if(inv1)
                {
                    bool const same = inv0 && n0.pos.chunk_pos == n1.pos.chunk_pos && n0.pos.vec_pos == n1.pos.vec_pos;
                    if(!same) { try_delete_not_if_unused(n1); }
                }

                changed = true;
            }

            return changed;
        }

        [[nodiscard]] inline bool optimize_factor_common_terms_in_pe_netlist(
            ::phy_engine::netlist::netlist& nl,
            ::std::vector<::phy_engine::model::node_t*> const& protected_nodes) noexcept
        {
            enum class kind : std::uint8_t
            {
                and_gate,
                or_gate,
            };

            struct bin_gate
            {
                kind k{};
                ::phy_engine::netlist::model_pos pos{};
                ::phy_engine::model::node_t* in0{};
                ::phy_engine::model::node_t* in1{};
                ::phy_engine::model::node_t* out{};
                ::phy_engine::model::pin const* out_pin{};
            };

            ::std::unordered_map<::phy_engine::model::node_t*, bool> protected_map{};
            protected_map.reserve(protected_nodes.size() * 2u + 1u);
            for(auto* n : protected_nodes) { protected_map.emplace(n, true); }
            auto const is_protected = [&](::phy_engine::model::node_t* n) noexcept -> bool { return protected_map.contains(n); };

            auto const fan = build_gate_opt_fanout(nl);

            ::std::unordered_map<::phy_engine::model::node_t*, bin_gate> gate_by_out{};
            gate_by_out.reserve(1 << 14);

            ::std::vector<bin_gate> ors{};
            ors.reserve(1 << 14);
            ::std::vector<bin_gate> ands{};
            ands.reserve(1 << 14);

            auto classify = [&](::phy_engine::model::model_base const& mb,
                                ::phy_engine::netlist::model_pos pos) noexcept -> ::std::optional<bin_gate>
            {
                if(mb.type != ::phy_engine::model::model_type::normal || mb.ptr == nullptr) { return ::std::nullopt; }
                auto const name = model_name_u8(mb);
                auto pv = mb.ptr->generate_pin_view();
                if(pv.size != 3) { return ::std::nullopt; }

                bin_gate g{};
                g.pos = pos;
                g.in0 = pv.pins[0].nodes;
                g.in1 = pv.pins[1].nodes;
                g.out = pv.pins[2].nodes;
                g.out_pin = __builtin_addressof(pv.pins[2]);

                if(name == u8"AND") { g.k = kind::and_gate; }
                else if(name == u8"OR") { g.k = kind::or_gate; }
                else { return ::std::nullopt; }

                return g;
            };

            for(std::size_t chunk_pos{}; chunk_pos < nl.models.size(); ++chunk_pos)
            {
                auto& blk = nl.models.index_unchecked(chunk_pos);
                for(std::size_t vec_pos{}; blk.begin + vec_pos < blk.curr; ++vec_pos)
                {
                    auto const& mb = blk.begin[vec_pos];
                    auto og = classify(mb, {vec_pos, chunk_pos});
                    if(!og || og->out == nullptr) { continue; }
                    gate_by_out.emplace(og->out, *og);
                    if(og->k == kind::or_gate) { ors.push_back(*og); }
                    else { ands.push_back(*og); }
                }
            }

            auto find_common = [](::phy_engine::model::node_t* a0,
                                  ::phy_engine::model::node_t* a1,
                                  ::phy_engine::model::node_t* b0,
                                  ::phy_engine::model::node_t* b1) noexcept
                -> ::std::optional<::std::tuple<::phy_engine::model::node_t*, ::phy_engine::model::node_t*, ::phy_engine::model::node_t*>>
            {
                if(a0 == nullptr || a1 == nullptr || b0 == nullptr || b1 == nullptr) { return ::std::nullopt; }
                if(a0 == b0) { return ::std::tuple{a0, a1, b1}; }
                if(a0 == b1) { return ::std::tuple{a0, a1, b0}; }
                if(a1 == b0) { return ::std::tuple{a1, a0, b1}; }
                if(a1 == b1) { return ::std::tuple{a1, a0, b0}; }
                return ::std::nullopt;
            };

            bool changed{};
            for(auto const& gor : ors)
            {
                if(gor.in0 == nullptr || gor.in1 == nullptr || gor.out == nullptr || gor.out_pin == nullptr) { continue; }
                if(is_protected(gor.out)) { continue; }
                if(!has_unique_driver_pin(gor.out, gor.out_pin, fan.pin_out)) { continue; }

                // Ensure the OR gate still exists and matches our snapshot.
                {
                    auto* mb = ::phy_engine::netlist::get_model(nl, gor.pos);
                    if(mb == nullptr || mb->type != ::phy_engine::model::model_type::normal || mb->ptr == nullptr) { continue; }
                    if(model_name_u8(*mb) != u8"OR") { continue; }
                }

                // Multi-term factoring on OR trees:
                // Try to turn OR-tree-of-(AND terms) with a shared literal into AND(common, OR(others...)).
                auto const try_factor_or_tree = [&]() noexcept -> bool
                {
                    constexpr std::size_t max_terms = 16;
                    constexpr std::size_t max_nodes = 64;

                    ::std::unordered_map<::phy_engine::model::node_t*, bool> visited{};
                    visited.reserve(max_nodes * 2u);
                    ::std::vector<bin_gate> or_nodes{};
                    or_nodes.reserve(max_nodes);
                    ::std::vector<bin_gate> and_terms{};
                    and_terms.reserve(max_terms);

                    auto exclusive_out = [&](bin_gate const& g) noexcept -> bool
                    {
                        if(g.out == nullptr || g.out_pin == nullptr) { return false; }
                        if(is_protected(g.out)) { return false; }
                        auto itd = fan.driver_count.find(g.out);
                        auto itc = fan.consumer_count.find(g.out);
                        if(itd == fan.driver_count.end() || itc == fan.consumer_count.end()) { return false; }
                        if(itd->second != 1 || itc->second != 1) { return false; }
                        return has_unique_driver_pin(g.out, g.out_pin, fan.pin_out);
                    };

                    bool ok{true};
                    auto dfs = [&](auto&& self, ::phy_engine::model::node_t* n, bool is_root) noexcept -> void
                    {
                        if(!ok || n == nullptr) { ok = false; return; }
                        if(visited.contains(n)) { return; }
                        visited.emplace(n, true);
                        if(or_nodes.size() + and_terms.size() >= max_nodes) { ok = false; return; }

                        auto it = gate_by_out.find(n);
                        if(it != gate_by_out.end() && it->second.k == kind::or_gate)
                        {
                            auto const& og = it->second;
                            if(og.out == nullptr || og.out_pin == nullptr) { ok = false; return; }
                            if(!has_unique_driver_pin(og.out, og.out_pin, fan.pin_out)) { ok = false; return; }
                            if(!is_root)
                            {
                                if(!exclusive_out(og)) { ok = false; return; }
                            }
                            or_nodes.push_back(og);
                            self(self, og.in0, false);
                            self(self, og.in1, false);
                            return;
                        }

                        if(it != gate_by_out.end() && it->second.k == kind::and_gate)
                        {
                            auto const& ag = it->second;
                            if(and_terms.size() >= max_terms) { ok = false; return; }
                            if(!exclusive_out(ag)) { ok = false; return; }
                            and_terms.push_back(ag);
                            return;
                        }

                        ok = false;
                    };

                    dfs(dfs, gor.out, true);
                    if(!ok) { return false; }
                    if(and_terms.size() < 3) { return false; }

                    // Find best common literal among all AND terms (either in0 or in1).
                    ::std::array<::phy_engine::model::node_t*, 2> candidates{and_terms[0].in0, and_terms[0].in1};
                    for(std::size_t i{1}; i < and_terms.size(); ++i)
                    {
                        auto const* t = __builtin_addressof(and_terms[i]);
                        for(auto& c : candidates)
                        {
                            if(c == nullptr) { continue; }
                            if(t->in0 != c && t->in1 != c) { c = nullptr; }
                        }
                    }

                    struct pick
                    {
                        ::phy_engine::model::node_t* common{};
                        ::std::vector<::phy_engine::model::node_t*> others{};
                        std::size_t new_cost{};
                    };
                    ::std::optional<pick> best{};

                    for(auto* c : candidates)
                    {
                        if(c == nullptr) { continue; }
                        ::std::unordered_map<::phy_engine::model::node_t*, bool> seen{};
                        seen.reserve(and_terms.size() * 2u);
                        ::std::vector<::phy_engine::model::node_t*> others{};
                        others.reserve(and_terms.size());
                        for(auto const& t : and_terms)
                        {
                            if(t.in0 != c && t.in1 != c) { others.clear(); break; }
                            auto* o = (t.in0 == c) ? t.in1 : t.in0;
                            if(o == nullptr) { others.clear(); break; }
                            if(!seen.contains(o))
                            {
                                seen.emplace(o, true);
                                others.push_back(o);
                            }
                        }
                        if(others.empty()) { continue; }

                        // new_cost: OR-chain on `others` (N-1) + one AND. Special-case if only one other.
                        std::size_t cost = 1;
                        if(others.size() >= 2) { cost += (others.size() - 1); }
                        // If all terms are effectively (c & c), others would be {c} and cost becomes 1; still ok.

                        if(!best || cost < best->new_cost)
                        {
                            best = pick{.common = c, .others = ::std::move(others), .new_cost = cost};
                        }
                    }

                    if(!best) { return false; }

                    auto const old_cost = and_terms.size() + or_nodes.size();
                    if(best->new_cost >= old_cost) { return false; }

                    // Build new network.
                    ::phy_engine::model::node_t* or_out_node{};
                    ::std::optional<::phy_engine::netlist::model_pos> mand_pos{};

                    if(best->others.size() == 1 && best->others[0] == best->common)
                    {
                        // OR-tree is just `common`. We'll rewire consumers of gor.out to `common`.
                        // (No new models.)
                    }
                    else
                    {
                        if(best->others.size() == 1)
                        {
                            or_out_node = best->others[0];
                        }
                        else
                        {
                            // Create OR chain to combine `others` into one node.
                            auto* acc = best->others[0];
                            for(std::size_t i{1}; i < best->others.size(); ++i)
                            {
                                auto& nref = ::phy_engine::netlist::create_node(nl);
                                auto* nout = __builtin_addressof(nref);
                                auto [mor, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OR{});
                                if(mor == nullptr) { return false; }
                                if(!::phy_engine::netlist::add_to_node(nl, *mor, 0, *acc) ||
                                   !::phy_engine::netlist::add_to_node(nl, *mor, 1, *best->others[i]) ||
                                   !::phy_engine::netlist::add_to_node(nl, *mor, 2, *nout))
                                {
                                    (void)::phy_engine::netlist::delete_model(nl, pos);
                                    return false;
                                }
                                acc = nout;
                            }
                            or_out_node = acc;
                        }

                        auto [mand, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::AND{});
                        if(mand == nullptr) { return false; }
                        if(!::phy_engine::netlist::add_to_node(nl, *mand, 0, *best->common) ||
                           !::phy_engine::netlist::add_to_node(nl, *mand, 1, *or_out_node))
                        {
                            (void)::phy_engine::netlist::delete_model(nl, pos);
                            return false;
                        }
                        mand_pos = pos;
                    }

                    // Delete old OR-tree and AND terms.
                    for(auto const& og : or_nodes) { (void)::phy_engine::netlist::delete_model(nl, og.pos); }
                    for(auto const& tg : and_terms) { (void)::phy_engine::netlist::delete_model(nl, tg.pos); }

                    if(mand_pos)
                    {
                        auto* mand_m = ::phy_engine::netlist::get_model(nl, *mand_pos);
                        if(mand_m == nullptr || mand_m->type != ::phy_engine::model::model_type::normal || mand_m->ptr == nullptr) { return false; }
                        if(!::phy_engine::netlist::add_to_node(nl, *mand_m, 2, *gor.out))
                        {
                            (void)::phy_engine::netlist::delete_model(nl, *mand_pos);
                            return false;
                        }
                        return true;
                    }

                    // Forward case: move consumers of gor.out to best->common.
                    ::std::vector<::phy_engine::model::pin*> pins_to_move{};
                    pins_to_move.reserve(gor.out->pins.size());
                    for(auto* p : gor.out->pins)
                    {
                        auto it = fan.pin_out.find(p);
                        bool const is_out = (it != fan.pin_out.end()) ? it->second : false;
                        if(is_out) { continue; }
                        pins_to_move.push_back(p);
                    }
                    for(auto* p : pins_to_move)
                    {
                        gor.out->pins.erase(p);
                        p->nodes = best->common;
                        best->common->pins.insert(p);
                    }
                    return true;
                };

                if(try_factor_or_tree())
                {
                    changed = true;
                    continue;
                }

                auto it_a = gate_by_out.find(gor.in0);
                auto it_b = gate_by_out.find(gor.in1);
                if(it_a == gate_by_out.end() || it_b == gate_by_out.end()) { continue; }
                auto const& ga = it_a->second;
                auto const& gb = it_b->second;
                if(ga.k != kind::and_gate || gb.k != kind::and_gate) { continue; }
                if(ga.out == nullptr || gb.out == nullptr || ga.out_pin == nullptr || gb.out_pin == nullptr) { continue; }
                if(is_protected(ga.out) || is_protected(gb.out)) { continue; }
                if(ga.out != gor.in0 || gb.out != gor.in1) { continue; }

                auto const ga_dc = fan.driver_count.find(ga.out);
                auto const ga_cc = fan.consumer_count.find(ga.out);
                auto const gb_dc = fan.driver_count.find(gb.out);
                auto const gb_cc = fan.consumer_count.find(gb.out);
                if(ga_dc == fan.driver_count.end() || ga_dc->second != 1) { continue; }
                if(gb_dc == fan.driver_count.end() || gb_dc->second != 1) { continue; }
                if(ga_cc == fan.consumer_count.end() || ga_cc->second != 1) { continue; }
                if(gb_cc == fan.consumer_count.end() || gb_cc->second != 1) { continue; }
                if(!has_unique_driver_pin(ga.out, ga.out_pin, fan.pin_out)) { continue; }
                if(!has_unique_driver_pin(gb.out, gb.out_pin, fan.pin_out)) { continue; }

                auto const common = find_common(ga.in0, ga.in1, gb.in0, gb.in1);
                if(!common) { continue; }
                auto const [c, x, y] = *common;
                if(c == nullptr || x == nullptr || y == nullptr) { continue; }

                // new network: t = OR(x,y); out = AND(c,t)
                auto& tnode_ref = ::phy_engine::netlist::create_node(nl);
                auto* tnode = __builtin_addressof(tnode_ref);

                auto [mor, mor_pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OR{});
                if(mor == nullptr) { continue; }
                if(!::phy_engine::netlist::add_to_node(nl, *mor, 0, *x) ||
                   !::phy_engine::netlist::add_to_node(nl, *mor, 1, *y) ||
                   !::phy_engine::netlist::add_to_node(nl, *mor, 2, *tnode))
                {
                    (void)::phy_engine::netlist::delete_model(nl, mor_pos);
                    continue;
                }

                auto [mand, mand_pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::AND{});
                if(mand == nullptr)
                {
                    (void)::phy_engine::netlist::delete_model(nl, mor_pos);
                    continue;
                }
                if(!::phy_engine::netlist::add_to_node(nl, *mand, 0, *c) ||
                   !::phy_engine::netlist::add_to_node(nl, *mand, 1, *tnode))
                {
                    (void)::phy_engine::netlist::delete_model(nl, mand_pos);
                    (void)::phy_engine::netlist::delete_model(nl, mor_pos);
                    continue;
                }

                // Remove old drivers before connecting the new output.
                (void)::phy_engine::netlist::delete_model(nl, gor.pos);
                (void)::phy_engine::netlist::delete_model(nl, ga.pos);
                (void)::phy_engine::netlist::delete_model(nl, gb.pos);

                auto* mand_m = ::phy_engine::netlist::get_model(nl, mand_pos);
                if(mand_m == nullptr || mand_m->type != ::phy_engine::model::model_type::normal || mand_m->ptr == nullptr)
                {
                    continue;
                }
                if(!::phy_engine::netlist::add_to_node(nl, *mand_m, 2, *gor.out))
                {
                    (void)::phy_engine::netlist::delete_model(nl, mand_pos);
                    (void)::phy_engine::netlist::delete_model(nl, mor_pos);
                    continue;
                }

                changed = true;
            }

            for(auto const& gand : ands)
            {
                if(gand.in0 == nullptr || gand.in1 == nullptr || gand.out == nullptr || gand.out_pin == nullptr) { continue; }
                if(is_protected(gand.out)) { continue; }
                if(!has_unique_driver_pin(gand.out, gand.out_pin, fan.pin_out)) { continue; }

                // Ensure the AND gate still exists and matches our snapshot.
                {
                    auto* mb = ::phy_engine::netlist::get_model(nl, gand.pos);
                    if(mb == nullptr || mb->type != ::phy_engine::model::model_type::normal || mb->ptr == nullptr) { continue; }
                    if(model_name_u8(*mb) != u8"AND") { continue; }
                }

                // Dual multi-term factoring on AND trees (AND-of-(OR terms)).
                auto const try_factor_and_tree = [&]() noexcept -> bool
                {
                    constexpr std::size_t max_terms = 16;
                    constexpr std::size_t max_nodes = 64;

                    ::std::unordered_map<::phy_engine::model::node_t*, bool> visited{};
                    visited.reserve(max_nodes * 2u);
                    ::std::vector<bin_gate> and_nodes{};
                    and_nodes.reserve(max_nodes);
                    ::std::vector<bin_gate> or_terms{};
                    or_terms.reserve(max_terms);

                    auto exclusive_out = [&](bin_gate const& g) noexcept -> bool
                    {
                        if(g.out == nullptr || g.out_pin == nullptr) { return false; }
                        if(is_protected(g.out)) { return false; }
                        auto itd = fan.driver_count.find(g.out);
                        auto itc = fan.consumer_count.find(g.out);
                        if(itd == fan.driver_count.end() || itc == fan.consumer_count.end()) { return false; }
                        if(itd->second != 1 || itc->second != 1) { return false; }
                        return has_unique_driver_pin(g.out, g.out_pin, fan.pin_out);
                    };

                    bool ok{true};
                    auto dfs = [&](auto&& self, ::phy_engine::model::node_t* n, bool is_root) noexcept -> void
                    {
                        if(!ok || n == nullptr) { ok = false; return; }
                        if(visited.contains(n)) { return; }
                        visited.emplace(n, true);
                        if(and_nodes.size() + or_terms.size() >= max_nodes) { ok = false; return; }

                        auto it = gate_by_out.find(n);
                        if(it != gate_by_out.end() && it->second.k == kind::and_gate)
                        {
                            auto const& ag = it->second;
                            if(ag.out == nullptr || ag.out_pin == nullptr) { ok = false; return; }
                            if(!has_unique_driver_pin(ag.out, ag.out_pin, fan.pin_out)) { ok = false; return; }
                            if(!is_root)
                            {
                                if(!exclusive_out(ag)) { ok = false; return; }
                            }
                            and_nodes.push_back(ag);
                            self(self, ag.in0, false);
                            self(self, ag.in1, false);
                            return;
                        }

                        if(it != gate_by_out.end() && it->second.k == kind::or_gate)
                        {
                            auto const& og = it->second;
                            if(or_terms.size() >= max_terms) { ok = false; return; }
                            if(!exclusive_out(og)) { ok = false; return; }
                            or_terms.push_back(og);
                            return;
                        }

                        ok = false;
                    };

                    dfs(dfs, gand.out, true);
                    if(!ok) { return false; }
                    if(or_terms.size() < 3) { return false; }

                    ::std::array<::phy_engine::model::node_t*, 2> candidates{or_terms[0].in0, or_terms[0].in1};
                    for(std::size_t i{1}; i < or_terms.size(); ++i)
                    {
                        auto const* t = __builtin_addressof(or_terms[i]);
                        for(auto& c : candidates)
                        {
                            if(c == nullptr) { continue; }
                            if(t->in0 != c && t->in1 != c) { c = nullptr; }
                        }
                    }

                    struct pick
                    {
                        ::phy_engine::model::node_t* common{};
                        ::std::vector<::phy_engine::model::node_t*> others{};
                        std::size_t new_cost{};
                    };
                    ::std::optional<pick> best{};

                    for(auto* c : candidates)
                    {
                        if(c == nullptr) { continue; }
                        ::std::unordered_map<::phy_engine::model::node_t*, bool> seen{};
                        seen.reserve(or_terms.size() * 2u);
                        ::std::vector<::phy_engine::model::node_t*> others{};
                        others.reserve(or_terms.size());
                        for(auto const& t : or_terms)
                        {
                            if(t.in0 != c && t.in1 != c) { others.clear(); break; }
                            auto* o = (t.in0 == c) ? t.in1 : t.in0;
                            if(o == nullptr) { others.clear(); break; }
                            if(!seen.contains(o))
                            {
                                seen.emplace(o, true);
                                others.push_back(o);
                            }
                        }
                        if(others.empty()) { continue; }

                        // new_cost: AND-chain on `others` (N-1) + one OR. Special-case if only one other.
                        std::size_t cost = 1;
                        if(others.size() >= 2) { cost += (others.size() - 1); }

                        if(!best || cost < best->new_cost)
                        {
                            best = pick{.common = c, .others = ::std::move(others), .new_cost = cost};
                        }
                    }

                    if(!best) { return false; }

                    auto const old_cost = or_terms.size() + and_nodes.size();
                    if(best->new_cost >= old_cost) { return false; }

                    ::phy_engine::model::node_t* and_out_node{};
                    ::std::optional<::phy_engine::netlist::model_pos> mor_pos{};

                    if(best->others.size() == 1 && best->others[0] == best->common)
                    {
                        // AND-tree is just `common`. We'll rewire consumers of gand.out to `common`.
                        // (No new models.)
                    }
                    else
                    {
                        if(best->others.size() == 1)
                        {
                            and_out_node = best->others[0];
                        }
                        else
                        {
                            auto* acc = best->others[0];
                            for(std::size_t i{1}; i < best->others.size(); ++i)
                            {
                                auto& nref = ::phy_engine::netlist::create_node(nl);
                                auto* nout = __builtin_addressof(nref);
                                auto [mand, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::AND{});
                                if(mand == nullptr) { return false; }
                                if(!::phy_engine::netlist::add_to_node(nl, *mand, 0, *acc) ||
                                   !::phy_engine::netlist::add_to_node(nl, *mand, 1, *best->others[i]) ||
                                   !::phy_engine::netlist::add_to_node(nl, *mand, 2, *nout))
                                {
                                    (void)::phy_engine::netlist::delete_model(nl, pos);
                                    return false;
                                }
                                acc = nout;
                            }
                            and_out_node = acc;
                        }

                        auto [mor, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OR{});
                        if(mor == nullptr) { return false; }
                        if(!::phy_engine::netlist::add_to_node(nl, *mor, 0, *best->common) ||
                           !::phy_engine::netlist::add_to_node(nl, *mor, 1, *and_out_node))
                        {
                            (void)::phy_engine::netlist::delete_model(nl, pos);
                            return false;
                        }
                        mor_pos = pos;
                    }

                    for(auto const& ag : and_nodes) { (void)::phy_engine::netlist::delete_model(nl, ag.pos); }
                    for(auto const& og : or_terms) { (void)::phy_engine::netlist::delete_model(nl, og.pos); }

                    if(mor_pos)
                    {
                        auto* mor_m = ::phy_engine::netlist::get_model(nl, *mor_pos);
                        if(mor_m == nullptr || mor_m->type != ::phy_engine::model::model_type::normal || mor_m->ptr == nullptr) { return false; }
                        if(!::phy_engine::netlist::add_to_node(nl, *mor_m, 2, *gand.out))
                        {
                            (void)::phy_engine::netlist::delete_model(nl, *mor_pos);
                            return false;
                        }
                        return true;
                    }

                    ::std::vector<::phy_engine::model::pin*> pins_to_move{};
                    pins_to_move.reserve(gand.out->pins.size());
                    for(auto* p : gand.out->pins)
                    {
                        auto it = fan.pin_out.find(p);
                        bool const is_out = (it != fan.pin_out.end()) ? it->second : false;
                        if(is_out) { continue; }
                        pins_to_move.push_back(p);
                    }
                    for(auto* p : pins_to_move)
                    {
                        gand.out->pins.erase(p);
                        p->nodes = best->common;
                        best->common->pins.insert(p);
                    }
                    return true;
                };

                if(try_factor_and_tree())
                {
                    changed = true;
                    continue;
                }

                // 2-term dual factoring: (c|x) & (c|y) -> c | (x&y)
                auto it_a = gate_by_out.find(gand.in0);
                auto it_b = gate_by_out.find(gand.in1);
                if(it_a == gate_by_out.end() || it_b == gate_by_out.end()) { continue; }
                auto const& ga = it_a->second;
                auto const& gb = it_b->second;
                if(ga.k != kind::or_gate || gb.k != kind::or_gate) { continue; }
                if(ga.out == nullptr || gb.out == nullptr || ga.out_pin == nullptr || gb.out_pin == nullptr) { continue; }
                if(is_protected(ga.out) || is_protected(gb.out)) { continue; }
                if(ga.out != gand.in0 || gb.out != gand.in1) { continue; }

                auto const ga_dc = fan.driver_count.find(ga.out);
                auto const ga_cc = fan.consumer_count.find(ga.out);
                auto const gb_dc = fan.driver_count.find(gb.out);
                auto const gb_cc = fan.consumer_count.find(gb.out);
                if(ga_dc == fan.driver_count.end() || ga_dc->second != 1) { continue; }
                if(gb_dc == fan.driver_count.end() || gb_dc->second != 1) { continue; }
                if(ga_cc == fan.consumer_count.end() || ga_cc->second != 1) { continue; }
                if(gb_cc == fan.consumer_count.end() || gb_cc->second != 1) { continue; }
                if(!has_unique_driver_pin(ga.out, ga.out_pin, fan.pin_out)) { continue; }
                if(!has_unique_driver_pin(gb.out, gb.out_pin, fan.pin_out)) { continue; }

                auto const common = find_common(ga.in0, ga.in1, gb.in0, gb.in1);
                if(!common) { continue; }
                auto const [c, x, y] = *common;
                if(c == nullptr || x == nullptr || y == nullptr) { continue; }

                auto& tnode_ref = ::phy_engine::netlist::create_node(nl);
                auto* tnode = __builtin_addressof(tnode_ref);

                auto [mand, mand_pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::AND{});
                if(mand == nullptr) { continue; }
                if(!::phy_engine::netlist::add_to_node(nl, *mand, 0, *x) ||
                   !::phy_engine::netlist::add_to_node(nl, *mand, 1, *y) ||
                   !::phy_engine::netlist::add_to_node(nl, *mand, 2, *tnode))
                {
                    (void)::phy_engine::netlist::delete_model(nl, mand_pos);
                    continue;
                }

                auto [mor, mor_pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OR{});
                if(mor == nullptr)
                {
                    (void)::phy_engine::netlist::delete_model(nl, mand_pos);
                    continue;
                }
                if(!::phy_engine::netlist::add_to_node(nl, *mor, 0, *c) || !::phy_engine::netlist::add_to_node(nl, *mor, 1, *tnode))
                {
                    (void)::phy_engine::netlist::delete_model(nl, mor_pos);
                    (void)::phy_engine::netlist::delete_model(nl, mand_pos);
                    continue;
                }

                (void)::phy_engine::netlist::delete_model(nl, gand.pos);
                (void)::phy_engine::netlist::delete_model(nl, ga.pos);
                (void)::phy_engine::netlist::delete_model(nl, gb.pos);

                auto* mor_m = ::phy_engine::netlist::get_model(nl, mor_pos);
                if(mor_m == nullptr || mor_m->type != ::phy_engine::model::model_type::normal || mor_m->ptr == nullptr) { continue; }
                if(!::phy_engine::netlist::add_to_node(nl, *mor_m, 2, *gand.out))
                {
                    (void)::phy_engine::netlist::delete_model(nl, mor_pos);
                    (void)::phy_engine::netlist::delete_model(nl, mand_pos);
                    continue;
                }

                changed = true;
            }

            return changed;
        }

        [[nodiscard]] inline bool optimize_rewrite_xor_xnor_in_pe_netlist(::phy_engine::netlist::netlist& nl,
                                                                          ::std::vector<::phy_engine::model::node_t*> const& protected_nodes) noexcept
        {
            // AIG-style local rewriting: detect classic SOP XOR/XNOR patterns and replace them with XOR/XNOR gates.
            // This is a gate-count driven tech-mapping step (for the Phy-Engine logical gate library).

            struct not_gate
            {
                ::phy_engine::netlist::model_pos pos{};
                ::phy_engine::model::node_t* in{};
                ::phy_engine::model::node_t* out{};
                ::phy_engine::model::pin const* out_pin{};
            };
            struct and_gate
            {
                ::phy_engine::netlist::model_pos pos{};
                ::phy_engine::model::node_t* a{};
                ::phy_engine::model::node_t* b{};
                ::phy_engine::model::node_t* out{};
                ::phy_engine::model::pin const* out_pin{};
            };
            struct or_gate
            {
                ::phy_engine::netlist::model_pos pos{};
                ::phy_engine::model::node_t* a{};
                ::phy_engine::model::node_t* b{};
                ::phy_engine::model::node_t* out{};
                ::phy_engine::model::pin const* out_pin{};
            };
            struct nimp_gate
            {
                ::phy_engine::netlist::model_pos pos{};
                ::phy_engine::model::node_t* a{};
                ::phy_engine::model::node_t* b{};
                ::phy_engine::model::node_t* out{};
                ::phy_engine::model::pin const* out_pin{};
            };

            struct lit
            {
                ::phy_engine::model::node_t* v{};
                bool neg{};
            };

            ::std::unordered_map<::phy_engine::model::node_t*, bool> protected_map{};
            protected_map.reserve(protected_nodes.size() * 2u + 1u);
            for(auto* n : protected_nodes) { protected_map.emplace(n, true); }
            auto const is_protected = [&](::phy_engine::model::node_t* n) noexcept -> bool { return protected_map.contains(n); };

            auto const fan = build_gate_opt_fanout(nl);

            ::std::unordered_map<::phy_engine::model::node_t*, not_gate> not_by_out{};
            ::std::unordered_map<::phy_engine::model::node_t*, and_gate> and_by_out{};
            ::std::unordered_map<::phy_engine::model::node_t*, nimp_gate> nimp_by_out{};
            ::std::vector<or_gate> ors{};
            ::std::unordered_map<::phy_engine::model::node_t*, or_gate> or_by_out{};
            ::std::vector<and_gate> ands{};
            not_by_out.reserve(1 << 14);
            and_by_out.reserve(1 << 14);
            nimp_by_out.reserve(1 << 14);
            ors.reserve(1 << 14);
            or_by_out.reserve(1 << 14);
            ands.reserve(1 << 14);

            auto classify_not = [&](::phy_engine::model::model_base const& mb,
                                    ::phy_engine::netlist::model_pos pos) noexcept -> void
            {
                if(mb.type != ::phy_engine::model::model_type::normal || mb.ptr == nullptr) { return; }
                if(model_name_u8(mb) != u8"NOT") { return; }
                auto pv = mb.ptr->generate_pin_view();
                if(pv.size != 2) { return; }
                not_gate g{};
                g.pos = pos;
                g.in = pv.pins[0].nodes;
                g.out = pv.pins[1].nodes;
                g.out_pin = __builtin_addressof(pv.pins[1]);
                if(g.out != nullptr) { not_by_out.emplace(g.out, g); }
            };

            auto classify_and = [&](::phy_engine::model::model_base const& mb,
                                    ::phy_engine::netlist::model_pos pos) noexcept -> void
            {
                if(mb.type != ::phy_engine::model::model_type::normal || mb.ptr == nullptr) { return; }
                if(model_name_u8(mb) != u8"AND") { return; }
                auto pv = mb.ptr->generate_pin_view();
                if(pv.size != 3) { return; }
                and_gate g{};
                g.pos = pos;
                g.a = pv.pins[0].nodes;
                g.b = pv.pins[1].nodes;
                g.out = pv.pins[2].nodes;
                g.out_pin = __builtin_addressof(pv.pins[2]);
                if(g.out != nullptr) { and_by_out.emplace(g.out, g); }
            };

            auto classify_nimp = [&](::phy_engine::model::model_base const& mb,
                                     ::phy_engine::netlist::model_pos pos) noexcept -> void
            {
                if(mb.type != ::phy_engine::model::model_type::normal || mb.ptr == nullptr) { return; }
                if(model_name_u8(mb) != u8"NIMP") { return; }
                auto pv = mb.ptr->generate_pin_view();
                if(pv.size != 3) { return; }
                nimp_gate g{};
                g.pos = pos;
                g.a = pv.pins[0].nodes;
                g.b = pv.pins[1].nodes;
                g.out = pv.pins[2].nodes;
                g.out_pin = __builtin_addressof(pv.pins[2]);
                if(g.out != nullptr) { nimp_by_out.emplace(g.out, g); }
            };

            auto classify_or = [&](::phy_engine::model::model_base const& mb,
                                   ::phy_engine::netlist::model_pos pos) noexcept -> void
            {
                if(mb.type != ::phy_engine::model::model_type::normal || mb.ptr == nullptr) { return; }
                if(model_name_u8(mb) != u8"OR") { return; }
                auto pv = mb.ptr->generate_pin_view();
                if(pv.size != 3) { return; }
                or_gate g{};
                g.pos = pos;
                g.a = pv.pins[0].nodes;
                g.b = pv.pins[1].nodes;
                g.out = pv.pins[2].nodes;
                g.out_pin = __builtin_addressof(pv.pins[2]);
                ors.push_back(g);
                if(g.out != nullptr) { or_by_out.emplace(g.out, g); }
            };

            auto classify_and2 = [&](::phy_engine::model::model_base const& mb,
                                     ::phy_engine::netlist::model_pos pos) noexcept -> void
            {
                if(mb.type != ::phy_engine::model::model_type::normal || mb.ptr == nullptr) { return; }
                if(model_name_u8(mb) != u8"AND") { return; }
                auto pv = mb.ptr->generate_pin_view();
                if(pv.size != 3) { return; }
                and_gate g{};
                g.pos = pos;
                g.a = pv.pins[0].nodes;
                g.b = pv.pins[1].nodes;
                g.out = pv.pins[2].nodes;
                g.out_pin = __builtin_addressof(pv.pins[2]);
                ands.push_back(g);
            };

            for(std::size_t chunk_pos{}; chunk_pos < nl.models.size(); ++chunk_pos)
            {
                auto& blk = nl.models.index_unchecked(chunk_pos);
                for(std::size_t vec_pos{}; blk.begin + vec_pos < blk.curr; ++vec_pos)
                {
                    auto const& mb = blk.begin[vec_pos];
                    ::phy_engine::netlist::model_pos const pos{vec_pos, chunk_pos};
                    classify_not(mb, pos);
                    classify_and(mb, pos);
                    classify_nimp(mb, pos);
                    classify_or(mb, pos);
                    classify_and2(mb, pos);
                }
            }

            auto is_single_use_internal = [&](::phy_engine::model::node_t* n, ::phy_engine::model::pin const* expected_driver) noexcept -> bool
            {
                if(n == nullptr) { return false; }
                if(is_protected(n)) { return false; }
                auto itd = fan.driver_count.find(n);
                auto itc = fan.consumer_count.find(n);
                if(itd == fan.driver_count.end() || itc == fan.consumer_count.end()) { return false; }
                if(itd->second != 1 || itc->second != 1) { return false; }
                return has_unique_driver_pin(n, expected_driver, fan.pin_out);
            };

            auto get_lit = [&](::phy_engine::model::node_t* n) noexcept -> lit
            {
                if(auto itn = not_by_out.find(n); itn != not_by_out.end() && itn->second.in != nullptr)
                {
                    return lit{itn->second.in, true};
                }
                return lit{n, false};
            };

            auto canon2 = [](lit a, lit b) noexcept -> ::std::array<lit, 2>
            {
                if(reinterpret_cast<std::uintptr_t>(a.v) > reinterpret_cast<std::uintptr_t>(b.v)) { ::std::swap(a, b); }
                return {a, b};
            };

            auto add_xor_like = [&](bool is_xnor, ::phy_engine::model::node_t* a, ::phy_engine::model::node_t* b) noexcept
                -> ::std::optional<::phy_engine::netlist::model_pos>
            {
                if(a == nullptr || b == nullptr) { return ::std::nullopt; }
                if(is_xnor)
                {
                    auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::XNOR{});
                    (void)pos;
                    if(m == nullptr) { return ::std::nullopt; }
                    if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *a) || !::phy_engine::netlist::add_to_node(nl, *m, 1, *b))
                    {
                        (void)::phy_engine::netlist::delete_model(nl, pos);
                        return ::std::nullopt;
                    }
                    return pos;
                }
                auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::XOR{});
                (void)pos;
                if(m == nullptr) { return ::std::nullopt; }
                if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *a) || !::phy_engine::netlist::add_to_node(nl, *m, 1, *b))
                {
                    (void)::phy_engine::netlist::delete_model(nl, pos);
                    return ::std::nullopt;
                }
                return pos;
            };

            struct rewrite_action
            {
                bool to_xnor{};
                ::phy_engine::model::node_t* a{};
                ::phy_engine::model::node_t* b{};
                ::phy_engine::model::node_t* out{};
                ::phy_engine::netlist::model_pos or_pos{};
                ::phy_engine::netlist::model_pos t0_pos{};
                ::phy_engine::netlist::model_pos t1_pos{};
                bool delete_t0{};
                bool delete_t1{};
            };
            struct rewrite_action_pos
            {
                bool to_xnor{};
                ::phy_engine::model::node_t* a{};
                ::phy_engine::model::node_t* b{};
                ::phy_engine::model::node_t* out{};
                ::phy_engine::netlist::model_pos and_pos{};
                ::phy_engine::netlist::model_pos t0_pos{};
                ::phy_engine::netlist::model_pos t1_pos{};
                bool delete_t0{};
                bool delete_t1{};
            };

            ::std::vector<rewrite_action> actions{};
            actions.reserve(256);
            ::std::vector<rewrite_action_pos> actions_pos{};
            actions_pos.reserve(256);

            for(auto const& gor : ors)
            {
                if(gor.a == nullptr || gor.b == nullptr || gor.out == nullptr || gor.out_pin == nullptr) { continue; }
                if(is_protected(gor.out)) { continue; }
                if(!has_unique_driver_pin(gor.out, gor.out_pin, fan.pin_out)) { continue; }

                // Term extraction: each OR input must be a 2-literal product term from AND or NIMP.
                auto extract_term = [&](::phy_engine::model::node_t* term_out,
                                        ::std::array<lit, 2>& term_lits,
                                        ::phy_engine::netlist::model_pos& term_pos,
                                        bool& can_delete) noexcept -> bool
                {
                    if(term_out == nullptr) { return false; }
                    if(auto ita = and_by_out.find(term_out); ita != and_by_out.end())
                    {
                        auto const& ga = ita->second;
                        if(ga.a == nullptr || ga.b == nullptr || ga.out == nullptr || ga.out_pin == nullptr) { return false; }
                        term_pos = ga.pos;
                        can_delete = is_single_use_internal(ga.out, ga.out_pin);
                        term_lits = canon2(get_lit(ga.a), get_lit(ga.b));
                        return true;
                    }
                    if(auto itn = nimp_by_out.find(term_out); itn != nimp_by_out.end())
                    {
                        auto const& gn = itn->second;
                        if(gn.a == nullptr || gn.b == nullptr || gn.out == nullptr || gn.out_pin == nullptr) { return false; }
                        term_pos = gn.pos;
                        can_delete = is_single_use_internal(gn.out, gn.out_pin);
                        term_lits = canon2(lit{gn.a, false}, lit{gn.b, true});
                        return true;
                    }
                    return false;
                };

                ::std::array<lit, 2> t0{};
                ::std::array<lit, 2> t1{};
                ::phy_engine::netlist::model_pos t0_pos{};
                ::phy_engine::netlist::model_pos t1_pos{};
                bool del0{};
                bool del1{};
                if(!extract_term(gor.a, t0, t0_pos, del0)) { continue; }
                if(!extract_term(gor.b, t1, t1_pos, del1)) { continue; }

                // Must be over the same 2 variables.
                if(t0[0].v == nullptr || t0[1].v == nullptr || t1[0].v == nullptr || t1[1].v == nullptr) { continue; }
                if(t0[0].v != t1[0].v || t0[1].v != t1[1].v) { continue; }
                if(t0[0].v == t0[1].v) { continue; }

                // Complementary products: neg masks must be bitwise complements.
                bool const same0 = (t0[0].neg == t1[0].neg);
                bool const same1 = (t0[1].neg == t1[1].neg);
                if(same0 || same1) { continue; }

                unsigned const nneg0 = static_cast<unsigned>(t0[0].neg) + static_cast<unsigned>(t0[1].neg);
                unsigned const nneg1 = static_cast<unsigned>(t1[0].neg) + static_cast<unsigned>(t1[1].neg);

                bool to_xnor{};
                if((nneg0 == 1u && nneg1 == 1u))
                {
                    to_xnor = false;  // XOR
                }
                else if((nneg0 == 0u && nneg1 == 2u) || (nneg0 == 2u && nneg1 == 0u))
                {
                    to_xnor = true;  // XNOR
                }
                else
                {
                    continue;
                }

                actions.push_back(rewrite_action{
                    .to_xnor = to_xnor,
                    .a = t0[0].v,
                    .b = t0[1].v,
                    .out = gor.out,
                    .or_pos = gor.pos,
                    .t0_pos = t0_pos,
                    .t1_pos = t1_pos,
                    .delete_t0 = del0,
                    .delete_t1 = del1,
                });
            }

            // POS XOR/XNOR: (a|b)&(~a|~b) = XOR(a,b) ; (a|~b)&(~a|b) = XNOR(a,b)
            for(auto const& gand : ands)
            {
                if(gand.a == nullptr || gand.b == nullptr || gand.out == nullptr || gand.out_pin == nullptr) { continue; }
                if(is_protected(gand.out)) { continue; }
                if(!has_unique_driver_pin(gand.out, gand.out_pin, fan.pin_out)) { continue; }

                auto it0 = or_by_out.find(gand.a);
                auto it1 = or_by_out.find(gand.b);
                if(it0 == or_by_out.end() || it1 == or_by_out.end()) { continue; }
                auto const& t0g = it0->second;
                auto const& t1g = it1->second;
                if(t0g.a == nullptr || t0g.b == nullptr || t1g.a == nullptr || t1g.b == nullptr) { continue; }
                if(t0g.out == nullptr || t1g.out == nullptr || t0g.out_pin == nullptr || t1g.out_pin == nullptr) { continue; }

                bool const del0 = is_single_use_internal(t0g.out, t0g.out_pin);
                bool const del1 = is_single_use_internal(t1g.out, t1g.out_pin);

                auto t0 = canon2(get_lit(t0g.a), get_lit(t0g.b));
                auto t1 = canon2(get_lit(t1g.a), get_lit(t1g.b));
                if(t0[0].v == nullptr || t0[1].v == nullptr || t1[0].v == nullptr || t1[1].v == nullptr) { continue; }
                if(t0[0].v != t1[0].v || t0[1].v != t1[1].v) { continue; }
                if(t0[0].v == t0[1].v) { continue; }

                bool const same0 = (t0[0].neg == t1[0].neg);
                bool const same1 = (t0[1].neg == t1[1].neg);
                if(same0 || same1) { continue; }

                unsigned const nneg0 = static_cast<unsigned>(t0[0].neg) + static_cast<unsigned>(t0[1].neg);
                unsigned const nneg1 = static_cast<unsigned>(t1[0].neg) + static_cast<unsigned>(t1[1].neg);

                bool to_xnor{};
                if((nneg0 == 0u && nneg1 == 2u) || (nneg0 == 2u && nneg1 == 0u))
                {
                    to_xnor = false;  // XOR
                }
                else if((nneg0 == 1u && nneg1 == 1u))
                {
                    to_xnor = true;  // XNOR
                }
                else
                {
                    continue;
                }

                actions_pos.push_back(rewrite_action_pos{
                    .to_xnor = to_xnor,
                    .a = t0[0].v,
                    .b = t0[1].v,
                    .out = gand.out,
                    .and_pos = gand.pos,
                    .t0_pos = t0g.pos,
                    .t1_pos = t1g.pos,
                    .delete_t0 = del0,
                    .delete_t1 = del1,
                });
            }

            bool changed{};
            for(auto const& a : actions)
            {
                if(a.out == nullptr || a.a == nullptr || a.b == nullptr) { continue; }
                if(is_protected(a.out)) { continue; }

                // Remove OR driver first, then connect the new XOR/XNOR gate output to the same node.
                (void)::phy_engine::netlist::delete_model(nl, a.or_pos);

                auto new_pos = add_xor_like(a.to_xnor, a.a, a.b);
                if(!new_pos) { continue; }

                if(a.delete_t0) { (void)::phy_engine::netlist::delete_model(nl, a.t0_pos); }
                if(a.delete_t1) { (void)::phy_engine::netlist::delete_model(nl, a.t1_pos); }

                auto* nm = ::phy_engine::netlist::get_model(nl, *new_pos);
                if(nm == nullptr || nm->type != ::phy_engine::model::model_type::normal || nm->ptr == nullptr)
                {
                    continue;
                }
                if(!::phy_engine::netlist::add_to_node(nl, *nm, 2, *a.out))
                {
                    (void)::phy_engine::netlist::delete_model(nl, *new_pos);
                    continue;
                }

                changed = true;
            }

            for(auto const& a : actions_pos)
            {
                if(a.out == nullptr || a.a == nullptr || a.b == nullptr) { continue; }
                if(is_protected(a.out)) { continue; }

                (void)::phy_engine::netlist::delete_model(nl, a.and_pos);
                auto new_pos = add_xor_like(a.to_xnor, a.a, a.b);
                if(!new_pos) { continue; }

                if(a.delete_t0) { (void)::phy_engine::netlist::delete_model(nl, a.t0_pos); }
                if(a.delete_t1) { (void)::phy_engine::netlist::delete_model(nl, a.t1_pos); }

                auto* nm = ::phy_engine::netlist::get_model(nl, *new_pos);
                if(nm == nullptr || nm->type != ::phy_engine::model::model_type::normal || nm->ptr == nullptr) { continue; }
                if(!::phy_engine::netlist::add_to_node(nl, *nm, 2, *a.out))
                {
                    (void)::phy_engine::netlist::delete_model(nl, *new_pos);
                    continue;
                }
                changed = true;
            }

            return changed;
        }

        [[nodiscard]] inline bool optimize_eliminate_double_not_in_pe_netlist(::phy_engine::netlist::netlist& nl,
                                                                              ::std::vector<::phy_engine::model::node_t*> const& protected_nodes) noexcept
        {
            // Local resubstitution: ~~x -> x (or ~~x -> YES(x) when the output node is protected).

            struct not_gate
            {
                ::phy_engine::netlist::model_pos pos{};
                ::phy_engine::model::node_t* in{};
                ::phy_engine::model::node_t* out{};
                ::phy_engine::model::pin const* out_pin{};
            };

            ::std::unordered_map<::phy_engine::model::node_t*, bool> protected_map{};
            protected_map.reserve(protected_nodes.size() * 2u + 1u);
            for(auto* n : protected_nodes) { protected_map.emplace(n, true); }
            auto const is_protected = [&](::phy_engine::model::node_t* n) noexcept -> bool { return protected_map.contains(n); };

            auto const fan = build_gate_opt_fanout(nl);

            ::std::unordered_map<::phy_engine::model::node_t*, not_gate> not_by_out{};
            not_by_out.reserve(1 << 14);
            ::std::vector<not_gate> nots{};
            nots.reserve(1 << 14);

            for(std::size_t chunk_pos{}; chunk_pos < nl.models.size(); ++chunk_pos)
            {
                auto& blk = nl.models.index_unchecked(chunk_pos);
                for(std::size_t vec_pos{}; blk.begin + vec_pos < blk.curr; ++vec_pos)
                {
                    auto const& mb = blk.begin[vec_pos];
                    if(mb.type != ::phy_engine::model::model_type::normal || mb.ptr == nullptr) { continue; }
                    if(model_name_u8(mb) != u8"NOT") { continue; }
                    auto pv = mb.ptr->generate_pin_view();
                    if(pv.size != 2) { continue; }
                    not_gate g{};
                    g.pos = ::phy_engine::netlist::model_pos{vec_pos, chunk_pos};
                    g.in = pv.pins[0].nodes;
                    g.out = pv.pins[1].nodes;
                    g.out_pin = __builtin_addressof(pv.pins[1]);
                    if(g.out == nullptr || g.in == nullptr || g.out_pin == nullptr) { continue; }
                    not_by_out.emplace(g.out, g);
                    nots.push_back(g);
                }
            }

            auto move_consumers = [&](::phy_engine::model::node_t* from,
                                      ::phy_engine::model::node_t* to) noexcept -> bool
            {
                if(from == nullptr || to == nullptr || from == to) { return false; }
                if(is_protected(from)) { return false; }

                ::std::vector<::phy_engine::model::pin*> pins_to_move{};
                pins_to_move.reserve(from->pins.size());
                for(auto* p : from->pins)
                {
                    auto it = fan.pin_out.find(p);
                    bool const is_out = (it != fan.pin_out.end()) ? it->second : false;
                    if(is_out) { continue; }
                    pins_to_move.push_back(p);
                }

                for(auto* p : pins_to_move)
                {
                    from->pins.erase(p);
                    p->nodes = to;
                    to->pins.insert(p);
                }
                return true;
            };

            bool changed{};
            for(auto const& outer : nots)
            {
                if(outer.in == nullptr || outer.out == nullptr || outer.out_pin == nullptr) { continue; }
                if(!has_unique_driver_pin(outer.out, outer.out_pin, fan.pin_out)) { continue; }

                auto it_inner = not_by_out.find(outer.in);
                if(it_inner == not_by_out.end()) { continue; }
                auto const& inner = it_inner->second;
                if(inner.in == nullptr || inner.out == nullptr || inner.out_pin == nullptr) { continue; }
                if(inner.out != outer.in) { continue; }

                // inner output must be exclusively consumed by this outer NOT, so we can safely remove it.
                if(is_protected(inner.out)) { continue; }
                auto itd = fan.driver_count.find(inner.out);
                auto itc = fan.consumer_count.find(inner.out);
                if(itd == fan.driver_count.end() || itc == fan.consumer_count.end()) { continue; }
                if(itd->second != 1 || itc->second != 1) { continue; }
                if(!has_unique_driver_pin(inner.out, inner.out_pin, fan.pin_out)) { continue; }

                if(is_protected(outer.out))
                {
                    // Keep the protected node, but collapse ~~ into a single YES driver.
                    (void)::phy_engine::netlist::delete_model(nl, outer.pos);
                    (void)::phy_engine::netlist::delete_model(nl, inner.pos);

                    auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::YES{});
                    (void)pos;
                    if(m == nullptr) { continue; }
                    if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *inner.in) || !::phy_engine::netlist::add_to_node(nl, *m, 1, *outer.out))
                    {
                        (void)::phy_engine::netlist::delete_model(nl, pos);
                        continue;
                    }
                    changed = true;
                    continue;
                }

                if(!move_consumers(outer.out, inner.in)) { continue; }
                (void)::phy_engine::netlist::delete_model(nl, outer.pos);
                (void)::phy_engine::netlist::delete_model(nl, inner.pos);
                changed = true;
            }

            return changed;
        }

        [[nodiscard]] inline bool optimize_constant_propagation_in_pe_netlist(::phy_engine::netlist::netlist& nl,
                                                                              ::std::vector<::phy_engine::model::node_t*> const& protected_nodes,
                                                                              pe_synth_options const& opt) noexcept
        {
            // Safe constant propagation for 4-valued logic:
            // - Applies only identities that hold even with X/Z (e.g. x&0=0, x|1=1, x^0=x, etc.)
            // - Avoids rewrites like x^x=0 unless assume_binary_inputs is enabled.

            ::std::unordered_map<::phy_engine::model::node_t*, bool> protected_map{};
            protected_map.reserve(protected_nodes.size() * 2u + 1u);
            for(auto* n : protected_nodes) { protected_map.emplace(n, true); }
            auto const is_protected = [&](::phy_engine::model::node_t* n) noexcept -> bool { return protected_map.contains(n); };

            auto const fan = build_gate_opt_fanout(nl);

            using dns = ::phy_engine::model::digital_node_statement_t;

            // Build node->(0/1) constant map from unnamed INPUT drivers.
            ::std::unordered_map<::phy_engine::model::node_t*, dns> node_const{};
            node_const.reserve(1 << 14);
            for(auto& blk : nl.models)
            {
                for(auto* m = blk.begin; m != blk.curr; ++m)
                {
                    if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }
                    if(m->name.size() != 0) { continue; }  // named inputs are external IO
                    if(model_name_u8(*m) != u8"INPUT") { continue; }
                    auto vi = m->ptr->get_attribute(0);
                    if(vi.type != ::phy_engine::model::variant_type::digital) { continue; }
                    if(vi.digital != dns::false_state && vi.digital != dns::true_state) { continue; }
                    auto pv = m->ptr->generate_pin_view();
                    if(pv.size != 1) { continue; }
                    auto* n = pv.pins[0].nodes;
                    if(n == nullptr) { continue; }
                    node_const.emplace(n, vi.digital);
                }
            }

            auto get_const = [&](::phy_engine::model::node_t* n, dns& out) noexcept -> bool
            {
                if(n == nullptr) { return false; }
                if(auto it = node_const.find(n); it != node_const.end())
                {
                    out = it->second;
                    return true;
                }
                return false;
            };

            auto move_consumers = [&](::phy_engine::model::node_t* from,
                                      ::phy_engine::model::node_t* to) noexcept -> bool
            {
                if(from == nullptr || to == nullptr || from == to) { return false; }
                if(is_protected(from)) { return false; }
                ::std::vector<::phy_engine::model::pin*> pins_to_move{};
                pins_to_move.reserve(from->pins.size());
                for(auto* p : from->pins)
                {
                    auto it = fan.pin_out.find(p);
                    bool const is_out = (it != fan.pin_out.end()) ? it->second : false;
                    if(is_out) { continue; }
                    pins_to_move.push_back(p);
                }
                for(auto* p : pins_to_move)
                {
                    from->pins.erase(p);
                    p->nodes = to;
                    to->pins.insert(p);
                }
                return true;
            };

            struct cand
            {
                ::phy_engine::netlist::model_pos pos{};
                ::fast_io::u8string_view name{};
                ::phy_engine::model::node_t* a{};
                ::phy_engine::model::node_t* b{};
                ::phy_engine::model::node_t* out{};
                ::phy_engine::model::pin const* out_pin{};
                bool is_unary{};
            };
            ::std::vector<cand> cands{};
            cands.reserve(1 << 14);

            for(std::size_t chunk_pos{}; chunk_pos < nl.models.size(); ++chunk_pos)
            {
                auto& blk = nl.models.index_unchecked(chunk_pos);
                for(std::size_t vec_pos{}; blk.begin + vec_pos < blk.curr; ++vec_pos)
                {
                    auto const& mb = blk.begin[vec_pos];
                    if(mb.type != ::phy_engine::model::model_type::normal || mb.ptr == nullptr) { continue; }
                    auto const nm = model_name_u8(mb);
                    if(nm != u8"NOT" && nm != u8"AND" && nm != u8"OR" && nm != u8"XOR" && nm != u8"XNOR" && nm != u8"NAND" && nm != u8"NOR" &&
                       nm != u8"IMP" && nm != u8"NIMP")
                    {
                        continue;
                    }
                    auto pv = mb.ptr->generate_pin_view();
                    cand c{};
                    c.pos = ::phy_engine::netlist::model_pos{vec_pos, chunk_pos};
                    c.name = nm;
                    if(nm == u8"NOT")
                    {
                        if(pv.size != 2) { continue; }
                        c.is_unary = true;
                        c.a = pv.pins[0].nodes;
                        c.out = pv.pins[1].nodes;
                        c.out_pin = __builtin_addressof(pv.pins[1]);
                    }
                    else
                    {
                        if(pv.size != 3) { continue; }
                        c.is_unary = false;
                        c.a = pv.pins[0].nodes;
                        c.b = pv.pins[1].nodes;
                        c.out = pv.pins[2].nodes;
                        c.out_pin = __builtin_addressof(pv.pins[2]);
                    }
                    if(c.out == nullptr || c.out_pin == nullptr) { continue; }
                    cands.push_back(c);
                }
            }

            bool changed{};
            for(auto const& g : cands)
            {
                if(g.out == nullptr || g.out_pin == nullptr) { continue; }
                if(is_protected(g.out)) { continue; }
                if(!has_unique_driver_pin(g.out, g.out_pin, fan.pin_out)) { continue; }

                dns av{}, bv{};
                bool const aconst = get_const(g.a, av);
                bool const bconst = get_const(g.b, bv);

                ::phy_engine::model::node_t* replacement{};
                if(g.name == u8"NOT")
                {
                    if(!aconst) { continue; }
                    replacement = find_or_make_const_node(nl, (av == dns::true_state) ? dns::false_state : dns::true_state);
                }
                else if(g.name == u8"AND")
                {
                    if(aconst && av == dns::false_state) { replacement = find_or_make_const_node(nl, dns::false_state); }
                    else if(bconst && bv == dns::false_state) { replacement = find_or_make_const_node(nl, dns::false_state); }
                    else if(aconst && av == dns::true_state) { replacement = g.b; }
                    else if(bconst && bv == dns::true_state) { replacement = g.a; }
                    else if(g.a != nullptr && g.a == g.b) { replacement = g.a; }
                    else { continue; }
                }
                else if(g.name == u8"OR")
                {
                    if(aconst && av == dns::true_state) { replacement = find_or_make_const_node(nl, dns::true_state); }
                    else if(bconst && bv == dns::true_state) { replacement = find_or_make_const_node(nl, dns::true_state); }
                    else if(aconst && av == dns::false_state) { replacement = g.b; }
                    else if(bconst && bv == dns::false_state) { replacement = g.a; }
                    else if(g.a != nullptr && g.a == g.b) { replacement = g.a; }
                    else { continue; }
                }
                else if(g.name == u8"XOR")
                {
                    if(aconst && bconst)
                    {
                        bool const r = (av != bv);
                        replacement = find_or_make_const_node(nl, r ? dns::true_state : dns::false_state);
                    }
                    else if(aconst && av == dns::false_state) { replacement = g.b; }
                    else if(bconst && bv == dns::false_state) { replacement = g.a; }
                    else if(opt.assume_binary_inputs && g.a != nullptr && g.a == g.b)
                    {
                        replacement = find_or_make_const_node(nl, dns::false_state);
                    }
                    else { continue; }
                }
                else if(g.name == u8"XNOR")
                {
                    if(aconst && bconst)
                    {
                        bool const r = (av == bv);
                        replacement = find_or_make_const_node(nl, r ? dns::true_state : dns::false_state);
                    }
                    else if(aconst && av == dns::true_state) { replacement = g.b; }
                    else if(bconst && bv == dns::true_state) { replacement = g.a; }
                    else if(opt.assume_binary_inputs && g.a != nullptr && g.a == g.b)
                    {
                        replacement = find_or_make_const_node(nl, dns::true_state);
                    }
                    else { continue; }
                }
                else if(g.name == u8"NAND")
                {
                    if(aconst && av == dns::false_state) { replacement = find_or_make_const_node(nl, dns::true_state); }
                    else if(bconst && bv == dns::false_state) { replacement = find_or_make_const_node(nl, dns::true_state); }
                    else if(aconst && bconst)
                    {
                        bool const r = !(av == dns::true_state && bv == dns::true_state);
                        replacement = find_or_make_const_node(nl, r ? dns::true_state : dns::false_state);
                    }
                    else { continue; }
                }
                else if(g.name == u8"NOR")
                {
                    if(aconst && av == dns::true_state) { replacement = find_or_make_const_node(nl, dns::false_state); }
                    else if(bconst && bv == dns::true_state) { replacement = find_or_make_const_node(nl, dns::false_state); }
                    else if(aconst && bconst)
                    {
                        bool const r = !(av == dns::true_state || bv == dns::true_state);
                        replacement = find_or_make_const_node(nl, r ? dns::true_state : dns::false_state);
                    }
                    else { continue; }
                }
                else if(g.name == u8"IMP")
                {
                    // IMP(a,b) = ~a | b
                    if(bconst && bv == dns::true_state) { replacement = find_or_make_const_node(nl, dns::true_state); }
                    else if(aconst && av == dns::false_state) { replacement = find_or_make_const_node(nl, dns::true_state); }
                    else if(aconst && av == dns::true_state) { replacement = g.b; }
                    else { continue; }
                }
                else if(g.name == u8"NIMP")
                {
                    // NIMP(a,b) = a & ~b
                    if(aconst && av == dns::false_state) { replacement = find_or_make_const_node(nl, dns::false_state); }
                    else if(bconst && bv == dns::true_state) { replacement = find_or_make_const_node(nl, dns::false_state); }
                    else if(bconst && bv == dns::false_state) { replacement = g.a; }
                    else { continue; }
                }
                else
                {
                    continue;
                }

                if(replacement == nullptr) { continue; }
                if(!move_consumers(g.out, replacement)) { continue; }
                (void)::phy_engine::netlist::delete_model(nl, g.pos);
                changed = true;
            }

            return changed;
        }

        [[nodiscard]] inline bool optimize_absorption_in_pe_netlist(::phy_engine::netlist::netlist& nl,
                                                                    ::std::vector<::phy_engine::model::node_t*> const& protected_nodes) noexcept
        {
            // Multi-level boolean simplification (safe for 4-valued logic):
            // - a & (a | b) -> a
            // - a | (a & b) -> a
            // - a & (a & b) -> (a & b)
            // - a | (a | b) -> (a | b)

            enum class kind : std::uint8_t
            {
                and_gate,
                or_gate,
            };

            struct bin_gate
            {
                kind k{};
                ::phy_engine::netlist::model_pos pos{};
                ::phy_engine::model::node_t* in0{};
                ::phy_engine::model::node_t* in1{};
                ::phy_engine::model::node_t* out{};
                ::phy_engine::model::pin const* out_pin{};
            };

            ::std::unordered_map<::phy_engine::model::node_t*, bool> protected_map{};
            protected_map.reserve(protected_nodes.size() * 2u + 1u);
            for(auto* n : protected_nodes) { protected_map.emplace(n, true); }
            auto const is_protected = [&](::phy_engine::model::node_t* n) noexcept -> bool { return protected_map.contains(n); };

            auto const fan = build_gate_opt_fanout(nl);

            ::std::unordered_map<::phy_engine::model::node_t*, bin_gate> gate_by_out{};
            gate_by_out.reserve(1 << 14);
            ::std::vector<bin_gate> parents{};
            parents.reserve(1 << 14);

            auto classify = [&](::phy_engine::model::model_base const& mb,
                                ::phy_engine::netlist::model_pos pos) noexcept -> ::std::optional<bin_gate>
            {
                if(mb.type != ::phy_engine::model::model_type::normal || mb.ptr == nullptr) { return ::std::nullopt; }
                auto const name = model_name_u8(mb);
                if(name != u8"AND" && name != u8"OR") { return ::std::nullopt; }
                auto pv = mb.ptr->generate_pin_view();
                if(pv.size != 3) { return ::std::nullopt; }
                bin_gate g{};
                g.pos = pos;
                g.in0 = pv.pins[0].nodes;
                g.in1 = pv.pins[1].nodes;
                g.out = pv.pins[2].nodes;
                g.out_pin = __builtin_addressof(pv.pins[2]);
                g.k = (name == u8"AND") ? kind::and_gate : kind::or_gate;
                return g;
            };

            for(std::size_t chunk_pos{}; chunk_pos < nl.models.size(); ++chunk_pos)
            {
                auto& blk = nl.models.index_unchecked(chunk_pos);
                for(std::size_t vec_pos{}; blk.begin + vec_pos < blk.curr; ++vec_pos)
                {
                    auto const& mb = blk.begin[vec_pos];
                    auto og = classify(mb, {vec_pos, chunk_pos});
                    if(!og || og->out == nullptr || og->out_pin == nullptr) { continue; }
                    gate_by_out.emplace(og->out, *og);
                    parents.push_back(*og);
                }
            }

            auto move_consumers = [&](::phy_engine::model::node_t* from,
                                      ::phy_engine::model::node_t* to) noexcept -> bool
            {
                if(from == nullptr || to == nullptr || from == to) { return false; }
                if(is_protected(from)) { return false; }
                ::std::vector<::phy_engine::model::pin*> pins_to_move{};
                pins_to_move.reserve(from->pins.size());
                for(auto* p : from->pins)
                {
                    auto it = fan.pin_out.find(p);
                    bool const is_out = (it != fan.pin_out.end()) ? it->second : false;
                    if(is_out) { continue; }
                    pins_to_move.push_back(p);
                }
                for(auto* p : pins_to_move)
                {
                    from->pins.erase(p);
                    p->nodes = to;
                    to->pins.insert(p);
                }
                return true;
            };

            bool changed{};
            for(auto const& pg : parents)
            {
                if(pg.in0 == nullptr || pg.in1 == nullptr || pg.out == nullptr || pg.out_pin == nullptr) { continue; }
                if(is_protected(pg.out)) { continue; }
                if(!has_unique_driver_pin(pg.out, pg.out_pin, fan.pin_out)) { continue; }

                // Ensure parent still exists and matches.
                {
                    auto* mb = ::phy_engine::netlist::get_model(nl, pg.pos);
                    if(mb == nullptr || mb->type != ::phy_engine::model::model_type::normal || mb->ptr == nullptr) { continue; }
                    auto const nm = model_name_u8(*mb);
                    if((pg.k == kind::and_gate && nm != u8"AND") || (pg.k == kind::or_gate && nm != u8"OR")) { continue; }
                }

                auto try_rewrite = [&](::phy_engine::model::node_t* plain, ::phy_engine::model::node_t* subexpr) noexcept -> ::phy_engine::model::node_t*
                {
                    auto it = gate_by_out.find(subexpr);
                    if(it == gate_by_out.end()) { return nullptr; }
                    auto const& cg = it->second;
                    if(cg.in0 == nullptr || cg.in1 == nullptr || cg.out == nullptr) { return nullptr; }

                    // Ensure subexpression still exists.
                    auto* cmb = ::phy_engine::netlist::get_model(nl, cg.pos);
                    if(cmb == nullptr || cmb->type != ::phy_engine::model::model_type::normal || cmb->ptr == nullptr) { return nullptr; }
                    auto const cnm = model_name_u8(*cmb);
                    if((cg.k == kind::and_gate && cnm != u8"AND") || (cg.k == kind::or_gate && cnm != u8"OR")) { return nullptr; }

                    bool const hits = (plain == cg.in0) || (plain == cg.in1);
                    if(!hits) { return nullptr; }

                    // a & (a | b) -> a ; a | (a & b) -> a
                    if(pg.k == kind::and_gate && cg.k == kind::or_gate) { return plain; }
                    if(pg.k == kind::or_gate && cg.k == kind::and_gate) { return plain; }

                    // a & (a & b) -> (a & b) ; a | (a | b) -> (a | b)
                    if(pg.k == kind::and_gate && cg.k == kind::and_gate) { return cg.out; }
                    if(pg.k == kind::or_gate && cg.k == kind::or_gate) { return cg.out; }

                    return nullptr;
                };

                ::phy_engine::model::node_t* rep{};
                rep = try_rewrite(pg.in0, pg.in1);
                if(rep == nullptr) { rep = try_rewrite(pg.in1, pg.in0); }
                if(rep == nullptr) { continue; }

                if(!move_consumers(pg.out, rep)) { continue; }
                (void)::phy_engine::netlist::delete_model(nl, pg.pos);
                changed = true;
            }

            return changed;
        }

        [[nodiscard]] inline bool optimize_binary_complement_simplify_in_pe_netlist(::phy_engine::netlist::netlist& nl,
                                                                                     ::std::vector<::phy_engine::model::node_t*> const& protected_nodes) noexcept
        {
            // Binary-only multi-level simplifications (valid when inputs are guaranteed 0/1):
            // - (x&y) | (x&~y) -> x  and  (x&y) | (~x&y) -> y
            // - (x|y) & (x|~y) -> x  and  (x|y) & (~x|y) -> y
            // - (x|~x) -> 1 and (x&~x) -> 0 (when expressed structurally)

            struct not_gate
            {
                ::phy_engine::netlist::model_pos pos{};
                ::phy_engine::model::node_t* in{};
                ::phy_engine::model::node_t* out{};
                ::phy_engine::model::pin const* out_pin{};
            };
            struct bin_gate
            {
                ::fast_io::u8string_view name{};
                ::phy_engine::netlist::model_pos pos{};
                ::phy_engine::model::node_t* a{};
                ::phy_engine::model::node_t* b{};
                ::phy_engine::model::node_t* out{};
                ::phy_engine::model::pin const* out_pin{};
            };

            ::std::unordered_map<::phy_engine::model::node_t*, bool> protected_map{};
            protected_map.reserve(protected_nodes.size() * 2u + 1u);
            for(auto* n : protected_nodes) { protected_map.emplace(n, true); }
            auto const is_protected = [&](::phy_engine::model::node_t* n) noexcept -> bool { return protected_map.contains(n); };

            auto const fan = build_gate_opt_fanout(nl);

            ::std::unordered_map<::phy_engine::model::node_t*, not_gate> not_by_out{};
            not_by_out.reserve(1 << 14);
            ::std::unordered_map<::phy_engine::model::node_t*, bin_gate> and_by_out{};
            ::std::unordered_map<::phy_engine::model::node_t*, bin_gate> or_by_out{};
            and_by_out.reserve(1 << 14);
            or_by_out.reserve(1 << 14);
            ::std::vector<bin_gate> ors{};
            ::std::vector<bin_gate> ands{};
            ors.reserve(1 << 14);
            ands.reserve(1 << 14);

            for(std::size_t chunk_pos{}; chunk_pos < nl.models.size(); ++chunk_pos)
            {
                auto& blk = nl.models.index_unchecked(chunk_pos);
                for(std::size_t vec_pos{}; blk.begin + vec_pos < blk.curr; ++vec_pos)
                {
                    auto const& mb = blk.begin[vec_pos];
                    if(mb.type != ::phy_engine::model::model_type::normal || mb.ptr == nullptr) { continue; }
                    auto const name = model_name_u8(mb);
                    auto pv = mb.ptr->generate_pin_view();

                    if(name == u8"NOT")
                    {
                        if(pv.size != 2) { continue; }
                        not_gate g{};
                        g.pos = {vec_pos, chunk_pos};
                        g.in = pv.pins[0].nodes;
                        g.out = pv.pins[1].nodes;
                        g.out_pin = __builtin_addressof(pv.pins[1]);
                        if(g.out != nullptr) { not_by_out.emplace(g.out, g); }
                        continue;
                    }

                    if(name != u8"AND" && name != u8"OR") { continue; }
                    if(pv.size != 3) { continue; }
                    bin_gate g{};
                    g.name = name;
                    g.pos = {vec_pos, chunk_pos};
                    g.a = pv.pins[0].nodes;
                    g.b = pv.pins[1].nodes;
                    g.out = pv.pins[2].nodes;
                    g.out_pin = __builtin_addressof(pv.pins[2]);
                    if(g.out == nullptr || g.out_pin == nullptr) { continue; }
                    if(name == u8"AND")
                    {
                        and_by_out.emplace(g.out, g);
                        ands.push_back(g);
                    }
                    else
                    {
                        or_by_out.emplace(g.out, g);
                        ors.push_back(g);
                    }
                }
            }

            auto is_single_use_internal = [&](::phy_engine::model::node_t* n, ::phy_engine::model::pin const* expected_driver) noexcept -> bool
            {
                if(n == nullptr) { return false; }
                if(is_protected(n)) { return false; }
                auto itd = fan.driver_count.find(n);
                auto itc = fan.consumer_count.find(n);
                if(itd == fan.driver_count.end() || itc == fan.consumer_count.end()) { return false; }
                if(itd->second != 1 || itc->second != 1) { return false; }
                return has_unique_driver_pin(n, expected_driver, fan.pin_out);
            };

            struct lit
            {
                ::phy_engine::model::node_t* v{};
                bool neg{};
            };
            auto get_lit = [&](::phy_engine::model::node_t* n) noexcept -> lit
            {
                if(auto it = not_by_out.find(n); it != not_by_out.end() && it->second.in != nullptr) { return lit{it->second.in, true}; }
                return lit{n, false};
            };

            auto move_consumers = [&](::phy_engine::model::node_t* from, ::phy_engine::model::node_t* to) noexcept -> bool
            {
                if(from == nullptr || to == nullptr || from == to) { return false; }
                if(is_protected(from)) { return false; }
                ::std::vector<::phy_engine::model::pin*> pins_to_move{};
                pins_to_move.reserve(from->pins.size());
                for(auto* p : from->pins)
                {
                    auto it = fan.pin_out.find(p);
                    bool const is_out = (it != fan.pin_out.end()) ? it->second : false;
                    if(is_out) { continue; }
                    pins_to_move.push_back(p);
                }
                for(auto* p : pins_to_move)
                {
                    from->pins.erase(p);
                    p->nodes = to;
                    to->pins.insert(p);
                }
                return true;
            };

            auto make_const = [&](bool v) noexcept -> ::phy_engine::model::node_t*
            {
                return find_or_make_const_node(nl, v ? ::phy_engine::model::digital_node_statement_t::true_state
                                                     : ::phy_engine::model::digital_node_statement_t::false_state);
            };

            bool changed{};

            // Simplify OR pairs: (x&y)|(x&~y) -> x, and (x&y)|(~x&y) -> y.
            for(auto const& og : ors)
            {
                if(og.a == nullptr || og.b == nullptr || og.out == nullptr || og.out_pin == nullptr) { continue; }
                if(is_protected(og.out)) { continue; }
                if(!has_unique_driver_pin(og.out, og.out_pin, fan.pin_out)) { continue; }

                auto it0 = and_by_out.find(og.a);
                auto it1 = and_by_out.find(og.b);
                if(it0 == and_by_out.end() || it1 == and_by_out.end()) { continue; }
                auto const& a0 = it0->second;
                auto const& a1 = it1->second;
                if(a0.a == nullptr || a0.b == nullptr || a1.a == nullptr || a1.b == nullptr) { continue; }

                auto l00 = get_lit(a0.a);
                auto l01 = get_lit(a0.b);
                auto l10 = get_lit(a1.a);
                auto l11 = get_lit(a1.b);
                ::std::array<lit, 2> t0{l00, l01};
                ::std::array<lit, 2> t1{l10, l11};

                auto canon = [](lit a, lit b) noexcept -> ::std::array<lit, 2>
                {
                    if(reinterpret_cast<std::uintptr_t>(a.v) > reinterpret_cast<std::uintptr_t>(b.v)) { ::std::swap(a, b); }
                    return {a, b};
                };
                t0 = canon(t0[0], t0[1]);
                t1 = canon(t1[0], t1[1]);
                if(t0[0].v == nullptr || t0[1].v == nullptr || t1[0].v == nullptr || t1[1].v == nullptr) { continue; }
                if(t0[0].v != t1[0].v || t0[1].v != t1[1].v) { continue; }

                // Look for one literal with same polarity and the other with opposite polarity.
                bool const same0 = (t0[0].neg == t1[0].neg);
                bool const same1 = (t0[1].neg == t1[1].neg);
                if(same0 == same1) { continue; }  // need exactly one same and one opposite

                ::phy_engine::model::node_t* rep{};
                if(same0) { rep = t0[0].v; }
                else { rep = t0[1].v; }
                // If the common literal is negated, result is ~x (still valid). We'll materialize via NOT.
                bool const rep_neg = same0 ? t0[0].neg : t0[1].neg;
                if(rep == nullptr) { continue; }
                if(rep_neg)
                {
                    // Find existing NOT output if available, else build a new NOT (allowed even if shared).
                    // Prefer reusing by scanning not_by_out; reverse map would be heavier so just create.
                    auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::NOT{});
                    (void)pos;
                    if(m == nullptr) { continue; }
                    auto& nref = ::phy_engine::netlist::create_node(nl);
                    auto* nout = __builtin_addressof(nref);
                    if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *rep) || !::phy_engine::netlist::add_to_node(nl, *m, 1, *nout))
                    {
                        (void)::phy_engine::netlist::delete_model(nl, pos);
                        continue;
                    }
                    rep = nout;
                }

                if(!move_consumers(og.out, rep)) { continue; }
                (void)::phy_engine::netlist::delete_model(nl, og.pos);
                if(is_single_use_internal(a0.out, a0.out_pin)) { (void)::phy_engine::netlist::delete_model(nl, a0.pos); }
                if(is_single_use_internal(a1.out, a1.out_pin)) { (void)::phy_engine::netlist::delete_model(nl, a1.pos); }
                changed = true;
            }

            // Simplify AND pairs: (x|y)&(x|~y) -> x, and (x|y)&(~x|y) -> y.
            for(auto const& ag : ands)
            {
                if(ag.a == nullptr || ag.b == nullptr || ag.out == nullptr || ag.out_pin == nullptr) { continue; }
                if(is_protected(ag.out)) { continue; }
                if(!has_unique_driver_pin(ag.out, ag.out_pin, fan.pin_out)) { continue; }

                auto it0 = or_by_out.find(ag.a);
                auto it1 = or_by_out.find(ag.b);
                if(it0 == or_by_out.end() || it1 == or_by_out.end()) { continue; }
                auto const& o0 = it0->second;
                auto const& o1 = it1->second;
                if(o0.a == nullptr || o0.b == nullptr || o1.a == nullptr || o1.b == nullptr) { continue; }

                auto l00 = get_lit(o0.a);
                auto l01 = get_lit(o0.b);
                auto l10 = get_lit(o1.a);
                auto l11 = get_lit(o1.b);
                ::std::array<lit, 2> t0{l00, l01};
                ::std::array<lit, 2> t1{l10, l11};
                auto canon = [](lit a, lit b) noexcept -> ::std::array<lit, 2>
                {
                    if(reinterpret_cast<std::uintptr_t>(a.v) > reinterpret_cast<std::uintptr_t>(b.v)) { ::std::swap(a, b); }
                    return {a, b};
                };
                t0 = canon(t0[0], t0[1]);
                t1 = canon(t1[0], t1[1]);
                if(t0[0].v == nullptr || t0[1].v == nullptr || t1[0].v == nullptr || t1[1].v == nullptr) { continue; }
                if(t0[0].v != t1[0].v || t0[1].v != t1[1].v) { continue; }

                bool const same0 = (t0[0].neg == t1[0].neg);
                bool const same1 = (t0[1].neg == t1[1].neg);
                if(same0 == same1) { continue; }

                ::phy_engine::model::node_t* rep{};
                bool rep_neg{};
                if(same0) { rep = t0[0].v; rep_neg = t0[0].neg; }
                else { rep = t0[1].v; rep_neg = t0[1].neg; }
                if(rep == nullptr) { continue; }
                if(rep_neg)
                {
                    auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::NOT{});
                    (void)pos;
                    if(m == nullptr) { continue; }
                    auto& nref = ::phy_engine::netlist::create_node(nl);
                    auto* nout = __builtin_addressof(nref);
                    if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *rep) || !::phy_engine::netlist::add_to_node(nl, *m, 1, *nout))
                    {
                        (void)::phy_engine::netlist::delete_model(nl, pos);
                        continue;
                    }
                    rep = nout;
                }

                if(!move_consumers(ag.out, rep)) { continue; }
                (void)::phy_engine::netlist::delete_model(nl, ag.pos);
                if(is_single_use_internal(o0.out, o0.out_pin)) { (void)::phy_engine::netlist::delete_model(nl, o0.pos); }
                if(is_single_use_internal(o1.out, o1.out_pin)) { (void)::phy_engine::netlist::delete_model(nl, o1.pos); }
                changed = true;
            }

            // Complement tautologies/contradictions: (x|~x)->1, (x&~x)->0 when expressed as OR/AND of a literal and its negation.
            for(auto const& og : ors)
            {
                if(og.a == nullptr || og.b == nullptr || og.out == nullptr || og.out_pin == nullptr) { continue; }
                if(is_protected(og.out)) { continue; }
                if(!has_unique_driver_pin(og.out, og.out_pin, fan.pin_out)) { continue; }
                auto la = get_lit(og.a);
                auto lb = get_lit(og.b);
                if(la.v != nullptr && la.v == lb.v && la.neg != lb.neg)
                {
                    auto* rep = make_const(true);
                    if(rep == nullptr) { continue; }
                    if(!move_consumers(og.out, rep)) { continue; }
                    (void)::phy_engine::netlist::delete_model(nl, og.pos);
                    changed = true;
                }
            }
            for(auto const& ag : ands)
            {
                if(ag.a == nullptr || ag.b == nullptr || ag.out == nullptr || ag.out_pin == nullptr) { continue; }
                if(is_protected(ag.out)) { continue; }
                if(!has_unique_driver_pin(ag.out, ag.out_pin, fan.pin_out)) { continue; }
                auto la = get_lit(ag.a);
                auto lb = get_lit(ag.b);
                if(la.v != nullptr && la.v == lb.v && la.neg != lb.neg)
                {
                    auto* rep = make_const(false);
                    if(rep == nullptr) { continue; }
                    if(!move_consumers(ag.out, rep)) { continue; }
                    (void)::phy_engine::netlist::delete_model(nl, ag.pos);
                    changed = true;
                }
            }

            return changed;
        }

        [[nodiscard]] inline bool optimize_flatten_associative_and_or_in_pe_netlist(::phy_engine::netlist::netlist& nl,
                                                                                    ::std::vector<::phy_engine::model::node_t*> const& protected_nodes,
                                                                                    pe_synth_options const& opt) noexcept
        {
            // Flatten AND/OR trees, remove duplicates, and fold constants.
            // - Safe 4-valued rules: x&0=0, x&1=x, x|1=1, x|0=x, idempotence x&x=x, x|x=x
            // - Binary-only (assume_binary_inputs): x&~x=0, x|~x=1 across the whole flattened set
            //
            // This is a multi-level optimization: it can reduce gate count even when duplicates/constants are not adjacent.

            enum class k2 : std::uint8_t
            {
                and_gate,
                or_gate,
            };

            struct not_gate
            {
                ::phy_engine::netlist::model_pos pos{};
                ::phy_engine::model::node_t* in{};
                ::phy_engine::model::node_t* out{};
                ::phy_engine::model::pin const* out_pin{};
            };

            struct bin_gate
            {
                k2 k{};
                ::phy_engine::netlist::model_pos pos{};
                ::phy_engine::model::node_t* in0{};
                ::phy_engine::model::node_t* in1{};
                ::phy_engine::model::node_t* out{};
                ::phy_engine::model::pin const* out_pin{};
            };

            struct leaf
            {
                ::phy_engine::model::node_t* node{};  // actual node in netlist
                ::phy_engine::model::node_t* base{};  // underlying (for NOT literals: input of NOT)
                bool neg{};
            };

            ::std::unordered_map<::phy_engine::model::node_t*, bool> protected_map{};
            protected_map.reserve(protected_nodes.size() * 2u + 1u);
            for(auto* n : protected_nodes) { protected_map.emplace(n, true); }
            auto const is_protected = [&](::phy_engine::model::node_t* n) noexcept -> bool { return protected_map.contains(n); };

            auto const fan = build_gate_opt_fanout(nl);

            using dns = ::phy_engine::model::digital_node_statement_t;
            ::std::unordered_map<::phy_engine::model::node_t*, dns> node_const{};
            node_const.reserve(1 << 14);
            for(auto& blk : nl.models)
            {
                for(auto* m = blk.begin; m != blk.curr; ++m)
                {
                    if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }
                    if(m->name.size() != 0) { continue; }
                    if(model_name_u8(*m) != u8"INPUT") { continue; }
                    auto vi = m->ptr->get_attribute(0);
                    if(vi.type != ::phy_engine::model::variant_type::digital) { continue; }
                    if(vi.digital != dns::false_state && vi.digital != dns::true_state) { continue; }
                    auto pv = m->ptr->generate_pin_view();
                    if(pv.size != 1) { continue; }
                    auto* n = pv.pins[0].nodes;
                    if(n == nullptr) { continue; }
                    node_const.emplace(n, vi.digital);
                }
            }

            auto get_const = [&](::phy_engine::model::node_t* n, bool& out) noexcept -> bool
            {
                if(n == nullptr) { return false; }
                if(auto it = node_const.find(n); it != node_const.end())
                {
                    out = (it->second == dns::true_state);
                    return true;
                }
                return false;
            };

            ::std::unordered_map<::phy_engine::model::node_t*, not_gate> not_by_out{};
            not_by_out.reserve(1 << 14);

            ::std::unordered_map<::phy_engine::model::node_t*, bin_gate> gate_by_out{};
            gate_by_out.reserve(1 << 14);

            ::std::vector<bin_gate> roots{};
            roots.reserve(1 << 14);

            for(std::size_t chunk_pos{}; chunk_pos < nl.models.size(); ++chunk_pos)
            {
                auto& blk = nl.models.index_unchecked(chunk_pos);
                for(std::size_t vec_pos{}; blk.begin + vec_pos < blk.curr; ++vec_pos)
                {
                    auto const& mb = blk.begin[vec_pos];
                    if(mb.type != ::phy_engine::model::model_type::normal || mb.ptr == nullptr) { continue; }
                    auto const name = model_name_u8(mb);
                    auto pv = mb.ptr->generate_pin_view();

                    if(name == u8"NOT")
                    {
                        if(pv.size != 2) { continue; }
                        not_gate ng{};
                        ng.pos = {vec_pos, chunk_pos};
                        ng.in = pv.pins[0].nodes;
                        ng.out = pv.pins[1].nodes;
                        ng.out_pin = __builtin_addressof(pv.pins[1]);
                        if(ng.out != nullptr) { not_by_out.emplace(ng.out, ng); }
                        continue;
                    }

                    if(name != u8"AND" && name != u8"OR") { continue; }
                    if(pv.size != 3) { continue; }

                    bin_gate g{};
                    g.pos = {vec_pos, chunk_pos};
                    g.in0 = pv.pins[0].nodes;
                    g.in1 = pv.pins[1].nodes;
                    g.out = pv.pins[2].nodes;
                    g.out_pin = __builtin_addressof(pv.pins[2]);
                    g.k = (name == u8"AND") ? k2::and_gate : k2::or_gate;
                    if(g.out == nullptr || g.out_pin == nullptr) { continue; }
                    gate_by_out.emplace(g.out, g);
                    roots.push_back(g);
                }
            }

            auto is_exclusive_internal = [&](bin_gate const& g) noexcept -> bool
            {
                if(g.out == nullptr || g.out_pin == nullptr) { return false; }
                if(is_protected(g.out)) { return false; }
                auto itd = fan.driver_count.find(g.out);
                auto itc = fan.consumer_count.find(g.out);
                if(itd == fan.driver_count.end() || itc == fan.consumer_count.end()) { return false; }
                if(itd->second != 1 || itc->second != 1) { return false; }
                return has_unique_driver_pin(g.out, g.out_pin, fan.pin_out);
            };

            auto as_leaf = [&](::phy_engine::model::node_t* n) noexcept -> leaf
            {
                if(auto it = not_by_out.find(n); it != not_by_out.end() && it->second.in != nullptr)
                {
                    return leaf{.node = n, .base = it->second.in, .neg = true};
                }
                return leaf{.node = n, .base = n, .neg = false};
            };

            auto move_consumers = [&](::phy_engine::model::node_t* from, ::phy_engine::model::node_t* to) noexcept -> bool
            {
                if(from == nullptr || to == nullptr || from == to) { return false; }
                if(is_protected(from)) { return false; }
                ::std::vector<::phy_engine::model::pin*> pins_to_move{};
                pins_to_move.reserve(from->pins.size());
                for(auto* p : from->pins)
                {
                    auto it = fan.pin_out.find(p);
                    bool const is_out = (it != fan.pin_out.end()) ? it->second : false;
                    if(is_out) { continue; }
                    pins_to_move.push_back(p);
                }
                for(auto* p : pins_to_move)
                {
                    from->pins.erase(p);
                    p->nodes = to;
                    to->pins.insert(p);
                }
                return true;
            };

            auto make_bin = [&](k2 k, ::phy_engine::model::node_t* a, ::phy_engine::model::node_t* b, ::phy_engine::model::node_t* out) noexcept
                -> ::std::optional<::phy_engine::netlist::model_pos>
            {
                if(a == nullptr || b == nullptr || out == nullptr) { return ::std::nullopt; }
                if(k == k2::and_gate)
                {
                    auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::AND{});
                    (void)pos;
                    if(m == nullptr) { return ::std::nullopt; }
                    if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *a) || !::phy_engine::netlist::add_to_node(nl, *m, 1, *b) ||
                       !::phy_engine::netlist::add_to_node(nl, *m, 2, *out))
                    {
                        (void)::phy_engine::netlist::delete_model(nl, pos);
                        return ::std::nullopt;
                    }
                    return pos;
                }
                auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OR{});
                (void)pos;
                if(m == nullptr) { return ::std::nullopt; }
                if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *a) || !::phy_engine::netlist::add_to_node(nl, *m, 1, *b) ||
                   !::phy_engine::netlist::add_to_node(nl, *m, 2, *out))
                {
                    (void)::phy_engine::netlist::delete_model(nl, pos);
                    return ::std::nullopt;
                }
                return pos;
            };

            auto make_const_node = [&](bool v) noexcept -> ::phy_engine::model::node_t*
            {
                return find_or_make_const_node(nl, v ? dns::true_state : dns::false_state);
            };

            bool changed{};
            for(auto const& root : roots)
            {
                if(root.in0 == nullptr || root.in1 == nullptr || root.out == nullptr || root.out_pin == nullptr) { continue; }
                if(is_protected(root.out)) { continue; }
                if(!has_unique_driver_pin(root.out, root.out_pin, fan.pin_out)) { continue; }

                // Ensure root still exists and matches.
                {
                    auto* mb = ::phy_engine::netlist::get_model(nl, root.pos);
                    if(mb == nullptr || mb->type != ::phy_engine::model::model_type::normal || mb->ptr == nullptr) { continue; }
                    auto const nm = model_name_u8(*mb);
                    if((root.k == k2::and_gate && nm != u8"AND") || (root.k == k2::or_gate && nm != u8"OR")) { continue; }
                }

                ::std::vector<leaf> leaves{};
                leaves.reserve(32);
                ::std::vector<::phy_engine::netlist::model_pos> to_delete{};
                to_delete.reserve(64);

                ::std::unordered_map<::phy_engine::model::node_t*, bool> visited{};
                visited.reserve(64);

                auto collect = [&](auto&& self, ::phy_engine::model::node_t* n) noexcept -> void
                {
                    if(n == nullptr) { return; }
                    if(visited.contains(n)) { leaves.push_back(as_leaf(n)); return; }
                    visited.emplace(n, true);

                    auto itg = gate_by_out.find(n);
                    if(itg == gate_by_out.end() || itg->second.k != root.k)
                    {
                        leaves.push_back(as_leaf(n));
                        return;
                    }
                    auto const& g = itg->second;
                    if(!is_exclusive_internal(g))
                    {
                        leaves.push_back(as_leaf(n));
                        return;
                    }
                    to_delete.push_back(g.pos);
                    self(self, g.in0);
                    self(self, g.in1);
                };

                collect(collect, root.in0);
                collect(collect, root.in1);
                if(leaves.empty()) { continue; }

                // Remove duplicates and fold constants.
                bool any_const{};
                bool const_value{};
                ::std::unordered_map<::std::uint64_t, leaf> uniq{};
                uniq.reserve(leaves.size() * 2u);

                auto key_of = [](leaf const& l) noexcept -> ::std::uint64_t
                {
                    auto const p = static_cast<::std::uint64_t>(reinterpret_cast<::std::uintptr_t>(l.base));
                    return (p >> 4) ^ (static_cast<::std::uint64_t>(l.neg) << 1);
                };

                bool force_const{};
                bool force_const_value{};

                for(auto const& l : leaves)
                {
                    if(l.node == nullptr || l.base == nullptr) { continue; }

                    bool cv{};
                    if(get_const(l.node, cv))
                    {
                        any_const = true;
                        const_value = cv;
                        if(root.k == k2::and_gate)
                        {
                            if(!cv) { force_const = true; force_const_value = false; break; }
                            // true leaf can be dropped
                            continue;
                        }
                        else
                        {
                            if(cv) { force_const = true; force_const_value = true; break; }
                            // false leaf can be dropped
                            continue;
                        }
                    }

                    uniq.emplace(key_of(l), l);
                }

                if(force_const)
                {
                    auto* rep = make_const_node(force_const_value);
                    if(rep == nullptr) { continue; }
                    if(!move_consumers(root.out, rep)) { continue; }
                    (void)::phy_engine::netlist::delete_model(nl, root.pos);
                    for(auto const mp : to_delete) { (void)::phy_engine::netlist::delete_model(nl, mp); }
                    changed = true;
                    continue;
                }

                // Binary-only complement check across uniq set.
                bool did_root_rewrite{};
                if(opt.assume_binary_inputs)
                {
                    ::std::unordered_map<::phy_engine::model::node_t*, unsigned> seen_mask{};
                    seen_mask.reserve(uniq.size() * 2u);
                    for(auto const& kv : uniq)
                    {
                        auto const& l = kv.second;
                        if(l.base == nullptr) { continue; }
                        auto& m = seen_mask[l.base];
                        m |= l.neg ? 2u : 1u;
                        if(m == 3u)
                        {
                            bool const v = (root.k == k2::or_gate);
                            auto* rep = make_const_node(v);
                            if(rep == nullptr) { did_root_rewrite = false; break; }
                            if(!move_consumers(root.out, rep)) { did_root_rewrite = false; break; }
                            (void)::phy_engine::netlist::delete_model(nl, root.pos);
                            for(auto const mp : to_delete) { (void)::phy_engine::netlist::delete_model(nl, mp); }
                            changed = true;
                            did_root_rewrite = true;
                            break;
                        }
                    }
                }
                if(did_root_rewrite) { continue; }

                {
                    ::std::vector<leaf> flat{};
                    flat.reserve(uniq.size());
                    for(auto const& kv : uniq) { flat.push_back(kv.second); }

                    // If everything dropped (e.g. AND of 1's, OR of 0's), reduce to identity constant.
                    if(flat.empty())
                    {
                        bool const v = (root.k == k2::and_gate);
                        auto* rep = make_const_node(v);
                        if(rep == nullptr) { continue; }
                        if(!move_consumers(root.out, rep)) { continue; }
                        (void)::phy_engine::netlist::delete_model(nl, root.pos);
                        for(auto const mp : to_delete) { (void)::phy_engine::netlist::delete_model(nl, mp); }
                        changed = true;
                        continue;
                    }

                    // Canonical order: sort by (base ptr, neg).
                    ::std::sort(flat.begin(), flat.end(), [](leaf const& x, leaf const& y) noexcept {
                        if(x.base != y.base) { return reinterpret_cast<std::uintptr_t>(x.base) < reinterpret_cast<std::uintptr_t>(y.base); }
                        return x.neg < y.neg;
                    });

                    auto const old_cost = 1u + to_delete.size();
                    auto const new_cost = (flat.size() <= 1) ? 0u : static_cast<std::size_t>(flat.size() - 1);
                    if(new_cost >= old_cost) { continue; }

                    if(flat.size() == 1)
                    {
                        auto* rep = flat[0].node;
                        if(rep == nullptr) { continue; }
                        if(!move_consumers(root.out, rep)) { continue; }
                        (void)::phy_engine::netlist::delete_model(nl, root.pos);
                        for(auto const mp : to_delete) { (void)::phy_engine::netlist::delete_model(nl, mp); }
                        changed = true;
                        continue;
                    }

                    // Build a balanced-ish chain from leaves.
                    auto* acc = flat[0].node;
                    if(acc == nullptr) { continue; }
                    ::std::vector<::phy_engine::netlist::model_pos> new_models{};
                    new_models.reserve(flat.size());

                    for(std::size_t i{1}; i + 1 < flat.size(); ++i)
                    {
                        auto& nref = ::phy_engine::netlist::create_node(nl);
                        auto* nout = __builtin_addressof(nref);
                        auto mp = make_bin(root.k, acc, flat[i].node, nout);
                        if(!mp) { goto rollback_new; }
                        new_models.push_back(*mp);
                        acc = nout;
                    }

                    // Final gate drives root.out node.
                    {
                        auto mp = make_bin(root.k, acc, flat.back().node, root.out);
                        if(!mp) { goto rollback_new; }
                        new_models.push_back(*mp);
                    }

                    // Delete old root + internal.
                    (void)::phy_engine::netlist::delete_model(nl, root.pos);
                    for(auto const mp : to_delete) { (void)::phy_engine::netlist::delete_model(nl, mp); }
                    changed = true;
                    continue;

                rollback_new:
                    for(auto const mp : new_models) { (void)::phy_engine::netlist::delete_model(nl, mp); }
                }

            }

            return changed;
        }

        [[nodiscard]] inline bool optimize_strash_in_pe_netlist(::phy_engine::netlist::netlist& nl,
                                                                ::std::vector<::phy_engine::model::node_t*> const& protected_nodes) noexcept
        {
            enum class kind : std::uint8_t
            {
                not_gate,
                and_gate,
                or_gate,
                xor_gate,
                xnor_gate,
                nand_gate,
                nor_gate,
                imp_gate,
                nimp_gate,
            };

            struct gate
            {
                kind k{};
                ::phy_engine::netlist::model_pos pos{};
                ::phy_engine::model::node_t* in0{};
                ::phy_engine::model::node_t* in1{};
                ::phy_engine::model::node_t* out{};
                ::phy_engine::model::pin const* out_pin{};
            };

            struct key
            {
                kind k{};
                ::phy_engine::model::node_t* a{};
                ::phy_engine::model::node_t* b{};
            };
            struct key_hash
            {
                std::size_t operator()(key const& x) const noexcept
                {
                    auto const mix = [](std::size_t h, std::size_t v) noexcept -> std::size_t {
                        return (h ^ (v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2)));
                    };
                    std::size_t h{};
                    h = mix(h, static_cast<std::size_t>(x.k));
                    h = mix(h, reinterpret_cast<std::size_t>(x.a));
                    h = mix(h, reinterpret_cast<std::size_t>(x.b));
                    return h;
                }
            };
            struct key_eq
            {
                bool operator()(key const& x, key const& y) const noexcept { return x.k == y.k && x.a == y.a && x.b == y.b; }
            };

            ::std::unordered_map<::phy_engine::model::node_t*, bool> protected_map{};
            protected_map.reserve(protected_nodes.size() * 2u + 1u);
            for(auto* n : protected_nodes) { protected_map.emplace(n, true); }
            auto const is_protected = [&](::phy_engine::model::node_t* n) noexcept -> bool { return protected_map.contains(n); };

            auto const fan = build_gate_opt_fanout(nl);

            auto canon_key = [](kind k, ::phy_engine::model::node_t* a, ::phy_engine::model::node_t* b) noexcept -> key
            {
                auto const commutative = (k == kind::and_gate || k == kind::or_gate || k == kind::xor_gate || k == kind::xnor_gate || k == kind::nand_gate ||
                                          k == kind::nor_gate);
                if(commutative && reinterpret_cast<std::uintptr_t>(a) > reinterpret_cast<std::uintptr_t>(b)) { ::std::swap(a, b); }
                return key{k, a, b};
            };

            auto classify = [&](::phy_engine::model::model_base const& mb,
                                ::phy_engine::netlist::model_pos pos) noexcept -> ::std::optional<gate>
            {
                if(mb.type != ::phy_engine::model::model_type::normal || mb.ptr == nullptr) { return ::std::nullopt; }
                auto const name = model_name_u8(mb);
                auto pv = mb.ptr->generate_pin_view();

                gate g{};
                g.pos = pos;

                if(name == u8"NOT")
                {
                    if(pv.size != 2) { return ::std::nullopt; }
                    g.k = kind::not_gate;
                    g.in0 = pv.pins[0].nodes;
                    g.out = pv.pins[1].nodes;
                    g.out_pin = __builtin_addressof(pv.pins[1]);
                    return g;
                }

                if(name == u8"AND" || name == u8"OR" || name == u8"XOR" || name == u8"XNOR" || name == u8"NAND" || name == u8"NOR" || name == u8"IMP" ||
                   name == u8"NIMP")
                {
                    if(pv.size != 3) { return ::std::nullopt; }
                    g.in0 = pv.pins[0].nodes;
                    g.in1 = pv.pins[1].nodes;
                    g.out = pv.pins[2].nodes;
                    g.out_pin = __builtin_addressof(pv.pins[2]);
                    if(name == u8"AND") { g.k = kind::and_gate; }
                    else if(name == u8"OR") { g.k = kind::or_gate; }
                    else if(name == u8"XOR") { g.k = kind::xor_gate; }
                    else if(name == u8"XNOR") { g.k = kind::xnor_gate; }
                    else if(name == u8"NAND") { g.k = kind::nand_gate; }
                    else if(name == u8"NOR") { g.k = kind::nor_gate; }
                    else if(name == u8"IMP") { g.k = kind::imp_gate; }
                    else { g.k = kind::nimp_gate; }
                    return g;
                }

                return ::std::nullopt;
            };

            auto move_consumers = [&](::phy_engine::model::node_t* from,
                                      ::phy_engine::model::node_t* to,
                                      ::phy_engine::model::pin const* from_driver_pin) noexcept -> bool
            {
                if(from == nullptr || to == nullptr || from_driver_pin == nullptr) { return false; }
                if(from == to) { return false; }
                if(is_protected(from)) { return false; }
                if(!has_unique_driver_pin(from, from_driver_pin, fan.pin_out)) { return false; }

                ::std::vector<::phy_engine::model::pin*> pins_to_move{};
                pins_to_move.reserve(from->pins.size());
                for(auto* p : from->pins)
                {
                    if(p == from_driver_pin) { continue; }
                    pins_to_move.push_back(p);
                }
                for(auto* p : pins_to_move)
                {
                    from->pins.erase(p);
                    p->nodes = to;
                    to->pins.insert(p);
                }
                return true;
            };

            ::std::unordered_map<key, gate, key_hash, key_eq> rep{};
            rep.reserve(1 << 14);

            bool changed{};
            for(std::size_t chunk_pos{}; chunk_pos < nl.models.size(); ++chunk_pos)
            {
                auto& blk = nl.models.index_unchecked(chunk_pos);
                for(std::size_t vec_pos{}; blk.begin + vec_pos < blk.curr; ++vec_pos)
                {
                    auto const& mb = blk.begin[vec_pos];
                    auto og = classify(mb, {vec_pos, chunk_pos});
                    if(!og || og->out == nullptr || og->out_pin == nullptr) { continue; }

                    key k{};
                    if(og->k == kind::not_gate)
                    {
                        if(og->in0 == nullptr) { continue; }
                        k = key{og->k, og->in0, nullptr};
                    }
                    else
                    {
                        if(og->in0 == nullptr || og->in1 == nullptr) { continue; }
                        k = canon_key(og->k, og->in0, og->in1);
                    }

                    auto it = rep.find(k);
                    if(it == rep.end())
                    {
                        rep.emplace(k, *og);
                        continue;
                    }

                    auto& r = it->second;
                    if(r.out == nullptr || r.out_pin == nullptr) { r = *og; continue; }

                    // Prefer keeping protected outputs. If both are protected, skip.
                    bool const r_prot = is_protected(r.out);
                    bool const g_prot = is_protected(og->out);
                    if(r_prot && g_prot) { continue; }

                    gate const* keep = __builtin_addressof(r);
                    gate const* drop = __builtin_addressof(*og);
                    if(!r_prot && g_prot)
                    {
                        keep = __builtin_addressof(*og);
                        drop = __builtin_addressof(r);
                    }

                    // Rewire consumers of `drop->out` to `keep->out`, then delete the dropped gate.
                    if(!move_consumers(drop->out, keep->out, drop->out_pin)) { continue; }

                    (void)::phy_engine::netlist::delete_model(nl, drop->pos);
                    changed = true;

                    if(keep == og.operator->()) { r = *og; }
                }
            }

            return changed;
        }

        [[nodiscard]] inline bool optimize_dce_in_pe_netlist(::phy_engine::netlist::netlist& nl,
                                                             ::std::vector<::phy_engine::model::node_t*> const& protected_nodes) noexcept
        {
            ::std::unordered_map<::phy_engine::model::node_t*, bool> protected_map{};
            protected_map.reserve(protected_nodes.size() * 2u + 1u);
            for(auto* n : protected_nodes) { protected_map.emplace(n, true); }
            auto const is_protected = [&](::phy_engine::model::node_t* n) noexcept -> bool { return protected_map.contains(n); };

            auto const fan = build_gate_opt_fanout(nl);

            bool changed{};
            for(std::size_t chunk_pos{}; chunk_pos < nl.models.size(); ++chunk_pos)
            {
                auto& blk = nl.models.index_unchecked(chunk_pos);
                for(std::size_t vec_pos{}; blk.begin + vec_pos < blk.curr; ++vec_pos)
                {
                    auto const& mb = blk.begin[vec_pos];
                    if(mb.type != ::phy_engine::model::model_type::normal || mb.ptr == nullptr) { continue; }

                    auto const name = model_name_u8(mb);
                    if(name == u8"OUTPUT" || name == u8"VERILOG_PORTS") { continue; }
                    if(name == u8"INPUT" && !mb.name.empty()) { continue; }  // keep named IO drivers

                    auto pv = mb.ptr->generate_pin_view();
                    bool any_out{};
                    bool all_unused{true};
                    for(std::size_t i{}; i < pv.size; ++i)
                    {
                        if(!is_output_pin(name, i, pv.size)) { continue; }
                        any_out = true;
                        auto* n = pv.pins[i].nodes;
                        if(n == nullptr) { continue; }
                        if(is_protected(n)) { all_unused = false; break; }
                        auto it = fan.consumer_count.find(n);
                        if(it != fan.consumer_count.end() && it->second != 0) { all_unused = false; break; }
                    }
                    if(!any_out || !all_unused) { continue; }

                    if(::phy_engine::netlist::delete_model(nl, vec_pos, chunk_pos)) { changed = true; }
                }
            }

            return changed;
        }

        struct qm_implicant
        {
            ::std::uint16_t value{};  // up to 8 vars
            ::std::uint16_t mask{};   // 1 => don't care on that bit
            bool combined{};
        };

        [[nodiscard]] inline std::size_t popcount16(::std::uint16_t x) noexcept
        {
            return static_cast<std::size_t>(__builtin_popcount(static_cast<unsigned>(x)));
        }

        [[nodiscard]] inline bool implicant_covers(qm_implicant const& imp, ::std::uint16_t m) noexcept
        {
            auto const keep = static_cast<::std::uint16_t>(~imp.mask);
            return static_cast<::std::uint16_t>(m & keep) == static_cast<::std::uint16_t>(imp.value & keep);
        }

        [[nodiscard]] inline std::size_t implicant_literals(qm_implicant const& imp, std::size_t var_count) noexcept
        {
            auto const keep = static_cast<::std::uint16_t>(~imp.mask);
            auto const masked = static_cast<::std::uint16_t>(keep & static_cast<::std::uint16_t>((1u << var_count) - 1u));
            return popcount16(masked);
        }

        [[nodiscard]] inline ::std::vector<qm_implicant>
            qm_prime_implicants(::std::vector<::std::uint16_t> const& on,
                                ::std::vector<::std::uint16_t> const& dc,
                                std::size_t var_count) noexcept
        {
            ::std::vector<qm_implicant> current{};
            current.reserve(on.size() + dc.size());
            for(auto const m : on) { current.push_back(qm_implicant{m, 0u, false}); }
            for(auto const m : dc) { current.push_back(qm_implicant{m, 0u, false}); }

            ::std::vector<qm_implicant> primes{};
            primes.reserve(256);

            auto const var_mask = static_cast<::std::uint16_t>((var_count >= 16) ? 0xFFFFu : ((1u << var_count) - 1u));

            for(;;)
            {
                for(auto& x : current) { x.combined = false; }

                ::std::vector<qm_implicant> next{};
                next.reserve(current.size());

                for(std::size_t i{}; i < current.size(); ++i)
                {
                    for(std::size_t j{i + 1}; j < current.size(); ++j)
                    {
                        auto const& a = current[i];
                        auto const& b = current[j];
                        if(a.mask != b.mask) { continue; }
                        auto const diff = static_cast<::std::uint16_t>((a.value ^ b.value) & static_cast<::std::uint16_t>(~a.mask) & var_mask);
                        if(popcount16(diff) != 1u) { continue; }

                        qm_implicant c{};
                        c.mask = static_cast<::std::uint16_t>(a.mask | diff);
                        c.value = static_cast<::std::uint16_t>(a.value & static_cast<::std::uint16_t>(~diff));
                        c.combined = false;

                        current[i].combined = true;
                        current[j].combined = true;

                        bool dup{};
                        for(auto const& e : next)
                        {
                            if(e.value == c.value && e.mask == c.mask) { dup = true; break; }
                        }
                        if(!dup) { next.push_back(c); }
                    }
                }

                for(auto const& a : current)
                {
                    if(a.combined) { continue; }
                    bool dup{};
                    for(auto const& p : primes)
                    {
                        if(p.value == a.value && p.mask == a.mask) { dup = true; break; }
                    }
                    if(!dup) { primes.push_back(a); }
                }

                if(next.empty()) { break; }
                current = ::std::move(next);
            }

            return primes;
        }

        struct qm_solution
        {
            ::std::vector<std::size_t> pick{};
            std::size_t cost{static_cast<std::size_t>(-1)};
        };

        [[nodiscard]] inline std::size_t qm_cover_cost(::std::vector<qm_implicant> const& primes,
                                                       ::std::vector<std::size_t> const& pick,
                                                       std::size_t var_count) noexcept
        {
            // Cost model (2-input gates only): NOTs for negated literals + AND trees + OR tree.
            bool neg_used[8]{};
            std::size_t terms{};
            std::size_t and_cost{};

            for(auto const idx : pick)
            {
                if(idx >= primes.size()) { continue; }
                auto const& imp = primes[idx];
                std::size_t lits{};
                for(std::size_t v{}; v < var_count; ++v)
                {
                    if((imp.mask >> v) & 1u) { continue; }
                    ++lits;
                    bool const bit_is_1 = ((imp.value >> v) & 1u) != 0;
                    if(!bit_is_1) { neg_used[v] = true; }
                }
                ++terms;
                if(lits >= 2) { and_cost += (lits - 1u); }
            }

            std::size_t not_cost{};
            for(std::size_t v{}; v < var_count; ++v)
            {
                if(neg_used[v]) { ++not_cost; }
            }

            std::size_t or_cost{};
            if(terms >= 2) { or_cost = terms - 1u; }

            return not_cost + and_cost + or_cost;
        }

        [[nodiscard]] inline qm_solution qm_exact_minimum_cover(::std::vector<qm_implicant> const& primes,
                                                                ::std::vector<::std::uint16_t> const& on,
                                                                std::size_t var_count) noexcept
        {
            // Branch-and-bound exact cover on ON-set minterms.
            qm_solution best{};

            if(on.empty())
            {
                best.cost = 0;
                return best;
            }

            // Precompute which primes cover each minterm.
            ::std::vector<::std::vector<std::size_t>> covers{};
            covers.resize(on.size());
            for(std::size_t mi{}; mi < on.size(); ++mi)
            {
                auto const m = on[mi];
                for(std::size_t pi{}; pi < primes.size(); ++pi)
                {
                    if(implicant_covers(primes[pi], m)) { covers[mi].push_back(pi); }
                }
                if(covers[mi].empty())
                {
                    // Shouldn't happen, but fail safe.
                    best.cost = static_cast<std::size_t>(-1);
                    return best;
                }
            }

            // Essential prime implicants.
            ::std::vector<bool> covered{};
            covered.resize(on.size());
            ::std::vector<std::size_t> picked{};
            picked.reserve(primes.size());

            bool changed{true};
            while(changed)
            {
                changed = false;
                for(std::size_t mi{}; mi < on.size(); ++mi)
                {
                    if(covered[mi]) { continue; }
                    auto const& cand = covers[mi];
                    std::size_t alive{};
                    std::size_t last{};
                    for(auto const pi : cand)
                    {
                        bool already{};
                        for(auto const ppi : picked)
                        {
                            if(ppi == pi) { already = true; break; }
                        }
                        if(already) { alive = 2; break; }  // already covered by picked
                        ++alive;
                        last = pi;
                        if(alive > 1) { break; }
                    }
                    if(alive == 1)
                    {
                        picked.push_back(last);
                        changed = true;
                        // mark all minterms covered by this implicant
                        for(std::size_t mj{}; mj < on.size(); ++mj)
                        {
                            if(covered[mj]) { continue; }
                            if(implicant_covers(primes[last], on[mj])) { covered[mj] = true; }
                        }
                    }
                }
            }

            auto const base_cost = qm_cover_cost(primes, picked, var_count);
            best.pick = picked;
            best.cost = base_cost;

            auto all_covered = [&]() noexcept -> bool {
                for(auto const v : covered)
                {
                    if(!v) { return false; }
                }
                return true;
            };
            if(all_covered()) { return best; }

            // Map: prime index -> whether selected.
            ::std::vector<bool> selected{};
            selected.resize(primes.size());
            for(auto const pi : picked)
            {
                if(pi < selected.size()) { selected[pi] = true; }
            }

            // Recursive search.
            auto dfs = [&](auto&& self, ::std::vector<bool>& cov, ::std::vector<bool>& sel, ::std::vector<std::size_t>& pick) noexcept -> void
            {
                if(best.cost != static_cast<std::size_t>(-1))
                {
                    auto const cost = qm_cover_cost(primes, pick, var_count);
                    if(cost >= best.cost) { return; }
                }

                std::size_t next_m{};
                bool found{};
                for(std::size_t mi{}; mi < on.size(); ++mi)
                {
                    if(!cov[mi])
                    {
                        next_m = mi;
                        found = true;
                        break;
                    }
                }
                if(!found)
                {
                    auto const cost = qm_cover_cost(primes, pick, var_count);
                    if(cost < best.cost)
                    {
                        best.pick = pick;
                        best.cost = cost;
                    }
                    return;
                }

                auto const& options = covers[next_m];
                for(auto const pi : options)
                {
                    if(pi >= primes.size()) { continue; }
                    bool newly_added{false};
                    if(!sel[pi])
                    {
                        sel[pi] = true;
                        pick.push_back(pi);
                        newly_added = true;
                    }

                    ::std::vector<std::size_t> flipped{};
                    flipped.reserve(on.size());
                    for(std::size_t mj{}; mj < on.size(); ++mj)
                    {
                        if(cov[mj]) { continue; }
                        if(implicant_covers(primes[pi], on[mj]))
                        {
                            cov[mj] = true;
                            flipped.push_back(mj);
                        }
                    }

                    self(self, cov, sel, pick);

                    for(auto const mj : flipped) { cov[mj] = false; }
                    if(newly_added)
                    {
                        pick.pop_back();
                        sel[pi] = false;
                    }
                }
            };

            dfs(dfs, covered, selected, picked);
            return best;
        }

        [[nodiscard]] inline qm_solution qm_greedy_cover(::std::vector<qm_implicant> const& primes,
                                                         ::std::vector<::std::uint16_t> const& on,
                                                         std::size_t var_count) noexcept
        {
            qm_solution sol{};
            if(on.empty())
            {
                sol.cost = 0;
                return sol;
            }

            auto const blocks = (on.size() + 63u) / 64u;
            ::std::vector<::std::vector<::std::uint64_t>> cov{};
            cov.resize(primes.size());
            for(auto& v : cov) { v.assign(blocks, 0ull); }

            ::std::vector<::std::size_t> prime_cost{};
            prime_cost.resize(primes.size());

            for(std::size_t pi{}; pi < primes.size(); ++pi)
            {
                prime_cost[pi] = qm_cover_cost(primes, ::std::vector<std::size_t>{pi}, var_count);
                for(std::size_t mi{}; mi < on.size(); ++mi)
                {
                    if(implicant_covers(primes[pi], on[mi]))
                    {
                        cov[pi][mi / 64u] |= (1ull << (mi % 64u));
                    }
                }
            }

            auto popcount64 = [](std::uint64_t x) noexcept -> std::size_t { return static_cast<std::size_t>(__builtin_popcountll(x)); };

            ::std::vector<std::uint64_t> uncovered{};
            uncovered.assign(blocks, ~0ull);
            if(on.size() % 64u)
            {
                auto const rem = on.size() % 64u;
                uncovered.back() = (rem == 64u) ? ~0ull : ((1ull << rem) - 1ull);
            }

            // Essential implicants.
            ::std::vector<std::size_t> cover_count{};
            ::std::vector<std::size_t> last_prime{};
            cover_count.assign(on.size(), 0u);
            last_prime.assign(on.size(), static_cast<std::size_t>(-1));

            for(std::size_t pi{}; pi < primes.size(); ++pi)
            {
                for(std::size_t mi{}; mi < on.size(); ++mi)
                {
                    if((cov[pi][mi / 64u] >> (mi % 64u)) & 1ull)
                    {
                        ++cover_count[mi];
                        last_prime[mi] = pi;
                    }
                }
            }

            ::std::vector<bool> picked{};
            picked.assign(primes.size(), false);

            auto apply_pick = [&](std::size_t pi) noexcept
            {
                if(pi >= primes.size() || picked[pi]) { return; }
                picked[pi] = true;
                sol.pick.push_back(pi);
                for(std::size_t b{}; b < blocks; ++b) { uncovered[b] &= ~cov[pi][b]; }
            };

            bool changed{true};
            while(changed)
            {
                changed = false;
                for(std::size_t mi{}; mi < on.size(); ++mi)
                {
                    if(((uncovered[mi / 64u] >> (mi % 64u)) & 1ull) == 0ull) { continue; }
                    if(cover_count[mi] == 1u && last_prime[mi] != static_cast<std::size_t>(-1))
                    {
                        apply_pick(last_prime[mi]);
                        changed = true;
                    }
                }
            }

            auto any_uncovered = [&]() noexcept -> bool
            {
                for(auto const w : uncovered)
                {
                    if(w) { return true; }
                }
                return false;
            };

            while(any_uncovered())
            {
                std::size_t best_pi{static_cast<std::size_t>(-1)};
                std::size_t best_gain{};
                std::size_t best_cost{static_cast<std::size_t>(-1)};
                std::int64_t best_score{::std::numeric_limits<std::int64_t>::min()};

                for(std::size_t pi{}; pi < primes.size(); ++pi)
                {
                    if(picked[pi]) { continue; }

                    std::size_t gain{};
                    for(std::size_t b{}; b < blocks; ++b)
                    {
                        gain += popcount64(cov[pi][b] & uncovered[b]);
                    }
                    if(gain == 0) { continue; }

                    auto const cost = prime_cost[pi];
                    // Score: prioritize coverage, then lower cost.
                    auto const score = static_cast<std::int64_t>(gain) * 64 - static_cast<std::int64_t>(cost);
                    if(score > best_score || (score == best_score && (gain > best_gain || (gain == best_gain && cost < best_cost))))
                    {
                        best_score = score;
                        best_pi = pi;
                        best_gain = gain;
                        best_cost = cost;
                    }
                }

                if(best_pi == static_cast<std::size_t>(-1))
                {
                    sol.cost = static_cast<std::size_t>(-1);
                    return sol;
                }
                apply_pick(best_pi);
            }

            sol.cost = qm_cover_cost(primes, sol.pick, var_count);
            return sol;
        }

        [[nodiscard]] inline bool optimize_qm_two_level_minimize_in_pe_netlist(::phy_engine::netlist::netlist& nl,
                                                                               ::std::vector<::phy_engine::model::node_t*> const& protected_nodes,
                                                                               pe_synth_options const& opt) noexcept
        {
            // Exact Quine–McCluskey minimization on small binary cones (<=6 vars, <=32 gates).
            // Only safe when the cone's internal nets have fanout=1 (exclusive to the cone).
            constexpr std::size_t exact_vars = 6;
            constexpr std::size_t max_vars = 10;
            constexpr std::size_t max_gates = 64;
            constexpr std::size_t max_primes = 4096;

            ::std::unordered_map<::phy_engine::model::node_t*, bool> protected_map{};
            protected_map.reserve(protected_nodes.size() * 2u + 1u);
            for(auto* n : protected_nodes) { protected_map.emplace(n, true); }
            auto const is_protected = [&](::phy_engine::model::node_t* n) noexcept -> bool { return protected_map.contains(n); };

            auto const fan = build_gate_opt_fanout(nl);

            enum class gkind : std::uint8_t
            {
                not_gate,
                and_gate,
                or_gate,
                xor_gate,
                xnor_gate,
                nand_gate,
                nor_gate,
                imp_gate,
                nimp_gate,
            };
            struct gate
            {
                gkind k{};
                ::phy_engine::netlist::model_pos pos{};
                ::phy_engine::model::node_t* in0{};
                ::phy_engine::model::node_t* in1{};
                ::phy_engine::model::node_t* out{};
            };

            ::std::unordered_map<::phy_engine::model::node_t*, gate> gate_by_out{};
            gate_by_out.reserve(1 << 14);

            auto classify = [&](::phy_engine::model::model_base const& mb,
                                ::phy_engine::netlist::model_pos pos) noexcept -> ::std::optional<gate>
            {
                if(mb.type != ::phy_engine::model::model_type::normal || mb.ptr == nullptr) { return ::std::nullopt; }
                auto const name = model_name_u8(mb);
                auto pv = mb.ptr->generate_pin_view();

                gate g{};
                g.pos = pos;
                if(name == u8"NOT")
                {
                    if(pv.size != 2) { return ::std::nullopt; }
                    g.k = gkind::not_gate;
                    g.in0 = pv.pins[0].nodes;
                    g.out = pv.pins[1].nodes;
                    return g;
                }

                if(name == u8"AND" || name == u8"OR" || name == u8"XOR" || name == u8"XNOR" || name == u8"NAND" || name == u8"NOR" ||
                   name == u8"IMP" || name == u8"NIMP")
                {
                    if(pv.size != 3) { return ::std::nullopt; }
                    g.in0 = pv.pins[0].nodes;
                    g.in1 = pv.pins[1].nodes;
                    g.out = pv.pins[2].nodes;
                    if(name == u8"AND") { g.k = gkind::and_gate; }
                    else if(name == u8"OR") { g.k = gkind::or_gate; }
                    else if(name == u8"XOR") { g.k = gkind::xor_gate; }
                    else if(name == u8"XNOR") { g.k = gkind::xnor_gate; }
                    else if(name == u8"NAND") { g.k = gkind::nand_gate; }
                    else if(name == u8"NOR") { g.k = gkind::nor_gate; }
                    else if(name == u8"IMP") { g.k = gkind::imp_gate; }
                    else { g.k = gkind::nimp_gate; }
                    return g;
                }

                return ::std::nullopt;
            };

            for(std::size_t chunk_pos{}; chunk_pos < nl.models.size(); ++chunk_pos)
            {
                auto& blk = nl.models.index_unchecked(chunk_pos);
                for(std::size_t vec_pos{}; blk.begin + vec_pos < blk.curr; ++vec_pos)
                {
                    auto const& mb = blk.begin[vec_pos];
                    auto og = classify(mb, {vec_pos, chunk_pos});
                    if(!og || og->out == nullptr) { continue; }
                    gate_by_out.emplace(og->out, *og);
                }
            }

            auto eval_gate = [&](auto&& self,
                                 ::phy_engine::model::node_t* node,
                                 ::std::unordered_map<::phy_engine::model::node_t*, ::phy_engine::model::digital_node_statement_t> const& leaf_val,
                                 ::std::unordered_map<::phy_engine::model::node_t*, ::phy_engine::model::digital_node_statement_t>& memo) noexcept
                -> ::phy_engine::model::digital_node_statement_t
            {
                if(node == nullptr) { return ::phy_engine::model::digital_node_statement_t::indeterminate_state; }
                if(auto it = memo.find(node); it != memo.end()) { return it->second; }
                if(auto it = leaf_val.find(node); it != leaf_val.end())
                {
                    memo.emplace(node, it->second);
                    return it->second;
                }
                auto itg = gate_by_out.find(node);
                if(itg == gate_by_out.end())
                {
                    memo.emplace(node, ::phy_engine::model::digital_node_statement_t::indeterminate_state);
                    return ::phy_engine::model::digital_node_statement_t::indeterminate_state;
                }
                auto const& g = itg->second;
                using dns = ::phy_engine::model::digital_node_statement_t;
                dns a = self(self, g.in0, leaf_val, memo);
                dns b = self(self, g.in1, leaf_val, memo);
                dns r{};
                switch(g.k)
                {
                    case gkind::not_gate: r = ~a; break;
                    case gkind::and_gate: r = (a & b); break;
                    case gkind::or_gate: r = (a | b); break;
                    case gkind::xor_gate: r = (a ^ b); break;
                    case gkind::xnor_gate: r = ~(a ^ b); break;
                    case gkind::nand_gate: r = ~(a & b); break;
                    case gkind::nor_gate: r = ~(a | b); break;
                    case gkind::imp_gate: r = ((~a) | b); break;
                    case gkind::nimp_gate: r = (a & (~b)); break;
                    default: r = dns::indeterminate_state; break;
                }
                memo.emplace(node, r);
                return r;
            };

            auto make_not = [&](::phy_engine::model::node_t* in) noexcept -> ::phy_engine::model::node_t*
            {
                if(in == nullptr) { return nullptr; }
                auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::NOT{});
                (void)pos;
                if(m == nullptr) { return nullptr; }
                auto& out_ref = ::phy_engine::netlist::create_node(nl);
                auto* out = __builtin_addressof(out_ref);
                if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *in) || !::phy_engine::netlist::add_to_node(nl, *m, 1, *out))
                {
                    return nullptr;
                }
                return out;
            };
            auto make_and = [&](::phy_engine::model::node_t* a, ::phy_engine::model::node_t* b) noexcept -> ::phy_engine::model::node_t*
            {
                if(a == nullptr || b == nullptr) { return nullptr; }
                auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::AND{});
                (void)pos;
                if(m == nullptr) { return nullptr; }
                auto& out_ref = ::phy_engine::netlist::create_node(nl);
                auto* out = __builtin_addressof(out_ref);
                if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *a) || !::phy_engine::netlist::add_to_node(nl, *m, 1, *b) ||
                   !::phy_engine::netlist::add_to_node(nl, *m, 2, *out))
                {
                    return nullptr;
                }
                return out;
            };
            auto make_or = [&](::phy_engine::model::node_t* a, ::phy_engine::model::node_t* b) noexcept -> ::phy_engine::model::node_t*
            {
                if(a == nullptr || b == nullptr) { return nullptr; }
                auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OR{});
                (void)pos;
                if(m == nullptr) { return nullptr; }
                auto& out_ref = ::phy_engine::netlist::create_node(nl);
                auto* out = __builtin_addressof(out_ref);
                if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *a) || !::phy_engine::netlist::add_to_node(nl, *m, 1, *b) ||
                   !::phy_engine::netlist::add_to_node(nl, *m, 2, *out))
                {
                    return nullptr;
                }
                return out;
            };
            auto make_yes = [&](::phy_engine::model::node_t* in, ::phy_engine::model::node_t* out) noexcept -> bool
            {
                if(in == nullptr || out == nullptr) { return false; }
                auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::YES{});
                (void)pos;
                if(m == nullptr) { return false; }
                return ::phy_engine::netlist::add_to_node(nl, *m, 0, *in) && ::phy_engine::netlist::add_to_node(nl, *m, 1, *out);
            };

            auto rewire_consumers = [&](::phy_engine::model::node_t* from, ::phy_engine::model::node_t* to) noexcept -> bool
            {
                if(from == nullptr || to == nullptr || from == to) { return false; }
                if(is_protected(from)) { return false; }
                ::std::vector<::phy_engine::model::pin*> pins_to_move{};
                pins_to_move.reserve(from->pins.size());
                for(auto* p : from->pins) { pins_to_move.push_back(p); }
                for(auto* p : pins_to_move)
                {
                    from->pins.erase(p);
                    p->nodes = to;
                    to->pins.insert(p);
                }
                return true;
            };

            bool changed{};

            // Collect candidate roots from current snapshot (avoid iterator invalidation).
            ::std::vector<::phy_engine::model::node_t*> roots{};
            roots.reserve(gate_by_out.size());
            for(auto const& kv : gate_by_out) { roots.push_back(kv.first); }

            for(auto* root : roots)
            {
                if(root == nullptr) { continue; }
                auto it_drv = fan.driver_count.find(root);
                if(it_drv == fan.driver_count.end() || it_drv->second != 1) { continue; }
                auto it_root = gate_by_out.find(root);
                if(it_root == gate_by_out.end()) { continue; }

                ::std::vector<::phy_engine::netlist::model_pos> to_delete{};
                to_delete.reserve(max_gates);
                ::std::vector<::phy_engine::model::node_t*> leaves{};
                leaves.reserve(max_vars);
                ::std::unordered_map<::phy_engine::model::node_t*, std::size_t> leaf_index{};
                leaf_index.reserve(max_vars * 2u);

                bool ok{true};
                ::std::unordered_map<::phy_engine::model::node_t*, bool> visited{};
                visited.reserve(max_gates * 2u);

                auto dfs = [&](auto&& self, ::phy_engine::model::node_t* n) noexcept -> void
                {
                    if(!ok || n == nullptr) { return; }
                    if(visited.contains(n)) { return; }
                    visited.emplace(n, true);

                    auto itg = gate_by_out.find(n);
                    if(itg == gate_by_out.end())
                    {
                        if(!leaf_index.contains(n))
                        {
                            if(leaves.size() >= max_vars) { ok = false; return; }
                            leaf_index.emplace(n, leaves.size());
                            leaves.push_back(n);
                        }
                        return;
                    }

                    auto const& g = itg->second;
                    if(n != root)
                    {
                        if(is_protected(n)) { ok = false; return; }
                        auto itc = fan.consumer_count.find(n);
                        if(itc == fan.consumer_count.end() || itc->second != 1) { ok = false; return; }
                        auto itd = fan.driver_count.find(n);
                        if(itd == fan.driver_count.end() || itd->second != 1) { ok = false; return; }
                    }

                    if(to_delete.size() >= max_gates) { ok = false; return; }
                    to_delete.push_back(g.pos);

                    self(self, g.in0);
                    if(g.k != gkind::not_gate) { self(self, g.in1); }
                };

                dfs(dfs, root);
                if(!ok) { continue; }
                if(leaves.empty()) { continue; }

                // Build ON/DC sets by truth-table enumeration on leaf vars (binary only).
                ::std::vector<::std::uint16_t> on{};
                ::std::vector<::std::uint16_t> dc{};
                on.reserve(1u << leaves.size());

                for(::std::uint16_t m{}; m < static_cast<::std::uint16_t>(1u << leaves.size()); ++m)
                {
                    ::std::unordered_map<::phy_engine::model::node_t*, ::phy_engine::model::digital_node_statement_t> leaf_val{};
                    leaf_val.reserve(leaves.size() * 2u);
                    for(std::size_t i{}; i < leaves.size(); ++i)
                    {
                        auto const bit = ((m >> i) & 1u) != 0;
                        leaf_val.emplace(leaves[i], bit ? ::phy_engine::model::digital_node_statement_t::true_state
                                                        : ::phy_engine::model::digital_node_statement_t::false_state);
                    }
                    ::std::unordered_map<::phy_engine::model::node_t*, ::phy_engine::model::digital_node_statement_t> memo{};
                    memo.reserve(to_delete.size() * 2u + 8u);
                    auto const r = eval_gate(eval_gate, root, leaf_val, memo);
                    if(r == ::phy_engine::model::digital_node_statement_t::true_state) { on.push_back(m); }
                    else if(r == ::phy_engine::model::digital_node_statement_t::false_state) { /* off */ }
                    else
                    {
                        // Conservative: treat X/Z as don't-care only if user opted out of X-propagation.
                        // (This is a best-effort DC-set exploitation.)
                        if(opt.assume_binary_inputs) { dc.push_back(m); }
                        else { ok = false; break; }
                    }
                }
                if(!ok) { continue; }

                auto const primes = qm_prime_implicants(on, dc, leaves.size());
                if(primes.size() > max_primes) { continue; }
                auto sol = (leaves.size() <= exact_vars) ? qm_exact_minimum_cover(primes, on, leaves.size())
                                                         : qm_greedy_cover(primes, on, leaves.size());
                if(sol.cost == static_cast<std::size_t>(-1)) { continue; }

                // Normalize deletion list and only apply if we estimate a gate-count win.
                ::std::sort(to_delete.begin(),
                            to_delete.end(),
                            [](auto const& a, auto const& b) noexcept {
                                if(a.chunk_pos != b.chunk_pos) { return a.chunk_pos > b.chunk_pos; }
                                return a.vec_pos > b.vec_pos;
                            });
                to_delete.erase(::std::unique(to_delete.begin(),
                                              to_delete.end(),
                                              [](auto const& a, auto const& b) noexcept { return a.chunk_pos == b.chunk_pos && a.vec_pos == b.vec_pos; }),
                                to_delete.end());

                if(sol.cost >= to_delete.size()) { continue; }

                // Delete old cone (descending order).
                for(auto const& mp : to_delete) { (void)::phy_engine::netlist::delete_model(nl, mp); }

                // Rebuild minimized SOP.
                auto* const0 = find_or_make_const_node(nl, ::phy_engine::model::digital_node_statement_t::false_state);
                auto* const1 = find_or_make_const_node(nl, ::phy_engine::model::digital_node_statement_t::true_state);
                if(const0 == nullptr || const1 == nullptr) { continue; }

                if(on.empty())
                {
                    (void)make_yes(const0, root);
                    changed = true;
                    continue;
                }
                if(on.size() == (1u << leaves.size()))
                {
                    (void)make_yes(const1, root);
                    changed = true;
                    continue;
                }

                // Cache negations used.
                ::phy_engine::model::node_t* neg[8]{};
                for(std::size_t v{}; v < leaves.size(); ++v)
                {
                    neg[v] = nullptr;
                }
                for(auto const pi : sol.pick)
                {
                    if(pi >= primes.size()) { continue; }
                    auto const& imp = primes[pi];
                    for(std::size_t v{}; v < leaves.size(); ++v)
                    {
                        if((imp.mask >> v) & 1u) { continue; }
                        bool const bit_is_1 = ((imp.value >> v) & 1u) != 0;
                        if(bit_is_1) { continue; }
                        if(neg[v] == nullptr) { neg[v] = make_not(leaves[v]); }
                    }
                }

                ::std::vector<::phy_engine::model::node_t*> terms{};
                terms.reserve(sol.pick.size());
                for(auto const pi : sol.pick)
                {
                    if(pi >= primes.size()) { continue; }
                    auto const& imp = primes[pi];

                    ::phy_engine::model::node_t* term = nullptr;
                    std::size_t lits{};
                    for(std::size_t v{}; v < leaves.size(); ++v)
                    {
                        if((imp.mask >> v) & 1u) { continue; }
                        bool const bit_is_1 = ((imp.value >> v) & 1u) != 0;
                        auto* lit = bit_is_1 ? leaves[v] : neg[v];
                        if(lit == nullptr) { ok = false; break; }
                        ++lits;
                        if(term == nullptr) { term = lit; }
                        else { term = make_and(term, lit); }
                    }
                    if(!ok) { break; }
                    if(lits == 0) { term = const1; }  // covers all
                    if(term != nullptr) { terms.push_back(term); }
                }
                if(!ok || terms.empty()) { continue; }

                ::phy_engine::model::node_t* out = terms[0];
                for(std::size_t i{1}; i < terms.size(); ++i)
                {
                    out = make_or(out, terms[i]);
                    if(out == nullptr) { ok = false; break; }
                }
                if(!ok || out == nullptr) { continue; }

                if(out == root)
                {
                    changed = true;
                    continue;
                }

                // Keep protected port nodes intact: use YES buffer if output is not directly driven by a newly created gate.
                // Otherwise, connect final output by a YES to ensure root has a driver.
                if(is_protected(root))
                {
                    (void)make_yes(out, root);
                }
                else
                {
                    // If root is not protected, alias by moving consumers to `out`.
                    (void)rewire_consumers(root, out);
                }

                changed = true;
            }

            return changed;
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

        auto const lvl = static_cast<::std::uint8_t>(opt.opt_level > 3 ? 3 : opt.opt_level);

        bool const do_wires = opt.optimize_wires || (lvl >= 1);
        bool const do_mul2 = opt.optimize_mul2 || (lvl >= 2);
        bool const do_adders = opt.optimize_adders || (lvl >= 2);

        bool const do_fuse_inverters = (lvl >= 1);
        bool const do_strash = (lvl >= 1);
        bool const do_dce = (lvl >= 1);
        bool const do_factoring = (lvl >= 2);
        bool const do_qm = (lvl >= 3);
        bool const do_input_inv_map = (lvl >= 2);
        bool const do_xor_rewrite = (lvl >= 2);
        bool const do_double_not = (lvl >= 1);
        bool const do_constprop = (lvl >= 1);
        bool const do_absorption = (lvl >= 2);
        bool const do_flatten = (lvl >= 2);
        bool const do_binary_simplify = (lvl >= 2) && opt.assume_binary_inputs;

        auto run_once = [&]() noexcept -> void {
            if(do_wires) { details::optimize_eliminate_yes_buffers(nl, top_port_nodes); }
            if(do_mul2) { details::optimize_mul2_in_pe_netlist(nl); }
            if(do_adders) { details::optimize_adders_in_pe_netlist(nl); }

            if(do_constprop) { (void)details::optimize_constant_propagation_in_pe_netlist(nl, top_port_nodes, opt); }
            if(do_factoring) { (void)details::optimize_factor_common_terms_in_pe_netlist(nl, top_port_nodes); }
            if(do_absorption) { (void)details::optimize_absorption_in_pe_netlist(nl, top_port_nodes); }
            if(do_xor_rewrite) { (void)details::optimize_rewrite_xor_xnor_in_pe_netlist(nl, top_port_nodes); }
            if(do_flatten) { (void)details::optimize_flatten_associative_and_or_in_pe_netlist(nl, top_port_nodes, opt); }
            if(do_binary_simplify) { (void)details::optimize_binary_complement_simplify_in_pe_netlist(nl, top_port_nodes); }
            if(do_binary_simplify) { (void)details::optimize_constant_propagation_in_pe_netlist(nl, top_port_nodes, opt); }
            if(do_qm) { (void)details::optimize_qm_two_level_minimize_in_pe_netlist(nl, top_port_nodes, opt); }
            if(do_input_inv_map) { (void)details::optimize_push_input_inverters_in_pe_netlist(nl, top_port_nodes); }
            if(do_double_not) { (void)details::optimize_eliminate_double_not_in_pe_netlist(nl, top_port_nodes); }
            if(do_fuse_inverters) { (void)details::optimize_fuse_inverters_in_pe_netlist(nl, top_port_nodes); }
            if(do_strash) { (void)details::optimize_strash_in_pe_netlist(nl, top_port_nodes); }
            if(do_dce) { (void)details::optimize_dce_in_pe_netlist(nl, top_port_nodes); }
        };

        if(lvl >= 3)
        {
            // Best-effort fixpoint-ish pipeline for gate count reduction.
            for(::std::size_t iter{}; iter < 8u; ++iter)
            {
                auto const before = ::phy_engine::netlist::get_num_of_model(nl);
                run_once();
                auto const after = ::phy_engine::netlist::get_num_of_model(nl);
                if(after >= before) { break; }
            }
        }
        else
        {
            run_once();
        }
        return ctx.ok();
    }
}  // namespace phy_engine::verilog::digital
