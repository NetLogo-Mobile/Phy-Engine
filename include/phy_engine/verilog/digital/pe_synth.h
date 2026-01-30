#pragma once

#include <algorithm>
#include <array>
#include <charconv>
#include <chrono>
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <limits>
#include <random>
#include <optional>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <fast_io/fast_io_dsal/string.h>

#include "../../netlist/operation.h"

// Header-only CUDA hygiene:
// - Normal (non-CUDA) builds must not require CUDA headers.
// - Provide stable macros that can be tested from .cpp even when CUDA is not present.
// Note: `__CUDA__` is not a standard macro; we define it (0/1) only if it doesn't exist.
#if !defined(__CUDA__)
    #if defined(__CUDACC__) || defined(__CUDA_ARCH__)
        #define __CUDA__ 1
    #else
        #define __CUDA__ 0
    #endif
#endif

// Build-time switch for pe_synth CUDA backend.
#if defined(PHY_ENGINE_ENABLE_CUDA_PE_SYNTH)
    #define PHY_ENGINE_PE_SYNTH_CUDA 1
#else
    #define PHY_ENGINE_PE_SYNTH_CUDA 0
#endif

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
    struct pe_synth_report;

    struct pe_synth_options
    {
        enum class omax_cost_model : ::std::uint8_t
        {
            gate_count,          // legacy: count primitive gate instances (includes YES)
            weighted_gate_count  // approximate "area": weighted sum of primitive instances (includes YES by default)
        };

        enum class two_level_cost_model : ::std::uint8_t
        {
            gate_count,     // approximate 2-input gate count (NOT/AND/OR), with shared NOTs per cone
            literal_count,  // SOP literal count (sum of specified vars across cubes)
        };

        struct gate_cost_weights
        {
            ::std::uint16_t not_w{1};
            ::std::uint16_t and_w{1};
            ::std::uint16_t or_w{1};
            ::std::uint16_t xor_w{1};
            ::std::uint16_t xnor_w{1};
            ::std::uint16_t nand_w{1};
            ::std::uint16_t nor_w{1};
            ::std::uint16_t imp_w{1};
            ::std::uint16_t nimp_w{1};
            ::std::uint16_t yes_w{1};
            ::std::uint16_t case_eq_w{1};
            ::std::uint16_t is_unknown_w{1};
        };

        struct two_level_cost_weights
        {
            ::std::uint16_t not_w{1};
            ::std::uint16_t and_w{1};
            ::std::uint16_t or_w{1};
        };

        bool allow_inout{false};
        bool allow_multi_driver{false};
        bool support_always_comb{true};
        bool support_always_ff{true};
        bool assume_binary_inputs{false};                // treat X/Z as absent: `is_unknown(...)` folds to 0, dropping X-propagation mux networks
        // Optimization level:
        // - 0=O0, 1=O1, 2=O2
        // - 3=O3 (fast medium-strength: bounded rewrites/resub/sweep; no QM/techmap/decompose fixpoint)
        // - 4=O4 (strong: full O4 fixpoint pipeline; historically this was "O3")
        // - 5=Omax (multi-start search over O4 pipeline; best-so-far kept)
        ::std::uint8_t opt_level{0};
        ::std::size_t omax_timeout_ms{0};                // Omax only: 0 disables wall-clock budget
        ::std::size_t omax_max_iter{32};                 // Omax only: max restarts/tries (0 disables Omax loop)
        bool omax_randomize{false};                      // Omax only: enable randomized heuristic variants (deterministic if false)
        ::std::uint64_t omax_rand_seed{1};               // Omax only: RNG seed for randomized variants
        bool omax_allow_regress{false};                  // Omax only: if false, only accept strictly better cost solutions
        bool omax_verify{false};                         // Omax only: verify candidate netlists against the pre-opt reference (best-effort, combinational only)
        ::std::size_t omax_verify_exact_max_inputs{12};  // if input count <= this, do an exhaustive equivalence check
        ::std::size_t omax_verify_random_vectors{256};   // otherwise, random-vector regression guard
        ::std::uint64_t omax_verify_seed{1};             // RNG seed for verification vectors
        omax_cost_model omax_cost{omax_cost_model::gate_count};
        gate_cost_weights omax_gate_weights{};

        using omax_dump_best_cb_t = void (*)(::phy_engine::netlist::netlist const&, void*) noexcept;
        omax_dump_best_cb_t omax_dump_best_cb{nullptr};  // optional callback invoked when a new best is found
        void* omax_dump_best_user{nullptr};

        // Optional CUDA acceleration (best-effort). Requires building with PHY_ENGINE_ENABLE_CUDA_PE_SYNTH.
        bool cuda_enable{false};
        ::std::uint32_t cuda_device_mask{0};  // bitmask of devices to use; 0 means "all available"
        ::std::size_t cuda_min_batch{1024};   // minimum number of cones/candidates before offloading (avoid launch overhead)
        ::std::size_t decomp_var_order_tries{4};    // BDD decomposition: max variable-order variants to try (bounded, deterministic unless randomized)
        bool optimize_wires{false};                 // best-effort: remove synthesized YES buffers (net aliasing), keeps top-level port nodes intact
        bool optimize_mul2{false};                  // best-effort: replace 2-bit multiplier tiles with MUL2 models
        bool optimize_adders{false};                // best-effort: replace gate-level adders with HALF_ADDER/FULL_ADDER models
        ::std::size_t loop_unroll_limit{64};        // bounded unrolling for dynamic for/while in procedural blocks
        bool infer_dc_from_xz{true};                // when assume_binary_inputs, allow X/Z-driven minterms as DC in 2-level minimization
        bool infer_dc_from_fsm{true};               // infer DC constraints from one-hot FSM-style state encodings (bounded)
        bool infer_dc_from_odc{true};               // local observability DC for masked internal nodes (bounded)
        ::std::size_t dc_fsm_max_bits{16};          // max state bits for FSM DC inference (0 disables)
        bool techmap_enable{true};                  // enable cut-based tech mapping (bounded, best-effort)
        bool techmap_richer_library{false};         // include AOI/OAI-style templates (lowered to primitives)
        ::std::size_t techmap_max_cut{4};           // max cut size (vars) for mapping
        ::std::size_t techmap_max_gates{96};        // max gates per cone for mapping
        ::std::size_t techmap_max_cuts{32};         // max cuts per node
        bool decompose_large_functions{true};       // bounded functional decomposition for large cones (2-valued)
        ::std::size_t decomp_min_vars{11};          // start decomposing at this leaf count
        ::std::size_t decomp_max_vars{16};          // max vars for truth-table/BDD decomposition
        ::std::size_t decomp_max_gates{256};        // max gates per cone for decomposition
        ::std::size_t decomp_bdd_node_limit{4096};  // BDD node budget (includes terminals)

        // Per-pass budgets (used by O3 and thus by Omax too). Use 0 to disable a pass's heavy inner loop.
        ::std::size_t qm_max_vars{10};            // 2-level minimization: max vars per cone (0 disables QM/Espresso pass)
        ::std::size_t qm_max_gates{64};           // 2-level minimization: max internal logic gates per cone
        ::std::size_t qm_max_primes{4096};        // 2-level minimization: max prime implicants to consider (best-effort)
        ::std::size_t qm_max_minterms{0};         // 2-level minimization: 0 = unlimited; else skip cones with 2^vars > this
        ::std::size_t resub_max_vars{6};          // Resub: max vars per local truth-table window (0 disables resub)
        ::std::size_t resub_max_gates{64};        // Resub: max gates in local cone
        ::std::size_t sweep_max_vars{6};          // Sweep: max vars per local truth-table window (0 disables sweep)
        ::std::size_t sweep_max_gates{64};        // Sweep: max gates in local cone
        ::std::size_t rewrite_max_candidates{0};  // AIG rewrite: 0 = unlimited; else cap candidate roots (deterministic prefix)

        // Global "blow-up" guards (best-effort). Note: nodes are allocated from arenas; they can't be shrunk.
        ::std::size_t max_total_nodes{0};        // 0 disables; if exceeded, remaining passes are skipped (best-effort)
        ::std::size_t max_total_models{0};       // 0 disables; if exceeded after a pass, it is rolled back (best-effort)
        ::std::size_t max_total_logic_gates{0};  // 0 disables; if exceeded after a pass, it is rolled back (best-effort)
        bool report_enable{false};               // collect per-pass gate-count deltas (best-effort)
        pe_synth_report* report{nullptr};        // optional sink when report_enable=true

        // Two-level minimization cost model (used in O3 cone minimization passes).
        two_level_cost_model two_level_cost{two_level_cost_model::gate_count};
        two_level_cost_weights two_level_weights{};
    };

    struct pe_synth_error
    {
        ::fast_io::u8string message{};
    };

    struct pe_synth_pass_stat
    {
        ::fast_io::u8string_view pass{};
        ::std::size_t before{};
        ::std::size_t after{};
    };

    struct pe_synth_report
    {
        ::std::vector<pe_synth_pass_stat> passes{};
        ::std::vector<::std::size_t> iter_gate_count{};
        ::std::vector<::std::size_t> omax_best_gate_count{};  // best-so-far cost after each Omax try (empty unless opt_level>=5)
        ::std::vector<::std::size_t> omax_best_cost{};        // best-so-far objective cost after each Omax try (matches opt.omax_cost)
        ::fast_io::u8string omax_summary{};                   // single-line, reproducible knobs summary
    };

    namespace details
    {
        inline bool is_output_pin(::fast_io::u8string_view model_name, std::size_t pin_idx, std::size_t pin_count) noexcept;

        inline ::fast_io::u8string_view model_name_u8(::phy_engine::model::model_base const& mb) noexcept
        { return (mb.ptr == nullptr) ? ::fast_io::u8string_view{} : mb.ptr->get_model_name(); }

        inline bool is_const_input_model(::phy_engine::model::model_base const& mb, ::phy_engine::model::digital_node_statement_t v) noexcept
        {
            if(mb.ptr == nullptr) { return false; }
            if(mb.name.size() != 0) { return false; }  // named INPUTs are external IO, not constants
            if(model_name_u8(mb) != u8"INPUT") { return false; }
            auto vi = mb.ptr->get_attribute(0);
            if(vi.type != ::phy_engine::model::variant_type::digital) { return false; }
            return vi.digital == v;
        }

        inline ::phy_engine::model::node_t* find_existing_const_node(::phy_engine::netlist::netlist& nl,
                                                                     ::phy_engine::model::digital_node_statement_t v) noexcept
        {
            for(auto& blk: nl.models)
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

        inline ::phy_engine::model::node_t* make_const_node(::phy_engine::netlist::netlist& nl, ::phy_engine::model::digital_node_statement_t v) noexcept
        {
            auto& n = ::phy_engine::netlist::create_node(nl);
            auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = v});
            (void)pos;
            if(m == nullptr) { return nullptr; }
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, n)) { return nullptr; }
            return __builtin_addressof(n);
        }

        inline ::phy_engine::model::node_t* find_or_make_const_node(::phy_engine::netlist::netlist& nl,
                                                                    ::phy_engine::model::digital_node_statement_t v) noexcept
        {
            if(auto* n = find_existing_const_node(nl, v); n != nullptr) { return n; }
            return make_const_node(nl, v);
        }

        [[nodiscard]] inline std::size_t count_logic_gates(::phy_engine::netlist::netlist const& nl) noexcept
        {
            std::size_t gates{};
            for(auto const& blk: nl.models)
            {
                for(auto const* m = blk.begin; m != blk.curr; ++m)
                {
                    if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }
                    auto const n = m->ptr->get_model_name();
                    if(n == u8"AND" || n == u8"OR" || n == u8"XOR" || n == u8"XNOR" || n == u8"NOT" || n == u8"NAND" || n == u8"NOR" || n == u8"IMP" ||
                       n == u8"NIMP" || n == u8"YES")
                    {
                        ++gates;
                    }
                }
            }
            return gates;
        }

        [[nodiscard]] inline std::size_t compute_omax_cost(::phy_engine::netlist::netlist const& nl, pe_synth_options const& opt) noexcept
        {
            if(opt.omax_cost == pe_synth_options::omax_cost_model::gate_count) { return count_logic_gates(nl); }

            std::size_t cost{};
            for(auto const& blk: nl.models)
            {
                for(auto const* m = blk.begin; m != blk.curr; ++m)
                {
                    if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }
                    auto const n = m->ptr->get_model_name();
                    if(n == u8"NOT") { cost += opt.omax_gate_weights.not_w; }
                    else if(n == u8"AND") { cost += opt.omax_gate_weights.and_w; }
                    else if(n == u8"OR") { cost += opt.omax_gate_weights.or_w; }
                    else if(n == u8"XOR") { cost += opt.omax_gate_weights.xor_w; }
                    else if(n == u8"XNOR") { cost += opt.omax_gate_weights.xnor_w; }
                    else if(n == u8"NAND") { cost += opt.omax_gate_weights.nand_w; }
                    else if(n == u8"NOR") { cost += opt.omax_gate_weights.nor_w; }
                    else if(n == u8"IMP") { cost += opt.omax_gate_weights.imp_w; }
                    else if(n == u8"NIMP") { cost += opt.omax_gate_weights.nimp_w; }
                    else if(n == u8"YES") { cost += opt.omax_gate_weights.yes_w; }
                    else if(n == u8"CASE_EQ") { cost += opt.omax_gate_weights.case_eq_w; }
                    else if(n == u8"IS_UNKNOWN") { cost += opt.omax_gate_weights.is_unknown_w; }
                }
            }
            return cost;
        }

        [[nodiscard]] inline std::size_t count_total_models(::phy_engine::netlist::netlist const& nl) noexcept
        {
            std::size_t models{};
            for(auto const& blk: nl.models)
            {
                for(auto const* m = blk.begin; m != blk.curr; ++m)
                {
                    if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }
                    ++models;
                }
            }
            return models;
        }

        [[nodiscard]] inline std::size_t count_total_nodes(::phy_engine::netlist::netlist const& nl) noexcept
        {
            std::size_t nodes{};
            for(auto const& blk: nl.nodes) { nodes += static_cast<std::size_t>(blk.curr - blk.begin); }
            return nodes + 1u;  // + ground_node
        }

        inline void restore_from_snapshot(::phy_engine::netlist::netlist& dst, ::phy_engine::netlist::netlist const& src) noexcept
        {
            auto gather_nodes = [](::phy_engine::netlist::netlist const& x, ::std::vector<::phy_engine::model::node_t const*>& out) noexcept
            {
                out.clear();
                for(auto const& blk: x.nodes)
                {
                    for(auto const* n = blk.begin; n != blk.curr; ++n) { out.push_back(n); }
                }
            };

            ::std::vector<::phy_engine::model::node_t const*> src_nodes{};
            ::std::vector<::phy_engine::model::node_t const*> dst_nodes_const{};
            src_nodes.reserve(4096);
            dst_nodes_const.reserve(4096);
            gather_nodes(src, src_nodes);
            gather_nodes(dst, dst_nodes_const);

            // Clear node pin sets (including extra nodes allocated after the snapshot).
            for(auto const* cn: dst_nodes_const)
            {
                auto* n = const_cast<::phy_engine::model::node_t*>(cn);
                n->pins.clear();
                n->num_of_analog_node = 0;
                n->node_information = {};
                n->node_index = SIZE_MAX;
            }
            dst.ground_node.pins.clear();
            dst.ground_node.num_of_analog_node = 0;
            dst.ground_node.node_information = {};
            dst.ground_node.node_index = SIZE_MAX;

            // Clear existing models.
            for(auto& blk: dst.models) { blk.clear(); }
            dst.models.clear();

            if(dst_nodes_const.size() < src_nodes.size()) { return; }  // best-effort: shouldn't happen

            // Restore node_information for the overlapping prefix.
            for(::std::size_t i{}; i < src_nodes.size(); ++i)
            {
                auto* dn = const_cast<::phy_engine::model::node_t*>(dst_nodes_const[i]);
                dn->node_information = src_nodes[i]->node_information;
            }
            dst.ground_node.node_information = src.ground_node.node_information;

            ::std::unordered_map<::phy_engine::model::node_t const*, ::phy_engine::model::node_t*> node_map{};
            node_map.reserve(src_nodes.size() * 2u + 8u);
            node_map.emplace(__builtin_addressof(src.ground_node), __builtin_addressof(dst.ground_node));
            for(::std::size_t i{}; i < src_nodes.size(); ++i) { node_map.emplace(src_nodes[i], const_cast<::phy_engine::model::node_t*>(dst_nodes_const[i])); }

            auto emplace_model_base = [&](::phy_engine::model::model_base&& mb) noexcept
            {
                if(dst.models.empty()) [[unlikely]]
                {
                    auto& nlb{dst.models.emplace_back()};
                    ::new(nlb.curr++)::phy_engine::model::model_base{::std::move(mb)};
                    return;
                }
                auto& nlb{dst.models.back_unchecked()};
                if(nlb.curr == nlb.begin + nlb.chunk_module_size) [[unlikely]]
                {
                    auto& new_nlb{dst.models.emplace_back()};
                    ::new(new_nlb.curr++)::phy_engine::model::model_base{::std::move(mb)};
                    return;
                }
                ::new(nlb.curr++)::phy_engine::model::model_base{::std::move(mb)};
            };

            // Recreate models from src, reconnecting pins to mapped dst nodes.
            for(auto const& blk: src.models)
            {
                for(auto const* m = blk.begin; m != blk.curr; ++m)
                {
                    if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }

                    ::phy_engine::model::model_base copy{};
                    copy.copy_with_node_ptr(*m);
                    if(copy.type != ::phy_engine::model::model_type::normal || copy.ptr == nullptr) { continue; }

                    auto const dev = copy.ptr->get_device_type();
                    bool const is_analog = (dev != ::phy_engine::model::model_device_type::digital);

                    auto pv = copy.ptr->generate_pin_view();
                    for(std::size_t pi{}; pi < pv.size; ++pi)
                    {
                        auto& pn = pv.pins[pi];
                        auto* sn = pn.nodes;
                        if(sn == nullptr) { continue; }
                        auto it = node_map.find(sn);
                        if(it == node_map.end())
                        {
                            pn.nodes = nullptr;
                            continue;
                        }
                        pn.nodes = it->second;
                        it->second->pins.insert(__builtin_addressof(pn));
                        if(is_analog) { ++it->second->num_of_analog_node; }
                    }

                    emplace_model_base(::std::move(copy));
                }
            }
        }

        struct u8sv_hash
        {
            std::size_t operator() (::fast_io::u8string_view s) const noexcept
            {
                auto mix = [](std::size_t h, std::size_t v) noexcept -> std::size_t { return (h ^ (v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2))); };
                std::size_t h{};
                for(std::size_t i{}; i < s.size(); ++i) { h = mix(h, static_cast<unsigned char>(reinterpret_cast<char const*>(s.data())[i])); }
                return h;
            }
        };

        struct u8sv_eq
        {
            bool operator() (::fast_io::u8string_view a, ::fast_io::u8string_view b) const noexcept { return a == b; }
        };

        struct comb_eval_net
        {
            struct gate
            {
                enum class kind : std::uint8_t
                {
                    not_gate,
                    yes_gate,
                    and_gate,
                    or_gate,
                    xor_gate,
                    xnor_gate,
                    nand_gate,
                    nor_gate,
                    imp_gate,
                    nimp_gate,
                    case_eq_gate,
                    is_unknown_gate,
                };
                kind k{};
                ::phy_engine::model::node_t* in0{};
                ::phy_engine::model::node_t* in1{};
            };

            ::std::unordered_map<::fast_io::u8string_view, ::phy_engine::model::node_t*, u8sv_hash, u8sv_eq> inputs{};
            ::std::unordered_map<::fast_io::u8string_view, ::phy_engine::model::node_t*, u8sv_hash, u8sv_eq> outputs{};
            ::std::unordered_map<::phy_engine::model::node_t*, bool> const_nodes{};
            ::std::unordered_map<::phy_engine::model::node_t*, gate> driver{};
            bool ok{true};
        };

        [[nodiscard]] inline bool build_comb_eval_net(::phy_engine::netlist::netlist& nl, comb_eval_net& out) noexcept
        {
            out = comb_eval_net{};
            out.inputs.reserve(128);
            out.outputs.reserve(128);
            out.const_nodes.reserve(128);
            out.driver.reserve(1 << 14);

            auto to_bool_const = [&](::phy_engine::model::model_base const& mb, bool& v) noexcept -> bool
            {
                if(mb.type != ::phy_engine::model::model_type::normal || mb.ptr == nullptr) { return false; }
                auto vi = mb.ptr->get_attribute(0);
                if(vi.type != ::phy_engine::model::variant_type::digital) { return false; }
                if(vi.digital == ::phy_engine::model::digital_node_statement_t::true_state)
                {
                    v = true;
                    return true;
                }
                if(vi.digital == ::phy_engine::model::digital_node_statement_t::false_state)
                {
                    v = false;
                    return true;
                }
                return false;
            };

            for(auto& blk: nl.models)
            {
                for(auto* m = blk.begin; m != blk.curr; ++m)
                {
                    if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }
                    if(m->ptr->get_device_type() != ::phy_engine::model::model_device_type::digital) { continue; }
                    auto const nm = model_name_u8(*m);
                    auto pv = m->ptr->generate_pin_view();

                    if(nm == u8"INPUT")
                    {
                        if(pv.size != 1)
                        {
                            out.ok = false;
                            return false;
                        }
                        auto* n = pv.pins[0].nodes;
                        if(n == nullptr) { continue; }
                        if(m->name.size() == 0)
                        {
                            bool cv{};
                            if(!to_bool_const(*m, cv))
                            {
                                out.ok = false;
                                return false;
                            }
                            out.const_nodes.emplace(n, cv);
                        }
                        else
                        {
                            ::fast_io::u8string_view key{m->name.data(), m->name.size()};
                            out.inputs.emplace(key, n);
                        }
                        continue;
                    }
                    if(nm == u8"OUTPUT")
                    {
                        if(m->name.size() == 0) { continue; }
                        if(pv.size != 1)
                        {
                            out.ok = false;
                            return false;
                        }
                        auto* n = pv.pins[0].nodes;
                        if(n == nullptr) { continue; }
                        ::fast_io::u8string_view key{m->name.data(), m->name.size()};
                        out.outputs.emplace(key, n);
                        continue;
                    }

                    // Only support a combinational subset for verification; skip/disable verification on other devices.
                    comb_eval_net::gate g{};
                    bool supported{true};
                    if(nm == u8"NOT") { g.k = comb_eval_net::gate::kind::not_gate; }
                    else if(nm == u8"YES") { g.k = comb_eval_net::gate::kind::yes_gate; }
                    else if(nm == u8"AND") { g.k = comb_eval_net::gate::kind::and_gate; }
                    else if(nm == u8"OR") { g.k = comb_eval_net::gate::kind::or_gate; }
                    else if(nm == u8"XOR") { g.k = comb_eval_net::gate::kind::xor_gate; }
                    else if(nm == u8"XNOR") { g.k = comb_eval_net::gate::kind::xnor_gate; }
                    else if(nm == u8"NAND") { g.k = comb_eval_net::gate::kind::nand_gate; }
                    else if(nm == u8"NOR") { g.k = comb_eval_net::gate::kind::nor_gate; }
                    else if(nm == u8"IMP") { g.k = comb_eval_net::gate::kind::imp_gate; }
                    else if(nm == u8"NIMP") { g.k = comb_eval_net::gate::kind::nimp_gate; }
                    else if(nm == u8"CASE_EQ") { g.k = comb_eval_net::gate::kind::case_eq_gate; }
                    else if(nm == u8"IS_UNKNOWN") { g.k = comb_eval_net::gate::kind::is_unknown_gate; }
                    else
                    {
                        supported = false;
                    }
                    if(!supported)
                    {
                        out.ok = false;
                        return false;
                    }

                    // Fill inputs by position (common for these primitives).
                    ::phy_engine::model::node_t* out_node{};
                    if(nm == u8"NOT" || nm == u8"YES" || nm == u8"IS_UNKNOWN")
                    {
                        if(pv.size != 2)
                        {
                            out.ok = false;
                            return false;
                        }
                        g.in0 = pv.pins[0].nodes;
                        out_node = pv.pins[1].nodes;
                        if(g.in0 == nullptr)
                        {
                            out.ok = false;
                            return false;
                        }
                    }
                    else
                    {
                        if(pv.size != 3)
                        {
                            out.ok = false;
                            return false;
                        }
                        g.in0 = pv.pins[0].nodes;
                        g.in1 = pv.pins[1].nodes;
                        out_node = pv.pins[2].nodes;
                        if(g.in0 == nullptr || g.in1 == nullptr)
                        {
                            out.ok = false;
                            return false;
                        }
                    }

                    if(out_node == nullptr)
                    {
                        out.ok = false;
                        return false;
                    }
                    if(out.driver.contains(out_node))
                    {
                        out.ok = false;  // multi-driver (in the supported subset)
                        return false;
                    }
                    out.driver.emplace(out_node, g);
                }
            }

            return out.ok;
        }

        [[nodiscard]] inline bool eval_comb_node(comb_eval_net const& net,
                                                 ::phy_engine::model::node_t* n,
                                                 ::std::unordered_map<::phy_engine::model::node_t*, bool> const& in_assign,
                                                 ::std::unordered_map<::phy_engine::model::node_t*, bool>& memo,
                                                 bool& outv) noexcept
        {
            if(n == nullptr) { return false; }
            if(auto it = memo.find(n); it != memo.end())
            {
                outv = it->second;
                return true;
            }
            if(auto itc = net.const_nodes.find(n); itc != net.const_nodes.end())
            {
                outv = itc->second;
                memo.emplace(n, outv);
                return true;
            }
            if(auto iti = in_assign.find(n); iti != in_assign.end())
            {
                outv = iti->second;
                memo.emplace(n, outv);
                return true;
            }

            auto itg = net.driver.find(n);
            if(itg == net.driver.end()) { return false; }
            auto const& g = itg->second;

            bool a{}, b{};
            if(!eval_comb_node(net, g.in0, in_assign, memo, a)) { return false; }
            if(g.k != comb_eval_net::gate::kind::not_gate && g.k != comb_eval_net::gate::kind::yes_gate && g.k != comb_eval_net::gate::kind::is_unknown_gate)
            {
                if(!eval_comb_node(net, g.in1, in_assign, memo, b)) { return false; }
            }

            bool r{};
            switch(g.k)
            {
                case comb_eval_net::gate::kind::not_gate: r = !a; break;
                case comb_eval_net::gate::kind::yes_gate: r = a; break;
                case comb_eval_net::gate::kind::and_gate: r = (a & b); break;
                case comb_eval_net::gate::kind::or_gate: r = (a | b); break;
                case comb_eval_net::gate::kind::xor_gate: r = (a != b); break;
                case comb_eval_net::gate::kind::xnor_gate: r = (a == b); break;
                case comb_eval_net::gate::kind::nand_gate: r = !(a & b); break;
                case comb_eval_net::gate::kind::nor_gate: r = !(a | b); break;
                case comb_eval_net::gate::kind::imp_gate: r = (!a) | b; break;
                case comb_eval_net::gate::kind::nimp_gate: r = a & (!b); break;
                case comb_eval_net::gate::kind::case_eq_gate: r = (a == b); break;
                case comb_eval_net::gate::kind::is_unknown_gate: r = false; break;  // binary-only mode
            }

            memo.emplace(n, r);
            outv = r;
            return true;
        }

        [[nodiscard]] inline bool omax_verify_equivalence(::phy_engine::netlist::netlist& ref_nl,
                                                          ::phy_engine::netlist::netlist& cand_nl,
                                                          pe_synth_options const& opt,
                                                          ::fast_io::u8string* why = nullptr) noexcept
        {
            comb_eval_net ref{};
            comb_eval_net cand{};
            if(!build_comb_eval_net(ref_nl, ref) || !build_comb_eval_net(cand_nl, cand))
            {
                if(why)
                {
                    why->clear();
                    why->append(u8"omax_verify: unsupported netlist for comb verification");
                }
                return false;
            }

            // Ensure IO sets are consistent (by name).
            for(auto const& kv: ref.inputs)
            {
                if(!cand.inputs.contains(kv.first))
                {
                    if(why)
                    {
                        why->clear();
                        why->append(u8"omax_verify: input set mismatch");
                    }
                    return false;
                }
            }
            for(auto const& kv: ref.outputs)
            {
                if(!cand.outputs.contains(kv.first))
                {
                    if(why)
                    {
                        why->clear();
                        why->append(u8"omax_verify: output set mismatch");
                    }
                    return false;
                }
            }

            ::std::vector<::fast_io::u8string_view> in_names{};
            in_names.reserve(ref.inputs.size());
            for(auto const& kv: ref.inputs) { in_names.push_back(kv.first); }

            ::std::vector<::fast_io::u8string_view> out_names{};
            out_names.reserve(ref.outputs.size());
            for(auto const& kv: ref.outputs) { out_names.push_back(kv.first); }

            auto run_one = [&](auto&& assign_fn, std::size_t idx) noexcept -> bool
            {
                ::std::unordered_map<::phy_engine::model::node_t*, bool> as_ref{};
                ::std::unordered_map<::phy_engine::model::node_t*, bool> as_cand{};
                as_ref.reserve(in_names.size() * 2u + 8u);
                as_cand.reserve(in_names.size() * 2u + 8u);
                for(std::size_t i{}; i < in_names.size(); ++i)
                {
                    auto const nm = in_names[i];
                    bool const bit = assign_fn(i);
                    as_ref.emplace(ref.inputs.find(nm)->second, bit);
                    as_cand.emplace(cand.inputs.find(nm)->second, bit);
                }

                for(auto const nm: out_names)
                {
                    bool vr{}, vc{};
                    ::std::unordered_map<::phy_engine::model::node_t*, bool> memo_r{};
                    ::std::unordered_map<::phy_engine::model::node_t*, bool> memo_c{};
                    memo_r.reserve(256);
                    memo_c.reserve(256);
                    auto* nr = ref.outputs.find(nm)->second;
                    auto* nc = cand.outputs.find(nm)->second;
                    if(!eval_comb_node(ref, nr, as_ref, memo_r, vr) || !eval_comb_node(cand, nc, as_cand, memo_c, vc))
                    {
                        if(why)
                        {
                            why->clear();
                            why->append(u8"omax_verify: evaluation failed");
                        }
                        return false;
                    }
                    if(vr != vc)
                    {
                        if(why)
                        {
                            // Keep it short (ASCII).
                            why->clear();
                            why->append(u8"omax_verify: output mismatch");
                        }
                        (void)idx;
                        return false;
                    }
                }
                return true;
            };

            auto const in_cnt = in_names.size();
            auto const exact_k = opt.omax_verify_exact_max_inputs;
            if(exact_k != 0u && in_cnt <= exact_k && in_cnt < 31u)
            {
                auto const total = static_cast<std::size_t>(1u) << in_cnt;
                for(std::size_t pat{}; pat < total; ++pat)
                {
                    auto assign_fn = [&](std::size_t i) noexcept -> bool { return ((pat >> i) & 1u) != 0u; };
                    if(!run_one(assign_fn, pat)) { return false; }
                }
                return true;
            }

            // Random vectors.
            std::uint64_t rng = opt.omax_verify_seed;
            auto next = [&]() noexcept -> std::uint64_t
            {
                // xorshift64*
                rng ^= rng >> 12;
                rng ^= rng << 25;
                rng ^= rng >> 27;
                return rng * 0x2545F4914F6CDD1Dull;
            };
            std::size_t vecs = opt.omax_verify_random_vectors;
            if(vecs == 0u) { vecs = 1u; }
            for(std::size_t v{}; v < vecs; ++v)
            {
                auto r = next();
                auto assign_fn = [&](std::size_t i) noexcept -> bool
                {
                    if((i & 63u) == 0u) { r = next(); }
                    return ((r >> (i & 63u)) & 1ull) != 0ull;
                };
                if(!run_one(assign_fn, v)) { return false; }
            }
            return true;
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
                std::size_t operator() (gate_key const& x) const noexcept
                {
                    auto const mix = [](std::size_t h, std::size_t v) noexcept -> std::size_t
                    { return (h ^ (v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2))); };
                    std::size_t h{};
                    h = mix(h, static_cast<std::size_t>(x.k));
                    h = mix(h, reinterpret_cast<std::size_t>(x.a));
                    h = mix(h, reinterpret_cast<std::size_t>(x.b));
                    return h;
                }
            };

            struct gate_key_eq
            {
                bool operator() (gate_key const& x, gate_key const& y) const noexcept { return x.k == y.k && x.a == y.a && x.b == y.b; }
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

            ::std::unordered_map<::phy_engine::model::pin const*, bool> pin_is_output{};
            pin_is_output.reserve(1 << 14);

            ::std::unordered_map<::phy_engine::model::node_t*, std::size_t> consumer_count{};
            ::std::unordered_map<::phy_engine::model::node_t*, std::size_t> driver_count{};
            consumer_count.reserve(1 << 14);
            driver_count.reserve(1 << 14);

            auto note_pin = [&](::phy_engine::model::pin const* p, bool is_out) noexcept
            {
                if(p == nullptr) { return; }
                pin_is_output.emplace(p, is_out);
                if(p->nodes == nullptr) { return; }
                if(is_out) { ++driver_count[p->nodes]; }
                else
                {
                    ++consumer_count[p->nodes];
                }
            };

            auto classify_gate = [&](::phy_engine::model::model_base& mb, std::size_t vec_pos, std::size_t chunk_pos) noexcept -> std::optional<gate>
            {
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
                    else
                    {
                        g.k = gate::kind::xnor_gate;
                    }

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

                    if(og->k == gate::kind::and_gate || og->k == gate::kind::or_gate || og->k == gate::kind::xor_gate || og->k == gate::kind::xnor_gate)
                    {
                        if(og->in0 != nullptr && og->in1 != nullptr) { out_by_inputs.emplace(canon_pair(og->k, og->in0, og->in1), og->out); }
                    }
                }
            }

            auto gate_out = [&](gate::kind k, ::phy_engine::model::node_t* a, ::phy_engine::model::node_t* b) noexcept -> ::phy_engine::model::node_t*
            {
                auto it = out_by_inputs.find(canon_pair(k, a, b));
                return it == out_by_inputs.end() ? nullptr : it->second;
            };

            auto gate_ptr_by_out = [&](::phy_engine::model::node_t* out) noexcept -> gate const*
            {
                auto it = gate_by_out.find(out);
                if(it == gate_by_out.end()) { return nullptr; }
                return __builtin_addressof(gates[it->second]);
            };

            auto can_delete_gate_output = [&](::phy_engine::model::node_t* out) noexcept -> bool
            {
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

            auto mark_used = [&](gate const* g) noexcept -> bool
            {
                if(g == nullptr || g->mb == nullptr) { return false; }
                if(used_models.contains(g->mb)) { return false; }
                used_models.emplace(g->mb, true);
                return true;
            };

            auto try_add_half_adder = [&](gate const& gx) noexcept
            {
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

            auto try_add_full_adder_cin1 = [&](gate const& gnot) noexcept
            {
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

            auto try_add_full_adder_general = [&](gate const& g2) noexcept
            {
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
            for(auto const& g: gates) { try_add_full_adder_general(g); }
            // Pass 2: cin=0 half adders (common in LSB).
            for(auto const& g: gates) { try_add_half_adder(g); }
            // Pass 3: cin=1 simplified form (two's complement +1).
            for(auto const& g: gates) { try_add_full_adder_cin1(g); }

            ::std::vector<model_pos> to_delete{};
            to_delete.reserve(actions.size() * 8);
            for(auto const& a: actions)
            {
                for(auto const& p: a.del) { to_delete.push_back(p); }
            }

            auto less_desc = [](model_pos const& x, model_pos const& y) noexcept
            {
                if(x.chunk_pos != y.chunk_pos) { return x.chunk_pos > y.chunk_pos; }
                return x.vec_pos > y.vec_pos;
            };
            ::std::sort(to_delete.begin(), to_delete.end(), less_desc);
            to_delete.erase(::std::unique(to_delete.begin(),
                                          to_delete.end(),
                                          [](model_pos const& x, model_pos const& y) noexcept { return x.chunk_pos == y.chunk_pos && x.vec_pos == y.vec_pos; }),
                            to_delete.end());

            for(auto const& p: to_delete) { (void)::phy_engine::netlist::delete_model(nl, p.vec_pos, p.chunk_pos); }

            for(auto const& a: actions)
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
                std::size_t operator() (gate_key const& x) const noexcept
                {
                    auto const mix = [](std::size_t h, std::size_t v) noexcept -> std::size_t
                    { return (h ^ (v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2))); };
                    std::size_t h{};
                    h = mix(h, static_cast<std::size_t>(x.k));
                    h = mix(h, reinterpret_cast<std::size_t>(x.a));
                    h = mix(h, reinterpret_cast<std::size_t>(x.b));
                    return h;
                }
            };

            struct gate_key_eq
            {
                bool operator() (gate_key const& x, gate_key const& y) const noexcept { return x.k == y.k && x.a == y.a && x.b == y.b; }
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
            for(auto& blk: nl.models)
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
                        else
                        {
                            ++consumer_count[n];
                        }

                        if(is_output_pin(name, i, pv.size))
                        {
                            if(name == u8"INPUT" && !mb->name.empty())
                            {
                                driver_is_named_input[n] = true;
                                named_input_name.emplace(n, ::fast_io::u8string_view{mb->name.data(), mb->name.size()});
                            }
                            else if(name != u8"INPUT") { driver_is_non_input[n] = true; }
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
                else
                {
                    return std::nullopt;
                }
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
            { return x == p0 || x == p1; };

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
                for(auto const p2_xor_idx: uses_it->second)
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
                        else if(base_a0 != u8"a" && base_b0 != u8"a") { desired_a = (base_a0 <= base_b0) ? base_a0 : base_b0; }

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
                    for(auto const i: ids) { kill(i); }
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

            auto const is_protected = [&](::phy_engine::model::node_t* n) noexcept -> bool
            {
                for(auto* p: protected_nodes)
                {
                    if(p == n) { return true; }
                }
                return false;
            };

            // Build pin->is_output map for the whole netlist (best-effort).
            for(auto& blk: nl.models)
            {
                for(auto* m = blk.begin; m != blk.curr; ++m)
                {
                    if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }
                    auto const name = model_name_u8(*m);
                    auto pv = m->ptr->generate_pin_view();
                    for(std::size_t i{}; i < pv.size; ++i) { pin_out.emplace(__builtin_addressof(pv.pins[i]), is_output_pin(name, i, pv.size)); }
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

            for(auto const mp: yes_models)
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
                for(auto const* p: out_node->pins)
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
                for(auto* p: out_node->pins)
                {
                    if(p == out_pin) { continue; }
                    pins_to_move.push_back(p);
                }
                for(auto* p: pins_to_move)
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

        struct dc_group
        {
            ::std::vector<::phy_engine::model::node_t*> nodes{};
            ::std::vector<::std::uint16_t> allowed{};  // allowed minterms (bit order matches nodes)
        };

        struct dc_constraints
        {
            ::std::vector<dc_group> groups{};
        };

        // CUDA truth-table helper: evaluate many small (<=64 gates, <=6 vars) cones as 64-bit masks.
        // This is used to accelerate truth-table hashing (e.g. sweep/resub) when available.
        struct cuda_u64_cone_desc
        {
            // Leaf indices are 0..5, gate outputs are stored at 6+i.
            // Special indices:
            // - 254: CONST0
            // - 255: CONST1
            ::std::uint8_t var_count{};   // 0..6
            ::std::uint8_t gate_count{};  // 1..64
            ::std::uint8_t pad0{};
            ::std::uint8_t pad1{};
            ::std::uint8_t kind[64]{};  // encoded kind per gate
            ::std::uint8_t in0[64]{};
            ::std::uint8_t in1[64]{};
        };

        // CUDA truth-table helper: evaluate larger cones (<=256 gates, <=16 vars) into a packed bitset TT.
        // Output format is `stride_blocks` words per cone, with only the first `blocks=(2^var_count+63)/64`
        // words defined (remaining words are set to 0).
        struct cuda_tt_cone_desc
        {
            // Leaf indices are 0..15, gate outputs are stored at 16+i.
            // Special indices:
            // - 65534: CONST0
            // - 65535: CONST1
            ::std::uint8_t var_count{};     // 0..16
            ::std::uint8_t pad0{};
            ::std::uint16_t gate_count{};  // 1..256
            ::std::uint8_t kind[256]{};    // encoded kind per gate (same encoding as cuda_u64_cone_desc)
            ::std::uint16_t in0[256]{};
            ::std::uint16_t in1[256]{};
        };

        // CUDA Espresso helper: batch "cube hits OFF-set" checks.
        // Each cube is a (value,mask) over <=16 vars (same encoding as qm_implicant: mask bit 1 = don't-care).
        struct cuda_cube_desc
        {
            ::std::uint16_t value{};
            ::std::uint16_t mask{};
        };

#if defined(PHY_ENGINE_ENABLE_CUDA_PE_SYNTH)
        extern "C" int phy_engine_pe_synth_cuda_get_device_count() noexcept;
        extern "C" bool phy_engine_pe_synth_cuda_eval_u64_cones(::std::uint32_t device_mask,
                                                                cuda_u64_cone_desc const* cones,
                                                                ::std::size_t cone_count,
                                                                ::std::uint64_t* out_masks) noexcept;
        extern "C" bool phy_engine_pe_synth_cuda_eval_tt_cones(::std::uint32_t device_mask,
                                                               cuda_tt_cone_desc const* cones,
                                                               ::std::size_t cone_count,
                                                               ::std::uint32_t stride_blocks,
                                                               ::std::uint64_t* out_blocks) noexcept;

        // CUDA bitset helper: keep a row-major u64 matrix on device(s) for repeated masked popcount / any checks.
        // This is used to accelerate QM/Espresso "cover/search" bitset scoring when available.
        extern "C" void* phy_engine_pe_synth_cuda_bitset_matrix_create(::std::uint32_t device_mask,
                                                                       ::std::uint64_t const* rows,
                                                                       ::std::size_t row_count,
                                                                       ::std::uint32_t stride_words) noexcept;
        extern "C" void phy_engine_pe_synth_cuda_bitset_matrix_destroy(void* handle) noexcept;
        extern "C" bool phy_engine_pe_synth_cuda_bitset_matrix_row_and_popcount(void* handle,
                                                                                ::std::uint64_t const* mask,
                                                                                ::std::uint32_t mask_words,
                                                                                ::std::uint32_t* out_counts) noexcept;
        extern "C" bool phy_engine_pe_synth_cuda_bitset_matrix_row_any_and(void* handle,
                                                                           ::std::uint64_t const* mask,
                                                                           ::std::uint32_t mask_words,
                                                                           ::std::uint8_t* out_any) noexcept;

        extern "C" bool phy_engine_pe_synth_cuda_espresso_cube_hits_off(::std::uint32_t device_mask,
                                                                        cuda_cube_desc const* cubes,
                                                                        ::std::size_t cube_count,
                                                                        ::std::uint32_t var_count,
                                                                        ::std::uint64_t const* off_blocks,
                                                                        ::std::uint32_t off_words,
                                                                        ::std::uint8_t* out_hits) noexcept;
#endif

        [[nodiscard]] inline bool cuda_eval_u64_cones(::std::uint32_t device_mask,
                                                      cuda_u64_cone_desc const* cones,
                                                      ::std::size_t cone_count,
                                                      ::std::uint64_t* out_masks) noexcept
        {
#if defined(PHY_ENGINE_ENABLE_CUDA_PE_SYNTH)
            (void)device_mask;
            if(cones == nullptr || out_masks == nullptr || cone_count == 0u) { return false; }
            return phy_engine_pe_synth_cuda_eval_u64_cones(device_mask, cones, cone_count, out_masks);
#else
            (void)device_mask;
            (void)cones;
            (void)cone_count;
            (void)out_masks;
            return false;
#endif
        }

        [[nodiscard]] inline bool cuda_eval_tt_cones(::std::uint32_t device_mask,
                                                     cuda_tt_cone_desc const* cones,
                                                     ::std::size_t cone_count,
                                                     ::std::uint32_t stride_blocks,
                                                     ::std::uint64_t* out_blocks) noexcept
        {
#if defined(PHY_ENGINE_ENABLE_CUDA_PE_SYNTH)
            (void)device_mask;
            if(cones == nullptr || out_blocks == nullptr || cone_count == 0u || stride_blocks == 0u) { return false; }
            return phy_engine_pe_synth_cuda_eval_tt_cones(device_mask, cones, cone_count, stride_blocks, out_blocks);
#else
            (void)device_mask;
            (void)cones;
            (void)cone_count;
            (void)stride_blocks;
            (void)out_blocks;
            return false;
#endif
        }

        struct cuda_bitset_matrix_raii
        {
            void* handle{};

            cuda_bitset_matrix_raii() noexcept = default;
            cuda_bitset_matrix_raii(cuda_bitset_matrix_raii const&) = delete;
            cuda_bitset_matrix_raii& operator=(cuda_bitset_matrix_raii const&) = delete;

            cuda_bitset_matrix_raii(cuda_bitset_matrix_raii&& o) noexcept : handle(o.handle) { o.handle = nullptr; }
            cuda_bitset_matrix_raii& operator=(cuda_bitset_matrix_raii&& o) noexcept
            {
                if(this == &o) { return *this; }
                reset();
                handle = o.handle;
                o.handle = nullptr;
                return *this;
            }

            ~cuda_bitset_matrix_raii() noexcept { reset(); }

            void reset() noexcept
            {
#if defined(PHY_ENGINE_ENABLE_CUDA_PE_SYNTH)
                if(handle != nullptr) { phy_engine_pe_synth_cuda_bitset_matrix_destroy(handle); }
#endif
                handle = nullptr;
            }
        };

        [[nodiscard]] inline cuda_bitset_matrix_raii cuda_bitset_matrix_create(::std::uint32_t device_mask,
                                                                               ::std::uint64_t const* rows,
                                                                               ::std::size_t row_count,
                                                                               ::std::uint32_t stride_words) noexcept
        {
            cuda_bitset_matrix_raii r{};
#if defined(PHY_ENGINE_ENABLE_CUDA_PE_SYNTH)
            if(rows == nullptr || row_count == 0u || stride_words == 0u) { return r; }
            r.handle = phy_engine_pe_synth_cuda_bitset_matrix_create(device_mask, rows, row_count, stride_words);
#else
            (void)device_mask;
            (void)rows;
            (void)row_count;
            (void)stride_words;
#endif
            return r;
        }

        [[nodiscard]] inline bool cuda_bitset_matrix_row_and_popcount(cuda_bitset_matrix_raii const& m,
                                                                      ::std::uint64_t const* mask,
                                                                      ::std::uint32_t mask_words,
                                                                      ::std::uint32_t* out_counts) noexcept
        {
#if defined(PHY_ENGINE_ENABLE_CUDA_PE_SYNTH)
            if(m.handle == nullptr || mask == nullptr || out_counts == nullptr || mask_words == 0u) { return false; }
            return phy_engine_pe_synth_cuda_bitset_matrix_row_and_popcount(m.handle, mask, mask_words, out_counts);
#else
            (void)m;
            (void)mask;
            (void)mask_words;
            (void)out_counts;
            return false;
#endif
        }

        [[nodiscard]] inline bool cuda_bitset_matrix_row_any_and(cuda_bitset_matrix_raii const& m,
                                                                 ::std::uint64_t const* mask,
                                                                 ::std::uint32_t mask_words,
                                                                 ::std::uint8_t* out_any) noexcept
        {
#if defined(PHY_ENGINE_ENABLE_CUDA_PE_SYNTH)
            if(m.handle == nullptr || mask == nullptr || out_any == nullptr || mask_words == 0u) { return false; }
            return phy_engine_pe_synth_cuda_bitset_matrix_row_any_and(m.handle, mask, mask_words, out_any);
#else
            (void)m;
            (void)mask;
            (void)mask_words;
            (void)out_any;
            return false;
#endif
        }

        [[nodiscard]] inline bool cuda_espresso_cube_hits_off(::std::uint32_t device_mask,
                                                              cuda_cube_desc const* cubes,
                                                              ::std::size_t cube_count,
                                                              ::std::uint32_t var_count,
                                                              ::std::uint64_t const* off_blocks,
                                                              ::std::uint32_t off_words,
                                                              ::std::uint8_t* out_hits) noexcept
        {
#if defined(PHY_ENGINE_ENABLE_CUDA_PE_SYNTH)
            if(cubes == nullptr || off_blocks == nullptr || out_hits == nullptr || cube_count == 0u || off_words == 0u) { return false; }
            return phy_engine_pe_synth_cuda_espresso_cube_hits_off(device_mask, cubes, cube_count, var_count, off_blocks, off_words, out_hits);
#else
            (void)device_mask;
            (void)cubes;
            (void)cube_count;
            (void)var_count;
            (void)off_blocks;
            (void)off_words;
            (void)out_hits;
            return false;
#endif
        }

        [[nodiscard]] inline ::std::uint64_t eval_u64_cone_cpu(cuda_u64_cone_desc const& c) noexcept
        {
            // Leaf patterns (variable i toggles every 2^i minterms, with bit0 corresponding to minterm 0).
            constexpr ::std::uint64_t leaf_pat[6] = {
                0xAAAAAAAAAAAAAAAAull,  // v0: 0101...
                0xCCCCCCCCCCCCCCCCull,  // v1: 0011...
                0xF0F0F0F0F0F0F0F0ull,  // v2: 00001111...
                0xFF00FF00FF00FF00ull,  // v3
                0xFFFF0000FFFF0000ull,  // v4
                0xFFFFFFFF00000000ull,  // v5
            };

            auto const vc = static_cast<::std::size_t>(c.var_count);
            if(vc > 6u) { return 0ull; }
            auto const U = (vc >= 6u) ? 64u : (static_cast<::std::size_t>(1u) << vc);
            auto const all_mask = (U == 64u) ? ~0ull : ((1ull << U) - 1ull);

            ::std::uint64_t val[6 + 64]{};
            for(::std::size_t i{}; i < vc; ++i) { val[i] = leaf_pat[i] & all_mask; }

            auto load = [&](::std::uint8_t idx) noexcept -> ::std::uint64_t
            {
                if(idx == 254u) { return 0ull; }
                if(idx == 255u) { return all_mask; }
                if(idx < (6u + 64u)) { return val[idx]; }
                return 0ull;
            };

            auto const gc = static_cast<::std::size_t>(c.gate_count);
            if(gc == 0u || gc > 64u) { return 0ull; }

            for(::std::size_t gi{}; gi < gc; ++gi)
            {
                auto const a = load(c.in0[gi]);
                auto const b = load(c.in1[gi]);
                ::std::uint64_t r{};
                switch(c.kind[gi])
                {
                    case 0u: r = ~a; break;            // NOT
                    case 1u: r = a & b; break;         // AND
                    case 2u: r = a | b; break;         // OR
                    case 3u: r = a ^ b; break;         // XOR
                    case 4u: r = ~(a ^ b); break;      // XNOR
                    case 5u: r = ~(a & b); break;      // NAND
                    case 6u: r = ~(a | b); break;      // NOR
                    case 7u: r = (~a) | b; break;      // IMP
                    case 8u: r = a & (~b); break;      // NIMP
                    default: r = 0ull; break;
                }
                val[6u + gi] = r & all_mask;
            }

            return val[6u + (gc - 1u)] & all_mask;
        }

        inline void eval_tt_cone_cpu(cuda_tt_cone_desc const& c, ::std::uint32_t stride_blocks, ::std::uint64_t* out_blocks) noexcept
        {
            if(out_blocks == nullptr || stride_blocks == 0u) { return; }

            constexpr ::std::uint64_t leaf_pat64[6] = {
                0xAAAAAAAAAAAAAAAAull,  // v0
                0xCCCCCCCCCCCCCCCCull,  // v1
                0xF0F0F0F0F0F0F0F0ull,  // v2
                0xFF00FF00FF00FF00ull,  // v3
                0xFFFF0000FFFF0000ull,  // v4
                0xFFFFFFFF00000000ull,  // v5
            };

            auto const vc = static_cast<::std::size_t>(c.var_count);
            if(vc > 16u) { return; }
            auto const U = static_cast<::std::size_t>(1u) << vc;
            auto const blocks = static_cast<::std::uint32_t>((U + 63u) / 64u);
            if(blocks == 0u || blocks > stride_blocks) { return; }

            auto const rem = static_cast<::std::uint32_t>(U & 63u);
            auto const last_mask = (rem == 0u) ? ~0ull : ((1ull << rem) - 1ull);

            auto leaf_word = [&](::std::size_t v, ::std::uint32_t w) noexcept -> ::std::uint64_t
            {
                if(v < 6u) { return leaf_pat64[v]; }
                // For w-th 64-bit block, minterm base is 64*w, so ((base>>v)&1) == ((w>>(v-6))&1) for v>=6.
                return (((w >> static_cast<::std::uint32_t>(v - 6u)) & 1u) != 0u) ? ~0ull : 0ull;
            };

            auto load = [&](::std::uint16_t idx, ::std::uint64_t const* val) noexcept -> ::std::uint64_t
            {
                if(idx == 65534u) { return 0ull; }
                if(idx == 65535u) { return ~0ull; }
                if(idx < (16u + 256u)) { return val[idx]; }
                return 0ull;
            };

            auto const gc = static_cast<::std::size_t>(c.gate_count);
            if(gc == 0u || gc > 256u) { return; }

            // Fill only the defined words; zero the rest for determinism.
            for(::std::uint32_t w = 0u; w < stride_blocks; ++w) { out_blocks[w] = 0ull; }

            for(::std::uint32_t w = 0u; w < blocks; ++w)
            {
                auto const word_mask = (w + 1u == blocks) ? last_mask : ~0ull;

                ::std::uint64_t val[16u + 256u]{};
                for(::std::size_t i{}; i < vc; ++i) { val[i] = leaf_word(i, w) & word_mask; }

                for(::std::size_t gi{}; gi < gc; ++gi)
                {
                    auto const a = load(c.in0[gi], val) & word_mask;
                    auto const b = load(c.in1[gi], val) & word_mask;
                    ::std::uint64_t r{};
                    switch(c.kind[gi])
                    {
                        case 0u: r = ~a; break;
                        case 1u: r = a & b; break;
                        case 2u: r = a | b; break;
                        case 3u: r = a ^ b; break;
                        case 4u: r = ~(a ^ b); break;
                        case 5u: r = ~(a & b); break;
                        case 6u: r = ~(a | b); break;
                        case 7u: r = (~a) | b; break;
                        case 8u: r = a & (~b); break;
                        default: r = 0ull; break;
                    }
                    val[16u + gi] = r & word_mask;
                }

                out_blocks[w] = val[16u + (gc - 1u)] & word_mask;
            }
        }

        [[nodiscard]] inline gate_opt_fanout build_gate_opt_fanout(::phy_engine::netlist::netlist& nl) noexcept
        {
            gate_opt_fanout info{};
            info.pin_out.reserve(1 << 16);
            info.consumer_count.reserve(1 << 14);
            info.driver_count.reserve(1 << 14);

            for(auto& blk: nl.models)
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
                        else
                        {
                            ++info.consumer_count[n];
                        }
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
            for(auto const* p: node->pins)
            {
                auto it = pin_out.find(p);
                bool const is_out = (it != pin_out.end()) ? it->second : false;
                if(!is_out) { continue; }
                ++drivers;
                if(p != expected_driver) { return false; }
            }
            return drivers == 1;
        }

        [[nodiscard]] inline bool optimize_fuse_inverters_in_pe_netlist(::phy_engine::netlist::netlist& nl,
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
            for(auto* n: protected_nodes) { protected_map.emplace(n, true); }
            auto const is_protected = [&](::phy_engine::model::node_t* n) noexcept -> bool { return protected_map.contains(n); };

            auto const fan = build_gate_opt_fanout(nl);

            ::std::unordered_map<::phy_engine::model::node_t*, bin_gate> gate_by_out{};
            gate_by_out.reserve(1 << 14);
            ::std::vector<not_gate> nots{};
            nots.reserve(1 << 14);

            auto classify_bin = [&](::phy_engine::model::model_base const& mb, ::phy_engine::netlist::model_pos pos) noexcept -> ::std::optional<bin_gate>
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
                else
                {
                    return ::std::nullopt;
                }

                return g;
            };

            auto classify_not = [&](::phy_engine::model::model_base const& mb, ::phy_engine::netlist::model_pos pos) noexcept -> ::std::optional<not_gate>
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

            auto const complement = [](bin_kind k) noexcept -> ::std::optional<bin_kind>
            {
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

            auto add_bin_gate = [&](bin_kind k, ::phy_engine::model::node_t* a, ::phy_engine::model::node_t* b, ::phy_engine::model::node_t* out) noexcept
                -> ::std::optional<::phy_engine::netlist::model_pos>
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
                if(!::phy_engine::netlist::add_to_node(nl, *r.mod, 0, *a) || !::phy_engine::netlist::add_to_node(nl, *r.mod, 1, *b))
                {
                    (void)::phy_engine::netlist::delete_model(nl, r.mod_pos);
                    return ::std::nullopt;
                }

                return r.mod_pos;
            };

            bool changed{};
            for(auto const& n: nots)
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
                if(nm == nullptr || nm->type != ::phy_engine::model::model_type::normal || nm->ptr == nullptr) { continue; }
                if(!::phy_engine::netlist::add_to_node(nl, *nm, 2, *n.out))
                {
                    (void)::phy_engine::netlist::delete_model(nl, *new_pos);
                    continue;
                }

                changed = true;
            }

            return changed;
        }

        [[nodiscard]] inline bool optimize_push_input_inverters_in_pe_netlist(::phy_engine::netlist::netlist& nl,
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
            for(auto* n: protected_nodes) { protected_map.emplace(n, true); }
            auto const is_protected = [&](::phy_engine::model::node_t* n) noexcept -> bool { return protected_map.contains(n); };

            auto const fan = build_gate_opt_fanout(nl);

            ::std::unordered_map<::phy_engine::model::node_t*, not_gate> not_by_out{};
            not_by_out.reserve(1 << 14);
            ::std::vector<bin_gate> bins{};
            bins.reserve(1 << 14);

            auto classify_not = [&](::phy_engine::model::model_base const& mb, ::phy_engine::netlist::model_pos pos) noexcept -> ::std::optional<not_gate>
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

            auto classify_bin = [&](::phy_engine::model::model_base const& mb, ::phy_engine::netlist::model_pos pos) noexcept -> ::std::optional<bin_gate>
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
                else
                {
                    g.k = kind::xnor_gate;
                }
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

            auto add_model_inputs_only =
                [&](::phy_engine::model::model_base* m, ::phy_engine::model::node_t* a, ::phy_engine::model::node_t* b) noexcept -> bool
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
                else
                {
                    return ::std::nullopt;
                }
                if(r.mod == nullptr) { return ::std::nullopt; }
                if(!add_model_inputs_only(r.mod, a, b))
                {
                    (void)::phy_engine::netlist::delete_model(nl, r.mod_pos);
                    return ::std::nullopt;
                }
                return r;
            };

            bool changed{};

            for(auto const& g: bins)
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
                if(inv0)
                {
                    n0 = it0->second;
                    if(!ok_not(n0)) { continue; }
                    a = n0.in;
                }
                if(inv1)
                {
                    n1 = it1->second;
                    if(!ok_not(n1)) { continue; }
                    b = n1.in;
                }

                ::fast_io::u8string_view new_name{};
                ::phy_engine::model::node_t* na{};
                ::phy_engine::model::node_t* nb{};

                // Select a cheaper equivalent primitive.
                if(g.k == kind::and_gate)
                {
                    if(inv0 && inv1)
                    {
                        new_name = u8"NOR";
                        na = a;
                        nb = b;
                    }  // ~a & ~b = ~(a|b)
                    else if(inv0)
                    {
                        new_name = u8"NIMP";
                        na = g.in1;
                        nb = a;
                    }  // b & ~a = NIMP(b,a)
                    else
                    {
                        new_name = u8"NIMP";
                        na = g.in0;
                        nb = b;
                    }  // a & ~b = NIMP(a,b)
                }
                else if(g.k == kind::or_gate)
                {
                    if(inv0 && inv1)
                    {
                        new_name = u8"NAND";
                        na = a;
                        nb = b;
                    }  // ~a | ~b = ~(a&b)
                    else if(inv0)
                    {
                        new_name = u8"IMP";
                        na = a;
                        nb = g.in1;
                    }  // ~a | b = IMP(a,b)
                    else
                    {
                        new_name = u8"IMP";
                        na = b;
                        nb = g.in0;
                    }  // a | ~b = IMP(b,a)
                }
                else if(g.k == kind::xor_gate)
                {
                    if(inv0 && inv1)
                    {
                        new_name = u8"XOR";
                        na = a;
                        nb = b;
                    }
                    else
                    {
                        new_name = u8"XNOR";
                        na = inv0 ? a : g.in0;
                        nb = inv1 ? b : g.in1;
                    }
                }
                else  // XNOR
                {
                    if(inv0 && inv1)
                    {
                        new_name = u8"XNOR";
                        na = a;
                        nb = b;
                    }
                    else
                    {
                        new_name = u8"XOR";
                        na = inv0 ? a : g.in0;
                        nb = inv1 ? b : g.in1;
                    }
                }

                if(new_name.empty()) { continue; }
                auto const r = add_bin(new_name, na, nb);
                if(!r) { continue; }

                // Remove old drivers before connecting the new output.
                (void)::phy_engine::netlist::delete_model(nl, g.pos);

                auto* nm = ::phy_engine::netlist::get_model(nl, r->mod_pos);
                if(nm == nullptr || nm->type != ::phy_engine::model::model_type::normal || nm->ptr == nullptr) { continue; }
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
                    for(auto const* p: n.out->pins)
                    {
                        if(p == n.out_pin) { continue; }
                        auto it = fan.pin_out.find(p);
                        bool const is_out = (it != fan.pin_out.end()) ? it->second : false;
                        if(!is_out)
                        {
                            has_consumer = true;
                            break;
                        }
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

        [[nodiscard]] inline bool optimize_factor_common_terms_in_pe_netlist(::phy_engine::netlist::netlist& nl,
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
            for(auto* n: protected_nodes) { protected_map.emplace(n, true); }
            auto const is_protected = [&](::phy_engine::model::node_t* n) noexcept -> bool { return protected_map.contains(n); };

            auto const fan = build_gate_opt_fanout(nl);

            ::std::unordered_map<::phy_engine::model::node_t*, bin_gate> gate_by_out{};
            gate_by_out.reserve(1 << 14);

            ::std::vector<bin_gate> ors{};
            ors.reserve(1 << 14);
            ::std::vector<bin_gate> ands{};
            ands.reserve(1 << 14);

            auto classify = [&](::phy_engine::model::model_base const& mb, ::phy_engine::netlist::model_pos pos) noexcept -> ::std::optional<bin_gate>
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
                else
                {
                    return ::std::nullopt;
                }

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
                    else
                    {
                        ands.push_back(*og);
                    }
                }
            }

            auto find_common =
                [](::phy_engine::model::node_t* a0, ::phy_engine::model::node_t* a1, ::phy_engine::model::node_t* b0, ::phy_engine::model::node_t* b1) noexcept
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
            for(auto const& gor: ors)
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
                        if(!ok || n == nullptr)
                        {
                            ok = false;
                            return;
                        }
                        if(visited.contains(n)) { return; }
                        visited.emplace(n, true);
                        if(or_nodes.size() + and_terms.size() >= max_nodes)
                        {
                            ok = false;
                            return;
                        }

                        auto it = gate_by_out.find(n);
                        if(it != gate_by_out.end() && it->second.k == kind::or_gate)
                        {
                            auto const& og = it->second;
                            if(og.out == nullptr || og.out_pin == nullptr)
                            {
                                ok = false;
                                return;
                            }
                            if(!has_unique_driver_pin(og.out, og.out_pin, fan.pin_out))
                            {
                                ok = false;
                                return;
                            }
                            if(!is_root)
                            {
                                if(!exclusive_out(og))
                                {
                                    ok = false;
                                    return;
                                }
                            }
                            or_nodes.push_back(og);
                            self(self, og.in0, false);
                            self(self, og.in1, false);
                            return;
                        }

                        if(it != gate_by_out.end() && it->second.k == kind::and_gate)
                        {
                            auto const& ag = it->second;
                            if(and_terms.size() >= max_terms)
                            {
                                ok = false;
                                return;
                            }
                            if(!exclusive_out(ag))
                            {
                                ok = false;
                                return;
                            }
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
                        for(auto& c: candidates)
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

                    for(auto* c: candidates)
                    {
                        if(c == nullptr) { continue; }
                        ::std::unordered_map<::phy_engine::model::node_t*, bool> seen{};
                        seen.reserve(and_terms.size() * 2u);
                        ::std::vector<::phy_engine::model::node_t*> others{};
                        others.reserve(and_terms.size());
                        for(auto const& t: and_terms)
                        {
                            if(t.in0 != c && t.in1 != c)
                            {
                                others.clear();
                                break;
                            }
                            auto* o = (t.in0 == c) ? t.in1 : t.in0;
                            if(o == nullptr)
                            {
                                others.clear();
                                break;
                            }
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

                        if(!best || cost < best->new_cost) { best = pick{.common = c, .others = ::std::move(others), .new_cost = cost}; }
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
                        if(best->others.size() == 1) { or_out_node = best->others[0]; }
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
                        if(!::phy_engine::netlist::add_to_node(nl, *mand, 0, *best->common) || !::phy_engine::netlist::add_to_node(nl, *mand, 1, *or_out_node))
                        {
                            (void)::phy_engine::netlist::delete_model(nl, pos);
                            return false;
                        }
                        mand_pos = pos;
                    }

                    // Delete old OR-tree and AND terms.
                    for(auto const& og: or_nodes) { (void)::phy_engine::netlist::delete_model(nl, og.pos); }
                    for(auto const& tg: and_terms) { (void)::phy_engine::netlist::delete_model(nl, tg.pos); }

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
                    for(auto* p: gor.out->pins)
                    {
                        auto it = fan.pin_out.find(p);
                        bool const is_out = (it != fan.pin_out.end()) ? it->second : false;
                        if(is_out) { continue; }
                        pins_to_move.push_back(p);
                    }
                    for(auto* p: pins_to_move)
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
                if(!::phy_engine::netlist::add_to_node(nl, *mor, 0, *x) || !::phy_engine::netlist::add_to_node(nl, *mor, 1, *y) ||
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
                if(!::phy_engine::netlist::add_to_node(nl, *mand, 0, *c) || !::phy_engine::netlist::add_to_node(nl, *mand, 1, *tnode))
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
                if(mand_m == nullptr || mand_m->type != ::phy_engine::model::model_type::normal || mand_m->ptr == nullptr) { continue; }
                if(!::phy_engine::netlist::add_to_node(nl, *mand_m, 2, *gor.out))
                {
                    (void)::phy_engine::netlist::delete_model(nl, mand_pos);
                    (void)::phy_engine::netlist::delete_model(nl, mor_pos);
                    continue;
                }

                changed = true;
            }

            for(auto const& gand: ands)
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
                        if(!ok || n == nullptr)
                        {
                            ok = false;
                            return;
                        }
                        if(visited.contains(n)) { return; }
                        visited.emplace(n, true);
                        if(and_nodes.size() + or_terms.size() >= max_nodes)
                        {
                            ok = false;
                            return;
                        }

                        auto it = gate_by_out.find(n);
                        if(it != gate_by_out.end() && it->second.k == kind::and_gate)
                        {
                            auto const& ag = it->second;
                            if(ag.out == nullptr || ag.out_pin == nullptr)
                            {
                                ok = false;
                                return;
                            }
                            if(!has_unique_driver_pin(ag.out, ag.out_pin, fan.pin_out))
                            {
                                ok = false;
                                return;
                            }
                            if(!is_root)
                            {
                                if(!exclusive_out(ag))
                                {
                                    ok = false;
                                    return;
                                }
                            }
                            and_nodes.push_back(ag);
                            self(self, ag.in0, false);
                            self(self, ag.in1, false);
                            return;
                        }

                        if(it != gate_by_out.end() && it->second.k == kind::or_gate)
                        {
                            auto const& og = it->second;
                            if(or_terms.size() >= max_terms)
                            {
                                ok = false;
                                return;
                            }
                            if(!exclusive_out(og))
                            {
                                ok = false;
                                return;
                            }
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
                        for(auto& c: candidates)
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

                    for(auto* c: candidates)
                    {
                        if(c == nullptr) { continue; }
                        ::std::unordered_map<::phy_engine::model::node_t*, bool> seen{};
                        seen.reserve(or_terms.size() * 2u);
                        ::std::vector<::phy_engine::model::node_t*> others{};
                        others.reserve(or_terms.size());
                        for(auto const& t: or_terms)
                        {
                            if(t.in0 != c && t.in1 != c)
                            {
                                others.clear();
                                break;
                            }
                            auto* o = (t.in0 == c) ? t.in1 : t.in0;
                            if(o == nullptr)
                            {
                                others.clear();
                                break;
                            }
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

                        if(!best || cost < best->new_cost) { best = pick{.common = c, .others = ::std::move(others), .new_cost = cost}; }
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
                        if(best->others.size() == 1) { and_out_node = best->others[0]; }
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
                        if(!::phy_engine::netlist::add_to_node(nl, *mor, 0, *best->common) || !::phy_engine::netlist::add_to_node(nl, *mor, 1, *and_out_node))
                        {
                            (void)::phy_engine::netlist::delete_model(nl, pos);
                            return false;
                        }
                        mor_pos = pos;
                    }

                    for(auto const& ag: and_nodes) { (void)::phy_engine::netlist::delete_model(nl, ag.pos); }
                    for(auto const& og: or_terms) { (void)::phy_engine::netlist::delete_model(nl, og.pos); }

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
                    for(auto* p: gand.out->pins)
                    {
                        auto it = fan.pin_out.find(p);
                        bool const is_out = (it != fan.pin_out.end()) ? it->second : false;
                        if(is_out) { continue; }
                        pins_to_move.push_back(p);
                    }
                    for(auto* p: pins_to_move)
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
                if(!::phy_engine::netlist::add_to_node(nl, *mand, 0, *x) || !::phy_engine::netlist::add_to_node(nl, *mand, 1, *y) ||
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
            for(auto* n: protected_nodes) { protected_map.emplace(n, true); }
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

            auto classify_not = [&](::phy_engine::model::model_base const& mb, ::phy_engine::netlist::model_pos pos) noexcept -> void
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

            auto classify_and = [&](::phy_engine::model::model_base const& mb, ::phy_engine::netlist::model_pos pos) noexcept -> void
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

            auto classify_nimp = [&](::phy_engine::model::model_base const& mb, ::phy_engine::netlist::model_pos pos) noexcept -> void
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

            auto classify_or = [&](::phy_engine::model::model_base const& mb, ::phy_engine::netlist::model_pos pos) noexcept -> void
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

            auto classify_and2 = [&](::phy_engine::model::model_base const& mb, ::phy_engine::netlist::model_pos pos) noexcept -> void
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
                if(auto itn = not_by_out.find(n); itn != not_by_out.end() && itn->second.in != nullptr) { return lit{itn->second.in, true}; }
                return lit{n, false};
            };

            auto canon2 = [](lit a, lit b) noexcept -> ::std::array<lit, 2>
            {
                if(reinterpret_cast<std::uintptr_t>(a.v) > reinterpret_cast<std::uintptr_t>(b.v)) { ::std::swap(a, b); }
                return {a, b};
            };

            auto add_xor_like =
                [&](bool is_xnor, ::phy_engine::model::node_t* a, ::phy_engine::model::node_t* b) noexcept -> ::std::optional<::phy_engine::netlist::model_pos>
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

            for(auto const& gor: ors)
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
            for(auto const& gand: ands)
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
            for(auto const& a: actions)
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
                if(nm == nullptr || nm->type != ::phy_engine::model::model_type::normal || nm->ptr == nullptr) { continue; }
                if(!::phy_engine::netlist::add_to_node(nl, *nm, 2, *a.out))
                {
                    (void)::phy_engine::netlist::delete_model(nl, *new_pos);
                    continue;
                }

                changed = true;
            }

            for(auto const& a: actions_pos)
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

        [[nodiscard]] inline bool optimize_aig_rewrite_in_pe_netlist(::phy_engine::netlist::netlist& nl,
                                                                     ::std::vector<::phy_engine::model::node_t*> const& protected_nodes,
                                                                     pe_synth_options const& opt) noexcept
        {
            if(!opt.assume_binary_inputs) { return false; }

            enum class kind : std::uint8_t
            {
                and_gate,
                or_gate,
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

            struct not_gate
            {
                ::phy_engine::netlist::model_pos pos{};
                ::phy_engine::model::node_t* in{};
                ::phy_engine::model::node_t* out{};
                ::phy_engine::model::pin const* out_pin{};
            };

            struct lit
            {
                ::phy_engine::model::node_t* base{};
                bool neg{};
                ::phy_engine::model::node_t* node{};
            };

            ::std::unordered_map<::phy_engine::model::node_t*, bool> protected_map{};
            protected_map.reserve(protected_nodes.size() * 2u + 1u);
            for(auto* n: protected_nodes) { protected_map.emplace(n, true); }
            auto const is_protected = [&](::phy_engine::model::node_t* n) noexcept -> bool { return protected_map.contains(n); };

            auto const fan = build_gate_opt_fanout(nl);

            ::std::unordered_map<::phy_engine::model::node_t*, not_gate> not_by_out{};
            not_by_out.reserve(1 << 14);
            ::std::unordered_map<::phy_engine::model::node_t*, gate> and_by_out{};
            ::std::unordered_map<::phy_engine::model::node_t*, gate> or_by_out{};
            and_by_out.reserve(1 << 14);
            or_by_out.reserve(1 << 14);

            auto classify_not = [&](::phy_engine::model::model_base const& mb, ::phy_engine::netlist::model_pos pos) noexcept -> ::std::optional<not_gate>
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

            auto classify_bin = [&](::phy_engine::model::model_base const& mb, ::phy_engine::netlist::model_pos pos) noexcept -> ::std::optional<gate>
            {
                if(mb.type != ::phy_engine::model::model_type::normal || mb.ptr == nullptr) { return ::std::nullopt; }
                auto const name = model_name_u8(mb);
                if(name != u8"AND" && name != u8"OR") { return ::std::nullopt; }
                auto pv = mb.ptr->generate_pin_view();
                if(pv.size != 3) { return ::std::nullopt; }
                gate g{};
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
                    if(auto on = classify_not(mb, {vec_pos, chunk_pos}); on && on->out != nullptr)
                    {
                        not_by_out.emplace(on->out, *on);
                        continue;
                    }
                    if(auto og = classify_bin(mb, {vec_pos, chunk_pos}); og && og->out != nullptr)
                    {
                        if(og->k == kind::and_gate) { and_by_out.emplace(og->out, *og); }
                        else
                        {
                            or_by_out.emplace(og->out, *og);
                        }
                    }
                }
            }

            auto make_lit = [&](::phy_engine::model::node_t* n) noexcept -> lit
            {
                if(n == nullptr) { return {}; }
                auto it = not_by_out.find(n);
                if(it == not_by_out.end()) { return lit{n, false, n}; }
                return lit{it->second.in, true, n};
            };

            auto same_lit = [](lit const& a, lit const& b) noexcept -> bool { return a.base == b.base && a.neg == b.neg; };
            auto comp_lit = [](lit const& a, lit const& b) noexcept -> bool { return a.base == b.base && a.neg != b.neg; };

            auto find_common_consensus = [&](lit const& a0, lit const& a1, lit const& b0, lit const& b1) noexcept -> ::std::optional<lit>
            {
                if(a0.base == nullptr || a1.base == nullptr || b0.base == nullptr || b1.base == nullptr) { return ::std::nullopt; }
                if(same_lit(a0, b0) && comp_lit(a1, b1)) { return a0; }
                if(same_lit(a0, b1) && comp_lit(a1, b0)) { return a0; }
                if(same_lit(a1, b0) && comp_lit(a0, b1)) { return a1; }
                if(same_lit(a1, b1) && comp_lit(a0, b0)) { return a1; }
                return ::std::nullopt;
            };

            auto move_consumers =
                [&](::phy_engine::model::node_t* from, ::phy_engine::model::node_t* to, ::phy_engine::model::pin const* from_driver_pin) noexcept -> bool
            {
                if(from == nullptr || to == nullptr || from_driver_pin == nullptr) { return false; }
                if(from == to) { return false; }
                if(is_protected(from)) { return false; }
                if(!has_unique_driver_pin(from, from_driver_pin, fan.pin_out)) { return false; }

                ::std::vector<::phy_engine::model::pin*> pins_to_move{};
                pins_to_move.reserve(from->pins.size());
                for(auto* p: from->pins)
                {
                    if(p == from_driver_pin) { continue; }
                    pins_to_move.push_back(p);
                }
                for(auto* p: pins_to_move)
                {
                    from->pins.erase(p);
                    p->nodes = to;
                    to->pins.insert(p);
                }
                return true;
            };

            auto replace_root = [&](gate const& root, ::phy_engine::model::node_t* repl) noexcept -> bool
            {
                if(repl == nullptr || root.out == nullptr || root.out_pin == nullptr) { return false; }
                if(is_protected(root.out))
                {
                    auto [buf, pos]{::phy_engine::netlist::add_model(nl, ::phy_engine::model::YES{})};
                    (void)pos;
                    if(buf == nullptr) { return false; }
                    if(!::phy_engine::netlist::add_to_node(nl, *buf, 0, *repl) || !::phy_engine::netlist::add_to_node(nl, *buf, 1, *root.out))
                    {
                        (void)::phy_engine::netlist::delete_model(nl, pos);
                        return false;
                    }
                    return true;
                }
                return move_consumers(root.out, repl, root.out_pin);
            };

            auto gate_exclusive = [&](gate const& g) noexcept -> bool
            {
                if(g.in0 == nullptr || g.in1 == nullptr || g.out == nullptr || g.out_pin == nullptr) { return false; }
                if(is_protected(g.out)) { return false; }
                auto const dc_it = fan.driver_count.find(g.out);
                auto const cc_it = fan.consumer_count.find(g.out);
                if(dc_it == fan.driver_count.end() || dc_it->second != 1) { return false; }
                if(cc_it == fan.consumer_count.end() || cc_it->second != 1) { return false; }
                return has_unique_driver_pin(g.out, g.out_pin, fan.pin_out);
            };

            auto gate_root_ok = [&](gate const& g) noexcept -> bool
            {
                if(g.in0 == nullptr || g.in1 == nullptr || g.out == nullptr || g.out_pin == nullptr) { return false; }
                auto const dc_it = fan.driver_count.find(g.out);
                if(dc_it == fan.driver_count.end() || dc_it->second != 1) { return false; }
                return has_unique_driver_pin(g.out, g.out_pin, fan.pin_out);
            };

            bool changed{};

            for(auto const& kv: or_by_out)
            {
                auto const& og = kv.second;
                if(!gate_root_ok(og)) { continue; }

                auto it_a = and_by_out.find(og.in0);
                auto it_b = and_by_out.find(og.in1);
                if(it_a == and_by_out.end() || it_b == and_by_out.end()) { continue; }
                auto const& ga = it_a->second;
                auto const& gb = it_b->second;
                if(!gate_exclusive(ga) || !gate_exclusive(gb)) { continue; }
                if(ga.out != og.in0 || gb.out != og.in1) { continue; }

                auto const a0 = make_lit(ga.in0);
                auto const a1 = make_lit(ga.in1);
                auto const b0 = make_lit(gb.in0);
                auto const b1 = make_lit(gb.in1);
                auto common = find_common_consensus(a0, a1, b0, b1);
                if(!common || common->node == nullptr) { continue; }

                if(!replace_root(og, common->node)) { continue; }
                (void)::phy_engine::netlist::delete_model(nl, og.pos);
                (void)::phy_engine::netlist::delete_model(nl, ga.pos);
                (void)::phy_engine::netlist::delete_model(nl, gb.pos);
                changed = true;
            }

            for(auto const& kv: and_by_out)
            {
                auto const& ag = kv.second;
                if(!gate_root_ok(ag)) { continue; }

                auto it_a = or_by_out.find(ag.in0);
                auto it_b = or_by_out.find(ag.in1);
                if(it_a == or_by_out.end() || it_b == or_by_out.end()) { continue; }
                auto const& ga = it_a->second;
                auto const& gb = it_b->second;
                if(!gate_exclusive(ga) || !gate_exclusive(gb)) { continue; }
                if(ga.out != ag.in0 || gb.out != ag.in1) { continue; }

                auto const a0 = make_lit(ga.in0);
                auto const a1 = make_lit(ga.in1);
                auto const b0 = make_lit(gb.in0);
                auto const b1 = make_lit(gb.in1);
                auto common = find_common_consensus(a0, a1, b0, b1);
                if(!common || common->node == nullptr) { continue; }

                if(!replace_root(ag, common->node)) { continue; }
                (void)::phy_engine::netlist::delete_model(nl, ag.pos);
                (void)::phy_engine::netlist::delete_model(nl, ga.pos);
                (void)::phy_engine::netlist::delete_model(nl, gb.pos);
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
            for(auto* n: protected_nodes) { protected_map.emplace(n, true); }
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

            auto move_consumers = [&](::phy_engine::model::node_t* from, ::phy_engine::model::node_t* to) noexcept -> bool
            {
                if(from == nullptr || to == nullptr || from == to) { return false; }
                if(is_protected(from)) { return false; }

                ::std::vector<::phy_engine::model::pin*> pins_to_move{};
                pins_to_move.reserve(from->pins.size());
                for(auto* p: from->pins)
                {
                    auto it = fan.pin_out.find(p);
                    bool const is_out = (it != fan.pin_out.end()) ? it->second : false;
                    if(is_out) { continue; }
                    pins_to_move.push_back(p);
                }

                for(auto* p: pins_to_move)
                {
                    from->pins.erase(p);
                    p->nodes = to;
                    to->pins.insert(p);
                }
                return true;
            };

            bool changed{};
            for(auto const& outer: nots)
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
            for(auto* n: protected_nodes) { protected_map.emplace(n, true); }
            auto const is_protected = [&](::phy_engine::model::node_t* n) noexcept -> bool { return protected_map.contains(n); };

            auto const fan = build_gate_opt_fanout(nl);

            using dns = ::phy_engine::model::digital_node_statement_t;

            // Build node->(0/1) constant map from unnamed INPUT drivers.
            ::std::unordered_map<::phy_engine::model::node_t*, dns> node_const{};
            node_const.reserve(1 << 14);
            for(auto& blk: nl.models)
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

            auto move_consumers = [&](::phy_engine::model::node_t* from, ::phy_engine::model::node_t* to) noexcept -> bool
            {
                if(from == nullptr || to == nullptr || from == to) { return false; }
                if(is_protected(from)) { return false; }
                ::std::vector<::phy_engine::model::pin*> pins_to_move{};
                pins_to_move.reserve(from->pins.size());
                for(auto* p: from->pins)
                {
                    auto it = fan.pin_out.find(p);
                    bool const is_out = (it != fan.pin_out.end()) ? it->second : false;
                    if(is_out) { continue; }
                    pins_to_move.push_back(p);
                }
                for(auto* p: pins_to_move)
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
                    if(nm != u8"NOT" && nm != u8"AND" && nm != u8"OR" && nm != u8"XOR" && nm != u8"XNOR" && nm != u8"NAND" && nm != u8"NOR" && nm != u8"IMP" &&
                       nm != u8"NIMP")
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
            if(opt.rewrite_max_candidates != 0u && cands.size() > opt.rewrite_max_candidates) { cands.resize(opt.rewrite_max_candidates); }

            bool changed{};
            for(auto const& g: cands)
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
                    else
                    {
                        continue;
                    }
                }
                else if(g.name == u8"OR")
                {
                    if(aconst && av == dns::true_state) { replacement = find_or_make_const_node(nl, dns::true_state); }
                    else if(bconst && bv == dns::true_state) { replacement = find_or_make_const_node(nl, dns::true_state); }
                    else if(aconst && av == dns::false_state) { replacement = g.b; }
                    else if(bconst && bv == dns::false_state) { replacement = g.a; }
                    else if(g.a != nullptr && g.a == g.b) { replacement = g.a; }
                    else
                    {
                        continue;
                    }
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
                    else if(opt.assume_binary_inputs && g.a != nullptr && g.a == g.b) { replacement = find_or_make_const_node(nl, dns::false_state); }
                    else
                    {
                        continue;
                    }
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
                    else if(opt.assume_binary_inputs && g.a != nullptr && g.a == g.b) { replacement = find_or_make_const_node(nl, dns::true_state); }
                    else
                    {
                        continue;
                    }
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
                    else
                    {
                        continue;
                    }
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
                    else
                    {
                        continue;
                    }
                }
                else if(g.name == u8"IMP")
                {
                    // IMP(a,b) = ~a | b
                    if(bconst && bv == dns::true_state) { replacement = find_or_make_const_node(nl, dns::true_state); }
                    else if(aconst && av == dns::false_state) { replacement = find_or_make_const_node(nl, dns::true_state); }
                    else if(aconst && av == dns::true_state) { replacement = g.b; }
                    else
                    {
                        continue;
                    }
                }
                else if(g.name == u8"NIMP")
                {
                    // NIMP(a,b) = a & ~b
                    if(aconst && av == dns::false_state) { replacement = find_or_make_const_node(nl, dns::false_state); }
                    else if(bconst && bv == dns::true_state) { replacement = find_or_make_const_node(nl, dns::false_state); }
                    else if(bconst && bv == dns::false_state) { replacement = g.a; }
                    else
                    {
                        continue;
                    }
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
            for(auto* n: protected_nodes) { protected_map.emplace(n, true); }
            auto const is_protected = [&](::phy_engine::model::node_t* n) noexcept -> bool { return protected_map.contains(n); };

            auto const fan = build_gate_opt_fanout(nl);

            ::std::unordered_map<::phy_engine::model::node_t*, bin_gate> gate_by_out{};
            gate_by_out.reserve(1 << 14);
            ::std::vector<bin_gate> parents{};
            parents.reserve(1 << 14);

            auto classify = [&](::phy_engine::model::model_base const& mb, ::phy_engine::netlist::model_pos pos) noexcept -> ::std::optional<bin_gate>
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

            auto move_consumers = [&](::phy_engine::model::node_t* from, ::phy_engine::model::node_t* to) noexcept -> bool
            {
                if(from == nullptr || to == nullptr || from == to) { return false; }
                if(is_protected(from)) { return false; }
                ::std::vector<::phy_engine::model::pin*> pins_to_move{};
                pins_to_move.reserve(from->pins.size());
                for(auto* p: from->pins)
                {
                    auto it = fan.pin_out.find(p);
                    bool const is_out = (it != fan.pin_out.end()) ? it->second : false;
                    if(is_out) { continue; }
                    pins_to_move.push_back(p);
                }
                for(auto* p: pins_to_move)
                {
                    from->pins.erase(p);
                    p->nodes = to;
                    to->pins.insert(p);
                }
                return true;
            };

            bool changed{};
            for(auto const& pg: parents)
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
            for(auto* n: protected_nodes) { protected_map.emplace(n, true); }
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
                for(auto* p: from->pins)
                {
                    auto it = fan.pin_out.find(p);
                    bool const is_out = (it != fan.pin_out.end()) ? it->second : false;
                    if(is_out) { continue; }
                    pins_to_move.push_back(p);
                }
                for(auto* p: pins_to_move)
                {
                    from->pins.erase(p);
                    p->nodes = to;
                    to->pins.insert(p);
                }
                return true;
            };

            auto make_const = [&](bool v) noexcept -> ::phy_engine::model::node_t*
            {
                return find_or_make_const_node(nl,
                                               v ? ::phy_engine::model::digital_node_statement_t::true_state
                                                 : ::phy_engine::model::digital_node_statement_t::false_state);
            };

            bool changed{};

            // Simplify OR pairs: (x&y)|(x&~y) -> x, and (x&y)|(~x&y) -> y.
            for(auto const& og: ors)
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
                else
                {
                    rep = t0[1].v;
                }
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
            for(auto const& ag: ands)
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
                if(same0)
                {
                    rep = t0[0].v;
                    rep_neg = t0[0].neg;
                }
                else
                {
                    rep = t0[1].v;
                    rep_neg = t0[1].neg;
                }
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
            for(auto const& og: ors)
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
            for(auto const& ag: ands)
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
            for(auto* n: protected_nodes) { protected_map.emplace(n, true); }
            auto const is_protected = [&](::phy_engine::model::node_t* n) noexcept -> bool { return protected_map.contains(n); };

            auto const fan = build_gate_opt_fanout(nl);

            using dns = ::phy_engine::model::digital_node_statement_t;
            ::std::unordered_map<::phy_engine::model::node_t*, dns> node_const{};
            node_const.reserve(1 << 14);
            for(auto& blk: nl.models)
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
                for(auto* p: from->pins)
                {
                    auto it = fan.pin_out.find(p);
                    bool const is_out = (it != fan.pin_out.end()) ? it->second : false;
                    if(is_out) { continue; }
                    pins_to_move.push_back(p);
                }
                for(auto* p: pins_to_move)
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
            { return find_or_make_const_node(nl, v ? dns::true_state : dns::false_state); };

            bool changed{};
            for(auto const& root: roots)
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
                    if(visited.contains(n))
                    {
                        leaves.push_back(as_leaf(n));
                        return;
                    }
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

                for(auto const& l: leaves)
                {
                    if(l.node == nullptr || l.base == nullptr) { continue; }

                    bool cv{};
                    if(get_const(l.node, cv))
                    {
                        any_const = true;
                        const_value = cv;
                        if(root.k == k2::and_gate)
                        {
                            if(!cv)
                            {
                                force_const = true;
                                force_const_value = false;
                                break;
                            }
                            // true leaf can be dropped
                            continue;
                        }
                        else
                        {
                            if(cv)
                            {
                                force_const = true;
                                force_const_value = true;
                                break;
                            }
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
                    for(auto const mp: to_delete) { (void)::phy_engine::netlist::delete_model(nl, mp); }
                    changed = true;
                    continue;
                }

                // Binary-only complement check across uniq set.
                bool did_root_rewrite{};
                if(opt.assume_binary_inputs)
                {
                    ::std::unordered_map<::phy_engine::model::node_t*, unsigned> seen_mask{};
                    seen_mask.reserve(uniq.size() * 2u);
                    for(auto const& kv: uniq)
                    {
                        auto const& l = kv.second;
                        if(l.base == nullptr) { continue; }
                        auto& m = seen_mask[l.base];
                        m |= l.neg ? 2u : 1u;
                        if(m == 3u)
                        {
                            bool const v = (root.k == k2::or_gate);
                            auto* rep = make_const_node(v);
                            if(rep == nullptr)
                            {
                                did_root_rewrite = false;
                                break;
                            }
                            if(!move_consumers(root.out, rep))
                            {
                                did_root_rewrite = false;
                                break;
                            }
                            (void)::phy_engine::netlist::delete_model(nl, root.pos);
                            for(auto const mp: to_delete) { (void)::phy_engine::netlist::delete_model(nl, mp); }
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
                    for(auto const& kv: uniq) { flat.push_back(kv.second); }

                    // If everything dropped (e.g. AND of 1's, OR of 0's), reduce to identity constant.
                    if(flat.empty())
                    {
                        bool const v = (root.k == k2::and_gate);
                        auto* rep = make_const_node(v);
                        if(rep == nullptr) { continue; }
                        if(!move_consumers(root.out, rep)) { continue; }
                        (void)::phy_engine::netlist::delete_model(nl, root.pos);
                        for(auto const mp: to_delete) { (void)::phy_engine::netlist::delete_model(nl, mp); }
                        changed = true;
                        continue;
                    }

                    // Canonical order: sort by (base ptr, neg).
                    ::std::sort(flat.begin(),
                                flat.end(),
                                [](leaf const& x, leaf const& y) noexcept
                                {
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
                        for(auto const mp: to_delete) { (void)::phy_engine::netlist::delete_model(nl, mp); }
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
                    for(auto const mp: to_delete) { (void)::phy_engine::netlist::delete_model(nl, mp); }
                    changed = true;
                    continue;

                rollback_new:
                    for(auto const mp: new_models) { (void)::phy_engine::netlist::delete_model(nl, mp); }
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
                std::size_t operator() (key const& x) const noexcept
                {
                    auto const mix = [](std::size_t h, std::size_t v) noexcept -> std::size_t
                    { return (h ^ (v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2))); };
                    std::size_t h{};
                    h = mix(h, static_cast<std::size_t>(x.k));
                    h = mix(h, reinterpret_cast<std::size_t>(x.a));
                    h = mix(h, reinterpret_cast<std::size_t>(x.b));
                    return h;
                }
            };

            struct key_eq
            {
                bool operator() (key const& x, key const& y) const noexcept { return x.k == y.k && x.a == y.a && x.b == y.b; }
            };

            ::std::unordered_map<::phy_engine::model::node_t*, bool> protected_map{};
            protected_map.reserve(protected_nodes.size() * 2u + 1u);
            for(auto* n: protected_nodes) { protected_map.emplace(n, true); }
            auto const is_protected = [&](::phy_engine::model::node_t* n) noexcept -> bool { return protected_map.contains(n); };

            auto const fan = build_gate_opt_fanout(nl);

            auto canon_key = [](kind k, ::phy_engine::model::node_t* a, ::phy_engine::model::node_t* b) noexcept -> key
            {
                auto const commutative =
                    (k == kind::and_gate || k == kind::or_gate || k == kind::xor_gate || k == kind::xnor_gate || k == kind::nand_gate || k == kind::nor_gate);
                if(commutative && reinterpret_cast<std::uintptr_t>(a) > reinterpret_cast<std::uintptr_t>(b)) { ::std::swap(a, b); }
                return key{k, a, b};
            };

            auto classify = [&](::phy_engine::model::model_base const& mb, ::phy_engine::netlist::model_pos pos) noexcept -> ::std::optional<gate>
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
                    else
                    {
                        g.k = kind::nimp_gate;
                    }
                    return g;
                }

                return ::std::nullopt;
            };

            auto move_consumers =
                [&](::phy_engine::model::node_t* from, ::phy_engine::model::node_t* to, ::phy_engine::model::pin const* from_driver_pin) noexcept -> bool
            {
                if(from == nullptr || to == nullptr || from_driver_pin == nullptr) { return false; }
                if(from == to) { return false; }
                if(is_protected(from)) { return false; }
                if(!has_unique_driver_pin(from, from_driver_pin, fan.pin_out)) { return false; }

                ::std::vector<::phy_engine::model::pin*> pins_to_move{};
                pins_to_move.reserve(from->pins.size());
                for(auto* p: from->pins)
                {
                    if(p == from_driver_pin) { continue; }
                    pins_to_move.push_back(p);
                }
                for(auto* p: pins_to_move)
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
                    if(r.out == nullptr || r.out_pin == nullptr)
                    {
                        r = *og;
                        continue;
                    }

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

        [[nodiscard]] inline bool optimize_cut_based_techmap_in_pe_netlist(::phy_engine::netlist::netlist& nl,
                                                                           ::std::vector<::phy_engine::model::node_t*> const& protected_nodes,
                                                                           pe_synth_options const& opt) noexcept
        {
            if(!opt.assume_binary_inputs || !opt.techmap_enable) { return false; }

            std::size_t const max_cut = (opt.techmap_max_cut > 4u) ? 4u : opt.techmap_max_cut;
            if(max_cut == 0u) { return false; }
            std::size_t const max_gates = (opt.techmap_max_gates == 0u) ? 64u : opt.techmap_max_gates;
            std::size_t const max_cuts = (opt.techmap_max_cuts == 0u) ? 16u : opt.techmap_max_cuts;

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

            struct cut
            {
                ::std::vector<::phy_engine::model::node_t*> leaves{};
            };

            struct choice
            {
                bool valid{};
                std::size_t pattern_idx{};
                ::std::vector<::phy_engine::model::node_t*> leaves{};
                ::std::array<unsigned, 4> perm{};
                ::std::uint32_t neg_mask{};
                std::size_t inputs{};
            };

            enum class pkind : std::uint8_t
            {
                not1,
                and2,
                or2,
                xor2,
                xnor2,
                nand2,
                nor2,
                imp2,
                nimp2,
                aoi21,
                oai21,
                aoi22,
                oai22,
            };

            struct pattern
            {
                pkind k{};
                std::size_t inputs{};
                ::std::uint64_t truth{};
                std::size_t cost{};
            };

            ::std::unordered_map<::phy_engine::model::node_t*, bool> protected_map{};
            protected_map.reserve(protected_nodes.size() * 2u + 1u);
            for(auto* n: protected_nodes) { protected_map.emplace(n, true); }
            auto const is_protected = [&](::phy_engine::model::node_t* n) noexcept -> bool { return protected_map.contains(n); };

            auto const fan = build_gate_opt_fanout(nl);

            ::std::unordered_map<::phy_engine::model::node_t*, gate> gate_by_out{};
            gate_by_out.reserve(1 << 14);

            auto classify = [&](::phy_engine::model::model_base const& mb, ::phy_engine::netlist::model_pos pos) noexcept -> ::std::optional<gate>
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
                    else
                    {
                        g.k = kind::nimp_gate;
                    }
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
                    if(!og || og->out == nullptr || og->out_pin == nullptr) { continue; }
                    gate_by_out.emplace(og->out, *og);
                }
            }

            // Detect constant nodes produced by unnamed INPUT models.
            ::std::unordered_map<::phy_engine::model::node_t*, ::phy_engine::model::digital_node_statement_t> const_val{};
            const_val.reserve(128);
            for(auto const& blk : nl.models)
            {
                for(auto const* m = blk.begin; m != blk.curr; ++m)
                {
                    if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }
                    if(model_name_u8(*m) != u8"INPUT") { continue; }
                    if(m->name.size() != 0) { continue; }
                    auto pv = m->ptr->generate_pin_view();
                    if(pv.size != 1 || pv.pins[0].nodes == nullptr) { continue; }
                    auto vi = m->ptr->get_attribute(0);
                    if(vi.type != ::phy_engine::model::variant_type::digital) { continue; }
                    const_val.emplace(pv.pins[0].nodes, vi.digital);
                }
            }

            auto truth_mask = [&](std::size_t inputs, auto&& fn) noexcept -> ::std::uint64_t
            {
                ::std::uint64_t mask{};
                auto const U = static_cast<std::size_t>(1u << inputs);
                for(std::size_t m{}; m < U; ++m)
                {
                    bool args[4]{};
                    for(std::size_t i{}; i < inputs && i < 4; ++i) { args[i] = ((m >> i) & 1u) != 0u; }
                    if(fn(args)) { mask |= (1ull << m); }
                }
                return mask;
            };

            ::std::vector<pattern> patterns{};
            patterns.reserve(16);
            patterns.push_back(pattern{pkind::not1, 1u, truth_mask(1u, [](bool const* a) noexcept { return !a[0]; }), 1u});
            patterns.push_back(pattern{pkind::and2, 2u, truth_mask(2u, [](bool const* a) noexcept { return a[0] & a[1]; }), 1u});
            patterns.push_back(pattern{pkind::or2, 2u, truth_mask(2u, [](bool const* a) noexcept { return a[0] | a[1]; }), 1u});
            patterns.push_back(pattern{pkind::xor2, 2u, truth_mask(2u, [](bool const* a) noexcept { return a[0] ^ a[1]; }), 1u});
            patterns.push_back(pattern{pkind::xnor2, 2u, truth_mask(2u, [](bool const* a) noexcept { return !(a[0] ^ a[1]); }), 1u});
            patterns.push_back(pattern{pkind::nand2, 2u, truth_mask(2u, [](bool const* a) noexcept { return !(a[0] & a[1]); }), 1u});
            patterns.push_back(pattern{pkind::nor2, 2u, truth_mask(2u, [](bool const* a) noexcept { return !(a[0] | a[1]); }), 1u});
            patterns.push_back(pattern{pkind::imp2, 2u, truth_mask(2u, [](bool const* a) noexcept { return (!a[0]) | a[1]; }), 1u});
            patterns.push_back(pattern{pkind::nimp2, 2u, truth_mask(2u, [](bool const* a) noexcept { return a[0] & (!a[1]); }), 1u});

            if(opt.techmap_richer_library && max_cut >= 3u)
            {
                patterns.push_back(pattern{pkind::aoi21, 3u, truth_mask(3u, [](bool const* a) noexcept { return !((a[0] & a[1]) | a[2]); }), 3u});
                patterns.push_back(pattern{pkind::oai21, 3u, truth_mask(3u, [](bool const* a) noexcept { return !((a[0] | a[1]) & a[2]); }), 3u});
            }
            if(opt.techmap_richer_library && max_cut >= 4u)
            {
                patterns.push_back(pattern{pkind::aoi22, 4u, truth_mask(4u, [](bool const* a) noexcept { return !((a[0] & a[1]) | (a[2] & a[3])); }), 4u});
                patterns.push_back(pattern{pkind::oai22, 4u, truth_mask(4u, [](bool const* a) noexcept { return !((a[0] | a[1]) & (a[2] | a[3])); }), 4u});
            }

            [[maybe_unused]] auto eval_gate = [&](auto&& self,
                                 ::phy_engine::model::node_t* node,
                                 ::std::unordered_map<::phy_engine::model::node_t*, ::phy_engine::model::digital_node_statement_t> const& leaf_val,
                                 ::std::unordered_map<::phy_engine::model::node_t*, ::phy_engine::model::digital_node_statement_t>& memo,
                                 ::std::unordered_map<::phy_engine::model::node_t*, bool>& visiting) noexcept -> ::phy_engine::model::digital_node_statement_t
            {
                if(node == nullptr) { return ::phy_engine::model::digital_node_statement_t::indeterminate_state; }
                if(auto it = memo.find(node); it != memo.end()) { return it->second; }
                if(auto itc = const_val.find(node); itc != const_val.end())
                {
                    memo.emplace(node, itc->second);
                    return itc->second;
                }
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
                if(visiting.contains(node)) { return ::phy_engine::model::digital_node_statement_t::indeterminate_state; }
                visiting.emplace(node, true);
                auto const& g = itg->second;
                using dns = ::phy_engine::model::digital_node_statement_t;
                dns a = self(self, g.in0, leaf_val, memo, visiting);
                dns b = self(self, g.in1, leaf_val, memo, visiting);
                dns r{};
                switch(g.k)
                {
                    case kind::not_gate: r = ~a; break;
                    case kind::and_gate: r = (a & b); break;
                    case kind::or_gate: r = (a | b); break;
                    case kind::xor_gate: r = (a ^ b); break;
                    case kind::xnor_gate: r = ~(a ^ b); break;
                    case kind::nand_gate: r = ~(a & b); break;
                    case kind::nor_gate: r = ~(a | b); break;
                    case kind::imp_gate: r = ((~a) | b); break;
                    case kind::nimp_gate: r = (a & (~b)); break;
                    default: r = dns::indeterminate_state; break;
                }
                visiting.erase(node);
                memo.emplace(node, r);
                return r;
            };

            auto enc_kind = [](kind k) noexcept -> ::std::uint8_t
            {
                switch(k)
                {
                    case kind::not_gate: return 0u;
                    case kind::and_gate: return 1u;
                    case kind::or_gate: return 2u;
                    case kind::xor_gate: return 3u;
                    case kind::xnor_gate: return 4u;
                    case kind::nand_gate: return 5u;
                    case kind::nor_gate: return 6u;
                    case kind::imp_gate: return 7u;
                    case kind::nimp_gate: return 8u;
                    default: return 1u;
                }
            };

            auto build_u64_mask_cone = [&](::phy_engine::model::node_t* root,
                                           ::std::vector<::phy_engine::model::node_t*> const& leaves,
                                           cuda_u64_cone_desc& out) noexcept -> bool
            {
                auto const n = leaves.size();
                if(n == 0u || n > 6u) { return false; }

                ::std::unordered_map<::phy_engine::model::node_t*, ::std::uint8_t> leaf_index{};
                leaf_index.reserve(n * 2u);
                for(std::size_t i{}; i < n; ++i) { leaf_index.emplace(leaves[i], static_cast<::std::uint8_t>(i)); }

                cuda_u64_cone_desc cone{};
                cone.var_count = static_cast<::std::uint8_t>(n);
                cone.gate_count = 0u;

                ::std::unordered_map<::phy_engine::model::node_t*, ::std::uint8_t> out_index{};
                out_index.reserve(128);
                bool ok{true};

                auto dfs2 = [&](auto&& self, ::phy_engine::model::node_t* n0) noexcept -> ::std::uint8_t
                {
                    if(!ok || n0 == nullptr) { ok = false; return 254u; }
                    if(auto itc = const_val.find(n0); itc != const_val.end())
                    {
                        using dns = ::phy_engine::model::digital_node_statement_t;
                        if(itc->second == dns::false_state) { return 254u; }
                        if(itc->second == dns::true_state) { return 255u; }
                        ok = false;
                        return 254u;
                    }
                    if(auto itl = leaf_index.find(n0); itl != leaf_index.end()) { return itl->second; }
                    if(auto ito = out_index.find(n0); ito != out_index.end()) { return ito->second; }

                    auto itg = gate_by_out.find(n0);
                    if(itg == gate_by_out.end()) { ok = false; return 254u; }
                    auto const& gg = itg->second;
                    auto const a = self(self, gg.in0);
                    auto const b = (gg.k == kind::not_gate) ? static_cast<::std::uint8_t>(254u) : self(self, gg.in1);
                    if(!ok) { return 254u; }

                    if(cone.gate_count >= 64u) { ok = false; return 254u; }
                    auto const gi = static_cast<::std::uint8_t>(cone.gate_count++);
                    cone.kind[gi] = enc_kind(gg.k);
                    cone.in0[gi] = a;
                    cone.in1[gi] = b;
                    auto const outi = static_cast<::std::uint8_t>(6u + gi);
                    out_index.emplace(n0, outi);
                    return outi;
                };

                (void)dfs2(dfs2, root);
                if(!ok || cone.gate_count == 0u) { return false; }
                out = cone;
                return true;
            };

            auto match_best = [&](::std::uint64_t target,
                                  std::vector<::phy_engine::model::node_t*> const& leaves,
                                  std::size_t leaf_cost) noexcept -> ::std::optional<::std::pair<choice, std::size_t>>
            {
                auto const n = leaves.size();
                if(n == 0 || n > 4u) { return ::std::nullopt; }
                ::std::array<unsigned, 4> base_perm{};
                for(std::size_t i{}; i < n; ++i) { base_perm[i] = static_cast<unsigned>(i); }

                auto make_variant = [&](pattern const& pat, ::std::array<unsigned, 4> const& perm, ::std::uint32_t neg_mask) noexcept -> ::std::uint64_t
                {
                    auto const U = static_cast<std::size_t>(1u << n);
                    ::std::uint64_t mask{};
                    for(std::size_t m{}; m < U; ++m)
                    {
                        std::size_t p_m{};
                        for(std::size_t i{}; i < n; ++i)
                        {
                            bool bit = ((m >> perm[i]) & 1u) != 0u;
                            if((neg_mask >> i) & 1u) { bit = !bit; }
                            if(bit) { p_m |= (1u << i); }
                        }
                        if(((pat.truth >> p_m) & 1u) != 0u) { mask |= (1ull << m); }
                    }
                    return mask;
                };

                bool found{};
                choice best_choice{};
                std::size_t best_cost{static_cast<std::size_t>(-1)};

                for(std::size_t pi{}; pi < patterns.size(); ++pi)
                {
                    auto const& pat = patterns[pi];
                    if(pat.inputs != n) { continue; }

                    ::std::array<unsigned, 4> perm = base_perm;
                    do
                    {
                        auto const neg_max = static_cast<std::uint32_t>(1u << n);
                        for(::std::uint32_t neg{}; neg < neg_max; ++neg)
                        {
                            auto const vmask = make_variant(pat, perm, neg);
                            if(vmask != target) { continue; }
                            auto const inv_cost = static_cast<std::size_t>(__builtin_popcount(neg));
                            auto const total = pat.cost + inv_cost + leaf_cost;
                            if(!found || total < best_cost)
                            {
                                found = true;
                                best_cost = total;
                                best_choice.valid = true;
                                best_choice.pattern_idx = pi;
                                best_choice.leaves = leaves;
                                best_choice.perm = perm;
                                best_choice.neg_mask = neg;
                                best_choice.inputs = n;
                            }
                        }
                    }
                    while(::std::next_permutation(perm.begin(), perm.begin() + static_cast<std::ptrdiff_t>(n)));
                }

                if(!found) { return ::std::nullopt; }
                return ::std::make_pair(best_choice, best_cost);
            };

            auto merge_leaves = [&](::std::vector<::phy_engine::model::node_t*> const& a,
                                    ::std::vector<::phy_engine::model::node_t*> const& b,
                                    ::std::vector<::phy_engine::model::node_t*>& out) noexcept -> bool
            {
                out.clear();
                out.reserve(a.size() + b.size());
                std::size_t i{};
                std::size_t j{};
                while(i < a.size() || j < b.size())
                {
                    if(j >= b.size() || (i < a.size() && reinterpret_cast<std::uintptr_t>(a[i]) < reinterpret_cast<std::uintptr_t>(b[j])))
                    {
                        out.push_back(a[i++]);
                    }
                    else if(i >= a.size() || reinterpret_cast<std::uintptr_t>(b[j]) < reinterpret_cast<std::uintptr_t>(a[i])) { out.push_back(b[j++]); }
                    else
                    {
                        out.push_back(a[i]);
                        ++i;
                        ++j;
                    }
                    if(out.size() > max_cut) { return false; }
                }
                return true;
            };

            auto move_consumers_all = [&](::phy_engine::model::node_t* from, ::phy_engine::model::node_t* to) noexcept -> bool
            {
                if(from == nullptr || to == nullptr || from == to) { return false; }
                if(is_protected(from)) { return false; }
                ::std::vector<::phy_engine::model::pin*> pins_to_move{};
                pins_to_move.reserve(from->pins.size());
                for(auto* p: from->pins)
                {
                    auto it = fan.pin_out.find(p);
                    bool const is_out = (it != fan.pin_out.end()) ? it->second : false;
                    if(is_out) { continue; }
                    pins_to_move.push_back(p);
                }
                for(auto* p: pins_to_move)
                {
                    from->pins.erase(p);
                    p->nodes = to;
                    to->pins.insert(p);
                }
                return true;
            };

            struct cone
            {
                ::phy_engine::model::node_t* root{};
                ::std::vector<::phy_engine::model::node_t*> leaves{};
                ::std::vector<::phy_engine::model::node_t*> topo{};
                ::std::vector<::phy_engine::netlist::model_pos> to_delete{};
                ::std::unordered_map<::phy_engine::model::node_t*, bool> internal{};
            };

            auto collect_cone = [&](::phy_engine::model::node_t* root, cone& c) noexcept -> bool
            {
                if(root == nullptr) { return false; }
                auto it_root = gate_by_out.find(root);
                if(it_root == gate_by_out.end()) { return false; }
                auto const& rg = it_root->second;
                if(rg.out_pin == nullptr) { return false; }
                if(!has_unique_driver_pin(root, rg.out_pin, fan.pin_out)) { return false; }

                c.root = root;
                c.leaves.clear();
                c.topo.clear();
                c.to_delete.clear();
                c.internal.clear();
                c.internal.reserve(max_gates * 2u);

                ::std::unordered_map<::phy_engine::model::node_t*, bool> leaf_seen{};
                leaf_seen.reserve(max_cut * 2u);
                ::std::unordered_map<::phy_engine::model::node_t*, bool> visited{};
                visited.reserve(max_gates * 2u);
                bool ok{true};
                std::size_t gate_count{};

                auto dfs = [&](auto&& self, ::phy_engine::model::node_t* n, bool is_root) noexcept -> void
                {
                    if(!ok || n == nullptr)
                    {
                        ok = false;
                        return;
                    }
                    if(visited.contains(n)) { return; }
                    visited.emplace(n, true);

                    if(!is_root && is_protected(n))
                    {
                        if(!leaf_seen.contains(n))
                        {
                            leaf_seen.emplace(n, true);
                            c.leaves.push_back(n);
                        }
                        return;
                    }

                    auto itg = gate_by_out.find(n);
                    if(itg == gate_by_out.end())
                    {
                        if(!leaf_seen.contains(n))
                        {
                            leaf_seen.emplace(n, true);
                            c.leaves.push_back(n);
                        }
                        return;
                    }

                    auto const& g = itg->second;
                    if(!is_root)
                    {
                        auto itd = fan.driver_count.find(n);
                        auto itc = fan.consumer_count.find(n);
                        if(itd == fan.driver_count.end() || itc == fan.consumer_count.end() || itd->second != 1u || itc->second != 1u)
                        {
                            if(!leaf_seen.contains(n))
                            {
                                leaf_seen.emplace(n, true);
                                c.leaves.push_back(n);
                            }
                            return;
                        }
                        if(g.out_pin == nullptr || !has_unique_driver_pin(n, g.out_pin, fan.pin_out))
                        {
                            if(!leaf_seen.contains(n))
                            {
                                leaf_seen.emplace(n, true);
                                c.leaves.push_back(n);
                            }
                            return;
                        }
                    }

                    if(g.in0 == nullptr || (g.k != kind::not_gate && g.in1 == nullptr))
                    {
                        ok = false;
                        return;
                    }

                    if(++gate_count > max_gates)
                    {
                        ok = false;
                        return;
                    }
                    c.internal.emplace(n, true);
                    c.to_delete.push_back(g.pos);
                    self(self, g.in0, false);
                    if(g.k != kind::not_gate) { self(self, g.in1, false); }
                };

                dfs(dfs, root, true);
                if(!ok) { return false; }
                if(c.leaves.empty()) { return false; }
                if(c.leaves.size() > max_cut) { return false; }

                // Topological order (children before parents).
                visited.clear();
                auto topo_dfs = [&](auto&& self, ::phy_engine::model::node_t* n) noexcept -> void
                {
                    if(n == nullptr || visited.contains(n)) { return; }
                    visited.emplace(n, true);
                    if(!c.internal.contains(n)) { return; }
                    auto itg = gate_by_out.find(n);
                    if(itg == gate_by_out.end()) { return; }
                    auto const& g = itg->second;
                    self(self, g.in0);
                    if(g.k != kind::not_gate) { self(self, g.in1); }
                    c.topo.push_back(n);
                };

                topo_dfs(topo_dfs, root);
                return !c.topo.empty();
            };

            auto make_not = [&](::phy_engine::model::node_t* in,
                                ::std::vector<::phy_engine::netlist::model_pos>& new_models) noexcept -> ::phy_engine::model::node_t*
            {
                if(in == nullptr) { return nullptr; }
                auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::NOT{});
                if(m == nullptr) { return nullptr; }
                new_models.push_back(pos);
                auto& out_ref = ::phy_engine::netlist::create_node(nl);
                auto* out = __builtin_addressof(out_ref);
                if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *in) || !::phy_engine::netlist::add_to_node(nl, *m, 1, *out)) { return nullptr; }
                return out;
            };

            auto make_bin = [&](pkind k,
                                ::phy_engine::model::node_t* a,
                                ::phy_engine::model::node_t* b,
                                ::std::vector<::phy_engine::netlist::model_pos>& new_models) noexcept -> ::phy_engine::model::node_t*
            {
                if(a == nullptr || b == nullptr) { return nullptr; }
                ::phy_engine::netlist::add_model_retstr r{};
                switch(k)
                {
                    case pkind::and2: r = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::AND{}); break;
                    case pkind::or2: r = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OR{}); break;
                    case pkind::xor2: r = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::XOR{}); break;
                    case pkind::xnor2: r = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::XNOR{}); break;
                    case pkind::nand2: r = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::NAND{}); break;
                    case pkind::nor2: r = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::NOR{}); break;
                    case pkind::imp2: r = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::IMP{}); break;
                    case pkind::nimp2: r = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::NIMP{}); break;
                    default: return nullptr;
                }
                if(r.mod == nullptr) { return nullptr; }
                new_models.push_back(r.mod_pos);
                auto& out_ref = ::phy_engine::netlist::create_node(nl);
                auto* out = __builtin_addressof(out_ref);
                if(!::phy_engine::netlist::add_to_node(nl, *r.mod, 0, *a) || !::phy_engine::netlist::add_to_node(nl, *r.mod, 1, *b) ||
                   !::phy_engine::netlist::add_to_node(nl, *r.mod, 2, *out))
                {
                    return nullptr;
                }
                return out;
            };

            auto build_pattern = [&](pattern const& pat,
                                     ::std::array<::phy_engine::model::node_t*, 4> const& ins,
                                     ::std::vector<::phy_engine::netlist::model_pos>& new_models) noexcept -> ::phy_engine::model::node_t*
            {
                switch(pat.k)
                {
                    case pkind::not1: return make_not(ins[0], new_models);
                    case pkind::and2:
                    case pkind::or2:
                    case pkind::xor2:
                    case pkind::xnor2:
                    case pkind::nand2:
                    case pkind::nor2:
                    case pkind::imp2:
                    case pkind::nimp2: return make_bin(pat.k, ins[0], ins[1], new_models);
                    case pkind::aoi21:
                    {
                        auto* t = make_bin(pkind::and2, ins[0], ins[1], new_models);
                        auto* u = make_bin(pkind::or2, t, ins[2], new_models);
                        return make_not(u, new_models);
                    }
                    case pkind::oai21:
                    {
                        auto* t = make_bin(pkind::or2, ins[0], ins[1], new_models);
                        auto* u = make_bin(pkind::and2, t, ins[2], new_models);
                        return make_not(u, new_models);
                    }
                    case pkind::aoi22:
                    {
                        auto* t0 = make_bin(pkind::and2, ins[0], ins[1], new_models);
                        auto* t1 = make_bin(pkind::and2, ins[2], ins[3], new_models);
                        auto* u = make_bin(pkind::or2, t0, t1, new_models);
                        return make_not(u, new_models);
                    }
                    case pkind::oai22:
                    {
                        auto* t0 = make_bin(pkind::or2, ins[0], ins[1], new_models);
                        auto* t1 = make_bin(pkind::or2, ins[2], ins[3], new_models);
                        auto* u = make_bin(pkind::and2, t0, t1, new_models);
                        return make_not(u, new_models);
                    }
                    default: return nullptr;
                }
            };

            bool changed{};
            ::std::vector<::phy_engine::model::node_t*> roots{};
            roots.reserve(gate_by_out.size());
            for(auto const& kv: gate_by_out) { roots.push_back(kv.first); }

            for(auto* root: roots)
            {
                cone c{};
                if(!collect_cone(root, c)) { continue; }
                if(c.leaves.size() > max_cut) { continue; }

                ::std::unordered_map<::phy_engine::model::node_t*, ::std::vector<cut>> cuts{};
                cuts.reserve(c.topo.size() * 2u);

                auto ensure_leaf_cut = [&](::phy_engine::model::node_t* n) noexcept -> ::std::vector<cut>&
                {
                    auto it = cuts.find(n);
                    if(it != cuts.end()) { return it->second; }
                    cut ct{};
                    ct.leaves = {n};
                    auto [ins_it, ok] = cuts.emplace(n, ::std::vector<cut>{ct});
                    (void)ok;
                    return ins_it->second;
                };

                bool cuts_ok{true};
                for(auto* n: c.topo)
                {
                    auto itg = gate_by_out.find(n);
                    if(itg == gate_by_out.end())
                    {
                        cuts_ok = false;
                        break;
                    }
                    auto const& g = itg->second;
                    if(g.in0 == nullptr || (g.k != kind::not_gate && g.in1 == nullptr))
                    {
                        cuts_ok = false;
                        break;
                    }
                    ::std::vector<cut> node_cuts{};
                    node_cuts.reserve(max_cuts + 1u);

                    // Unit cut.
                    node_cuts.push_back(cut{::std::vector<::phy_engine::model::node_t*>{n}});

                    if(g.k == kind::not_gate)
                    {
                        auto& cin = (c.internal.contains(g.in0)) ? cuts[g.in0] : ensure_leaf_cut(g.in0);
                        for(auto const& ca: cin) { node_cuts.push_back(ca); }
                    }
                    else
                    {
                        auto& c0 = (c.internal.contains(g.in0)) ? cuts[g.in0] : ensure_leaf_cut(g.in0);
                        auto& c1 = (c.internal.contains(g.in1)) ? cuts[g.in1] : ensure_leaf_cut(g.in1);
                        ::std::vector<::phy_engine::model::node_t*> merged{};
                        for(auto const& ca: c0)
                        {
                            for(auto const& cb: c1)
                            {
                                if(!merge_leaves(ca.leaves, cb.leaves, merged)) { continue; }
                                node_cuts.push_back(cut{merged});
                            }
                        }
                    }

                    // Dedup by leaf set.
                    auto same_leaves = [](cut const& a, cut const& b) noexcept -> bool { return a.leaves == b.leaves; };
                    ::std::sort(node_cuts.begin(),
                                node_cuts.end(),
                                [](cut const& a, cut const& b) noexcept
                                {
                                    if(a.leaves.size() != b.leaves.size()) { return a.leaves.size() < b.leaves.size(); }
                                    return a.leaves < b.leaves;
                                });
                    node_cuts.erase(::std::unique(node_cuts.begin(), node_cuts.end(), same_leaves), node_cuts.end());

                    if(node_cuts.size() > max_cuts) { node_cuts.resize(max_cuts); }
                    cuts[n] = ::std::move(node_cuts);
                }
                if(!cuts_ok) { continue; }

                ::std::unordered_map<::phy_engine::model::node_t*, choice> best_choice{};
                ::std::unordered_map<::phy_engine::model::node_t*, std::size_t> best_cost{};
                best_choice.reserve(c.topo.size() * 2u);
                best_cost.reserve(c.topo.size() * 2u);

                bool ok{true};
                for(auto* n: c.topo)
                {
                    auto itc = cuts.find(n);
                    if(itc == cuts.end())
                    {
                        ok = false;
                        break;
                    }
                    std::size_t best{static_cast<std::size_t>(-1)};
                    choice bestc{};

                    struct mask_task
                    {
                        cuda_u64_cone_desc cone{};
                        ::std::uint64_t mask{};
                        ::std::vector<::phy_engine::model::node_t*> const* leaves{};
                        std::size_t leaf_cost{};
                        bool ok{};
                    };

                    ::std::vector<mask_task> tasks{};
                    tasks.reserve(itc->second.size());

                    for(auto const& ct: itc->second)
                    {
                        if(ct.leaves.size() == 1u && ct.leaves[0] == n) { continue; }  // unit cut only for parent enumeration

                        std::size_t leaf_cost{};
                        for(auto* leaf: ct.leaves)
                        {
                            if(c.internal.contains(leaf))
                            {
                                auto itb = best_cost.find(leaf);
                                if(itb == best_cost.end())
                                {
                                    leaf_cost = static_cast<std::size_t>(-1);
                                    break;
                                }
                                leaf_cost += itb->second;
                            }
                        }
                        if(leaf_cost == static_cast<std::size_t>(-1)) { continue; }

                        cuda_u64_cone_desc cone{};
                        if(!build_u64_mask_cone(n, ct.leaves, cone)) { continue; }
                        tasks.push_back(mask_task{.cone = cone, .mask = 0ull, .leaves = __builtin_addressof(ct.leaves), .leaf_cost = leaf_cost, .ok = true});
                    }

                    if(tasks.empty())
                    {
                        ok = false;
                        break;
                    }

                    // Evaluate masks (CUDA best-effort, batched).
                    bool used_cuda{};
                    if(opt.cuda_enable && tasks.size() >= opt.cuda_min_batch)
                    {
                        ::std::vector<cuda_u64_cone_desc> cones{};
                        cones.reserve(tasks.size());
                        for(auto const& t: tasks)
                        {
                            if(!t.ok) { continue; }
                            cones.push_back(t.cone);
                        }
                        ::std::vector<::std::uint64_t> masks{};
                        masks.resize(cones.size());
                        if(!cones.empty() && cuda_eval_u64_cones(opt.cuda_device_mask, cones.data(), cones.size(), masks.data()))
                        {
                            used_cuda = true;
                            std::size_t mi{};
                            for(auto& t: tasks)
                            {
                                if(!t.ok) { continue; }
                                t.mask = masks[mi++];
                            }
                        }
                    }
                    if(!used_cuda)
                    {
                        for(auto& t: tasks)
                        {
                            if(!t.ok) { continue; }
                            t.mask = eval_u64_cone_cpu(t.cone);
                        }
                    }

                    for(auto const& t: tasks)
                    {
                        if(!t.ok || t.leaves == nullptr) { continue; }
                        auto const nleaves = t.leaves->size();
                        if(nleaves == 0u || nleaves > 4u) { continue; }
                        auto const U = static_cast<std::size_t>(1u << nleaves);
                        auto const all_mask = (U == 64u) ? ~0ull : ((1ull << U) - 1ull);
                        auto const mask = t.mask & all_mask;

                        auto match = match_best(mask, *t.leaves, t.leaf_cost);
                        if(!match) { continue; }
                        auto const& cand_choice = match->first;
                        auto const cand_cost = match->second;
                        if(cand_cost < best)
                        {
                            best = cand_cost;
                            bestc = cand_choice;
                        }
                    }

                    if(best == static_cast<std::size_t>(-1))
                    {
                        ok = false;
                        break;
                    }
                    best_cost.emplace(n, best);
                    best_choice.emplace(n, bestc);
                }
                if(!ok) { continue; }

                auto itb = best_cost.find(root);
                if(itb == best_cost.end()) { continue; }
                auto const old_cost = c.to_delete.size();
                auto const new_cost = itb->second + (is_protected(root) ? 1u : 0u);
                if(new_cost >= old_cost) { continue; }

                ::std::unordered_map<::phy_engine::model::node_t*, ::phy_engine::model::node_t*> built{};
                built.reserve(c.topo.size() * 2u);
                ::std::vector<::phy_engine::netlist::model_pos> new_models{};
                new_models.reserve(old_cost + 8u);

                auto build_node = [&](auto&& self, ::phy_engine::model::node_t* n) noexcept -> ::phy_engine::model::node_t*
                {
                    if(n == nullptr) { return nullptr; }
                    if(!c.internal.contains(n)) { return n; }
                    if(auto it = built.find(n); it != built.end()) { return it->second; }
                    auto itc = best_choice.find(n);
                    if(itc == best_choice.end() || !itc->second.valid) { return nullptr; }
                    auto const& ch = itc->second;
                    auto const& pat = patterns[ch.pattern_idx];

                    ::std::array<::phy_engine::model::node_t*, 4> ins{};
                    for(std::size_t i{}; i < ch.inputs; ++i)
                    {
                        auto idx = static_cast<std::size_t>(ch.perm[i]);
                        if(idx >= ch.leaves.size()) { return nullptr; }
                        auto* leaf = ch.leaves[idx];
                        auto* in = self(self, leaf);
                        if(in == nullptr) { return nullptr; }
                        if((ch.neg_mask >> i) & 1u) { in = make_not(in, new_models); }
                        ins[i] = in;
                    }
                    auto* out = build_pattern(pat, ins, new_models);
                    if(out == nullptr) { return nullptr; }
                    built.emplace(n, out);
                    return out;
                };

                auto* new_out = build_node(build_node, root);
                if(new_out == nullptr)
                {
                    for(auto const mp: new_models) { (void)::phy_engine::netlist::delete_model(nl, mp); }
                    continue;
                }

                if(!is_protected(root))
                {
                    if(!move_consumers_all(root, new_out))
                    {
                        for(auto const mp: new_models) { (void)::phy_engine::netlist::delete_model(nl, mp); }
                        continue;
                    }
                }

                for(auto const mp: c.to_delete) { (void)::phy_engine::netlist::delete_model(nl, mp); }

                if(is_protected(root))
                {
                    auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::YES{});
                    if(m == nullptr || !::phy_engine::netlist::add_to_node(nl, *m, 0, *new_out) || !::phy_engine::netlist::add_to_node(nl, *m, 1, *root))
                    {
                        (void)::phy_engine::netlist::delete_model(nl, pos);
                    }
                }

                changed = true;
            }

            return changed;
        }

        [[nodiscard]] inline bool optimize_bdd_decompose_large_cones_in_pe_netlist(::phy_engine::netlist::netlist& nl,
                                                                                   ::std::vector<::phy_engine::model::node_t*> const& protected_nodes,
                                                                                   pe_synth_options const& opt) noexcept
        {
            if(!opt.assume_binary_inputs || !opt.decompose_large_functions) { return false; }

            std::size_t const min_vars = opt.decomp_min_vars;
            std::size_t const max_vars = (opt.decomp_max_vars > 16u) ? 16u : opt.decomp_max_vars;
            std::size_t const max_gates = (opt.decomp_max_gates == 0u) ? 256u : opt.decomp_max_gates;
            std::size_t const bdd_limit = (opt.decomp_bdd_node_limit < 2u) ? 2u : opt.decomp_bdd_node_limit;
            if(max_vars == 0u) { return false; }

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

            ::std::unordered_map<::phy_engine::model::node_t*, bool> protected_map{};
            protected_map.reserve(protected_nodes.size() * 2u + 1u);
            for(auto* n: protected_nodes) { protected_map.emplace(n, true); }
            auto const is_protected = [&](::phy_engine::model::node_t* n) noexcept -> bool { return protected_map.contains(n); };

            auto const fan = build_gate_opt_fanout(nl);

            ::std::unordered_map<::phy_engine::model::node_t*, gate> gate_by_out{};
            gate_by_out.reserve(1 << 14);

            auto classify = [&](::phy_engine::model::model_base const& mb, ::phy_engine::netlist::model_pos pos) noexcept -> ::std::optional<gate>
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
                    else
                    {
                        g.k = kind::nimp_gate;
                    }
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
                    if(!og || og->out == nullptr || og->out_pin == nullptr) { continue; }
                    gate_by_out.emplace(og->out, *og);
                }
            }

            // Constants (unnamed INPUT nodes) should not be treated as variable leaves for truth tables.
            ::std::unordered_map<::phy_engine::model::node_t*, ::std::uint16_t> const_idx{};
            const_idx.reserve(256);
            for(auto const& blk: nl.models)
            {
                for(auto const* m = blk.begin; m != blk.curr; ++m)
                {
                    if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }
                    if(m->name.size() != 0) { continue; }  // named INPUTs are external IO, not constants
                    if(model_name_u8(*m) != u8"INPUT") { continue; }
                    auto pv = m->ptr->generate_pin_view();
                    if(pv.size != 1 || pv.pins[0].nodes == nullptr) { continue; }
                    auto vi = m->ptr->get_attribute(0);
                    if(vi.type != ::phy_engine::model::variant_type::digital) { continue; }
                    if(vi.digital == ::phy_engine::model::digital_node_statement_t::true_state) { const_idx.emplace(pv.pins[0].nodes, 65535u); }
                    else if(vi.digital == ::phy_engine::model::digital_node_statement_t::false_state) { const_idx.emplace(pv.pins[0].nodes, 65534u); }
                }
            }

            auto enc_kind = [&](kind k) noexcept -> ::std::uint8_t
            {
                switch(k)
                {
                    case kind::not_gate: return 0u;
                    case kind::and_gate: return 1u;
                    case kind::or_gate: return 2u;
                    case kind::xor_gate: return 3u;
                    case kind::xnor_gate: return 4u;
                    case kind::nand_gate: return 5u;
                    case kind::nor_gate: return 6u;
                    case kind::imp_gate: return 7u;
                    case kind::nimp_gate: return 8u;
                    default: return 255u;
                }
            };

            struct cone
            {
                ::phy_engine::model::node_t* root{};
                ::phy_engine::model::pin const* root_out_pin{};
                ::std::vector<::phy_engine::model::node_t*> leaves{};
                ::std::unordered_map<::phy_engine::model::node_t*, bool> internal{};
                ::std::vector<::phy_engine::model::node_t*> topo{};
                ::std::vector<::phy_engine::netlist::model_pos> to_delete{};
            };

            auto collect_cone = [&](::phy_engine::model::node_t* root, cone& c) noexcept -> bool
            {
                auto it_root = gate_by_out.find(root);
                if(it_root == gate_by_out.end()) { return false; }
                auto const& rg = it_root->second;
                if(rg.out_pin == nullptr) { return false; }
                if(!has_unique_driver_pin(root, rg.out_pin, fan.pin_out)) { return false; }

                c.root = root;
                c.root_out_pin = rg.out_pin;
                c.leaves.clear();
                c.internal.clear();
                c.topo.clear();
                c.to_delete.clear();
                c.internal.reserve(max_gates * 2u);
                c.to_delete.reserve(max_gates);

                ::std::unordered_map<::phy_engine::model::node_t*, bool> visited{};
                visited.reserve(max_gates * 2u);
                ::std::unordered_map<::phy_engine::model::node_t*, bool> leaf_seen{};
                leaf_seen.reserve(max_vars * 2u);
                std::size_t gate_count{};
                bool ok{true};

                auto dfs = [&](auto&& self, ::phy_engine::model::node_t* n, bool is_root) noexcept -> void
                {
                    if(!ok || n == nullptr)
                    {
                        ok = false;
                        return;
                    }
                    if(visited.contains(n)) { return; }
                    visited.emplace(n, true);

                    auto itg = gate_by_out.find(n);
                    if(itg == gate_by_out.end())
                    {
                        if(!leaf_seen.contains(n))
                        {
                            leaf_seen.emplace(n, true);
                            c.leaves.push_back(n);
                        }
                        return;
                    }

                    auto const& g = itg->second;
                    if(g.in0 == nullptr || (g.k != kind::not_gate && g.in1 == nullptr))
                    {
                        ok = false;
                        return;
                    }

                    if(!is_root)
                    {
                        // Stop at protected or shared nodes (treat as leaves).
                        if(is_protected(n))
                        {
                            if(!leaf_seen.contains(n))
                            {
                                leaf_seen.emplace(n, true);
                                c.leaves.push_back(n);
                            }
                            return;
                        }

                        auto itd = fan.driver_count.find(n);
                        auto itc = fan.consumer_count.find(n);
                        if(itd == fan.driver_count.end() || itc == fan.consumer_count.end() || itd->second != 1u || itc->second != 1u)
                        {
                            if(!leaf_seen.contains(n))
                            {
                                leaf_seen.emplace(n, true);
                                c.leaves.push_back(n);
                            }
                            return;
                        }
                        if(g.out_pin == nullptr || !has_unique_driver_pin(n, g.out_pin, fan.pin_out))
                        {
                            if(!leaf_seen.contains(n))
                            {
                                leaf_seen.emplace(n, true);
                                c.leaves.push_back(n);
                            }
                            return;
                        }
                    }

                    if(++gate_count > max_gates)
                    {
                        ok = false;
                        return;
                    }
                    c.internal.emplace(n, true);
                    c.to_delete.push_back(g.pos);
                    self(self, g.in0, false);
                    if(g.k != kind::not_gate) { self(self, g.in1, false); }
                };

                dfs(dfs, root, true);
                if(!ok) { return false; }
                ::std::sort(c.leaves.begin(),
                            c.leaves.end(),
                            [](auto* a, auto* b) noexcept { return reinterpret_cast<std::uintptr_t>(a) < reinterpret_cast<std::uintptr_t>(b); });

                // Remove constant leaves (unnamed INPUT const0/const1).
                if(!const_idx.empty())
                {
                    ::std::vector<::phy_engine::model::node_t*> vars{};
                    vars.reserve(c.leaves.size());
                    for(auto* n: c.leaves)
                    {
                        if(const_idx.contains(n)) { continue; }
                        vars.push_back(n);
                    }
                    c.leaves.swap(vars);
                }

                if(c.leaves.size() < min_vars || c.leaves.size() > max_vars) { return false; }

                // Topological order (children before parents) restricted to internal nodes.
                visited.clear();
                auto topo_dfs = [&](auto&& self, ::phy_engine::model::node_t* n) noexcept -> void
                {
                    if(n == nullptr || visited.contains(n)) { return; }
                    visited.emplace(n, true);
                    if(!c.internal.contains(n)) { return; }
                    auto itg = gate_by_out.find(n);
                    if(itg == gate_by_out.end()) { return; }
                    auto const& g = itg->second;
                    self(self, g.in0);
                    if(g.k != kind::not_gate) { self(self, g.in1); }
                    c.topo.push_back(n);
                };
                topo_dfs(topo_dfs, root);
                return !c.topo.empty();
            };

            auto build_tt_desc = [&](cone const& c, cuda_tt_cone_desc& out) noexcept -> bool
            {
                auto const var_count = c.leaves.size();
                if(var_count == 0u || var_count > 16u) { return false; }
                if(c.topo.empty() || c.topo.size() > 256u) { return false; }

                ::std::unordered_map<::phy_engine::model::node_t*, ::std::uint16_t> leaf_index{};
                leaf_index.reserve(var_count * 2u + 8u);
                for(::std::uint16_t i{}; i < static_cast<::std::uint16_t>(var_count); ++i) { leaf_index.emplace(c.leaves[i], i); }

                ::std::unordered_map<::phy_engine::model::node_t*, ::std::uint16_t> out_index{};
                out_index.reserve(c.topo.size() * 2u + 8u);

                out.var_count = static_cast<::std::uint8_t>(var_count);
                out.gate_count = static_cast<::std::uint16_t>(c.topo.size());

                auto idx_of = [&](::phy_engine::model::node_t* n) noexcept -> ::std::optional<::std::uint16_t>
                {
                    if(n == nullptr) { return ::std::nullopt; }
                    if(auto itc = const_idx.find(n); itc != const_idx.end()) { return itc->second; }
                    if(auto itl = leaf_index.find(n); itl != leaf_index.end()) { return itl->second; }
                    if(auto ito = out_index.find(n); ito != out_index.end()) { return ito->second; }
                    return ::std::nullopt;
                };

                for(::std::uint16_t gi{}; gi < out.gate_count; ++gi)
                {
                    auto* n = c.topo[gi];
                    auto itg = gate_by_out.find(n);
                    if(itg == gate_by_out.end()) { return false; }
                    auto const& g = itg->second;

                    out.kind[gi] = enc_kind(g.k);
                    auto a = idx_of(g.in0);
                    if(!a) { return false; }
                    ::std::optional<::std::uint16_t> b{};
                    if(g.k == kind::not_gate) { b = static_cast<::std::uint16_t>(65534u); }
                    else { b = idx_of(g.in1); }
                    if(!b) { return false; }

                    out.in0[gi] = *a;
                    out.in1[gi] = *b;

                    out_index.emplace(n, static_cast<::std::uint16_t>(16u + gi));
                }

                return true;
            };

            auto move_consumers =
                [&](::phy_engine::model::node_t* from, ::phy_engine::model::node_t* to, ::phy_engine::model::pin const* from_driver_pin) noexcept -> bool
            {
                if(from == nullptr || to == nullptr || from_driver_pin == nullptr) { return false; }
                if(from == to) { return false; }
                if(is_protected(from)) { return false; }
                if(!has_unique_driver_pin(from, from_driver_pin, fan.pin_out)) { return false; }

                ::std::vector<::phy_engine::model::pin*> pins_to_move{};
                pins_to_move.reserve(from->pins.size());
                for(auto* p: from->pins)
                {
                    if(p == from_driver_pin) { continue; }
                    pins_to_move.push_back(p);
                }
                for(auto* p: pins_to_move)
                {
                    from->pins.erase(p);
                    p->nodes = to;
                    to->pins.insert(p);
                }
                return true;
            };

            auto add_yes = [&](::phy_engine::model::node_t* in, ::phy_engine::model::node_t* out) noexcept -> bool
            {
                if(in == nullptr || out == nullptr) { return false; }
                auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::YES{});
                (void)pos;
                if(m == nullptr) { return false; }
                if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *in) || !::phy_engine::netlist::add_to_node(nl, *m, 1, *out))
                {
                    (void)::phy_engine::netlist::delete_model(nl, pos);
                    return false;
                }
                return true;
            };

            auto make_not = [&](::phy_engine::model::node_t* in,
                                ::std::vector<::phy_engine::netlist::model_pos>& new_models) noexcept -> ::phy_engine::model::node_t*
            {
                if(in == nullptr) { return nullptr; }
                auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::NOT{});
                if(m == nullptr) { return nullptr; }
                new_models.push_back(pos);
                auto& out_ref = ::phy_engine::netlist::create_node(nl);
                auto* out = __builtin_addressof(out_ref);
                if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *in) || !::phy_engine::netlist::add_to_node(nl, *m, 1, *out)) { return nullptr; }
                return out;
            };
            auto make_and = [&](::phy_engine::model::node_t* a,
                                ::phy_engine::model::node_t* b,
                                ::phy_engine::model::node_t* const0,
                                ::phy_engine::model::node_t* const1,
                                ::std::vector<::phy_engine::netlist::model_pos>& new_models) noexcept -> ::phy_engine::model::node_t*
            {
                if(a == nullptr || b == nullptr) { return nullptr; }
                if(a == const0 || b == const0) { return const0; }
                if(a == const1) { return b; }
                if(b == const1) { return a; }
                auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::AND{});
                if(m == nullptr) { return nullptr; }
                new_models.push_back(pos);
                auto& out_ref = ::phy_engine::netlist::create_node(nl);
                auto* out = __builtin_addressof(out_ref);
                if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *a) || !::phy_engine::netlist::add_to_node(nl, *m, 1, *b) ||
                   !::phy_engine::netlist::add_to_node(nl, *m, 2, *out))
                {
                    return nullptr;
                }
                return out;
            };
            auto make_or = [&](::phy_engine::model::node_t* a,
                               ::phy_engine::model::node_t* b,
                               ::phy_engine::model::node_t* const0,
                               ::phy_engine::model::node_t* const1,
                               ::std::vector<::phy_engine::netlist::model_pos>& new_models) noexcept -> ::phy_engine::model::node_t*
            {
                if(a == nullptr || b == nullptr) { return nullptr; }
                if(a == const1 || b == const1) { return const1; }
                if(a == const0) { return b; }
                if(b == const0) { return a; }
                auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OR{});
                if(m == nullptr) { return nullptr; }
                new_models.push_back(pos);
                auto& out_ref = ::phy_engine::netlist::create_node(nl);
                auto* out = __builtin_addressof(out_ref);
                if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *a) || !::phy_engine::netlist::add_to_node(nl, *m, 1, *b) ||
                   !::phy_engine::netlist::add_to_node(nl, *m, 2, *out))
                {
                    return nullptr;
                }
                return out;
            };

            struct bdd_node
            {
                ::std::uint16_t var{};
                ::std::uint32_t lo{};
                ::std::uint32_t hi{};
            };

            struct uniq_key
            {
                ::std::uint16_t var{};
                ::std::uint32_t lo{};
                ::std::uint32_t hi{};
            };

            struct uniq_hash
            {
                std::size_t operator() (uniq_key const& k) const noexcept
                {
                    auto const mix = [](std::size_t h, std::size_t v) noexcept -> std::size_t
                    { return (h ^ (v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2))); };
                    std::size_t h{};
                    h = mix(h, static_cast<std::size_t>(k.var));
                    h = mix(h, static_cast<std::size_t>(k.lo));
                    h = mix(h, static_cast<std::size_t>(k.hi));
                    return h;
                }
            };

            struct uniq_eq
            {
                bool operator() (uniq_key const& a, uniq_key const& b) const noexcept { return a.var == b.var && a.lo == b.lo && a.hi == b.hi; }
            };

            auto hash_tt = [](std::size_t depth, ::std::vector<::std::uint64_t> const& tt) noexcept -> ::std::uint64_t
            {
                // 64-bit FNV-1a-ish over words (bounded, deterministic).
                ::std::uint64_t h = 1469598103934665603ull;
                h ^= static_cast<::std::uint64_t>(depth) + 0x9e3779b97f4a7c15ull;
                h *= 1099511628211ull;
                for(auto const w: tt)
                {
                    h ^= w;
                    h *= 1099511628211ull;
                }
                return h;
            };

            auto tt_all_zero = [](std::vector<std::uint64_t> const& tt) noexcept -> bool
            {
                for(auto const w: tt)
                {
                    if(w != 0ull) { return false; }
                }
                return true;
            };

            auto tt_all_one = [](std::vector<std::uint64_t> const& tt, std::uint64_t last_mask) noexcept -> bool
            {
                if(tt.empty()) { return false; }
                for(std::size_t i{}; i + 1 < tt.size(); ++i)
                {
                    if(tt[i] != ~0ull) { return false; }
                }
                return (tt.back() & last_mask) == last_mask;
            };

            auto tt_get_bit = [](std::vector<std::uint64_t> const& tt, std::size_t idx) noexcept -> bool
            { return ((tt[idx >> 6] >> (idx & 63u)) & 1ull) != 0ull; };

            auto tt_set_bit = [](std::vector<std::uint64_t>& tt, std::size_t idx) noexcept -> void { tt[idx >> 6] |= (1ull << (idx & 63u)); };

            bool changed{};
            ::std::vector<::phy_engine::model::node_t*> roots{};
            roots.reserve(gate_by_out.size());
            for(auto const& kv: gate_by_out) { roots.push_back(kv.first); }

            struct cand
            {
                ::phy_engine::model::node_t* root{};
                cone c{};
                ::std::uint32_t blocks{};
                ::std::uint64_t last_mask{};
            };

            ::std::vector<cand> cands{};
            ::std::vector<cuda_tt_cone_desc> descs{};
            cands.reserve(roots.size());
            descs.reserve(roots.size());

            ::std::uint32_t stride_blocks{};
            for(auto* root: roots)
            {
                cone c{};
                if(!collect_cone(root, c)) { continue; }

                auto const var_count = c.leaves.size();
                if(var_count < min_vars || var_count > max_vars) { continue; }

                cuda_tt_cone_desc d{};
                if(!build_tt_desc(c, d)) { continue; }

                auto const U = static_cast<std::size_t>(1u) << var_count;
                auto const blocks = static_cast<::std::uint32_t>((U + 63u) / 64u);
                if(blocks == 0u) { continue; }
                auto const rem = static_cast<::std::uint32_t>(U & 63u);
                auto const last_mask = (rem == 0u) ? ~0ull : ((1ull << rem) - 1ull);

                if(blocks > stride_blocks) { stride_blocks = blocks; }

                cands.push_back(cand{root, ::std::move(c), blocks, last_mask});
                descs.push_back(d);
            }

            if(cands.empty() || stride_blocks == 0u) { return changed; }

            ::std::vector<::std::uint64_t> tt_words{};
            tt_words.resize(cands.size() * static_cast<std::size_t>(stride_blocks));

            bool used_cuda{};
            if(opt.cuda_enable)
            {
                // Large truth-tables are expensive enough that even modest batches can amortize GPU overhead.
                if(cands.size() >= 16u || stride_blocks >= 64u)
                {
                    used_cuda = cuda_eval_tt_cones(opt.cuda_device_mask, descs.data(), descs.size(), stride_blocks, tt_words.data());
                }
            }
            if(!used_cuda)
            {
                for(std::size_t i{}; i < descs.size(); ++i)
                {
                    eval_tt_cone_cpu(descs[i], stride_blocks, tt_words.data() + i * static_cast<std::size_t>(stride_blocks));
                }
            }

            for(std::size_t ci{}; ci < cands.size(); ++ci)
            {
                auto* root = cands[ci].root;
                auto& c = cands[ci].c;
                auto const var_count = c.leaves.size();
                auto const U = static_cast<std::size_t>(1u) << var_count;
                auto const rem = static_cast<std::size_t>(U & 63u);
                auto const last_mask = cands[ci].last_mask;

                auto const blocks = static_cast<std::size_t>(cands[ci].blocks);
                ::std::vector<::std::uint64_t> root_tt{};
                root_tt.assign(tt_words.begin() + static_cast<std::size_t>(ci) * stride_blocks,
                               tt_words.begin() + static_cast<std::size_t>(ci) * stride_blocks + blocks);

                if(tt_all_zero(root_tt) || tt_all_one(root_tt, last_mask))
                {
                    // Let constprop handle this cheaply.
                    continue;
                }

                // Build a reduced ordered BDD. Try a few small variable-order variants and pick the smallest BDD (bounded).
                auto permute_tt = [&](::std::vector<::std::uint64_t> const& tt, ::std::vector<unsigned> const& perm) noexcept -> ::std::vector<::std::uint64_t>
                {
                    auto const UU = static_cast<std::size_t>(1u << var_count);
                    auto const blocks = (UU + 63u) / 64u;
                    ::std::vector<::std::uint64_t> out{};
                    out.assign(blocks, 0ull);
                    for(std::size_t m{}; m < UU; ++m)
                    {
                        std::size_t old_idx{};
                        for(std::size_t i{}; i < var_count; ++i)
                        {
                            unsigned const ov = perm[i];
                            bool const bit = ((m >> i) & 1u) != 0u;
                            if(bit) { old_idx |= (1u << ov); }
                        }
                        if(tt_get_bit(tt, old_idx)) { tt_set_bit(out, m); }
                    }
                    if(rem != 0u) { out.back() &= last_mask; }
                    return out;
                };

                auto influence_order = [&]() noexcept -> ::std::vector<unsigned>
                {
                    struct item
                    {
                        unsigned v{};
                        std::size_t inf{};
                    };
                    ::std::vector<item> items{};
                    items.reserve(var_count);
                    for(unsigned v{}; v < static_cast<unsigned>(var_count); ++v)
                    {
                        std::size_t cnt{};
                        auto const step = static_cast<std::size_t>(1u << v);
                        for(std::size_t base{}; base < U; base += (step << 1u))
                        {
                            for(std::size_t i{}; i < step; ++i)
                            {
                                auto const a = base + i;
                                auto const b = a + step;
                                cnt += static_cast<std::size_t>(tt_get_bit(root_tt, a) != tt_get_bit(root_tt, b));
                            }
                        }
                        items.push_back(item{v, cnt});
                    }
                    ::std::sort(items.begin(),
                                items.end(),
                                [](item const& a, item const& b) noexcept
                                {
                                    if(a.inf != b.inf) { return a.inf > b.inf; }
                                    return a.v < b.v;
                                });
                    ::std::vector<unsigned> perm{};
                    perm.reserve(var_count);
                    for(auto const& it: items) { perm.push_back(it.v); }
                    return perm;
                };

                auto try_build_bdd = [&](::std::vector<::std::uint64_t>&& tt_in, ::std::vector<bdd_node>& out_bdd) noexcept -> std::uint32_t
                {
                    out_bdd.clear();
                    out_bdd.reserve(1024);
                    out_bdd.push_back(bdd_node{0u, 0u, 0u});  // 0-terminal
                    out_bdd.push_back(bdd_node{0u, 1u, 1u});  // 1-terminal (dummy)

                    ::std::unordered_map<uniq_key, std::uint32_t, uniq_hash, uniq_eq> uniq{};
                    uniq.reserve(bdd_limit * 2u);

                    struct memo_entry
                    {
                        std::size_t depth{};
                        ::std::vector<::std::uint64_t> tt{};
                        std::uint32_t id{};
                    };
                    ::std::unordered_map<::std::uint64_t, ::std::vector<memo_entry>> memo{};
                    memo.reserve(1 << 12);

                    auto mk = [&](std::uint16_t v, std::uint32_t lo, std::uint32_t hi) noexcept -> std::uint32_t
                    {
                        if(lo == hi) { return lo; }
                        uniq_key k{v, lo, hi};
                        if(auto it = uniq.find(k); it != uniq.end()) { return it->second; }
                        if(out_bdd.size() >= bdd_limit) { return static_cast<std::uint32_t>(-1); }
                        auto const id = static_cast<std::uint32_t>(out_bdd.size());
                        out_bdd.push_back(bdd_node{v, lo, hi});
                        uniq.emplace(k, id);
                        return id;
                    };

                    auto build_bdd = [&](auto&& self, std::size_t depth, ::std::vector<std::uint64_t>&& tt) noexcept -> std::uint32_t
                    {
                        auto const bits_n = static_cast<std::size_t>(1u << (var_count - depth));
                        auto const remn = (bits_n % 64u);
                        auto const lastm = (remn == 0u) ? ~0ull : ((1ull << remn) - 1ull);

                        auto const h = hash_tt(depth, tt);
                        auto& bucket = memo[h];
                        for(auto const& e: bucket)
                        {
                            if(e.depth == depth && e.tt == tt) { return e.id; }
                        }

                        if(tt_all_zero(tt))
                        {
                            bucket.push_back(memo_entry{depth, ::std::move(tt), 0u});
                            return 0u;
                        }
                        if(tt_all_one(tt, lastm))
                        {
                            bucket.push_back(memo_entry{depth, ::std::move(tt), 1u});
                            return 1u;
                        }
                        if(depth >= var_count) { return 0u; }

                        auto const half = bits_n / 2u;
                        ::std::vector<std::uint64_t> t0((half + 63u) / 64u, 0ull);
                        ::std::vector<std::uint64_t> t1((half + 63u) / 64u, 0ull);
                        for(std::size_t i{}; i < half; ++i)
                        {
                            if(tt_get_bit(tt, 2u * i)) { tt_set_bit(t0, i); }
                            if(tt_get_bit(tt, 2u * i + 1u)) { tt_set_bit(t1, i); }
                        }
                        if((half % 64u) != 0u)
                        {
                            auto const lm2 = (1ull << (half % 64u)) - 1ull;
                            t0.back() &= lm2;
                            t1.back() &= lm2;
                        }

                        if(t0 == t1)
                        {
                            auto lo = self(self, depth + 1u, ::std::move(t0));
                            if(lo == static_cast<std::uint32_t>(-1)) { return lo; }
                            bucket.push_back(memo_entry{depth, ::std::move(tt), lo});
                            return lo;
                        }

                        auto lo = self(self, depth + 1u, ::std::move(t0));
                        if(lo == static_cast<std::uint32_t>(-1)) { return lo; }
                        auto hi = self(self, depth + 1u, ::std::move(t1));
                        if(hi == static_cast<std::uint32_t>(-1)) { return hi; }

                        auto id = mk(static_cast<std::uint16_t>(depth), lo, hi);
                        if(id == static_cast<std::uint32_t>(-1)) { return id; }
                        bucket.push_back(memo_entry{depth, ::std::move(tt), id});
                        return id;
                    };

                    return build_bdd(build_bdd, 0u, ::std::move(tt_in));
                };

                auto is_ident_perm = [&](::std::vector<unsigned> const& p) noexcept -> bool
                {
                    if(p.size() != var_count) { return false; }
                    for(std::size_t i{}; i < p.size(); ++i)
                    {
                        if(p[i] != i) { return false; }
                    }
                    return true;
                };

                ::std::vector<::std::vector<unsigned>> perms{};
                perms.reserve(4);
                {
                    ::std::vector<unsigned> id{};
                    id.reserve(var_count);
                    for(unsigned i{}; i < static_cast<unsigned>(var_count); ++i) { id.push_back(i); }
                    perms.push_back(::std::move(id));
                }
                perms.push_back(influence_order());
                {
                    auto rev = perms.front();
                    ::std::reverse(rev.begin(), rev.end());
                    perms.push_back(::std::move(rev));
                }
                {
                    auto inv = influence_order();
                    ::std::reverse(inv.begin(), inv.end());
                    perms.push_back(::std::move(inv));
                }

                // Limit + (optional) randomized extra variable orders (Omax).
                std::size_t perm_limit = opt.decomp_var_order_tries;
                if(perm_limit == 0u) { perm_limit = 1u; }
                if(perm_limit > 64u) { perm_limit = 64u; }

                if(opt.omax_randomize && perms.size() < perm_limit)
                {
                    auto seed = opt.omax_rand_seed;
                    seed ^= static_cast<::std::uint64_t>(reinterpret_cast<::std::uintptr_t>(root) >> 4);
                    seed ^= static_cast<::std::uint64_t>(var_count) * 0x9e3779b97f4a7c15ull;
                    ::std::mt19937_64 rng{seed};

                    ::std::vector<unsigned> base{};
                    base.reserve(var_count);
                    for(unsigned i{}; i < static_cast<unsigned>(var_count); ++i) { base.push_back(i); }

                    // Over-generate a bit; stable dedup below will trim.
                    std::size_t attempts{};
                    while(perms.size() < perm_limit && attempts++ < perm_limit * 8u)
                    {
                        auto p = base;
                        ::std::shuffle(p.begin(), p.end(), rng);
                        perms.push_back(::std::move(p));
                    }
                }

                // Stable dedup (keep insertion order) and apply limit.
                ::std::vector<::std::vector<unsigned>> uniq{};
                uniq.reserve(perms.size());
                ::std::vector<::std::uint64_t> keys{};
                keys.reserve(perms.size());
                auto perm_key = [&](::std::vector<unsigned> const& p) noexcept -> ::std::uint64_t
                {
                    // var_count is <= 16, so 4 bits per entry is enough.
                    ::std::uint64_t k = static_cast<::std::uint64_t>(p.size());
                    for(std::size_t i{}; i < p.size() && i < 16u; ++i) { k ^= (static_cast<::std::uint64_t>(p[i] & 0xFu) << (4u * i)); }
                    return k;
                };
                for(auto& p: perms)
                {
                    auto const k = perm_key(p);
                    bool seen{};
                    for(auto const kk: keys)
                    {
                        if(kk == k)
                        {
                            seen = true;
                            break;
                        }
                    }
                    if(seen) { continue; }
                    keys.push_back(k);
                    uniq.push_back(::std::move(p));
                    if(uniq.size() >= perm_limit) { break; }
                }
                perms = ::std::move(uniq);

                ::std::vector<unsigned> best_perm{};
                ::std::vector<bdd_node> bdd{};
                std::uint32_t root_id{static_cast<std::uint32_t>(-1)};
                std::size_t best_nodes{static_cast<std::size_t>(-1)};

                for(auto const& perm: perms)
                {
                    ::std::vector<std::uint64_t> tt_try = is_ident_perm(perm) ? root_tt : permute_tt(root_tt, perm);
                    ::std::vector<bdd_node> tmp_bdd{};
                    auto tmp_root = try_build_bdd(::std::move(tt_try), tmp_bdd);
                    if(tmp_root == static_cast<std::uint32_t>(-1)) { continue; }
                    if(tmp_bdd.size() < best_nodes)
                    {
                        best_nodes = tmp_bdd.size();
                        best_perm = perm;
                        bdd = ::std::move(tmp_bdd);
                        root_id = tmp_root;
                    }
                }

                if(root_id == static_cast<std::uint32_t>(-1) || bdd.size() < 2u) { continue; }
                ::std::vector<::phy_engine::model::node_t*> leaves_ord{};
                leaves_ord.reserve(var_count);
                for(std::size_t i{}; i < best_perm.size(); ++i)
                {
                    auto const old = static_cast<std::size_t>(best_perm[i]);
                    if(old >= c.leaves.size())
                    {
                        leaves_ord.clear();
                        break;
                    }
                    leaves_ord.push_back(c.leaves[old]);
                }
                if(leaves_ord.size() != var_count) { continue; }

                // Synthesize BDD into primitives (mux tree).
                auto* const0 = find_or_make_const_node(nl, ::phy_engine::model::digital_node_statement_t::false_state);
                auto* const1 = find_or_make_const_node(nl, ::phy_engine::model::digital_node_statement_t::true_state);
                if(const0 == nullptr || const1 == nullptr) { continue; }

                ::std::unordered_map<std::uint32_t, ::phy_engine::model::node_t*> net{};
                net.reserve(bdd.size() * 2u);
                net.emplace(0u, const0);
                net.emplace(1u, const1);

                ::std::vector<::phy_engine::netlist::model_pos> new_models{};
                new_models.reserve(c.to_delete.size() + 16u);
                ::std::unordered_map<::phy_engine::model::node_t*, ::phy_engine::model::node_t*> not_cache{};
                not_cache.reserve(1024);
                ::std::vector<::phy_engine::model::node_t*> neg_var{};
                neg_var.assign(var_count, nullptr);

                struct bin_key
                {
                    std::uint8_t op{};
                    ::phy_engine::model::node_t* a{};
                    ::phy_engine::model::node_t* b{};
                };

                struct bin_hash
                {
                    std::size_t operator() (bin_key const& k) const noexcept
                    {
                        auto const mix = [](std::size_t h, std::size_t v) noexcept -> std::size_t
                        { return (h ^ (v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2))); };
                        std::size_t h{};
                        h = mix(h, static_cast<std::size_t>(k.op));
                        h = mix(h, reinterpret_cast<std::size_t>(k.a));
                        h = mix(h, reinterpret_cast<std::size_t>(k.b));
                        return h;
                    }
                };

                struct bin_eq
                {
                    bool operator() (bin_key const& x, bin_key const& y) const noexcept { return x.op == y.op && x.a == y.a && x.b == y.b; }
                };

                ::std::unordered_map<bin_key, ::phy_engine::model::node_t*, bin_hash, bin_eq> bin_cache{};
                bin_cache.reserve(4096);

                auto get_not_node = [&](::phy_engine::model::node_t* in) noexcept -> ::phy_engine::model::node_t*
                {
                    if(in == nullptr) { return nullptr; }
                    if(in == const0) { return const1; }
                    if(in == const1) { return const0; }
                    if(auto it = not_cache.find(in); it != not_cache.end()) { return it->second; }
                    auto* out = make_not(in, new_models);
                    if(out != nullptr) { not_cache.emplace(in, out); }
                    return out;
                };

                auto get_not_var = [&](std::size_t v) noexcept -> ::phy_engine::model::node_t*
                {
                    if(v >= var_count) { return nullptr; }
                    if(neg_var[v] != nullptr) { return neg_var[v]; }
                    neg_var[v] = get_not_node(leaves_ord[v]);
                    return neg_var[v];
                };

                auto make_and_cached = [&](::phy_engine::model::node_t* a, ::phy_engine::model::node_t* b) noexcept -> ::phy_engine::model::node_t*
                {
                    if(a == nullptr || b == nullptr) { return nullptr; }
                    if(a == const0 || b == const0) { return const0; }
                    if(a == const1) { return b; }
                    if(b == const1) { return a; }
                    if(reinterpret_cast<std::uintptr_t>(a) > reinterpret_cast<std::uintptr_t>(b)) { ::std::swap(a, b); }
                    bin_key k{1u, a, b};
                    if(auto it = bin_cache.find(k); it != bin_cache.end()) { return it->second; }
                    auto* out = make_and(a, b, const0, const1, new_models);
                    if(out != nullptr) { bin_cache.emplace(k, out); }
                    return out;
                };
                auto make_or_cached = [&](::phy_engine::model::node_t* a, ::phy_engine::model::node_t* b) noexcept -> ::phy_engine::model::node_t*
                {
                    if(a == nullptr || b == nullptr) { return nullptr; }
                    if(a == const1 || b == const1) { return const1; }
                    if(a == const0) { return b; }
                    if(b == const0) { return a; }
                    if(reinterpret_cast<std::uintptr_t>(a) > reinterpret_cast<std::uintptr_t>(b)) { ::std::swap(a, b); }
                    bin_key k{2u, a, b};
                    if(auto it = bin_cache.find(k); it != bin_cache.end()) { return it->second; }
                    auto* out = make_or(a, b, const0, const1, new_models);
                    if(out != nullptr) { bin_cache.emplace(k, out); }
                    return out;
                };
                auto make_xor_cached = [&](::phy_engine::model::node_t* a, ::phy_engine::model::node_t* b) noexcept -> ::phy_engine::model::node_t*
                {
                    if(a == nullptr || b == nullptr) { return nullptr; }
                    if(a == const0) { return b; }
                    if(b == const0) { return a; }
                    if(a == const1) { return get_not_node(b); }
                    if(b == const1) { return get_not_node(a); }
                    if(a == b) { return const0; }
                    if(reinterpret_cast<std::uintptr_t>(a) > reinterpret_cast<std::uintptr_t>(b)) { ::std::swap(a, b); }
                    bin_key k{3u, a, b};
                    if(auto it = bin_cache.find(k); it != bin_cache.end()) { return it->second; }
                    auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::XOR{});
                    if(m == nullptr) { return nullptr; }
                    new_models.push_back(pos);
                    auto& out_ref = ::phy_engine::netlist::create_node(nl);
                    auto* out = __builtin_addressof(out_ref);
                    if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *a) || !::phy_engine::netlist::add_to_node(nl, *m, 1, *b) ||
                       !::phy_engine::netlist::add_to_node(nl, *m, 2, *out))
                    {
                        (void)::phy_engine::netlist::delete_model(nl, pos);
                        new_models.pop_back();
                        return nullptr;
                    }
                    bin_cache.emplace(k, out);
                    return out;
                };
                auto make_xnor_cached = [&](::phy_engine::model::node_t* a, ::phy_engine::model::node_t* b) noexcept -> ::phy_engine::model::node_t*
                {
                    if(a == nullptr || b == nullptr) { return nullptr; }
                    if(a == const0) { return get_not_node(b); }
                    if(b == const0) { return get_not_node(a); }
                    if(a == const1) { return b; }
                    if(b == const1) { return a; }
                    if(a == b) { return const1; }
                    if(reinterpret_cast<std::uintptr_t>(a) > reinterpret_cast<std::uintptr_t>(b)) { ::std::swap(a, b); }
                    bin_key k{4u, a, b};
                    if(auto it = bin_cache.find(k); it != bin_cache.end()) { return it->second; }
                    auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::XNOR{});
                    if(m == nullptr) { return nullptr; }
                    new_models.push_back(pos);
                    auto& out_ref = ::phy_engine::netlist::create_node(nl);
                    auto* out = __builtin_addressof(out_ref);
                    if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *a) || !::phy_engine::netlist::add_to_node(nl, *m, 1, *b) ||
                       !::phy_engine::netlist::add_to_node(nl, *m, 2, *out))
                    {
                        (void)::phy_engine::netlist::delete_model(nl, pos);
                        new_models.pop_back();
                        return nullptr;
                    }
                    bin_cache.emplace(k, out);
                    return out;
                };

                auto build_net = [&](auto&& self, std::uint32_t id) noexcept -> ::phy_engine::model::node_t*
                {
                    if(id == 0u) { return const0; }
                    if(id == 1u) { return const1; }
                    if(auto it = net.find(id); it != net.end()) { return it->second; }
                    if(id >= bdd.size()) { return nullptr; }
                    auto const& bn = bdd[id];
                    if(static_cast<std::size_t>(bn.var) >= var_count) { return nullptr; }
                    auto* x = leaves_ord[bn.var];
                    if(x == nullptr) { return nullptr; }
                    auto* lo = self(self, bn.lo);
                    auto* hi = self(self, bn.hi);
                    if(lo == nullptr || hi == nullptr) { return nullptr; }
                    if(lo == hi)
                    {
                        net.emplace(id, lo);
                        return lo;
                    }
                    if(x == const0)
                    {
                        net.emplace(id, lo);
                        return lo;
                    }
                    if(x == const1)
                    {
                        net.emplace(id, hi);
                        return hi;
                    }

                    if(lo == const0 && hi == const1)
                    {
                        net.emplace(id, x);
                        return x;
                    }
                    if(lo == const1 && hi == const0)
                    {
                        auto* nx = get_not_var(bn.var);
                        if(nx == nullptr) { return nullptr; }
                        net.emplace(id, nx);
                        return nx;
                    }

                    if(lo == const0)
                    {
                        auto* out = make_and_cached(x, hi);
                        net.emplace(id, out);
                        return out;
                    }
                    if(hi == const0)
                    {
                        auto* nx = get_not_var(bn.var);
                        if(nx == nullptr) { return nullptr; }
                        auto* out = make_and_cached(nx, lo);
                        net.emplace(id, out);
                        return out;
                    }
                    if(lo == const1)
                    {
                        auto* nx = get_not_var(bn.var);
                        if(nx == nullptr) { return nullptr; }
                        auto* out = make_or_cached(nx, hi);
                        net.emplace(id, out);
                        return out;
                    }
                    if(hi == const1)
                    {
                        auto* out = make_or_cached(x, lo);
                        net.emplace(id, out);
                        return out;
                    }

                    auto* nx = get_not_var(bn.var);
                    if(nx == nullptr) { return nullptr; }

                    // Complemented child shortcut (only when we already have the NOT node available):
                    // mux(x, a, ~a) == a ^ x ; mux(x, ~a, a) == a XNOR x.
                    if(auto it = not_cache.find(lo); it != not_cache.end() && it->second == hi)
                    {
                        auto* out = make_xor_cached(x, lo);
                        net.emplace(id, out);
                        return out;
                    }
                    if(auto it = not_cache.find(hi); it != not_cache.end() && it->second == lo)
                    {
                        auto* out = make_xnor_cached(x, hi);
                        net.emplace(id, out);
                        return out;
                    }

                    auto* t1 = make_and_cached(x, hi);
                    auto* t0 = make_and_cached(nx, lo);
                    auto* out = make_or_cached(t0, t1);
                    net.emplace(id, out);
                    return out;
                };

                auto* new_out = build_net(build_net, root_id);
                if(new_out == nullptr)
                {
                    for(auto const mp: new_models) { (void)::phy_engine::netlist::delete_model(nl, mp); }
                    continue;
                }

                auto const old_cost = c.to_delete.size();
                auto const new_cost = new_models.size() + (is_protected(root) ? 1u : 0u);
                if(new_cost >= old_cost)
                {
                    for(auto const mp: new_models) { (void)::phy_engine::netlist::delete_model(nl, mp); }
                    continue;
                }

                if(is_protected(root))
                {
                    // Delete old driver cone, then connect via YES buffer to keep the protected node intact.
                    for(auto const mp: c.to_delete) { (void)::phy_engine::netlist::delete_model(nl, mp); }
                    if(!add_yes(new_out, root))
                    {
                        // best-effort; leave the new cone to be DCE'd if unconnected.
                    }
                }
                else
                {
                    if(!move_consumers(root, new_out, c.root_out_pin))
                    {
                        for(auto const mp: new_models) { (void)::phy_engine::netlist::delete_model(nl, mp); }
                        continue;
                    }
                    for(auto const mp: c.to_delete) { (void)::phy_engine::netlist::delete_model(nl, mp); }
                }

                changed = true;
            }

            return changed;
        }

        [[nodiscard]] inline bool optimize_bounded_resubstitute_in_pe_netlist(::phy_engine::netlist::netlist& nl,
                                                                              ::std::vector<::phy_engine::model::node_t*> const& protected_nodes,
                                                                              pe_synth_options const& opt) noexcept
        {
            if(!opt.assume_binary_inputs) { return false; }

            if(opt.resub_max_vars == 0u) { return false; }

            // If CUDA is enabled and the user requests a larger window, switch to a bitset truth-table path (<=16 vars, <=256 gates).
            if(opt.cuda_enable && opt.resub_max_vars > 6u)
            {
                std::size_t const max_vars = ::std::min<std::size_t>(16u, opt.resub_max_vars);
                std::size_t const max_gates = ::std::min<std::size_t>(256u, (opt.resub_max_gates == 0u) ? 64u : opt.resub_max_gates);
                if(max_vars <= 6u || max_gates == 0u) { return false; }

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

                ::std::unordered_map<::phy_engine::model::node_t*, bool> protected_map{};
                protected_map.reserve(protected_nodes.size() * 2u + 1u);
                for(auto* n: protected_nodes) { protected_map.emplace(n, true); }
                auto const is_protected = [&](::phy_engine::model::node_t* n) noexcept -> bool { return protected_map.contains(n); };

                auto const fan = build_gate_opt_fanout(nl);

                ::std::unordered_map<::phy_engine::model::node_t*, gate> gate_by_out{};
                gate_by_out.reserve(1 << 14);

                auto classify = [&](::phy_engine::model::model_base const& mb, ::phy_engine::netlist::model_pos pos) noexcept -> ::std::optional<gate>
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

                for(std::size_t chunk_pos{}; chunk_pos < nl.models.size(); ++chunk_pos)
                {
                    auto& blk = nl.models.index_unchecked(chunk_pos);
                    for(std::size_t vec_pos{}; blk.begin + vec_pos < blk.curr; ++vec_pos)
                    {
                        auto const& mb = blk.begin[vec_pos];
                        auto og = classify(mb, {vec_pos, chunk_pos});
                        if(!og || og->out == nullptr || og->out_pin == nullptr) { continue; }
                        gate_by_out.emplace(og->out, *og);
                    }
                }

                ::std::unordered_map<::phy_engine::model::node_t*, ::phy_engine::model::node_t*> not_of{};
                not_of.reserve(1 << 12);
                for(auto const& kv: gate_by_out)
                {
                    auto const& g = kv.second;
                    if(g.k != kind::not_gate) { continue; }
                    if(g.in0 == nullptr || g.out == nullptr) { continue; }
                    not_of.emplace(g.in0, g.out);
                }

                ::std::unordered_map<::phy_engine::model::node_t*, ::std::uint16_t> const_idx{};
                const_idx.reserve(256);
                for(auto const& blk: nl.models)
                {
                    for(auto const* m = blk.begin; m != blk.curr; ++m)
                    {
                        if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }
                        if(m->name.size() != 0) { continue; }
                        if(model_name_u8(*m) != u8"INPUT") { continue; }
                        auto pv = m->ptr->generate_pin_view();
                        if(pv.size != 1 || pv.pins[0].nodes == nullptr) { continue; }
                        auto vi = m->ptr->get_attribute(0);
                        if(vi.type != ::phy_engine::model::variant_type::digital) { continue; }
                        if(vi.digital == ::phy_engine::model::digital_node_statement_t::true_state) { const_idx.emplace(pv.pins[0].nodes, 65535u); }
                        else if(vi.digital == ::phy_engine::model::digital_node_statement_t::false_state) { const_idx.emplace(pv.pins[0].nodes, 65534u); }
                    }
                }

                auto enc_kind = [&](kind k) noexcept -> ::std::uint8_t
                {
                    switch(k)
                    {
                        case kind::not_gate: return 0u;
                        case kind::and_gate: return 1u;
                        case kind::or_gate: return 2u;
                        case kind::xor_gate: return 3u;
                        case kind::xnor_gate: return 4u;
                        case kind::nand_gate: return 5u;
                        case kind::nor_gate: return 6u;
                        case kind::imp_gate: return 7u;
                        case kind::nimp_gate: return 8u;
                        default: return 255u;
                    }
                };

                auto move_consumers =
                    [&](::phy_engine::model::node_t* from, ::phy_engine::model::node_t* to, ::phy_engine::model::pin const* from_driver_pin) noexcept -> bool
                {
                    if(from == nullptr || to == nullptr || from_driver_pin == nullptr) { return false; }
                    if(from == to) { return false; }
                    if(is_protected(from)) { return false; }
                    if(!has_unique_driver_pin(from, from_driver_pin, fan.pin_out)) { return false; }

                    ::std::vector<::phy_engine::model::pin*> pins_to_move{};
                    pins_to_move.reserve(from->pins.size());
                    for(auto* p: from->pins)
                    {
                        if(p == from_driver_pin) { continue; }
                        pins_to_move.push_back(p);
                    }
                    for(auto* p: pins_to_move)
                    {
                        from->pins.erase(p);
                        p->nodes = to;
                        to->pins.insert(p);
                    }
                    return true;
                };

                struct cand
                {
                    ::phy_engine::model::node_t* root{};
                    gate g{};
                    ::std::vector<::phy_engine::model::node_t*> leaves{};
                    cuda_tt_cone_desc cone{};
                    ::std::uint32_t blocks{};
                    ::std::uint64_t last_mask{};
                    ::std::uint64_t sig{};
                    ::std::uint64_t sig_c{};
                    bool ok{};
                };

                ::std::vector<::phy_engine::model::node_t*> roots{};
                roots.reserve(gate_by_out.size());
                for(auto const& kv: gate_by_out) { roots.push_back(kv.first); }

                ::std::vector<cand> cands{};
                cands.reserve(roots.size());
                ::std::uint32_t stride_blocks{};

                auto sig_tt = [&](::std::uint64_t const* w, ::std::uint32_t blocks) noexcept -> ::std::uint64_t
                {
                    ::std::uint64_t h = 1469598103934665603ull;
                    for(::std::uint32_t i{}; i < blocks; ++i)
                    {
                        h ^= w[i];
                        h *= 1099511628211ull;
                    }
                    return h;
                };

                auto hash_leaves = [&](::std::vector<::phy_engine::model::node_t*> const& leaves) noexcept -> ::std::uint64_t
                {
                    auto mix = [](std::uint64_t h, std::uint64_t v) noexcept -> std::uint64_t
                    { return (h ^ (v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2))); };
                    std::uint64_t h{};
                    for(auto* p: leaves) { h = mix(h, static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(p))); }
                    return h;
                };

                for(auto* root: roots)
                {
                    auto it_root = gate_by_out.find(root);
                    if(it_root == gate_by_out.end()) { continue; }
                    auto const& g = it_root->second;

                    // Ensure the gate still exists and matches our snapshot.
                    {
                        auto* mb = ::phy_engine::netlist::get_model(nl, g.pos);
                        if(mb == nullptr || mb->type != ::phy_engine::model::model_type::normal || mb->ptr == nullptr) { continue; }
                    }

                    cand cd{};
                    cd.root = root;
                    cd.g = g;

                    ::std::vector<::phy_engine::model::node_t*> leaves{};
                    leaves.reserve(max_vars);
                    ::std::unordered_map<::phy_engine::model::node_t*, bool> visited{};
                    visited.reserve(max_gates * 2u);
                    ::std::unordered_map<::phy_engine::model::node_t*, bool> leaf_seen{};
                    leaf_seen.reserve(max_vars * 2u);

                    std::size_t gate_count{};
                    bool ok{true};
                    auto dfs = [&](auto&& self, ::phy_engine::model::node_t* n) noexcept -> void
                    {
                        if(!ok || n == nullptr) { return; }
                        if(visited.contains(n)) { return; }
                        visited.emplace(n, true);

                        if(const_idx.contains(n)) { return; }  // constants are not variable leaves

                        auto itg = gate_by_out.find(n);
                        if(itg == gate_by_out.end())
                        {
                            if(!leaf_seen.contains(n))
                            {
                                if(leaves.size() >= max_vars) { ok = false; return; }
                                leaf_seen.emplace(n, true);
                                leaves.push_back(n);
                            }
                            return;
                        }

                        if(++gate_count > max_gates) { ok = false; return; }
                        auto const& gg = itg->second;
                        self(self, gg.in0);
                        if(gg.k != kind::not_gate) { self(self, gg.in1); }
                    };

                    dfs(dfs, root);
                    if(!ok) { continue; }

                    ::std::sort(leaves.begin(),
                                leaves.end(),
                                [](auto* a, auto* b) noexcept { return reinterpret_cast<std::uintptr_t>(a) < reinterpret_cast<std::uintptr_t>(b); });

                    if(leaves.size() <= 6u || leaves.size() > max_vars) { continue; }  // this TT path is for >6 vars
                    auto const var_count = leaves.size();
                    auto const U = static_cast<std::size_t>(1u) << var_count;
                    auto const blocks = static_cast<::std::uint32_t>((U + 63u) / 64u);
                    if(blocks == 0u) { continue; }
                    if(blocks > stride_blocks) { stride_blocks = blocks; }

                    ::std::unordered_map<::phy_engine::model::node_t*, ::std::uint16_t> leaf_index{};
                    leaf_index.reserve(var_count * 2u + 8u);
                    for(::std::uint16_t i{}; i < static_cast<::std::uint16_t>(var_count); ++i) { leaf_index.emplace(leaves[i], i); }

                    cuda_tt_cone_desc cone{};
                    cone.var_count = static_cast<::std::uint8_t>(var_count);
                    cone.gate_count = 0u;

                    ::std::unordered_map<::phy_engine::model::node_t*, ::std::uint16_t> out_index{};
                    out_index.reserve(gate_count * 2u + 8u);
                    bool ok2{true};

                    auto dfs2 = [&](auto&& self, ::phy_engine::model::node_t* n) noexcept -> ::std::uint16_t
                    {
                        if(!ok2 || n == nullptr) { ok2 = false; return 65534u; }
                        if(auto itc = const_idx.find(n); itc != const_idx.end()) { return itc->second; }
                        if(auto itl = leaf_index.find(n); itl != leaf_index.end()) { return itl->second; }
                        if(auto ito = out_index.find(n); ito != out_index.end()) { return ito->second; }

                        auto itg = gate_by_out.find(n);
                        if(itg == gate_by_out.end()) { ok2 = false; return 65534u; }
                        auto const& gg = itg->second;

                        auto const a = self(self, gg.in0);
                        auto const b = (gg.k == kind::not_gate) ? static_cast<::std::uint16_t>(65534u) : self(self, gg.in1);
                        if(!ok2) { return 65534u; }

                        if(cone.gate_count >= 256u) { ok2 = false; return 65534u; }
                        auto const gi = static_cast<::std::uint16_t>(cone.gate_count++);
                        cone.kind[gi] = enc_kind(gg.k);
                        cone.in0[gi] = a;
                        cone.in1[gi] = b;
                        auto const out = static_cast<::std::uint16_t>(16u + gi);
                        out_index.emplace(n, out);
                        return out;
                    };

                    (void)dfs2(dfs2, root);
                    if(!ok2 || cone.gate_count == 0u) { continue; }

                    auto const rem = static_cast<::std::uint32_t>(U & 63u);
                    auto const last_mask = (rem == 0u) ? ~0ull : ((1ull << rem) - 1ull);

                    cd.leaves = ::std::move(leaves);
                    cd.cone = cone;
                    cd.blocks = blocks;
                    cd.last_mask = last_mask;
                    cd.ok = true;
                    cands.push_back(::std::move(cd));
                }

                if(cands.empty() || stride_blocks == 0u) { return false; }

                ::std::vector<::std::uint64_t> tt_words{};
                tt_words.resize(cands.size() * static_cast<std::size_t>(stride_blocks));

                ::std::vector<cuda_tt_cone_desc> cones{};
                cones.reserve(cands.size());
                for(auto const& cd: cands) { cones.push_back(cd.cone); }

                bool used_cuda{};
                if(cands.size() >= 16u || stride_blocks >= 64u)
                {
                    used_cuda = cuda_eval_tt_cones(opt.cuda_device_mask, cones.data(), cones.size(), stride_blocks, tt_words.data());
                }
                if(!used_cuda)
                {
                    for(std::size_t i{}; i < cands.size(); ++i)
                    {
                        eval_tt_cone_cpu(cones[i], stride_blocks, tt_words.data() + i * static_cast<std::size_t>(stride_blocks));
                    }
                }

                for(std::size_t i{}; i < cands.size(); ++i)
                {
                    auto const* base = tt_words.data() + i * static_cast<std::size_t>(stride_blocks);
                    auto const blocks = cands[i].blocks;

                    cands[i].sig = sig_tt(base, blocks);

                    // complemented signature (masked)
                    ::std::uint64_t h = 1469598103934665603ull;
                    for(::std::uint32_t w{}; w < blocks; ++w)
                    {
                        ::std::uint64_t vv = ~base[w];
                        if(w + 1u == blocks) { vv &= cands[i].last_mask; }
                        h ^= vv;
                        h *= 1099511628211ull;
                    }
                    cands[i].sig_c = h;
                }

                auto tt_equal = [&](std::size_t i, std::size_t j, bool complemented_j) noexcept -> bool
                {
                    auto const bi = cands[i].blocks;
                    if(bi != cands[j].blocks) { return false; }
                    auto const* wi = tt_words.data() + i * static_cast<std::size_t>(stride_blocks);
                    auto const* wj = tt_words.data() + j * static_cast<std::size_t>(stride_blocks);
                    for(::std::uint32_t w{}; w < bi; ++w)
                    {
                        ::std::uint64_t a = wi[w];
                        ::std::uint64_t b = complemented_j ? ~wj[w] : wj[w];
                        if(w + 1u == bi)
                        {
                            a &= cands[i].last_mask;
                            b &= cands[i].last_mask;
                        }
                        if(a != b) { return false; }
                    }
                    return true;
                };

                ::std::unordered_map<::std::uint64_t, ::std::vector<std::size_t>> rep{};
                rep.reserve(cands.size() * 2u + 1u);

                auto key_of = [&](std::vector<::phy_engine::model::node_t*> const& leaves, ::std::uint64_t sig) noexcept -> ::std::uint64_t
                {
                    auto const lh = hash_leaves(leaves);
                    auto mix = [](std::uint64_t h, std::uint64_t v) noexcept -> std::uint64_t
                    { return (h ^ (v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2))); };
                    std::uint64_t k{};
                    k = mix(k, lh);
                    k = mix(k, sig);
                    return k;
                };

                bool changed{};
                for(std::size_t i{}; i < cands.size(); ++i)
                {
                    auto& cd = cands[i];
                    if(!cd.ok) { continue; }

                    auto* root = cd.root;
                    auto const& g = cd.g;

                    // Ensure the gate still exists and matches our snapshot.
                    {
                        auto* mb = ::phy_engine::netlist::get_model(nl, g.pos);
                        if(mb == nullptr || mb->type != ::phy_engine::model::model_type::normal || mb->ptr == nullptr) { continue; }
                    }

                    // Complement reuse: if we can replace with NOT(rep) cheaply.
                    if(g.k != kind::not_gate && !is_protected(root))
                    {
                        auto const kcomp = key_of(cd.leaves, cd.sig_c);
                        if(auto it = rep.find(kcomp); it != rep.end())
                        {
                            for(auto const j: it->second)
                            {
                                if(cands[j].leaves != cd.leaves) { continue; }
                                if(!tt_equal(i, j, true)) { continue; }
                                auto* rep_node = cands[j].root;
                                if(rep_node == nullptr || rep_node == root) { continue; }
                                auto itn = not_of.find(rep_node);
                                if(itn == not_of.end() || itn->second == nullptr) { continue; }
                                auto* rep_not = itn->second;
                                if(rep_not == root) { continue; }
                                if(move_consumers(root, rep_not, g.out_pin))
                                {
                                    (void)::phy_engine::netlist::delete_model(nl, g.pos);
                                    changed = true;
                                    goto next_root;
                                }
                            }
                        }
                    }

                    {
                        auto const kself = key_of(cd.leaves, cd.sig);
                        auto& vec = rep[kself];
                        // Check existing reps for a true match (avoid rare hash collisions).
                        for(auto const j: vec)
                        {
                            if(cands[j].leaves != cd.leaves) { continue; }
                            if(!tt_equal(i, j, false)) { continue; }
                            // Matched: keep first rep, try to delete current.
                            if(is_protected(root)) { goto next_root; }
                            auto* rep_node = cands[j].root;
                            if(rep_node == nullptr || rep_node == root) { goto next_root; }
                            if(move_consumers(root, rep_node, g.out_pin))
                            {
                                (void)::phy_engine::netlist::delete_model(nl, g.pos);
                                changed = true;
                            }
                            goto next_root;
                        }
                        vec.push_back(i);
                    }

                next_root:
                    continue;
                }

                return changed;
            }

            // The default implementation uses a 64-bit truth-table mask, so vars are capped at 6.
            std::size_t const max_vars = ::std::min<std::size_t>(6u, opt.resub_max_vars);
            std::size_t const max_gates = (opt.resub_max_gates == 0u) ? 64u : opt.resub_max_gates;

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
                ::std::vector<::phy_engine::model::node_t*> leaves{};
                ::std::uint64_t mask{};
            };

            struct key_hash
            {
                std::size_t operator() (key const& k) const noexcept
                {
                    auto const mix = [](std::size_t h, std::size_t v) noexcept -> std::size_t
                    { return (h ^ (v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2))); };
                    std::size_t h{};
                    h = mix(h, static_cast<std::size_t>(k.mask ^ (k.mask >> 32)));
                    for(auto* p: k.leaves) { h = mix(h, reinterpret_cast<std::size_t>(p)); }
                    return h;
                }
            };

            struct key_eq
            {
                bool operator() (key const& a, key const& b) const noexcept { return a.mask == b.mask && a.leaves == b.leaves; }
            };

            ::std::unordered_map<::phy_engine::model::node_t*, bool> protected_map{};
            protected_map.reserve(protected_nodes.size() * 2u + 1u);
            for(auto* n: protected_nodes) { protected_map.emplace(n, true); }
            auto const is_protected = [&](::phy_engine::model::node_t* n) noexcept -> bool { return protected_map.contains(n); };

            auto const fan = build_gate_opt_fanout(nl);

            ::std::unordered_map<::phy_engine::model::node_t*, gate> gate_by_out{};
            gate_by_out.reserve(1 << 14);
            ::std::unordered_map<::phy_engine::model::node_t*, ::phy_engine::model::node_t*> not_of{};
            not_of.reserve(1 << 14);

            auto classify = [&](::phy_engine::model::model_base const& mb, ::phy_engine::netlist::model_pos pos) noexcept -> ::std::optional<gate>
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
                    else
                    {
                        g.k = kind::nimp_gate;
                    }
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
                    if(!og || og->out == nullptr || og->out_pin == nullptr) { continue; }
                    gate_by_out.emplace(og->out, *og);
                    if(og->k == kind::not_gate && og->in0 != nullptr) { not_of.emplace(og->in0, og->out); }
                }
            }

            // Detect constant nodes produced by unnamed INPUT models.
            ::std::unordered_map<::phy_engine::model::node_t*, ::std::uint8_t> const_idx{};
            const_idx.reserve(128);
            for(auto const& blk : nl.models)
            {
                for(auto const* m = blk.begin; m != blk.curr; ++m)
                {
                    if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }
                    if(model_name_u8(*m) != u8"INPUT") { continue; }
                    if(m->name.size() != 0) { continue; }
                    auto pv = m->ptr->generate_pin_view();
                    if(pv.size != 1 || pv.pins[0].nodes == nullptr) { continue; }
                    auto vi = m->ptr->get_attribute(0);
                    if(vi.type != ::phy_engine::model::variant_type::digital) { continue; }
                    if(vi.digital == ::phy_engine::model::digital_node_statement_t::true_state) { const_idx.emplace(pv.pins[0].nodes, 255u); }
                    else if(vi.digital == ::phy_engine::model::digital_node_statement_t::false_state) { const_idx.emplace(pv.pins[0].nodes, 254u); }
                }
            }

            auto move_consumers =
                [&](::phy_engine::model::node_t* from, ::phy_engine::model::node_t* to, ::phy_engine::model::pin const* from_driver_pin) noexcept -> bool
            {
                if(from == nullptr || to == nullptr || from_driver_pin == nullptr) { return false; }
                if(from == to) { return false; }
                if(is_protected(from)) { return false; }
                if(!has_unique_driver_pin(from, from_driver_pin, fan.pin_out)) { return false; }

                ::std::vector<::phy_engine::model::pin*> pins_to_move{};
                pins_to_move.reserve(from->pins.size());
                for(auto* p: from->pins)
                {
                    if(p == from_driver_pin) { continue; }
                    pins_to_move.push_back(p);
                }
                for(auto* p: pins_to_move)
                {
                    from->pins.erase(p);
                    p->nodes = to;
                    to->pins.insert(p);
                }
                return true;
            };

            ::std::unordered_map<key, ::phy_engine::model::node_t*, key_hash, key_eq> rep{};
            rep.reserve(1 << 12);

            struct cand
            {
                ::phy_engine::model::node_t* root{};
                gate g{};
                ::std::vector<::phy_engine::model::node_t*> leaves{};
                cuda_u64_cone_desc cone{};
                ::std::uint64_t mask{};
                bool ok{};
            };

            ::std::vector<::phy_engine::model::node_t*> roots{};
            roots.reserve(gate_by_out.size());
            for(auto const& kv: gate_by_out) { roots.push_back(kv.first); }

            ::std::vector<cand> cands{};
            cands.reserve(roots.size());

            auto enc_kind = [&](kind k) noexcept -> ::std::uint8_t
            {
                switch(k)
                {
                    case kind::not_gate: return 0u;
                    case kind::and_gate: return 1u;
                    case kind::or_gate: return 2u;
                    case kind::xor_gate: return 3u;
                    case kind::xnor_gate: return 4u;
                    case kind::nand_gate: return 5u;
                    case kind::nor_gate: return 6u;
                    case kind::imp_gate: return 7u;
                    case kind::nimp_gate: return 8u;
                    default: return 255u;
                }
            };

            for(auto* root: roots)
            {
                auto it_root = gate_by_out.find(root);
                if(it_root == gate_by_out.end()) { continue; }
                auto const& g = it_root->second;

                // Ensure the gate still exists and matches our snapshot.
                {
                    auto* mb = ::phy_engine::netlist::get_model(nl, g.pos);
                    if(mb == nullptr || mb->type != ::phy_engine::model::model_type::normal || mb->ptr == nullptr) { continue; }
                }

                cand cd{};
                cd.root = root;
                cd.g = g;

                ::std::vector<::phy_engine::model::node_t*> leaves{};
                leaves.reserve(max_vars);
                ::std::unordered_map<::phy_engine::model::node_t*, bool> visited{};
                visited.reserve(max_gates * 2u);
                ::std::unordered_map<::phy_engine::model::node_t*, bool> leaf_seen{};
                leaf_seen.reserve(max_vars * 2u);

                std::size_t gate_count{};
                bool ok{true};
                auto dfs = [&](auto&& self, ::phy_engine::model::node_t* n) noexcept -> void
                {
                    if(!ok || n == nullptr) { return; }
                    if(visited.contains(n)) { return; }
                    visited.emplace(n, true);

                    if(const_idx.contains(n)) { return; }  // treat as constant, not as a variable leaf

                    auto itg = gate_by_out.find(n);
                    if(itg == gate_by_out.end())
                    {
                        if(!leaf_seen.contains(n))
                        {
                            if(leaves.size() >= max_vars)
                            {
                                ok = false;
                                return;
                            }
                            leaf_seen.emplace(n, true);
                            leaves.push_back(n);
                        }
                        return;
                    }

                    if(++gate_count > max_gates)
                    {
                        ok = false;
                        return;
                    }
                    auto const& gg = itg->second;
                    self(self, gg.in0);
                    if(gg.k != kind::not_gate) { self(self, gg.in1); }
                };

                dfs(dfs, root);
                if(!ok) { continue; }

                ::std::sort(leaves.begin(),
                            leaves.end(),
                            [](auto* a, auto* b) noexcept { return reinterpret_cast<std::uintptr_t>(a) < reinterpret_cast<std::uintptr_t>(b); });

                if(leaves.size() > max_vars) { continue; }
                auto const var_count = leaves.size();
                auto const U = static_cast<std::size_t>(1u << var_count);
                if(U > 64u) { continue; }

                ::std::unordered_map<::phy_engine::model::node_t*, ::std::uint8_t> leaf_index{};
                leaf_index.reserve(var_count * 2u + 8u);
                for(std::size_t i{}; i < var_count; ++i) { leaf_index.emplace(leaves[i], static_cast<::std::uint8_t>(i)); }

                cuda_u64_cone_desc cone{};
                cone.var_count = static_cast<::std::uint8_t>(var_count);
                cone.gate_count = 0u;

                ::std::unordered_map<::phy_engine::model::node_t*, ::std::uint8_t> out_index{};
                out_index.reserve(gate_count * 2u + 8u);
                bool ok2{true};

                auto dfs2 = [&](auto&& self, ::phy_engine::model::node_t* n) noexcept -> ::std::uint8_t
                {
                    if(!ok2 || n == nullptr) { ok2 = false; return 254u; }
                    if(auto itc = const_idx.find(n); itc != const_idx.end()) { return itc->second; }
                    if(auto itl = leaf_index.find(n); itl != leaf_index.end()) { return itl->second; }
                    if(auto ito = out_index.find(n); ito != out_index.end()) { return ito->second; }

                    auto itg = gate_by_out.find(n);
                    if(itg == gate_by_out.end()) { ok2 = false; return 254u; }
                    auto const& gg = itg->second;

                    auto const a = self(self, gg.in0);
                    auto const b = (gg.k == kind::not_gate) ? static_cast<::std::uint8_t>(254u) : self(self, gg.in1);
                    if(!ok2) { return 254u; }

                    if(cone.gate_count >= 64u) { ok2 = false; return 254u; }
                    auto const gi = static_cast<::std::uint8_t>(cone.gate_count++);
                    cone.kind[gi] = enc_kind(gg.k);
                    cone.in0[gi] = a;
                    cone.in1[gi] = b;
                    auto const out = static_cast<::std::uint8_t>(6u + gi);
                    out_index.emplace(n, out);
                    return out;
                };

                (void)dfs2(dfs2, root);
                if(!ok2 || cone.gate_count == 0u) { continue; }

                cd.leaves = ::std::move(leaves);
                cd.cone = cone;
                cd.ok = true;
                cands.push_back(::std::move(cd));
            }

            // Evaluate truth-table masks (CUDA best-effort).
            bool used_cuda{};
            if(opt.cuda_enable && cands.size() >= opt.cuda_min_batch)
            {
                ::std::vector<cuda_u64_cone_desc> cones{};
                ::std::vector<std::size_t> map_idx{};
                cones.reserve(cands.size());
                map_idx.reserve(cands.size());
                for(std::size_t i{}; i < cands.size(); ++i)
                {
                    if(!cands[i].ok) { continue; }
                    cones.push_back(cands[i].cone);
                    map_idx.push_back(i);
                }
                ::std::vector<::std::uint64_t> masks{};
                masks.resize(cones.size());
                if(!cones.empty() && cuda_eval_u64_cones(opt.cuda_device_mask, cones.data(), cones.size(), masks.data()))
                {
                    used_cuda = true;
                    for(std::size_t i{}; i < map_idx.size(); ++i) { cands[map_idx[i]].mask = masks[i]; }
                }
            }
            if(!used_cuda)
            {
                for(auto& cd : cands)
                {
                    if(!cd.ok) { continue; }
                    cd.mask = eval_u64_cone_cpu(cd.cone);
                }
            }

            bool changed{};

            for(auto& cd : cands)
            {
                auto* root = cd.root;
                auto const& g = cd.g;

                // Ensure the gate still exists and matches our snapshot.
                {
                    auto* mb = ::phy_engine::netlist::get_model(nl, g.pos);
                    if(mb == nullptr || mb->type != ::phy_engine::model::model_type::normal || mb->ptr == nullptr) { continue; }
                }

                auto const var_count = static_cast<std::size_t>(cd.cone.var_count);
                auto const U = (var_count >= 6u) ? 64u : (static_cast<std::size_t>(1u) << var_count);
                auto const all_mask = (U == 64u) ? ~0ull : ((1ull << U) - 1ull);
                auto const mask = cd.mask & all_mask;

                key k{cd.leaves, mask};
                key kc{cd.leaves, static_cast<::std::uint64_t>(mask ^ all_mask)};

                if(g.k != kind::not_gate && !is_protected(root))
                {
                    auto itc = rep.find(kc);
                    if(itc != rep.end())
                    {
                        auto* rep_node = itc->second;
                        if(rep_node != nullptr && rep_node != root)
                        {
                            auto it_not = not_of.find(rep_node);
                            if(it_not != not_of.end() && it_not->second != nullptr)
                            {
                                auto* rep_not = it_not->second;
                                if(rep_not != root && move_consumers(root, rep_not, g.out_pin))
                                {
                                    (void)::phy_engine::netlist::delete_model(nl, g.pos);
                                    changed = true;
                                    continue;
                                }
                            }
                        }
                    }
                }

                if(rep.find(k) == rep.end()) { rep.emplace(::std::move(k), root); }
            }

            return changed;
        }

        [[nodiscard]] inline bool optimize_bounded_sweep_in_pe_netlist(::phy_engine::netlist::netlist& nl,
                                                                       ::std::vector<::phy_engine::model::node_t*> const& protected_nodes,
                                                                       pe_synth_options const& opt) noexcept
        {
            if(!opt.assume_binary_inputs) { return false; }

            if(opt.sweep_max_vars == 0u) { return false; }

            // If CUDA is enabled and the user requests a larger window, switch to a bitset truth-table path (<=16 vars, <=256 gates).
            if(opt.cuda_enable && opt.sweep_max_vars > 6u)
            {
                std::size_t const max_vars = ::std::min<std::size_t>(16u, opt.sweep_max_vars);
                std::size_t const max_gates = ::std::min<std::size_t>(256u, (opt.sweep_max_gates == 0u) ? 64u : opt.sweep_max_gates);
                if(max_vars <= 6u || max_gates == 0u) { return false; }

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

                struct rep_entry
                {
                    std::size_t idx{};
                };

                ::std::unordered_map<::phy_engine::model::node_t*, bool> protected_map{};
                protected_map.reserve(protected_nodes.size() * 2u + 1u);
                for(auto* n: protected_nodes) { protected_map.emplace(n, true); }
                auto const is_protected = [&](::phy_engine::model::node_t* n) noexcept -> bool { return protected_map.contains(n); };

                auto const fan = build_gate_opt_fanout(nl);

                ::std::unordered_map<::phy_engine::model::node_t*, gate> gate_by_out{};
                gate_by_out.reserve(1 << 14);

                auto classify = [&](::phy_engine::model::model_base const& mb, ::phy_engine::netlist::model_pos pos) noexcept -> ::std::optional<gate>
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

                for(std::size_t chunk_pos{}; chunk_pos < nl.models.size(); ++chunk_pos)
                {
                    auto& blk = nl.models.index_unchecked(chunk_pos);
                    for(std::size_t vec_pos{}; blk.begin + vec_pos < blk.curr; ++vec_pos)
                    {
                        auto const& mb = blk.begin[vec_pos];
                        auto og = classify(mb, {vec_pos, chunk_pos});
                        if(!og || og->out == nullptr || og->out_pin == nullptr) { continue; }
                        gate_by_out.emplace(og->out, *og);
                    }
                }

                ::std::unordered_map<::phy_engine::model::node_t*, ::std::uint16_t> const_idx{};
                const_idx.reserve(256);
                for(auto const& blk: nl.models)
                {
                    for(auto const* m = blk.begin; m != blk.curr; ++m)
                    {
                        if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }
                        if(m->name.size() != 0) { continue; }
                        if(model_name_u8(*m) != u8"INPUT") { continue; }
                        auto pv = m->ptr->generate_pin_view();
                        if(pv.size != 1 || pv.pins[0].nodes == nullptr) { continue; }
                        auto vi = m->ptr->get_attribute(0);
                        if(vi.type != ::phy_engine::model::variant_type::digital) { continue; }
                        if(vi.digital == ::phy_engine::model::digital_node_statement_t::true_state) { const_idx.emplace(pv.pins[0].nodes, 65535u); }
                        else if(vi.digital == ::phy_engine::model::digital_node_statement_t::false_state) { const_idx.emplace(pv.pins[0].nodes, 65534u); }
                    }
                }

                auto enc_kind = [&](kind k) noexcept -> ::std::uint8_t
                {
                    switch(k)
                    {
                        case kind::not_gate: return 0u;
                        case kind::and_gate: return 1u;
                        case kind::or_gate: return 2u;
                        case kind::xor_gate: return 3u;
                        case kind::xnor_gate: return 4u;
                        case kind::nand_gate: return 5u;
                        case kind::nor_gate: return 6u;
                        case kind::imp_gate: return 7u;
                        case kind::nimp_gate: return 8u;
                        default: return 255u;
                    }
                };

                auto move_consumers =
                    [&](::phy_engine::model::node_t* from, ::phy_engine::model::node_t* to, ::phy_engine::model::pin const* from_driver_pin) noexcept -> bool
                {
                    if(from == nullptr || to == nullptr || from_driver_pin == nullptr) { return false; }
                    if(from == to) { return false; }
                    if(is_protected(from)) { return false; }
                    if(!has_unique_driver_pin(from, from_driver_pin, fan.pin_out)) { return false; }

                    ::std::vector<::phy_engine::model::pin*> pins_to_move{};
                    pins_to_move.reserve(from->pins.size());
                    for(auto* p: from->pins)
                    {
                        if(p == from_driver_pin) { continue; }
                        pins_to_move.push_back(p);
                    }
                    for(auto* p: pins_to_move)
                    {
                        from->pins.erase(p);
                        p->nodes = to;
                        to->pins.insert(p);
                    }
                    return true;
                };

                auto sig_tt = [&](::std::uint64_t const* w, ::std::uint32_t blocks) noexcept -> ::std::uint64_t
                {
                    ::std::uint64_t h = 1469598103934665603ull;
                    for(::std::uint32_t i{}; i < blocks; ++i)
                    {
                        h ^= w[i];
                        h *= 1099511628211ull;
                    }
                    return h;
                };

                auto hash_leaves = [&](::std::vector<::phy_engine::model::node_t*> const& leaves) noexcept -> ::std::uint64_t
                {
                    auto mix = [](std::uint64_t h, std::uint64_t v) noexcept -> std::uint64_t
                    { return (h ^ (v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2))); };
                    std::uint64_t h{};
                    for(auto* p: leaves) { h = mix(h, static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(p))); }
                    return h;
                };

                struct cand
                {
                    ::phy_engine::model::node_t* root{};
                    gate g{};
                    ::std::vector<::phy_engine::model::node_t*> leaves{};
                    cuda_tt_cone_desc cone{};
                    ::std::uint32_t blocks{};
                    ::std::uint64_t last_mask{};
                    ::std::uint64_t sig{};
                    bool ok{};
                };

                ::std::vector<::phy_engine::model::node_t*> roots{};
                roots.reserve(gate_by_out.size());
                for(auto const& kv: gate_by_out) { roots.push_back(kv.first); }

                ::std::vector<cand> cands{};
                cands.reserve(roots.size());
                ::std::uint32_t stride_blocks{};

                for(auto* root: roots)
                {
                    auto it_root = gate_by_out.find(root);
                    if(it_root == gate_by_out.end()) { continue; }
                    auto const& g = it_root->second;

                    // Ensure the gate still exists and matches our snapshot.
                    {
                        auto* mb = ::phy_engine::netlist::get_model(nl, g.pos);
                        if(mb == nullptr || mb->type != ::phy_engine::model::model_type::normal || mb->ptr == nullptr) { continue; }
                    }

                    ::std::vector<::phy_engine::model::node_t*> leaves{};
                    leaves.reserve(max_vars);
                    ::std::unordered_map<::phy_engine::model::node_t*, bool> visited{};
                    visited.reserve(max_gates * 2u);
                    ::std::unordered_map<::phy_engine::model::node_t*, bool> leaf_seen{};
                    leaf_seen.reserve(max_vars * 2u);

                    std::size_t gate_count{};
                    bool ok{true};
                    auto dfs = [&](auto&& self, ::phy_engine::model::node_t* n) noexcept -> void
                    {
                        if(!ok || n == nullptr) { return; }
                        if(visited.contains(n)) { return; }
                        visited.emplace(n, true);

                        if(const_idx.contains(n)) { return; }

                        auto itg = gate_by_out.find(n);
                        if(itg == gate_by_out.end())
                        {
                            if(!leaf_seen.contains(n))
                            {
                                if(leaves.size() >= max_vars) { ok = false; return; }
                                leaf_seen.emplace(n, true);
                                leaves.push_back(n);
                            }
                            return;
                        }

                        if(++gate_count > max_gates) { ok = false; return; }
                        auto const& gg = itg->second;
                        self(self, gg.in0);
                        if(gg.k != kind::not_gate) { self(self, gg.in1); }
                    };

                    dfs(dfs, root);
                    if(!ok || leaves.empty()) { continue; }

                    ::std::sort(leaves.begin(),
                                leaves.end(),
                                [](auto* a, auto* b) noexcept { return reinterpret_cast<std::uintptr_t>(a) < reinterpret_cast<std::uintptr_t>(b); });

                    if(leaves.size() <= 6u || leaves.size() > max_vars) { continue; }
                    auto const var_count = leaves.size();
                    auto const U = static_cast<std::size_t>(1u) << var_count;
                    auto const blocks = static_cast<::std::uint32_t>((U + 63u) / 64u);
                    if(blocks == 0u) { continue; }
                    if(blocks > stride_blocks) { stride_blocks = blocks; }

                    ::std::unordered_map<::phy_engine::model::node_t*, ::std::uint16_t> leaf_index{};
                    leaf_index.reserve(var_count * 2u + 8u);
                    for(::std::uint16_t i{}; i < static_cast<::std::uint16_t>(var_count); ++i) { leaf_index.emplace(leaves[i], i); }

                    cuda_tt_cone_desc cone{};
                    cone.var_count = static_cast<::std::uint8_t>(var_count);
                    cone.gate_count = 0u;

                    ::std::unordered_map<::phy_engine::model::node_t*, ::std::uint16_t> out_index{};
                    out_index.reserve(gate_count * 2u + 8u);
                    bool ok2{true};

                    auto dfs2 = [&](auto&& self, ::phy_engine::model::node_t* n) noexcept -> ::std::uint16_t
                    {
                        if(!ok2 || n == nullptr) { ok2 = false; return 65534u; }
                        if(auto itc = const_idx.find(n); itc != const_idx.end()) { return itc->second; }
                        if(auto itl = leaf_index.find(n); itl != leaf_index.end()) { return itl->second; }
                        if(auto ito = out_index.find(n); ito != out_index.end()) { return ito->second; }

                        auto itg = gate_by_out.find(n);
                        if(itg == gate_by_out.end()) { ok2 = false; return 65534u; }
                        auto const& gg = itg->second;

                        auto const a = self(self, gg.in0);
                        auto const b = (gg.k == kind::not_gate) ? static_cast<::std::uint16_t>(65534u) : self(self, gg.in1);
                        if(!ok2) { return 65534u; }

                        if(cone.gate_count >= 256u) { ok2 = false; return 65534u; }
                        auto const gi = static_cast<::std::uint16_t>(cone.gate_count++);
                        cone.kind[gi] = enc_kind(gg.k);
                        cone.in0[gi] = a;
                        cone.in1[gi] = b;
                        auto const out = static_cast<::std::uint16_t>(16u + gi);
                        out_index.emplace(n, out);
                        return out;
                    };

                    (void)dfs2(dfs2, root);
                    if(!ok2 || cone.gate_count == 0u) { continue; }

                    auto const rem = static_cast<::std::uint32_t>(U & 63u);
                    auto const last_mask = (rem == 0u) ? ~0ull : ((1ull << rem) - 1ull);

                    cand cd{};
                    cd.root = root;
                    cd.g = g;
                    cd.leaves = ::std::move(leaves);
                    cd.cone = cone;
                    cd.blocks = blocks;
                    cd.last_mask = last_mask;
                    cd.ok = true;
                    cands.push_back(::std::move(cd));
                }

                if(cands.empty() || stride_blocks == 0u) { return false; }

                ::std::vector<::std::uint64_t> tt_words{};
                tt_words.resize(cands.size() * static_cast<std::size_t>(stride_blocks));
                ::std::vector<cuda_tt_cone_desc> cones{};
                cones.reserve(cands.size());
                for(auto const& cd: cands) { cones.push_back(cd.cone); }

                bool used_cuda{};
                if(cands.size() >= 16u || stride_blocks >= 64u)
                {
                    used_cuda = cuda_eval_tt_cones(opt.cuda_device_mask, cones.data(), cones.size(), stride_blocks, tt_words.data());
                }
                if(!used_cuda)
                {
                    for(std::size_t i{}; i < cands.size(); ++i)
                    {
                        eval_tt_cone_cpu(cones[i], stride_blocks, tt_words.data() + i * static_cast<std::size_t>(stride_blocks));
                    }
                }

                for(std::size_t i{}; i < cands.size(); ++i)
                {
                    auto const* base = tt_words.data() + i * static_cast<std::size_t>(stride_blocks);
                    cands[i].sig = sig_tt(base, cands[i].blocks);
                }

                auto tt_equal = [&](std::size_t i, std::size_t j) noexcept -> bool
                {
                    auto const bi = cands[i].blocks;
                    if(bi != cands[j].blocks) { return false; }
                    auto const* wi = tt_words.data() + i * static_cast<std::size_t>(stride_blocks);
                    auto const* wj = tt_words.data() + j * static_cast<std::size_t>(stride_blocks);
                    for(::std::uint32_t w{}; w < bi; ++w)
                    {
                        ::std::uint64_t a = wi[w];
                        ::std::uint64_t b = wj[w];
                        if(w + 1u == bi)
                        {
                            a &= cands[i].last_mask;
                            b &= cands[i].last_mask;
                        }
                        if(a != b) { return false; }
                    }
                    return true;
                };

                auto key_of = [&](std::vector<::phy_engine::model::node_t*> const& leaves, ::std::uint64_t sig) noexcept -> ::std::uint64_t
                {
                    auto const lh = hash_leaves(leaves);
                    auto mix = [](std::uint64_t h, std::uint64_t v) noexcept -> std::uint64_t
                    { return (h ^ (v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2))); };
                    std::uint64_t k{};
                    k = mix(k, lh);
                    k = mix(k, sig);
                    return k;
                };

                ::std::unordered_map<::std::uint64_t, ::std::vector<std::size_t>> rep{};
                rep.reserve(cands.size() * 2u + 1u);

                bool changed{};
                for(std::size_t i{}; i < cands.size(); ++i)
                {
                    auto& cd = cands[i];
                    if(!cd.ok) { continue; }

                    auto* root = cd.root;
                    auto const& g = cd.g;

                    // Ensure the gate still exists and matches our snapshot.
                    {
                        auto* mb = ::phy_engine::netlist::get_model(nl, g.pos);
                        if(mb == nullptr || mb->type != ::phy_engine::model::model_type::normal || mb->ptr == nullptr) { continue; }
                    }

                    auto const* base = tt_words.data() + i * static_cast<std::size_t>(stride_blocks);
                    auto const blocks = cd.blocks;

                    bool all0{true};
                    bool all1{true};
                    for(::std::uint32_t w{}; w < blocks; ++w)
                    {
                        ::std::uint64_t v = base[w];
                        if(w + 1u == blocks) { v &= cd.last_mask; }
                        if(v != 0ull) { all0 = false; }
                        if(v != ((w + 1u == blocks) ? cd.last_mask : ~0ull)) { all1 = false; }
                    }

                    if(all0 || all1)
                    {
                        auto* cnode = find_or_make_const_node(nl,
                                                              all0 ? ::phy_engine::model::digital_node_statement_t::false_state
                                                                   : ::phy_engine::model::digital_node_statement_t::true_state);
                        if(cnode != nullptr && !is_protected(root))
                        {
                            if(move_consumers(root, cnode, g.out_pin))
                            {
                                (void)::phy_engine::netlist::delete_model(nl, g.pos);
                                changed = true;
                            }
                        }
                        continue;
                    }

                    if(cd.leaves.size() == 1u)
                    {
                        // identity: var0 => mask 0b10
                        ::std::uint64_t m = base[0] & 0x3ull;
                        if(m == 0b10ull)
                        {
                            auto* leaf = cd.leaves.front();
                            if(leaf != nullptr && !is_protected(root))
                            {
                                if(move_consumers(root, leaf, g.out_pin))
                                {
                                    (void)::phy_engine::netlist::delete_model(nl, g.pos);
                                    changed = true;
                                }
                            }
                            continue;
                        }
                    }

                    auto const kself = key_of(cd.leaves, cd.sig);
                    auto& vec = rep[kself];
                    for(auto const j: vec)
                    {
                        if(cands[j].leaves != cd.leaves) { continue; }
                        if(!tt_equal(i, j)) { continue; }
                        if(is_protected(root)) { goto next_root; }
                        auto* rep_node = cands[j].root;
                        if(rep_node == nullptr || rep_node == root) { goto next_root; }
                        if(move_consumers(root, rep_node, g.out_pin))
                        {
                            (void)::phy_engine::netlist::delete_model(nl, g.pos);
                            changed = true;
                        }
                        goto next_root;
                    }
                    vec.push_back(i);

                next_root:
                    continue;
                }

                return changed;
            }

            // The implementation uses a 64-bit truth-table mask, so vars are capped at 6.
            std::size_t const max_vars = ::std::min<std::size_t>(6u, opt.sweep_max_vars);
            std::size_t const max_gates = (opt.sweep_max_gates == 0u) ? 64u : opt.sweep_max_gates;

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
                ::std::vector<::phy_engine::model::node_t*> leaves{};
                ::std::uint64_t mask{};
            };

            struct key_hash
            {
                std::size_t operator() (key const& k) const noexcept
                {
                    auto const mix = [](std::size_t h, std::size_t v) noexcept -> std::size_t
                    { return (h ^ (v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2))); };
                    std::size_t h{};
                    h = mix(h, static_cast<std::size_t>(k.mask ^ (k.mask >> 32)));
                    for(auto* p: k.leaves) { h = mix(h, reinterpret_cast<std::size_t>(p)); }
                    return h;
                }
            };

            struct key_eq
            {
                bool operator() (key const& a, key const& b) const noexcept { return a.mask == b.mask && a.leaves == b.leaves; }
            };

            ::std::unordered_map<::phy_engine::model::node_t*, bool> protected_map{};
            protected_map.reserve(protected_nodes.size() * 2u + 1u);
            for(auto* n: protected_nodes) { protected_map.emplace(n, true); }
            auto const is_protected = [&](::phy_engine::model::node_t* n) noexcept -> bool { return protected_map.contains(n); };

            auto const fan = build_gate_opt_fanout(nl);

            ::std::unordered_map<::phy_engine::model::node_t*, gate> gate_by_out{};
            gate_by_out.reserve(1 << 14);

            auto classify = [&](::phy_engine::model::model_base const& mb, ::phy_engine::netlist::model_pos pos) noexcept -> ::std::optional<gate>
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
                    else
                    {
                        g.k = kind::nimp_gate;
                    }
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
                    if(!og || og->out == nullptr || og->out_pin == nullptr) { continue; }
                    gate_by_out.emplace(og->out, *og);
                }
            }

            // Constants (unnamed INPUT nodes) should not be treated as variable leaves for truth tables.
            ::std::unordered_map<::phy_engine::model::node_t*, ::std::uint8_t> const_idx{};
            const_idx.reserve(256);
            for(auto const& blk: nl.models)
            {
                for(auto const* m = blk.begin; m != blk.curr; ++m)
                {
                    if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }
                    if(m->name.size() != 0) { continue; }  // named INPUTs are external IO, not constants
                    if(model_name_u8(*m) != u8"INPUT") { continue; }
                    auto pv = m->ptr->generate_pin_view();
                    if(pv.size != 1 || pv.pins[0].nodes == nullptr) { continue; }
                    auto vi = m->ptr->get_attribute(0);
                    if(vi.type != ::phy_engine::model::variant_type::digital) { continue; }
                    if(vi.digital == ::phy_engine::model::digital_node_statement_t::true_state) { const_idx.emplace(pv.pins[0].nodes, 255u); }
                    else if(vi.digital == ::phy_engine::model::digital_node_statement_t::false_state) { const_idx.emplace(pv.pins[0].nodes, 254u); }
                }
            }

            auto move_consumers =
                [&](::phy_engine::model::node_t* from, ::phy_engine::model::node_t* to, ::phy_engine::model::pin const* from_driver_pin) noexcept -> bool
            {
                if(from == nullptr || to == nullptr || from_driver_pin == nullptr) { return false; }
                if(from == to) { return false; }
                if(is_protected(from)) { return false; }
                if(!has_unique_driver_pin(from, from_driver_pin, fan.pin_out)) { return false; }

                ::std::vector<::phy_engine::model::pin*> pins_to_move{};
                pins_to_move.reserve(from->pins.size());
                for(auto* p: from->pins)
                {
                    if(p == from_driver_pin) { continue; }
                    pins_to_move.push_back(p);
                }
                for(auto* p: pins_to_move)
                {
                    from->pins.erase(p);
                    p->nodes = to;
                    to->pins.insert(p);
                }
                return true;
            };

            ::std::unordered_map<key, ::phy_engine::model::node_t*, key_hash, key_eq> rep{};
            rep.reserve(1 << 12);

            ::std::vector<::phy_engine::model::node_t*> roots{};
            roots.reserve(gate_by_out.size());
            for(auto const& kv: gate_by_out) { roots.push_back(kv.first); }

            struct cand
            {
                ::phy_engine::model::node_t* root{};
                gate g{};
                ::std::vector<::phy_engine::model::node_t*> leaves{};
                cuda_u64_cone_desc cone{};
                ::std::uint64_t mask{};
                bool ok{};
            };

            ::std::vector<cand> cands{};
            cands.reserve(roots.size());

            auto enc_kind = [&](kind k) noexcept -> ::std::uint8_t
            {
                switch(k)
                {
                    case kind::not_gate: return 0u;
                    case kind::and_gate: return 1u;
                    case kind::or_gate: return 2u;
                    case kind::xor_gate: return 3u;
                    case kind::xnor_gate: return 4u;
                    case kind::nand_gate: return 5u;
                    case kind::nor_gate: return 6u;
                    case kind::imp_gate: return 7u;
                    case kind::nimp_gate: return 8u;
                    default: return 255u;
                }
            };

            for(auto* root: roots)
            {
                auto it_root = gate_by_out.find(root);
                if(it_root == gate_by_out.end()) { continue; }
                auto const& g = it_root->second;

                // Ensure the gate still exists and matches our snapshot.
                {
                    auto* mb = ::phy_engine::netlist::get_model(nl, g.pos);
                    if(mb == nullptr || mb->type != ::phy_engine::model::model_type::normal || mb->ptr == nullptr) { continue; }
                }

                cand cd{};
                cd.root = root;
                cd.g = g;

                ::std::vector<::phy_engine::model::node_t*> leaves{};
                leaves.reserve(max_vars);
                ::std::unordered_map<::phy_engine::model::node_t*, bool> visited{};
                visited.reserve(max_gates * 2u);
                ::std::unordered_map<::phy_engine::model::node_t*, bool> leaf_seen{};
                leaf_seen.reserve(max_vars * 2u);

                std::size_t gate_count{};
                bool ok{true};
                auto dfs = [&](auto&& self, ::phy_engine::model::node_t* n) noexcept -> void
                {
                    if(!ok || n == nullptr) { return; }
                    if(visited.contains(n)) { return; }
                    visited.emplace(n, true);

                    if(const_idx.contains(n)) { return; }  // treat as constant, not as a variable leaf

                    auto itg = gate_by_out.find(n);
                    if(itg == gate_by_out.end())
                    {
                        if(!leaf_seen.contains(n))
                        {
                            if(leaves.size() >= max_vars)
                            {
                                ok = false;
                                return;
                            }
                            leaf_seen.emplace(n, true);
                            leaves.push_back(n);
                        }
                        return;
                    }

                    if(++gate_count > max_gates)
                    {
                        ok = false;
                        return;
                    }
                    auto const& gg = itg->second;
                    self(self, gg.in0);
                    if(gg.k != kind::not_gate) { self(self, gg.in1); }
                };

                dfs(dfs, root);
                if(!ok) { continue; }

                ::std::sort(leaves.begin(),
                            leaves.end(),
                            [](auto* a, auto* b) noexcept { return reinterpret_cast<std::uintptr_t>(a) < reinterpret_cast<std::uintptr_t>(b); });

                if(leaves.size() > max_vars) { continue; }
                auto const var_count = leaves.size();
                auto const U = static_cast<std::size_t>(1u << var_count);
                if(U > 64u) { continue; }

                ::std::unordered_map<::phy_engine::model::node_t*, ::std::uint8_t> leaf_index{};
                leaf_index.reserve(var_count * 2u + 8u);
                for(std::size_t i{}; i < var_count; ++i) { leaf_index.emplace(leaves[i], static_cast<::std::uint8_t>(i)); }

                cuda_u64_cone_desc cone{};
                cone.var_count = static_cast<::std::uint8_t>(var_count);
                cone.gate_count = 0u;

                ::std::unordered_map<::phy_engine::model::node_t*, ::std::uint8_t> out_index{};
                out_index.reserve(gate_count * 2u + 8u);
                bool ok2{true};

                auto dfs2 = [&](auto&& self, ::phy_engine::model::node_t* n) noexcept -> ::std::uint8_t
                {
                    if(!ok2 || n == nullptr) { ok2 = false; return 254u; }
                    if(auto itc = const_idx.find(n); itc != const_idx.end()) { return itc->second; }
                    if(auto itl = leaf_index.find(n); itl != leaf_index.end()) { return itl->second; }
                    if(auto ito = out_index.find(n); ito != out_index.end()) { return ito->second; }

                    auto itg = gate_by_out.find(n);
                    if(itg == gate_by_out.end()) { ok2 = false; return 254u; }
                    auto const& gg = itg->second;

                    auto const a = self(self, gg.in0);
                    auto const b = (gg.k == kind::not_gate) ? static_cast<::std::uint8_t>(254u) : self(self, gg.in1);
                    if(!ok2) { return 254u; }

                    if(cone.gate_count >= 64u) { ok2 = false; return 254u; }
                    auto const gi = static_cast<::std::uint8_t>(cone.gate_count++);
                    cone.kind[gi] = enc_kind(gg.k);
                    cone.in0[gi] = a;
                    cone.in1[gi] = b;
                    auto const out = static_cast<::std::uint8_t>(6u + gi);
                    out_index.emplace(n, out);
                    return out;
                };

                (void)dfs2(dfs2, root);
                if(!ok2 || cone.gate_count == 0u) { continue; }

                cd.leaves = ::std::move(leaves);
                cd.cone = cone;
                cd.ok = true;
                cands.push_back(::std::move(cd));
            }

            // Evaluate truth-table masks (CUDA best-effort).
            bool used_cuda{};
            if(opt.cuda_enable && cands.size() >= opt.cuda_min_batch)
            {
                ::std::vector<cuda_u64_cone_desc> cones{};
                ::std::vector<std::size_t> map_idx{};
                cones.reserve(cands.size());
                map_idx.reserve(cands.size());
                for(std::size_t i{}; i < cands.size(); ++i)
                {
                    if(!cands[i].ok) { continue; }
                    cones.push_back(cands[i].cone);
                    map_idx.push_back(i);
                }
                ::std::vector<::std::uint64_t> masks{};
                masks.resize(cones.size());
                if(!cones.empty() && cuda_eval_u64_cones(opt.cuda_device_mask, cones.data(), cones.size(), masks.data()))
                {
                    used_cuda = true;
                    for(std::size_t i{}; i < map_idx.size(); ++i) { cands[map_idx[i]].mask = masks[i]; }
                }
            }
            if(!used_cuda)
            {
                for(auto& cd: cands)
                {
                    if(!cd.ok) { continue; }
                    cd.mask = eval_u64_cone_cpu(cd.cone);
                }
            }

            bool changed{};

            for(auto& cd: cands)
            {
                auto* root = cd.root;
                auto const& g = cd.g;

                // Ensure the gate still exists and matches our snapshot.
                {
                    auto* mb = ::phy_engine::netlist::get_model(nl, g.pos);
                    if(mb == nullptr || mb->type != ::phy_engine::model::model_type::normal || mb->ptr == nullptr) { continue; }
                }

                auto const var_count = static_cast<std::size_t>(cd.cone.var_count);
                auto const U = (var_count >= 6u) ? 64u : (static_cast<std::size_t>(1u) << var_count);
                if(U == 0u || U > 64u) { continue; }
                auto const all_mask = (U == 64u) ? ~0ull : ((1ull << U) - 1ull);
                auto const mask = cd.mask & all_mask;
                if(mask == 0ull || mask == all_mask)
                {
                    auto* cnode = find_or_make_const_node(nl,
                                                          mask == 0ull ? ::phy_engine::model::digital_node_statement_t::false_state
                                                                       : ::phy_engine::model::digital_node_statement_t::true_state);
                    if(cnode != nullptr && !is_protected(root))
                    {
                        if(move_consumers(root, cnode, g.out_pin))
                        {
                            (void)::phy_engine::netlist::delete_model(nl, g.pos);
                            changed = true;
                        }
                    }
                    continue;
                }

                if(var_count == 1u && mask == 0b10u)
                {
                    auto* leaf = cd.leaves.front();
                    if(leaf != nullptr && !is_protected(root))
                    {
                        if(move_consumers(root, leaf, g.out_pin))
                        {
                            (void)::phy_engine::netlist::delete_model(nl, g.pos);
                            changed = true;
                        }
                    }
                    continue;
                }

                key k{cd.leaves, mask};
                auto it = rep.find(k);
                if(it == rep.end())
                {
                    rep.emplace(::std::move(k), root);
                    continue;
                }

                if(is_protected(root)) { continue; }
                auto* rep_node = it->second;
                if(rep_node == nullptr || rep_node == root) { continue; }

                if(move_consumers(root, rep_node, g.out_pin))
                {
                    (void)::phy_engine::netlist::delete_model(nl, g.pos);
                    changed = true;
                }
            }

            return changed;
        }

        [[nodiscard]] inline bool optimize_dce_in_pe_netlist(::phy_engine::netlist::netlist& nl,
                                                             ::std::vector<::phy_engine::model::node_t*> const& protected_nodes) noexcept
        {
            ::std::unordered_map<::phy_engine::model::node_t*, bool> protected_map{};
            protected_map.reserve(protected_nodes.size() * 2u + 1u);
            for(auto* n: protected_nodes) { protected_map.emplace(n, true); }
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
                        if(is_protected(n))
                        {
                            all_unused = false;
                            break;
                        }
                        auto it = fan.consumer_count.find(n);
                        if(it != fan.consumer_count.end() && it->second != 0)
                        {
                            all_unused = false;
                            break;
                        }
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
        { return static_cast<std::size_t>(__builtin_popcount(static_cast<unsigned>(x))); }

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
            qm_prime_implicants(::std::vector<::std::uint16_t> const& on, ::std::vector<::std::uint16_t> const& dc, std::size_t var_count) noexcept
        {
            ::std::vector<qm_implicant> current{};
            current.reserve(on.size() + dc.size());
            for(auto const m: on) { current.push_back(qm_implicant{m, 0u, false}); }
            for(auto const m: dc) { current.push_back(qm_implicant{m, 0u, false}); }

            ::std::vector<qm_implicant> primes{};
            primes.reserve(256);

            auto const var_mask = static_cast<::std::uint16_t>((var_count >= 16) ? 0xFFFFu : ((1u << var_count) - 1u));

            for(;;)
            {
                for(auto& x: current) { x.combined = false; }

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
                        for(auto const& e: next)
                        {
                            if(e.value == c.value && e.mask == c.mask)
                            {
                                dup = true;
                                break;
                            }
                        }
                        if(!dup) { next.push_back(c); }
                    }
                }

                for(auto const& a: current)
                {
                    if(a.combined) { continue; }
                    bool dup{};
                    for(auto const& p: primes)
                    {
                        if(p.value == a.value && p.mask == a.mask)
                        {
                            dup = true;
                            break;
                        }
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

        [[nodiscard]] inline std::size_t
            two_level_cover_cost(::std::vector<qm_implicant> const& cover, std::size_t var_count, pe_synth_options const& opt) noexcept
        {
            if(opt.two_level_cost == pe_synth_options::two_level_cost_model::literal_count)
            {
                std::size_t lits{};
                for(auto const& imp: cover) { lits += implicant_literals(imp, var_count); }
                return lits;
            }

            // Gate-count model (2-input gates only): NOTs for negated literals (shared per cone) + AND trees + OR tree.
            ::std::vector<bool> neg_used{};
            neg_used.assign(var_count, false);

            std::size_t terms{};
            std::size_t and_cost{};

            for(auto const& imp: cover)
            {
                std::size_t cube_lits{};
                for(std::size_t v{}; v < var_count; ++v)
                {
                    if((imp.mask >> v) & 1u) { continue; }
                    ++cube_lits;
                    bool const bit_is_1 = ((imp.value >> v) & 1u) != 0;
                    if(!bit_is_1) { neg_used[v] = true; }
                }
                ++terms;
                if(cube_lits >= 2) { and_cost += (cube_lits - 1u); }
            }

            std::size_t not_cost{};
            for(std::size_t v{}; v < var_count; ++v)
            {
                if(neg_used[v]) { ++not_cost; }
            }

            std::size_t or_cost{};
            if(terms >= 2) { or_cost = terms - 1u; }

            return static_cast<std::size_t>(opt.two_level_weights.not_w) * not_cost + static_cast<std::size_t>(opt.two_level_weights.and_w) * and_cost +
                   static_cast<std::size_t>(opt.two_level_weights.or_w) * or_cost;
        }

        [[nodiscard]] inline std::size_t qm_cover_cost(::std::vector<qm_implicant> const& primes,
                                                       ::std::vector<std::size_t> const& pick,
                                                       std::size_t var_count,
                                                       pe_synth_options const& opt) noexcept
        {
            ::std::vector<qm_implicant> cover{};
            cover.reserve(pick.size());
            for(auto const idx: pick)
            {
                if(idx >= primes.size()) { continue; }
                cover.push_back(primes[idx]);
            }
            return two_level_cover_cost(cover, var_count, opt);
        }

        [[nodiscard]] inline bool two_level_cover_covers_all_on(::std::vector<qm_implicant> const& cover, ::std::vector<::std::uint16_t> const& on) noexcept
        {
            for(auto const m: on)
            {
                bool hit{};
                for(auto const& c: cover)
                {
                    if(implicant_covers(c, m))
                    {
                        hit = true;
                        break;
                    }
                }
                if(!hit) { return false; }
            }
            return true;
        }

        [[nodiscard]] inline bool
            two_level_cover_hits_off(::std::vector<qm_implicant> const& cover, ::std::vector<bool> const& is_on, ::std::vector<bool> const& is_dc) noexcept
        {
            // Universe is implicit: [0, is_on.size()).
            auto const U = is_on.size();
            for(std::size_t m{}; m < U; ++m)
            {
                if(is_on[m] || is_dc[m]) { continue; }
                auto const mm = static_cast<::std::uint16_t>(m);
                for(auto const& c: cover)
                {
                    if(implicant_covers(c, mm)) { return true; }
                }
            }
            return false;
        }

        struct espresso_solution
        {
            ::std::vector<qm_implicant> cover{};
            std::size_t cost{static_cast<std::size_t>(-1)};
            bool complemented{};
        };

        struct multi_output_solution
        {
            // covers[o] is the SOP cover for output o (same var order for the whole group).
            ::std::vector<::std::vector<qm_implicant>> covers{};
            std::size_t cost{static_cast<std::size_t>(-1)};
        };

        struct cube_key
        {
            ::std::uint16_t mask{};
            ::std::uint16_t value{};
        };

        struct cube_key_hash
        {
            std::size_t operator() (cube_key const& k) const noexcept { return (static_cast<std::size_t>(k.mask) << 16) ^ static_cast<std::size_t>(k.value); }
        };

        struct cube_key_eq
        {
            bool operator() (cube_key const& a, cube_key const& b) const noexcept { return a.mask == b.mask && a.value == b.value; }
        };

        [[nodiscard]] inline cube_key to_cube_key(qm_implicant const& imp) noexcept
        {
            auto const keep = static_cast<::std::uint16_t>(~imp.mask);
            return cube_key{imp.mask, static_cast<::std::uint16_t>(imp.value & keep)};
        }

        [[nodiscard]] inline std::size_t cube_literals(qm_implicant const& imp, std::size_t var_count) noexcept { return implicant_literals(imp, var_count); }

        [[nodiscard]] inline std::size_t
            multi_output_gate_cost(::std::vector<::std::vector<qm_implicant>> const& covers, std::size_t var_count, pe_synth_options const& opt) noexcept
        {
            if(opt.two_level_cost == pe_synth_options::two_level_cost_model::literal_count)
            {
                // PLA-style literal count with shared product terms: count unique cube literals once.
                ::std::unordered_map<cube_key, bool, cube_key_hash, cube_key_eq> seen{};
                seen.reserve(256);
                std::size_t lit{};
                for(auto const& cv: covers)
                {
                    for(auto const& imp: cv)
                    {
                        auto const k = to_cube_key(imp);
                        if(seen.emplace(k, true).second) { lit += cube_literals(imp, var_count); }
                    }
                }
                return lit;
            }

            // Gate-count with shared NOTs and shared product terms across outputs.
            // - NOT cost: any var used in complemented form by any selected cube
            // - AND cost: sum over unique cubes of (lits-1) for lits>=2
            // - OR cost: per output (terms-1) for terms>=2
            ::std::uint16_t neg_used_mask{};
            ::std::unordered_map<cube_key, qm_implicant, cube_key_hash, cube_key_eq> uniq{};
            uniq.reserve(256);

            for(auto const& cv: covers)
            {
                for(auto const& imp: cv) { (void)uniq.emplace(to_cube_key(imp), imp); }
            }

            std::size_t and_cost{};
            for(auto const& kv: uniq)
            {
                auto const& imp = kv.second;
                auto const lits = cube_literals(imp, var_count);
                if(lits >= 2) { and_cost += (lits - 1u); }
                for(std::size_t v{}; v < var_count; ++v)
                {
                    if((imp.mask >> v) & 1u) { continue; }
                    bool const bit_is_1 = ((imp.value >> v) & 1u) != 0;
                    if(!bit_is_1) { neg_used_mask = static_cast<::std::uint16_t>(neg_used_mask | static_cast<::std::uint16_t>(1u << v)); }
                }
            }

            std::size_t not_cost = static_cast<std::size_t>(__builtin_popcount(static_cast<unsigned>(neg_used_mask)));
            std::size_t or_cost{};
            for(auto const& cv: covers)
            {
                if(cv.size() >= 2u) { or_cost += (cv.size() - 1u); }
            }

            return static_cast<std::size_t>(opt.two_level_weights.not_w) * not_cost + static_cast<std::size_t>(opt.two_level_weights.and_w) * and_cost +
                   static_cast<std::size_t>(opt.two_level_weights.or_w) * or_cost;
        }

        [[nodiscard]] inline multi_output_solution multi_output_two_level_minimize(::std::vector<::std::vector<::std::uint16_t>> const& on_list,
                                                                                   ::std::vector<::std::vector<::std::uint16_t>> const& dc_list,
                                                                                   std::size_t var_count,
                                                                                   pe_synth_options const& opt) noexcept
        {
            // Multi-output cover selection that can trade per-output optimality for shared product terms.
            // Bounded, deterministic greedy heuristic over the union of prime implicants.
            multi_output_solution sol{};
            if(on_list.empty() || on_list.size() != dc_list.size()) { return sol; }
            if(var_count == 0) { return sol; }
            if(var_count > 16) { return sol; }

            auto const outputs = on_list.size();
            sol.covers.resize(outputs);

            // Quick constant handling.
            for(std::size_t o{}; o < outputs; ++o)
            {
                if(on_list[o].empty()) { sol.covers[o].clear(); }
            }

            // Build prime implicants per output.
            ::std::vector<::std::vector<qm_implicant>> primes_by_out{};
            primes_by_out.resize(outputs);
            for(std::size_t o{}; o < outputs; ++o)
            {
                primes_by_out[o] = qm_prime_implicants(on_list[o], dc_list[o], var_count);
                if(primes_by_out[o].empty() && !on_list[o].empty()) { return sol; }
            }

            // Union of cubes across all outputs.
            ::std::unordered_map<cube_key, std::size_t, cube_key_hash, cube_key_eq> cube_to_idx{};
            cube_to_idx.reserve(512);
            ::std::vector<qm_implicant> cubes{};
            cubes.reserve(512);

            for(std::size_t o{}; o < outputs; ++o)
            {
                for(auto const& imp: primes_by_out[o])
                {
                    auto const k = to_cube_key(imp);
                    if(auto it = cube_to_idx.find(k); it == cube_to_idx.end())
                    {
                        cube_to_idx.emplace(k, cubes.size());
                        cubes.push_back(imp);
                    }
                }
            }
            if(cubes.empty()) { return sol; }

            auto const full_U = static_cast<std::size_t>(1u << var_count);
            ::std::vector<::std::vector<bool>> is_on_full{};
            ::std::vector<::std::vector<bool>> is_dc_full{};
            is_on_full.resize(outputs);
            is_dc_full.resize(outputs);
            for(std::size_t o{}; o < outputs; ++o)
            {
                is_on_full[o].assign(full_U, false);
                is_dc_full[o].assign(full_U, false);
                for(auto const m: on_list[o])
                {
                    if(static_cast<std::size_t>(m) < full_U) { is_on_full[o][static_cast<std::size_t>(m)] = true; }
                }
                for(auto const m: dc_list[o])
                {
                    if(static_cast<std::size_t>(m) < full_U) { is_dc_full[o][static_cast<std::size_t>(m)] = true; }
                }
            }

            // Coverage bitsets per output (over indices in on_list[o]).
            ::std::vector<std::size_t> blocks_by_out{};
            blocks_by_out.resize(outputs);
            ::std::vector<::std::vector<::std::uint64_t>> cov{};
            cov.resize(outputs);
            for(std::size_t o{}; o < outputs; ++o)
            {
                auto const blocks = (on_list[o].size() + 63u) / 64u;
                blocks_by_out[o] = blocks;
                cov[o].assign(cubes.size() * blocks, 0ull);
                auto cov_row = [&](std::size_t ci) noexcept -> ::std::uint64_t*
                {
                    return cov[o].data() + ci * blocks;
                };
                for(std::size_t ci{}; ci < cubes.size(); ++ci)
                {
                    // A cube can only be used for output `o` if it does not cover any OFF minterm for that output.
                    bool valid{true};
                    for(std::size_t m{}; m < full_U; ++m)
                    {
                        if(is_on_full[o][m] || is_dc_full[o][m]) { continue; }
                        if(implicant_covers(cubes[ci], static_cast<::std::uint16_t>(m)))
                        {
                            valid = false;
                            break;
                        }
                    }
                    if(!valid) { continue; }
                    for(std::size_t mi{}; mi < on_list[o].size(); ++mi)
                    {
                        if(implicant_covers(cubes[ci], on_list[o][mi])) { cov_row(ci)[mi / 64u] |= (1ull << (mi % 64u)); }
                    }
                }
            }

            auto popcount64 = [](std::uint64_t x) noexcept -> std::size_t { return static_cast<std::size_t>(__builtin_popcountll(x)); };

            // Uncovered sets per output.
            ::std::vector<::std::vector<std::uint64_t>> uncovered{};
            uncovered.resize(outputs);
            for(std::size_t o{}; o < outputs; ++o)
            {
                auto const blocks = (on_list[o].size() + 63u) / 64u;
                uncovered[o].assign(blocks, ~0ull);
                if(on_list[o].size() % 64u)
                {
                    auto const rem = on_list[o].size() % 64u;
                    uncovered[o].back() = (rem == 64u) ? ~0ull : ((1ull << rem) - 1ull);
                }
            }

            ::std::vector<::std::vector<std::size_t>> use_idx{};
            use_idx.resize(outputs);
            ::std::vector<bool> selected{};
            selected.assign(cubes.size(), false);
            ::std::vector<std::size_t> terms_count{};
            terms_count.assign(outputs, 0u);

            ::std::uint16_t neg_mask_used{};
            auto cube_neg_mask = [&](qm_implicant const& imp) noexcept -> std::uint16_t
            {
                std::uint16_t m{};
                for(std::size_t v{}; v < var_count; ++v)
                {
                    if((imp.mask >> v) & 1u) { continue; }
                    bool const bit_is_1 = ((imp.value >> v) & 1u) != 0;
                    if(!bit_is_1) { m = static_cast<std::uint16_t>(m | static_cast<std::uint16_t>(1u << v)); }
                }
                return m;
            };

            auto any_uncovered = [&]() noexcept -> bool
            {
                for(std::size_t o{}; o < outputs; ++o)
                {
                    for(auto const w: uncovered[o])
                    {
                        if(w) { return true; }
                    }
                }
                return false;
            };

            // Optional CUDA acceleration for multi-output masked popcount scoring (best-effort).
            ::std::vector<cuda_bitset_matrix_raii> cov_gpu{};
            ::std::vector<::std::vector<::std::uint32_t>> gains_gpu{};
            ::std::vector<bool> use_cuda_gains{};
            cov_gpu.resize(outputs);
            gains_gpu.resize(outputs);
            use_cuda_gains.assign(outputs, false);
            if(opt.cuda_enable && !cubes.empty())
            {
                for(std::size_t o{}; o < outputs; ++o)
                {
                    auto const blocks = blocks_by_out[o];
                    if(blocks == 0u) { continue; }
                    auto const workload = cubes.size() * blocks;
                    if(workload < opt.cuda_min_batch) { continue; }
                    cov_gpu[o] = cuda_bitset_matrix_create(opt.cuda_device_mask, cov[o].data(), cubes.size(), static_cast<::std::uint32_t>(blocks));
                    if(cov_gpu[o].handle != nullptr)
                    {
                        gains_gpu[o].resize(cubes.size());
                        use_cuda_gains[o] = true;
                    }
                }
            }

            while(any_uncovered())
            {
                for(std::size_t o{}; o < outputs; ++o)
                {
                    if(!use_cuda_gains[o]) { continue; }
                    auto const blocks = blocks_by_out[o];
                    if(!cuda_bitset_matrix_row_and_popcount(cov_gpu[o],
                                                           uncovered[o].data(),
                                                           static_cast<::std::uint32_t>(blocks),
                                                           gains_gpu[o].data()))
                    {
                        use_cuda_gains[o] = false;
                        gains_gpu[o].clear();
                    }
                }

                std::size_t best_ci{static_cast<std::size_t>(-1)};
                std::size_t best_gain{};
                std::int64_t best_score{::std::numeric_limits<std::int64_t>::min()};
                std::size_t best_cost_incr{static_cast<std::size_t>(-1)};

                for(std::size_t ci{}; ci < cubes.size(); ++ci)
                {
                    if(selected[ci]) { continue; }
                    std::size_t gain{};
                    ::std::vector<bool> hits{};
                    hits.assign(outputs, false);
                    for(std::size_t o{}; o < outputs; ++o)
                    {
                        std::size_t og{};
                        auto const blocks = blocks_by_out[o];
                        if(use_cuda_gains[o])
                        {
                            og = static_cast<std::size_t>(gains_gpu[o][ci]);
                        }
                        else
                        {
                            auto const* row = cov[o].data() + ci * blocks;
                            for(std::size_t b{}; b < blocks; ++b) { og += popcount64(row[b] & uncovered[o][b]); }
                        }
                        if(og)
                        {
                            hits[o] = true;
                            gain += og;
                        }
                    }
                    if(gain == 0) { continue; }

                    auto const lits = cube_literals(cubes[ci], var_count);
                    std::size_t and_incr{};
                    if(lits >= 2) { and_incr = (lits - 1u); }

                    auto const nmask = cube_neg_mask(cubes[ci]);
                    auto const new_neg = static_cast<std::uint16_t>(nmask & static_cast<std::uint16_t>(~neg_mask_used));
                    auto const not_incr = static_cast<std::size_t>(__builtin_popcount(static_cast<unsigned>(new_neg)));

                    std::size_t or_incr{};
                    for(std::size_t o{}; o < outputs; ++o)
                    {
                        if(!hits[o]) { continue; }
                        if(terms_count[o] >= 1u) { ++or_incr; }
                    }

                    auto const cost_incr = static_cast<std::size_t>(opt.two_level_weights.not_w) * not_incr +
                                           static_cast<std::size_t>(opt.two_level_weights.and_w) * and_incr +
                                           static_cast<std::size_t>(opt.two_level_weights.or_w) * or_incr;

                    auto const score = static_cast<std::int64_t>(gain) * 64 - static_cast<std::int64_t>(cost_incr);
                    if(score > best_score || (score == best_score && (gain > best_gain || (gain == best_gain && cost_incr < best_cost_incr))))
                    {
                        best_score = score;
                        best_ci = ci;
                        best_gain = gain;
                        best_cost_incr = cost_incr;
                    }
                }

                if(best_ci == static_cast<std::size_t>(-1)) { return sol; }
                selected[best_ci] = true;
                neg_mask_used = static_cast<std::uint16_t>(neg_mask_used | cube_neg_mask(cubes[best_ci]));

                for(std::size_t o{}; o < outputs; ++o)
                {
                    bool contributed{};
                    auto const blocks = blocks_by_out[o];
                    auto const* row = cov[o].data() + best_ci * blocks;
                    for(std::size_t b{}; b < blocks; ++b)
                    {
                        auto const hit = row[b] & uncovered[o][b];
                        if(hit) { contributed = true; }
                        uncovered[o][b] &= ~row[b];
                    }
                    if(contributed)
                    {
                        use_idx[o].push_back(best_ci);
                        ++terms_count[o];
                    }
                }
            }

            // Build covers per output and prune redundancies (per-output irredundant).
            for(std::size_t o{}; o < outputs; ++o)
            {
                auto const& on = on_list[o];
                if(on.empty())
                {
                    sol.covers[o].clear();
                    continue;
                }
                auto& picks = use_idx[o];
                if(picks.empty()) { return sol; }

                // Count coverage per ON minterm.
                auto const blocks = blocks_by_out[o];
                auto cov_row_o = [&](std::size_t ci) noexcept -> ::std::uint64_t const*
                {
                    return cov[o].data() + ci * blocks;
                };

                ::std::vector<::std::uint16_t> cnt{};
                cnt.assign(on.size(), 0u);
                for(auto const ci: picks)
                {
                    auto const* row = cov_row_o(ci);
                    for(std::size_t b{}; b < blocks; ++b)
                    {
                        auto x = row[b];
                        while(x)
                        {
                            auto const bit = static_cast<unsigned>(__builtin_ctzll(x));
                            x &= (x - 1ull);
                            auto const idx = b * 64u + static_cast<std::size_t>(bit);
                            if(idx >= on.size()) { continue; }
                            auto& v = cnt[idx];
                            if(v != ::std::numeric_limits<::std::uint16_t>::max()) { ++v; }
                        }
                    }
                }

                for(std::size_t i{}; i < picks.size();)
                {
                    auto const ci = picks[i];
                    bool redundant{true};
                    bool covers_any{};
                    {
                        auto const* row = cov_row_o(ci);
                        for(std::size_t b{}; b < blocks && redundant; ++b)
                        {
                            auto x = row[b];
                            while(x)
                            {
                                auto const bit = static_cast<unsigned>(__builtin_ctzll(x));
                                x &= (x - 1ull);
                                auto const idx = b * 64u + static_cast<std::size_t>(bit);
                                if(idx >= on.size()) { continue; }
                                covers_any = true;
                                if(cnt[idx] <= 1u)
                                {
                                    redundant = false;
                                    break;
                                }
                            }
                        }
                    }
                    if(!covers_any) { redundant = true; }
                    if(!redundant)
                    {
                        ++i;
                        continue;
                    }
                    {
                        auto const* row = cov_row_o(ci);
                        for(std::size_t b{}; b < blocks; ++b)
                        {
                            auto x = row[b];
                            while(x)
                            {
                                auto const bit = static_cast<unsigned>(__builtin_ctzll(x));
                                x &= (x - 1ull);
                                auto const idx = b * 64u + static_cast<std::size_t>(bit);
                                if(idx >= on.size()) { continue; }
                                auto& v = cnt[idx];
                                if(v) { --v; }
                            }
                        }
                    }
                    picks[i] = picks.back();
                    picks.pop_back();
                }

                sol.covers[o].reserve(picks.size());
                for(auto const ci: picks) { sol.covers[o].push_back(cubes[ci]); }
            }

            // Validate (best-effort): cover each ON and do not hit OFF (implicit, since cubes come from primes).
            for(std::size_t o{}; o < outputs; ++o)
            {
                if(!two_level_cover_covers_all_on(sol.covers[o], on_list[o])) { return multi_output_solution{}; }
            }

            sol.cost = multi_output_gate_cost(sol.covers, var_count, opt);
            return sol;
        }

        [[nodiscard]] inline ::std::vector<std::size_t>
            espresso_binate_var_order(::std::vector<::std::uint16_t> const& on, ::std::vector<::std::uint16_t> const& dc, std::size_t var_count) noexcept
        {
            ::std::vector<std::size_t> order{};
            order.reserve(var_count);
            if(var_count == 0) { return order; }

            struct var_stat
            {
                std::size_t v{};
                std::size_t c0{};
                std::size_t c1{};
                bool binate{};
            };

            ::std::vector<var_stat> stats{};
            stats.reserve(var_count);

            for(std::size_t v{}; v < var_count; ++v)
            {
                std::size_t c0{};
                std::size_t c1{};
                for(auto const m: on)
                {
                    if((m >> v) & 1u) { ++c1; }
                    else
                    {
                        ++c0;
                    }
                }
                for(auto const m: dc)
                {
                    if((m >> v) & 1u) { ++c1; }
                    else
                    {
                        ++c0;
                    }
                }
                bool const binate = (c0 != 0u && c1 != 0u);
                stats.push_back(var_stat{v, c0, c1, binate});
            }

            ::std::sort(stats.begin(),
                        stats.end(),
                        [](var_stat const& a, var_stat const& b) noexcept
                        {
                            if(a.binate != b.binate) { return a.binate > b.binate; }
                            auto const amin = (a.c0 < a.c1) ? a.c0 : a.c1;
                            auto const bmin = (b.c0 < b.c1) ? b.c0 : b.c1;
                            if(a.binate)
                            {
                                if(amin != bmin) { return amin > bmin; }
                            }
                            auto const atot = a.c0 + a.c1;
                            auto const btot = b.c0 + b.c1;
                            if(atot != btot) { return atot > btot; }
                            return a.v < b.v;
                        });

            for(auto const& s: stats) { order.push_back(s.v); }
            return order;
        }

        [[nodiscard]] inline espresso_solution espresso_two_level_minimize_base(::std::vector<::std::uint16_t> const& on,
                                                                                ::std::vector<::std::uint16_t> const& dc,
                                                                                std::size_t var_count,
                                                                                pe_synth_options const& opt) noexcept
        {
            // A small, deterministic Espresso-style loop (bounded truth-table cones):
            // - Start from minterm cover (ON-set)
            // - Iterate: EXPAND (w.r.t. OFF-set) → IRREDUNDANT → REDUCE (w.r.t. essential region) → IRREDUNDANT
            // - Optionally run a small "last gasp" perturbation (deterministic) to escape local minima
            //
            // Notes:
            // - This is not a full industrial Espresso implementation, but it captures the key passes in a bounded setting.
            // - Uses binate variable ordering to guide expansion/reduction.
            // - The cost function is configurable via `pe_synth_options::{two_level_cost,two_level_weights}`.

            espresso_solution sol{};
            if(var_count == 0)
            {
                sol.cost = on.empty() ? 0u : 0u;
                if(!on.empty()) { sol.cover.push_back(qm_implicant{0u, 0u, false}); }
                return sol;
            }
            if(var_count > 16) { return sol; }

            auto const U = static_cast<std::size_t>(1u << var_count);
            auto var_order = espresso_binate_var_order(on, dc, var_count);
            if(var_order.size() != var_count)
            {
                var_order.clear();
                for(std::size_t v{}; v < var_count; ++v) { var_order.push_back(v); }
            }

            ::std::vector<bool> is_on{};
            ::std::vector<bool> is_dc{};
            is_on.assign(U, false);
            is_dc.assign(U, false);
            for(auto const m: on)
            {
                if(static_cast<std::size_t>(m) < U) { is_on[static_cast<std::size_t>(m)] = true; }
            }
            for(auto const m: dc)
            {
                if(static_cast<std::size_t>(m) < U) { is_dc[static_cast<std::size_t>(m)] = true; }
            }

            // Initial cover: one cube per ON minterm.
            ::std::vector<qm_implicant> cover{};
            cover.reserve(on.size());
            for(auto const m: on) { cover.push_back(qm_implicant{m, 0u, false}); }

            // Precompute OFF-set as a u64 bitset over the full universe to make hits-OFF checks fast and batchable.
            auto const blocksU = (U + 63u) / 64u;
            ::std::vector<::std::uint64_t> off_bits{};
            off_bits.assign(blocksU, 0ull);
            for(std::size_t m{}; m < U; ++m)
            {
                if(is_on[m] || is_dc[m]) { continue; }
                off_bits[m / 64u] |= (1ull << (m % 64u));
            }
            if(U % 64u)
            {
                auto const rem = U % 64u;
                auto const mask = (rem == 64u) ? ~0ull : ((1ull << rem) - 1ull);
                off_bits.back() &= mask;
            }

            ::std::vector<::std::uint64_t> on_bits{};
            on_bits.assign(blocksU, 0ull);
            for(auto const m: on)
            {
                auto const mi = static_cast<std::size_t>(m);
                if(mi >= U) { continue; }
                on_bits[mi / 64u] |= (1ull << (mi % 64u));
            }

            auto cube_cover_word = [&](qm_implicant const& c, std::uint32_t word_idx) noexcept -> std::uint64_t
            {
                // Compute the cube's coverage mask for this 64-minterm word.
                std::uint64_t wmask = ~0ull;
                if(word_idx + 1u == blocksU && (U % 64u))
                {
                    auto const rem = static_cast<unsigned>(U % 64u);
                    wmask = (rem == 64u) ? ~0ull : ((1ull << rem) - 1ull);
                }

                auto var_word16 = [&](unsigned v) noexcept -> std::uint64_t
                {
                    // v0 toggles every 1 minterm, v1 every 2, ... (same encoding as CUDA helper).
                    // For v>=6, the word index encodes the higher minterm bits.
                    if(v < 6u)
                    {
                        constexpr std::uint64_t leaf_pat[6] = {
                            0xAAAAAAAAAAAAAAAAull,
                            0xCCCCCCCCCCCCCCCCull,
                            0xF0F0F0F0F0F0F0F0ull,
                            0xFF00FF00FF00FF00ull,
                            0xFFFF0000FFFF0000ull,
                            0xFFFFFFFF00000000ull,
                        };
                        return leaf_pat[v];
                    }
                    auto const wi = static_cast<unsigned>(word_idx);
                    return (((wi >> (v - 6u)) & 1u) != 0u) ? ~0ull : 0ull;
                };

                std::uint64_t r = wmask;
                for(unsigned v{}; v < static_cast<unsigned>(var_count); ++v)
                {
                    if(((c.mask >> v) & 1u) != 0u) { continue; }  // don't care
                    auto const pat = var_word16(v);
                    bool const bit_is_1 = ((c.value >> v) & 1u) != 0u;
                    r &= bit_is_1 ? pat : ~pat;
                    if(r == 0ull) { break; }
                }
                return r & wmask;
            };

            auto cube_hits_off_fast = [&](qm_implicant const& c) noexcept -> bool
            {
                for(std::uint32_t w{}; w < static_cast<std::uint32_t>(blocksU); ++w)
                {
                    auto const covw = cube_cover_word(c, w);
                    if((covw & off_bits[w]) != 0ull) { return true; }
                }
                return false;
            };

            auto cube_hits_off_batch = [&](::std::vector<qm_implicant> const& cubes, ::std::vector<std::uint8_t>& out_hits) noexcept -> void
            {
                out_hits.assign(cubes.size(), 0u);
                if(cubes.empty() || blocksU == 0u) { return; }

                bool used_cuda{};
                if(opt.cuda_enable)
                {
                    auto const workload = cubes.size() * blocksU;
                    if(workload >= opt.cuda_min_batch && var_count <= 16u)
                    {
                        ::std::vector<cuda_cube_desc> desc{};
                        desc.resize(cubes.size());
                        for(std::size_t i{}; i < cubes.size(); ++i)
                        {
                            desc[i].value = cubes[i].value;
                            desc[i].mask = cubes[i].mask;
                        }
                        used_cuda = cuda_espresso_cube_hits_off(opt.cuda_device_mask,
                                                               desc.data(),
                                                               desc.size(),
                                                               static_cast<std::uint32_t>(var_count),
                                                               off_bits.data(),
                                                               static_cast<std::uint32_t>(blocksU),
                                                               out_hits.data());
                    }
                }

                if(used_cuda) { return; }
                for(std::size_t i{}; i < cubes.size(); ++i) { out_hits[i] = cube_hits_off_fast(cubes[i]) ? 1u : 0u; }
            };

            auto cube_hits_off = [&](qm_implicant const& c) noexcept -> bool
            {
                return cube_hits_off_fast(c);
            };

            [[maybe_unused]] auto expand_one = [&](qm_implicant const& c0) noexcept -> qm_implicant
            {
                auto c = c0;
                bool changed{true};
                while(changed)
                {
                    changed = false;
                    for(auto const v: var_order)
                    {
                        if((c.mask >> v) & 1u) { continue; }
                        qm_implicant cand = c;
                        cand.mask = static_cast<::std::uint16_t>(cand.mask | static_cast<::std::uint16_t>(1u << v));
                        cand.value = static_cast<::std::uint16_t>(cand.value & static_cast<::std::uint16_t>(~(1u << v)));
                        if(!cube_hits_off(cand))
                        {
                            c = cand;
                            changed = true;
                        }
                    }
                }
                return c;
            };

            auto expand_cover_batch = [&]() noexcept -> bool
            {
                if(cover.empty() || var_order.empty()) { return false; }
                bool any{};
                bool changed{true};
                ::std::vector<std::size_t> idx{};
                ::std::vector<qm_implicant> cands{};
                ::std::vector<std::uint8_t> hits{};

                while(changed)
                {
                    changed = false;
                    for(auto const v: var_order)
                    {
                        idx.clear();
                        cands.clear();
                        idx.reserve(cover.size() / 2u + 1u);
                        cands.reserve(cover.size() / 2u + 1u);

                        auto const bit = static_cast<::std::uint16_t>(1u << v);
                        for(std::size_t i{}; i < cover.size(); ++i)
                        {
                            auto const& c = cover[i];
                            if((c.mask >> v) & 1u) { continue; }  // already don't care
                            qm_implicant cand = c;
                            cand.mask = static_cast<::std::uint16_t>(cand.mask | bit);
                            cand.value = static_cast<::std::uint16_t>(cand.value & static_cast<::std::uint16_t>(~bit));
                            idx.push_back(i);
                            cands.push_back(cand);
                        }
                        if(cands.empty()) { continue; }

                        cube_hits_off_batch(cands, hits);
                        if(hits.size() != cands.size()) { continue; }

                        for(std::size_t k{}; k < cands.size(); ++k)
                        {
                            if(hits[k]) { continue; }
                            cover[idx[k]] = cands[k];
                            changed = true;
                            any = true;
                        }
                    }
                }
                return any;
            };

            auto irredundant = [&]() noexcept -> bool
            {
                if(cover.empty()) { return false; }
                ::std::vector<::std::uint16_t> cnt{};
                cnt.assign(U, 0u);

                for(auto const& c: cover)
                {
                    for(auto const m: on)
                    {
                        if(implicant_covers(c, m))
                        {
                            auto& x = cnt[static_cast<std::size_t>(m)];
                            if(x != ::std::numeric_limits<::std::uint16_t>::max()) { ++x; }
                        }
                    }
                }

                bool changed{};
                for(std::size_t i{}; i < cover.size();)
                {
                    auto const c = cover[i];
                    bool redundant{true};
                    bool covers_any{};
                    for(auto const m: on)
                    {
                        if(!implicant_covers(c, m)) { continue; }
                        covers_any = true;
                        if(cnt[static_cast<std::size_t>(m)] <= 1u)
                        {
                            redundant = false;
                            break;
                        }
                    }

                    if(!covers_any) { redundant = true; }

                    if(!redundant)
                    {
                        ++i;
                        continue;
                    }

                    // Remove cube and update counts.
                    for(auto const m: on)
                    {
                        if(!implicant_covers(c, m)) { continue; }
                        auto& x = cnt[static_cast<std::size_t>(m)];
                        if(x) { --x; }
                    }
                    cover[i] = cover.back();
                    cover.pop_back();
                    changed = true;
                }
                return changed;
            };

            auto dedupe = [&]() noexcept
            {
                ::std::sort(cover.begin(),
                            cover.end(),
                            [](qm_implicant const& a, qm_implicant const& b) noexcept
                            {
                                if(a.mask != b.mask) { return a.mask < b.mask; }
                                return a.value < b.value;
                            });
                cover.erase(::std::unique(cover.begin(),
                                          cover.end(),
                                          [](qm_implicant const& a, qm_implicant const& b) noexcept { return a.mask == b.mask && a.value == b.value; }),
                            cover.end());
            };

            auto reduce_step = [&]() noexcept
            {
                if(cover.empty()) { return; }
                // Coverage counts over ON minterms.
                ::std::vector<::std::uint16_t> cnt{};
                cnt.assign(U, 0u);
                for(auto const& c: cover)
                {
                    for(auto const m: on)
                    {
                        if(!implicant_covers(c, m)) { continue; }
                        auto& x = cnt[static_cast<std::size_t>(m)];
                        if(x != ::std::numeric_limits<::std::uint16_t>::max()) { ++x; }
                    }
                }

                for(auto& c: cover)
                {
                    // Essential region R: ON minterms uniquely covered by c.
                    bool has_r{};
                    ::std::uint16_t r_min{};
                    for(auto const m: on)
                    {
                        if(!implicant_covers(c, m)) { continue; }
                        if(cnt[static_cast<std::size_t>(m)] == 1u)
                        {
                            r_min = m;
                            has_r = true;
                            break;
                        }
                    }
                    if(!has_r)
                    {
                        // No unique responsibility: anchor reduction to the first ON minterm covered by this cube (if any).
                        bool found{};
                        for(auto const m: on)
                        {
                            if(implicant_covers(c, m))
                            {
                                r_min = m;
                                found = true;
                                break;
                            }
                        }
                        if(!found) { continue; }
                    }

                    // Reduce: for each don't-care var, if all R minterms share the same bit, specialize to that bit.
                    // In the anchored case (single minterm), this specializes all don't-care vars, shrinking to that minterm.
                    for(auto const v: var_order)
                    {
                        auto const bit = static_cast<::std::uint16_t>(1u << v);
                        if(((c.mask >> v) & 1u) == 0u) { continue; }  // already specified

                        bool all0{true};
                        bool all1{true};

                        if(has_r)
                        {
                            for(auto const m: on)
                            {
                                if(!implicant_covers(c, m)) { continue; }
                                if(cnt[static_cast<std::size_t>(m)] != 1u) { continue; }
                                if(m & bit) { all0 = false; }
                                else
                                {
                                    all1 = false;
                                }
                                if(!all0 && !all1) { break; }
                            }
                        }
                        else
                        {
                            // anchored single minterm
                            if(r_min & bit) { all0 = false; }
                            else
                            {
                                all1 = false;
                            }
                        }

                        if(all0 != all1)  // exactly one is true
                        {
                            c.mask = static_cast<::std::uint16_t>(c.mask & static_cast<::std::uint16_t>(~bit));
                            if(all1) { c.value = static_cast<::std::uint16_t>(c.value | bit); }
                            else
                            {
                                c.value = static_cast<::std::uint16_t>(c.value & static_cast<::std::uint16_t>(~bit));
                            }
                        }
                    }
                }
            };

            auto step_expand = [&]() noexcept -> void
            {
                // Heuristic ordering: expand cubes that already cover more ON minterms first.
                ::std::vector<::std::pair<std::size_t, qm_implicant>> ranked{};
                ranked.reserve(cover.size());
                for(auto const& c: cover)
                {
                    std::size_t cnt{};
                    for(std::uint32_t w{}; w < static_cast<std::uint32_t>(blocksU); ++w)
                    {
                        auto const covw = cube_cover_word(c, w);
                        cnt += static_cast<std::size_t>(__builtin_popcountll(covw & on_bits[w]));
                    }
                    ranked.push_back({cnt, c});
                }
                ::std::sort(ranked.begin(), ranked.end(), [](auto const& a, auto const& b) noexcept { return a.first > b.first; });
                cover.clear();
                cover.reserve(ranked.size());
                for(auto& rc: ranked) { cover.push_back(rc.second); }

                // Structural expand: batch hits-OFF checks across cubes for each variable.
                // This is GPU-friendly when CUDA is enabled and the cover is large.
                (void)expand_cover_batch();
                dedupe();
                (void)irredundant();
            };

            auto step_reduce = [&]() noexcept -> void
            {
                reduce_step();
                dedupe();
                (void)irredundant();
            };

            // Main expand/reduce/irredundant iterations with best-so-far tracking.
            std::vector<qm_implicant> best_cover = cover;
            std::size_t best_cost = two_level_cover_cost(best_cover, var_count, opt);

            for(std::size_t iter{}; iter < 8u; ++iter)
            {
                auto const prev_best = best_cost;

                step_expand();
                if(two_level_cover_covers_all_on(cover, on) && !two_level_cover_hits_off(cover, is_on, is_dc))
                {
                    auto const cst = two_level_cover_cost(cover, var_count, opt);
                    if(cst < best_cost)
                    {
                        best_cost = cst;
                        best_cover = cover;
                    }
                }

                step_reduce();
                if(two_level_cover_covers_all_on(cover, on) && !two_level_cover_hits_off(cover, is_on, is_dc))
                {
                    auto const cst = two_level_cover_cost(cover, var_count, opt);
                    if(cst < best_cost)
                    {
                        best_cost = cst;
                        best_cover = cover;
                    }
                }

                if(best_cost >= prev_best) { break; }
            }

            // Last gasp: deterministic perturbation of one cube, then rerun a few iterations.
            if(!best_cover.empty())
            {
                for(std::size_t attempt{}; attempt < 4u; ++attempt)
                {
                    cover = best_cover;
                    if(cover.empty()) { break; }
                    auto const idx = static_cast<std::size_t>((attempt * 2654435761u) % cover.size());

                    // Pick an anchor ON minterm covered by this cube; shrink to that minterm.
                    std::optional<std::uint16_t> anchor{};
                    for(auto const m: on)
                    {
                        if(implicant_covers(cover[idx], m))
                        {
                            anchor = m;
                            break;
                        }
                    }
                    if(!anchor) { continue; }
                    cover[idx].mask = 0u;
                    cover[idx].value = *anchor;

                    for(std::size_t i{}; i < 4u; ++i)
                    {
                        step_expand();
                        step_reduce();
                    }
                    if(!two_level_cover_covers_all_on(cover, on) || two_level_cover_hits_off(cover, is_on, is_dc)) { continue; }
                    auto const cst = two_level_cover_cost(cover, var_count, opt);
                    if(cst < best_cost)
                    {
                        best_cost = cst;
                        best_cover = cover;
                    }
                }
            }

            // Validate cover (best-effort). If something went wrong, bail.
            if(!two_level_cover_covers_all_on(best_cover, on)) { return sol; }
            if(two_level_cover_hits_off(best_cover, is_on, is_dc)) { return sol; }

            sol.cover = ::std::move(best_cover);
            sol.cost = best_cost;
            return sol;
        }

        [[nodiscard]] inline espresso_solution espresso_two_level_minimize(::std::vector<::std::uint16_t> const& on,
                                                                           ::std::vector<::std::uint16_t> const& dc,
                                                                           std::size_t var_count,
                                                                           pe_synth_options const& opt) noexcept
        {
            auto best = espresso_two_level_minimize_base(on, dc, var_count, opt);
            if(best.cost == static_cast<std::size_t>(-1)) { return best; }
            if(var_count == 0 || var_count > 16) { return best; }

            // Complementation-based improvement: try minimizing F' (OFF-set), then invert output if cheaper.
            auto const U = static_cast<std::size_t>(1u << var_count);
            ::std::vector<bool> is_on{};
            ::std::vector<bool> is_dc{};
            is_on.assign(U, false);
            is_dc.assign(U, false);
            for(auto const m: on)
            {
                if(static_cast<std::size_t>(m) < U) { is_on[static_cast<std::size_t>(m)] = true; }
            }
            for(auto const m: dc)
            {
                if(static_cast<std::size_t>(m) < U) { is_dc[static_cast<std::size_t>(m)] = true; }
            }

            ::std::vector<::std::uint16_t> off{};
            off.reserve(U);
            for(std::size_t m{}; m < U; ++m)
            {
                if(is_on[m] || is_dc[m]) { continue; }
                off.push_back(static_cast<::std::uint16_t>(m));
            }
            if(off.empty()) { return best; }

            auto comp = espresso_two_level_minimize_base(off, dc, var_count, opt);
            if(comp.cost == static_cast<std::size_t>(-1)) { return best; }

            std::size_t penalty{};
            if(opt.two_level_cost == pe_synth_options::two_level_cost_model::literal_count) { penalty = 1u; }
            else
            {
                penalty = static_cast<std::size_t>(opt.two_level_weights.not_w);
            }

            if(comp.cost + penalty < best.cost)
            {
                comp.cost = comp.cost + penalty;
                comp.complemented = true;
                return comp;
            }

            return best;
        }

        [[nodiscard]] inline qm_solution qm_exact_minimum_cover(::std::vector<qm_implicant> const& primes,
                                                                ::std::vector<::std::uint16_t> const& on,
                                                                std::size_t var_count,
                                                                pe_synth_options const& opt) noexcept
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
                    for(auto const pi: cand)
                    {
                        bool already{};
                        for(auto const ppi: picked)
                        {
                            if(ppi == pi)
                            {
                                already = true;
                                break;
                            }
                        }
                        if(already)
                        {
                            alive = 2;
                            break;
                        }  // already covered by picked
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

            auto const base_cost = qm_cover_cost(primes, picked, var_count, opt);
            best.pick = picked;
            best.cost = base_cost;

            auto all_covered = [&]() noexcept -> bool
            {
                for(auto const v: covered)
                {
                    if(!v) { return false; }
                }
                return true;
            };
            if(all_covered()) { return best; }

            // Map: prime index -> whether selected.
            ::std::vector<bool> selected{};
            selected.resize(primes.size());
            for(auto const pi: picked)
            {
                if(pi < selected.size()) { selected[pi] = true; }
            }

            // Recursive search.
            auto dfs = [&](auto&& self, ::std::vector<bool>& cov, ::std::vector<bool>& sel, ::std::vector<std::size_t>& pick) noexcept -> void
            {
                if(best.cost != static_cast<std::size_t>(-1))
                {
                    auto const cost = qm_cover_cost(primes, pick, var_count, opt);
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
                    auto const cost = qm_cover_cost(primes, pick, var_count, opt);
                    if(cost < best.cost)
                    {
                        best.pick = pick;
                        best.cost = cost;
                    }
                    return;
                }

                auto const& options = covers[next_m];
                for(auto const pi: options)
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

                    for(auto const mj: flipped) { cov[mj] = false; }
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

        [[nodiscard]] inline qm_solution qm_petrick_minimum_cover(::std::vector<qm_implicant> const& primes,
                                                                  ::std::vector<::std::uint16_t> const& on,
                                                                  std::size_t var_count,
                                                                  pe_synth_options const& opt) noexcept
        {
            // Petrick's method (exact) for moderate problem sizes (bounded by primes<=64).
            // Returns minimal cost cover under the chosen cost model.
            qm_solution sol{};
            if(on.empty())
            {
                sol.cost = 0;
                return sol;
            }
            if(primes.size() == 0 || primes.size() > 64u) { return sol; }

            // Essential picks first.
            ::std::vector<bool> covered{};
            covered.assign(on.size(), false);

            auto covers_m = [&](std::size_t pi, ::std::uint16_t m) noexcept -> bool
            {
                if(pi >= primes.size()) { return false; }
                return implicant_covers(primes[pi], m);
            };

            bool changed{true};
            while(changed)
            {
                changed = false;
                for(std::size_t mi{}; mi < on.size(); ++mi)
                {
                    if(covered[mi]) { continue; }
                    auto const m = on[mi];
                    std::size_t cnt{};
                    std::size_t last{};
                    for(std::size_t pi{}; pi < primes.size(); ++pi)
                    {
                        if(!covers_m(pi, m)) { continue; }
                        ++cnt;
                        last = pi;
                        if(cnt > 1) { break; }
                    }
                    if(cnt == 0)
                    {
                        sol.cost = static_cast<std::size_t>(-1);
                        return sol;
                    }
                    if(cnt == 1)
                    {
                        bool already{};
                        for(auto const p: sol.pick)
                        {
                            if(p == last)
                            {
                                already = true;
                                break;
                            }
                        }
                        if(!already) { sol.pick.push_back(last); }
                        for(std::size_t mj{}; mj < on.size(); ++mj)
                        {
                            if(covered[mj]) { continue; }
                            if(covers_m(last, on[mj])) { covered[mj] = true; }
                        }
                        changed = true;
                    }
                }
            }

            // Build clauses for remaining uncovered minterms.
            ::std::vector<::std::uint16_t> remaining{};
            remaining.reserve(on.size());
            for(std::size_t mi{}; mi < on.size(); ++mi)
            {
                if(!covered[mi]) { remaining.push_back(on[mi]); }
            }
            if(remaining.empty())
            {
                sol.cost = qm_cover_cost(primes, sol.pick, var_count, opt);
                return sol;
            }

            auto bit_cost = [&](std::uint64_t bits) noexcept -> std::size_t
            {
                ::std::vector<std::size_t> pick = sol.pick;
                for(std::size_t pi{}; pi < primes.size(); ++pi)
                {
                    if((bits >> pi) & 1ull) { pick.push_back(pi); }
                }
                return qm_cover_cost(primes, pick, var_count, opt);
            };

            ::std::vector<std::uint64_t> terms{};
            terms.reserve(256);
            terms.push_back(0ull);

            constexpr std::size_t max_terms = 16384;

            for(auto const m: remaining)
            {
                std::uint64_t clause{};
                for(std::size_t pi{}; pi < primes.size(); ++pi)
                {
                    // skip already-picked primes
                    bool already{};
                    for(auto const p: sol.pick)
                    {
                        if(p == pi)
                        {
                            already = true;
                            break;
                        }
                    }
                    if(already) { continue; }
                    if(covers_m(pi, m)) { clause |= (1ull << pi); }
                }
                if(clause == 0ull)
                {
                    sol.cost = static_cast<std::size_t>(-1);
                    return sol;
                }

                ::std::vector<std::uint64_t> next{};
                next.reserve(terms.size() * 4u + 8u);
                for(auto const t: terms)
                {
                    auto c = clause;
                    while(c)
                    {
                        auto const b = static_cast<unsigned>(__builtin_ctzll(c));
                        c &= (c - 1ull);
                        next.push_back(t | (1ull << b));
                        if(next.size() > max_terms)
                        {
                            sol.cost = static_cast<std::size_t>(-1);
                            return sol;
                        }
                    }
                }

                // Dedup.
                ::std::sort(next.begin(), next.end());
                next.erase(::std::unique(next.begin(), next.end()), next.end());

                // Dominance pruning by subset + cost.
                ::std::vector<std::size_t> cost{};
                cost.resize(next.size());
                for(std::size_t i{}; i < next.size(); ++i) { cost[i] = bit_cost(next[i]); }

                ::std::vector<bool> drop{};
                drop.assign(next.size(), false);
                for(std::size_t i{}; i < next.size(); ++i)
                {
                    if(drop[i]) { continue; }
                    for(std::size_t j{i + 1}; j < next.size(); ++j)
                    {
                        if(drop[j]) { continue; }
                        auto const a = next[i];
                        auto const b = next[j];
                        if((a & b) == a)
                        {
                            if(cost[i] <= cost[j]) { drop[j] = true; }
                        }
                        else if((a & b) == b)
                        {
                            if(cost[j] <= cost[i])
                            {
                                drop[i] = true;
                                break;
                            }
                        }
                    }
                }

                ::std::vector<std::uint64_t> pruned{};
                pruned.reserve(next.size());
                for(std::size_t i{}; i < next.size(); ++i)
                {
                    if(!drop[i]) { pruned.push_back(next[i]); }
                }
                if(pruned.empty())
                {
                    sol.cost = static_cast<std::size_t>(-1);
                    return sol;
                }

                terms = ::std::move(pruned);
            }

            // Choose best term.
            std::uint64_t best_bits{};
            std::size_t best_cost{static_cast<std::size_t>(-1)};
            for(auto const t: terms)
            {
                auto const c = bit_cost(t);
                if(c < best_cost)
                {
                    best_cost = c;
                    best_bits = t;
                }
            }
            if(best_cost == static_cast<std::size_t>(-1))
            {
                sol.cost = static_cast<std::size_t>(-1);
                return sol;
            }
            for(std::size_t pi{}; pi < primes.size(); ++pi)
            {
                if((best_bits >> pi) & 1ull) { sol.pick.push_back(pi); }
            }
            sol.cost = qm_cover_cost(primes, sol.pick, var_count, opt);
            return sol;
        }

        [[nodiscard]] inline qm_solution qm_greedy_cover(::std::vector<qm_implicant> const& primes,
                                                         ::std::vector<::std::uint16_t> const& on,
                                                         std::size_t var_count,
                                                         pe_synth_options const& opt) noexcept
        {
            qm_solution sol{};
            if(on.empty())
            {
                sol.cost = 0;
                return sol;
            }

            auto const blocks = (on.size() + 63u) / 64u;
            ::std::vector<::std::uint64_t> cov{};
            cov.assign(primes.size() * blocks, 0ull);

            auto cov_row = [&](std::size_t pi) noexcept -> ::std::uint64_t*
            {
                return cov.data() + pi * blocks;
            };
            auto cov_row_c = [&](std::size_t pi) noexcept -> ::std::uint64_t const*
            {
                return cov.data() + pi * blocks;
            };

            ::std::vector<::std::size_t> prime_cost{};
            prime_cost.resize(primes.size());

            for(std::size_t pi{}; pi < primes.size(); ++pi)
            {
                prime_cost[pi] = qm_cover_cost(primes, ::std::vector<std::size_t>{pi}, var_count, opt);
                for(std::size_t mi{}; mi < on.size(); ++mi)
                {
                    if(implicant_covers(primes[pi], on[mi])) { cov_row(pi)[mi / 64u] |= (1ull << (mi % 64u)); }
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
                    if((cov_row_c(pi)[mi / 64u] >> (mi % 64u)) & 1ull)
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
                auto const* r = cov_row_c(pi);
                for(std::size_t b{}; b < blocks; ++b) { uncovered[b] &= ~r[b]; }
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
                for(auto const w: uncovered)
                {
                    if(w) { return true; }
                }
                return false;
            };

            // Optional CUDA acceleration for masked popcount scoring (best-effort).
            // We keep the cover matrix on the device(s) and only stream the changing mask (`uncovered`) per iteration.
            cuda_bitset_matrix_raii cov_gpu{};
            bool use_cuda_gains{false};
            ::std::vector<::std::uint32_t> gains_gpu{};
            if(opt.cuda_enable && blocks != 0u && !primes.empty())
            {
                auto const workload = primes.size() * blocks;
                if(workload >= opt.cuda_min_batch)
                {
                    cov_gpu = cuda_bitset_matrix_create(opt.cuda_device_mask, cov.data(), primes.size(), static_cast<::std::uint32_t>(blocks));
                    if(cov_gpu.handle != nullptr)
                    {
                        gains_gpu.resize(primes.size());
                        use_cuda_gains = true;
                    }
                }
            }

            while(any_uncovered())
            {
                if(use_cuda_gains)
                {
                    if(!cuda_bitset_matrix_row_and_popcount(cov_gpu, uncovered.data(), static_cast<::std::uint32_t>(blocks), gains_gpu.data()))
                    {
                        use_cuda_gains = false;
                        gains_gpu.clear();
                    }
                }

                std::size_t best_pi{static_cast<std::size_t>(-1)};
                std::size_t best_gain{};
                std::size_t best_cost{static_cast<std::size_t>(-1)};
                std::int64_t best_score{::std::numeric_limits<std::int64_t>::min()};

                struct cand_t
                {
                    std::size_t pi{};
                    std::size_t gain{};
                    std::size_t cost{};
                    std::int64_t score{};
                };
                ::std::vector<cand_t> cands{};
                cands.reserve(64);

                for(std::size_t pi{}; pi < primes.size(); ++pi)
                {
                    if(picked[pi]) { continue; }

                    std::size_t gain{};
                    if(use_cuda_gains) { gain = static_cast<std::size_t>(gains_gpu[pi]); }
                    else
                    {
                        auto const* r = cov_row_c(pi);
                        for(std::size_t b{}; b < blocks; ++b) { gain += popcount64(r[b] & uncovered[b]); }
                    }
                    if(gain == 0) { continue; }

                    auto const cost = prime_cost[pi];
                    // Score: prioritize coverage, then lower cost.
                    auto const score = static_cast<std::int64_t>(gain) * 64 - static_cast<std::int64_t>(cost);
                    if(cands.size() < 64u) { cands.push_back(cand_t{pi, gain, cost, score}); }
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

                // Bounded 2-step lookahead (deterministic): among the top candidates, pick one that leaves a good next move.
                // This tends to improve greedy cover quality when the prime set is large.
                if(cands.size() >= 2u && blocks != 0u)
                {
                    ::std::sort(cands.begin(),
                                cands.end(),
                                [](cand_t const& a, cand_t const& b) noexcept
                                {
                                    if(a.score != b.score) { return a.score > b.score; }
                                    if(a.gain != b.gain) { return a.gain > b.gain; }
                                    return a.cost < b.cost;
                                });

                    constexpr std::size_t lookahead_k = 16u;
                    auto const k = (cands.size() < lookahead_k) ? cands.size() : lookahead_k;

                    ::std::vector<::std::uint64_t> tmp_uncovered{};
                    tmp_uncovered.resize(blocks);

                    std::size_t best2_pi = best_pi;
                    std::int64_t best2_score = best_score;
                    std::size_t best2_gain = best_gain;
                    std::size_t best2_cost = best_cost;
                    std::int64_t best2_combined = ::std::numeric_limits<std::int64_t>::min();

                    for(std::size_t i{}; i < k; ++i)
                    {
                        auto const p = cands[i].pi;
                        auto const* pr = cov_row_c(p);
                        for(std::size_t b{}; b < blocks; ++b) { tmp_uncovered[b] = uncovered[b] & ~pr[b]; }

                        std::int64_t best_next_score = ::std::numeric_limits<std::int64_t>::min();
                        for(std::size_t j{}; j < k; ++j)
                        {
                            auto const q = cands[j].pi;
                            if(q == p) { continue; }
                            auto const* qr = cov_row_c(q);
                            std::size_t g2{};
                            for(std::size_t b{}; b < blocks; ++b) { g2 += popcount64(qr[b] & tmp_uncovered[b]); }
                            if(g2 == 0u) { continue; }
                            auto const c2 = prime_cost[q];
                            auto const s2 = static_cast<std::int64_t>(g2) * 64 - static_cast<std::int64_t>(c2);
                            if(s2 > best_next_score) { best_next_score = s2; }
                        }

                        auto const combined = cands[i].score + ((best_next_score == ::std::numeric_limits<std::int64_t>::min()) ? 0 : best_next_score);
                        if(combined > best2_combined)
                        {
                            best2_combined = combined;
                            best2_pi = p;
                            best2_score = cands[i].score;
                            best2_gain = cands[i].gain;
                            best2_cost = cands[i].cost;
                        }
                    }

                    if(best2_pi != best_pi)
                    {
                        best_pi = best2_pi;
                        best_score = best2_score;
                        best_gain = best2_gain;
                        best_cost = best2_cost;
                    }
                }

                apply_pick(best_pi);
            }

            // Irredundant cleanup (bitset-based): drop any selected prime that covers no unique ON minterm.
            // This improves the greedy cover (especially after 2-step lookahead) at low extra cost.
            if(!sol.pick.empty() && blocks != 0u)
            {
                ::std::vector<::std::uint16_t> cnt{};
                cnt.assign(on.size(), 0u);

                auto add_counts = [&](std::size_t pi) noexcept
                {
                    auto const* row = cov_row_c(pi);
                    for(std::size_t b{}; b < blocks; ++b)
                    {
                        auto x = row[b];
                        while(x)
                        {
                            auto const bit = static_cast<unsigned>(__builtin_ctzll(x));
                            x &= (x - 1ull);
                            auto const idx = b * 64u + static_cast<std::size_t>(bit);
                            if(idx >= on.size()) { continue; }
                            auto& v = cnt[idx];
                            if(v != ::std::numeric_limits<::std::uint16_t>::max()) { ++v; }
                        }
                    }
                };

                for(auto const pi: sol.pick) { add_counts(pi); }

                for(std::size_t i{}; i < sol.pick.size();)
                {
                    auto const pi = sol.pick[i];
                    auto const* row = cov_row_c(pi);
                    bool redundant{true};
                    bool covers_any{};
                    for(std::size_t b{}; b < blocks && redundant; ++b)
                    {
                        auto x = row[b];
                        while(x)
                        {
                            auto const bit = static_cast<unsigned>(__builtin_ctzll(x));
                            x &= (x - 1ull);
                            auto const idx = b * 64u + static_cast<std::size_t>(bit);
                            if(idx >= on.size()) { continue; }
                            covers_any = true;
                            if(cnt[idx] <= 1u)
                            {
                                redundant = false;
                                break;
                            }
                        }
                    }
                    if(!covers_any) { redundant = true; }
                    if(!redundant)
                    {
                        ++i;
                        continue;
                    }

                    // Remove prime and decrement its covered counts.
                    for(std::size_t b{}; b < blocks; ++b)
                    {
                        auto x = row[b];
                        while(x)
                        {
                            auto const bit = static_cast<unsigned>(__builtin_ctzll(x));
                            x &= (x - 1ull);
                            auto const idx = b * 64u + static_cast<std::size_t>(bit);
                            if(idx >= on.size()) { continue; }
                            auto& v = cnt[idx];
                            if(v) { --v; }
                        }
                    }

                    sol.pick[i] = sol.pick.back();
                    sol.pick.pop_back();
                }
            }

            sol.cost = qm_cover_cost(primes, sol.pick, var_count, opt);
            return sol;
        }

        [[nodiscard]] inline bool optimize_qm_two_level_minimize_in_pe_netlist(::phy_engine::netlist::netlist& nl,
                                                                               ::std::vector<::phy_engine::model::node_t*> const& protected_nodes,
                                                                               pe_synth_options const& opt,
                                                                               dc_constraints const* dc = nullptr) noexcept
        {
            // Two-level minimization on small binary cones (exclusive fanout):
            // - Quine–McCluskey prime implicants + exact cover (very small) / greedy cover (moderate)
            // - Espresso-style expand+irredundant heuristic (moderate), choose the best cost
            // Only safe when the cone's internal nets have fanout=1 (exclusive to the cone).
            if(!opt.assume_binary_inputs) { return false; }
            if(opt.qm_max_vars == 0u) { return false; }

            constexpr std::size_t exact_vars = 6;
            std::size_t const max_vars = ::std::min<std::size_t>(16u, (opt.qm_max_vars == 0u) ? 10u : opt.qm_max_vars);
            std::size_t const max_gates = (opt.qm_max_gates == 0u) ? 64u : opt.qm_max_gates;
            std::size_t const max_primes = (opt.qm_max_primes == 0u) ? 4096u : opt.qm_max_primes;

            ::std::unordered_map<::phy_engine::model::node_t*, bool> protected_map{};
            protected_map.reserve(protected_nodes.size() * 2u + 1u);
            for(auto* n: protected_nodes) { protected_map.emplace(n, true); }
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

            struct consumer_gate
            {
                gkind k{};
                ::phy_engine::model::node_t* other{};
                bool root_is_in0{};
            };

            ::std::unordered_map<::phy_engine::model::node_t*, gate> gate_by_out{};
            gate_by_out.reserve(1 << 14);

            auto classify = [&](::phy_engine::model::model_base const& mb, ::phy_engine::netlist::model_pos pos) noexcept -> ::std::optional<gate>
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

                if(name == u8"AND" || name == u8"OR" || name == u8"XOR" || name == u8"XNOR" || name == u8"NAND" || name == u8"NOR" || name == u8"IMP" ||
                   name == u8"NIMP")
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
                    else
                    {
                        g.k = gkind::nimp_gate;
                    }
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

            ::std::unordered_map<::phy_engine::model::node_t*, ::std::vector<consumer_gate>> consumers{};
            consumers.reserve(gate_by_out.size() * 2u + 1u);
            for(auto const& kv: gate_by_out)
            {
                auto const& g = kv.second;
                if(g.in0 != nullptr) { consumers[g.in0].push_back(consumer_gate{g.k, g.in1, true}); }
                if(g.k != gkind::not_gate && g.in1 != nullptr) { consumers[g.in1].push_back(consumer_gate{g.k, g.in0, false}); }
            }

            // Detect constant nodes produced by unnamed INPUT models. These are treated as constants (not leaf vars).
            ::std::unordered_map<::phy_engine::model::node_t*, ::phy_engine::model::digital_node_statement_t> const_val{};
            const_val.reserve(128);
            for(auto const& blk : nl.models)
            {
                for(auto const* m = blk.begin; m != blk.curr; ++m)
                {
                    if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }
                    if(model_name_u8(*m) != u8"INPUT") { continue; }
                    if(m->name.size() != 0) { continue; }
                    auto pv = m->ptr->generate_pin_view();
                    if(pv.size != 1 || pv.pins[0].nodes == nullptr) { continue; }
                    auto vi = m->ptr->get_attribute(0);
                    if(vi.type != ::phy_engine::model::variant_type::digital) { continue; }
                    const_val.emplace(pv.pins[0].nodes, vi.digital);
                }
            }

            struct dc_group_view
            {
                ::std::vector<std::size_t> leaf_pos{};
                ::std::vector<bool> allowed{};
            };

            auto build_dc_group_views = [&](::std::vector<::phy_engine::model::node_t*> const& leaves) noexcept -> ::std::vector<dc_group_view>
            {
                ::std::vector<dc_group_view> views{};
                if(dc == nullptr || dc->groups.empty()) { return views; }
                for(auto const& g: dc->groups)
                {
                    if(g.nodes.empty() || g.allowed.empty() || g.nodes.size() > 16u) { continue; }
                    ::std::vector<std::size_t> pos{};
                    pos.reserve(g.nodes.size());
                    bool ok{true};
                    for(auto* n: g.nodes)
                    {
                        std::size_t idx = SIZE_MAX;
                        for(std::size_t i{}; i < leaves.size(); ++i)
                        {
                            if(leaves[i] == n)
                            {
                                idx = i;
                                break;
                            }
                        }
                        if(idx == SIZE_MAX)
                        {
                            ok = false;
                            break;
                        }
                        pos.push_back(idx);
                    }
                    if(!ok) { continue; }
                    dc_group_view view{};
                    view.leaf_pos = ::std::move(pos);
                    view.allowed.assign(1u << g.nodes.size(), false);
                    for(auto const v: g.allowed)
                    {
                        if(v < view.allowed.size()) { view.allowed[v] = true; }
                    }
                    views.push_back(::std::move(view));
                }
                return views;
            };

            auto apply_dc_constraints = [&](::std::vector<::std::uint16_t>& dc_list,
                                            ::std::vector<::std::uint16_t> const& on_list,
                                            std::size_t var_count,
                                            ::std::vector<dc_group_view> const& views) noexcept
            {
                if(views.empty() || var_count == 0u || var_count > 16u) { return; }
                auto const U = static_cast<std::size_t>(1u << var_count);
                ::std::vector<bool> is_on{};
                ::std::vector<bool> is_dc{};
                is_on.assign(U, false);
                is_dc.assign(U, false);
                for(auto const m: on_list)
                {
                    if(static_cast<std::size_t>(m) < U) { is_on[static_cast<std::size_t>(m)] = true; }
                }
                for(auto const m: dc_list)
                {
                    if(static_cast<std::size_t>(m) < U) { is_dc[static_cast<std::size_t>(m)] = true; }
                }
                for(::std::size_t m{}; m < U; ++m)
                {
                    if(is_on[m] || is_dc[m]) { continue; }
                    bool dc_hit{false};
                    for(auto const& v: views)
                    {
                        ::std::uint16_t sub{};
                        for(std::size_t bi{}; bi < v.leaf_pos.size(); ++bi)
                        {
                            auto const lp = v.leaf_pos[bi];
                            if(((m >> lp) & 1u) != 0u) { sub = static_cast<::std::uint16_t>(sub | (1u << bi)); }
                        }
                        if(sub < v.allowed.size() && !v.allowed[sub])
                        {
                            dc_hit = true;
                            break;
                        }
                    }
                    if(dc_hit) { is_dc[m] = true; }
                }
                dc_list.clear();
                dc_list.reserve(U);
                for(::std::size_t m{}; m < U; ++m)
                {
                    if(is_dc[m]) { dc_list.push_back(static_cast<::std::uint16_t>(m)); }
                }
            };

            struct odc_condition
            {
                std::size_t leaf_idx{};
                bool value{};
            };

            auto build_odc_conditions =
                [&](::phy_engine::model::node_t* root,
                    ::std::unordered_map<::phy_engine::model::node_t*, std::size_t> const& leaf_index) noexcept -> ::std::vector<odc_condition>
            {
                ::std::vector<odc_condition> conds{};
                if(!opt.infer_dc_from_odc || root == nullptr) { return conds; }
                auto itc = fan.consumer_count.find(root);
                if(itc == fan.consumer_count.end() || itc->second != 1u) { return conds; }
                auto it = consumers.find(root);
                if(it == consumers.end() || it->second.size() != 1u) { return conds; }
                auto const& cg = it->second.front();
                if(cg.other == nullptr) { return conds; }
                auto it_leaf = leaf_index.find(cg.other);
                if(it_leaf == leaf_index.end()) { return conds; }

                bool ok{true};
                bool val{};
                switch(cg.k)
                {
                    case gkind::and_gate:
                    case gkind::nand_gate: val = false; break;
                    case gkind::or_gate:
                    case gkind::nor_gate: val = true; break;
                    case gkind::imp_gate: val = cg.root_is_in0 ? true : false; break;
                    case gkind::nimp_gate: val = cg.root_is_in0 ? true : false; break;
                    default: ok = false; break;
                }
                if(!ok) { return conds; }
                conds.push_back(odc_condition{it_leaf->second, val});
                return conds;
            };

            auto apply_odc_conditions = [&](::std::vector<::std::uint16_t>& on_list,
                                            ::std::vector<::std::uint16_t>& dc_list,
                                            std::size_t var_count,
                                            ::std::vector<odc_condition> const& conds) noexcept
            {
                if(conds.empty() || var_count == 0u || var_count > 16u) { return; }
                auto const U = static_cast<std::size_t>(1u << var_count);
                ::std::vector<bool> is_on{};
                ::std::vector<bool> is_dc{};
                is_on.assign(U, false);
                is_dc.assign(U, false);
                for(auto const m: on_list)
                {
                    if(static_cast<std::size_t>(m) < U) { is_on[static_cast<std::size_t>(m)] = true; }
                }
                for(auto const m: dc_list)
                {
                    if(static_cast<std::size_t>(m) < U) { is_dc[static_cast<std::size_t>(m)] = true; }
                }
                for(::std::size_t m{}; m < U; ++m)
                {
                    if(is_dc[m]) { continue; }
                    bool dc_hit{false};
                    for(auto const& c: conds)
                    {
                        if(((m >> c.leaf_idx) & 1u) == (c.value ? 1u : 0u))
                        {
                            dc_hit = true;
                            break;
                        }
                    }
                    if(dc_hit)
                    {
                        is_dc[m] = true;
                        is_on[m] = false;
                    }
                }
                on_list.clear();
                dc_list.clear();
                on_list.reserve(U);
                dc_list.reserve(U);
                for(::std::size_t m{}; m < U; ++m)
                {
                    if(is_on[m]) { on_list.push_back(static_cast<::std::uint16_t>(m)); }
                    else if(is_dc[m]) { dc_list.push_back(static_cast<::std::uint16_t>(m)); }
                }
            };

            auto eval_gate = [&](auto&& self,
                                 ::phy_engine::model::node_t* node,
                                 ::std::unordered_map<::phy_engine::model::node_t*, ::phy_engine::model::digital_node_statement_t> const& leaf_val,
                                 ::std::unordered_map<::phy_engine::model::node_t*, ::phy_engine::model::digital_node_statement_t>& memo,
                                 ::std::unordered_map<::phy_engine::model::node_t*, bool>& visiting) noexcept -> ::phy_engine::model::digital_node_statement_t
            {
                if(node == nullptr) { return ::phy_engine::model::digital_node_statement_t::indeterminate_state; }
                if(auto it = memo.find(node); it != memo.end()) { return it->second; }
                if(auto itc = const_val.find(node); itc != const_val.end())
                {
                    memo.emplace(node, itc->second);
                    return itc->second;
                }
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
                if(visiting.contains(node)) { return ::phy_engine::model::digital_node_statement_t::indeterminate_state; }
                visiting.emplace(node, true);
                auto const& g = itg->second;
                using dns = ::phy_engine::model::digital_node_statement_t;
                dns a = self(self, g.in0, leaf_val, memo, visiting);
                dns b = self(self, g.in1, leaf_val, memo, visiting);
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
                visiting.erase(node);
                memo.emplace(node, r);
                return r;
            };

            auto enc_kind_tt = [](gkind k) noexcept -> ::std::uint8_t
            {
                switch(k)
                {
                    case gkind::not_gate: return 0u;
                    case gkind::and_gate: return 1u;
                    case gkind::or_gate: return 2u;
                    case gkind::xor_gate: return 3u;
                    case gkind::xnor_gate: return 4u;
                    case gkind::nand_gate: return 5u;
                    case gkind::nor_gate: return 6u;
                    case gkind::imp_gate: return 7u;
                    case gkind::nimp_gate: return 8u;
                    default: return 1u;
                }
            };

            // Best-effort truth-table evaluation (binary only): if the cone depends on non-binary constants (X/Z),
            // caller should fall back to per-minterm 4-valued evaluation to preserve DC inference semantics.
            auto eval_binary_cone_on_dc =
                [&](::phy_engine::model::node_t* root,
                    ::std::vector<::phy_engine::model::node_t*> const& leaves,
                    ::std::vector<::std::uint16_t>& on_out,
                    ::std::vector<::std::uint16_t>& dc_out) noexcept -> bool
            {
                on_out.clear();
                dc_out.clear();
                auto const var_count = leaves.size();
                if(var_count == 0u) { return false; }
                if(var_count > 16u) { return false; }

                if(var_count <= 6u)
                {
                    cuda_u64_cone_desc cone{};
                    cone.var_count = static_cast<::std::uint8_t>(var_count);
                    cone.gate_count = 0u;
                    ::std::unordered_map<::phy_engine::model::node_t*, ::std::uint8_t> leaf_index{};
                    leaf_index.reserve(var_count * 2u);
                    for(std::size_t i{}; i < var_count; ++i) { leaf_index.emplace(leaves[i], static_cast<::std::uint8_t>(i)); }
                    ::std::unordered_map<::phy_engine::model::node_t*, ::std::uint8_t> out_index{};
                    out_index.reserve(128);
                    bool ok2{true};

                    auto dfs2 = [&](auto&& self, ::phy_engine::model::node_t* n) noexcept -> ::std::uint8_t
                    {
                        if(!ok2 || n == nullptr) { ok2 = false; return 254u; }
                        if(auto itc = const_val.find(n); itc != const_val.end())
                        {
                            using dns = ::phy_engine::model::digital_node_statement_t;
                            if(itc->second == dns::false_state) { return 254u; }
                            if(itc->second == dns::true_state) { return 255u; }
                            ok2 = false;
                            return 254u;
                        }
                        if(auto itl = leaf_index.find(n); itl != leaf_index.end()) { return itl->second; }
                        if(auto ito = out_index.find(n); ito != out_index.end()) { return ito->second; }

                        auto itg = gate_by_out.find(n);
                        if(itg == gate_by_out.end()) { ok2 = false; return 254u; }
                        auto const& gg = itg->second;
                        auto const a = self(self, gg.in0);
                        auto const b = (gg.k == gkind::not_gate) ? static_cast<::std::uint8_t>(254u) : self(self, gg.in1);
                        if(!ok2) { return 254u; }

                        if(cone.gate_count >= 64u) { ok2 = false; return 254u; }
                        auto const gi = static_cast<::std::uint8_t>(cone.gate_count++);
                        cone.kind[gi] = enc_kind_tt(gg.k);
                        cone.in0[gi] = a;
                        cone.in1[gi] = b;
                        auto const out = static_cast<::std::uint8_t>(6u + gi);
                        out_index.emplace(n, out);
                        return out;
                    };

                    (void)dfs2(dfs2, root);
                    if(!ok2 || cone.gate_count == 0u) { return false; }

                    ::std::uint64_t mask{};
                    if(opt.cuda_enable)
                    {
                        ::std::uint64_t out_mask{};
                        if(cuda_eval_u64_cones(opt.cuda_device_mask, __builtin_addressof(cone), 1u, __builtin_addressof(out_mask))) { mask = out_mask; }
                        else { mask = eval_u64_cone_cpu(cone); }
                    }
                    else { mask = eval_u64_cone_cpu(cone); }

                    auto const U = static_cast<::std::size_t>(1u << var_count);
                    auto const all_mask = (U == 64u) ? ~0ull : ((1ull << U) - 1ull);
                    mask &= all_mask;

                    on_out.reserve(U);
                    for(::std::uint16_t m{}; m < static_cast<::std::uint16_t>(U); ++m)
                    {
                        if((mask >> m) & 1ull) { on_out.push_back(m); }
                    }
                    return true;
                }

                // var_count > 6u
                cuda_tt_cone_desc cone{};
                cone.var_count = static_cast<::std::uint8_t>(var_count);
                cone.gate_count = 0u;
                ::std::unordered_map<::phy_engine::model::node_t*, ::std::uint16_t> leaf_index{};
                leaf_index.reserve(var_count * 2u);
                for(std::size_t i{}; i < var_count; ++i) { leaf_index.emplace(leaves[i], static_cast<::std::uint16_t>(i)); }
                ::std::unordered_map<::phy_engine::model::node_t*, ::std::uint16_t> out_index{};
                out_index.reserve(256);
                bool ok2{true};

                auto dfs2 = [&](auto&& self, ::phy_engine::model::node_t* n) noexcept -> ::std::uint16_t
                {
                    if(!ok2 || n == nullptr) { ok2 = false; return 65534u; }
                    if(auto itc = const_val.find(n); itc != const_val.end())
                    {
                        using dns = ::phy_engine::model::digital_node_statement_t;
                        if(itc->second == dns::false_state) { return 65534u; }
                        if(itc->second == dns::true_state) { return 65535u; }
                        ok2 = false;
                        return 65534u;
                    }
                    if(auto itl = leaf_index.find(n); itl != leaf_index.end()) { return itl->second; }
                    if(auto ito = out_index.find(n); ito != out_index.end()) { return ito->second; }

                    auto itg = gate_by_out.find(n);
                    if(itg == gate_by_out.end()) { ok2 = false; return 65534u; }
                    auto const& gg = itg->second;
                    auto const a = self(self, gg.in0);
                    auto const b = (gg.k == gkind::not_gate) ? static_cast<::std::uint16_t>(65534u) : self(self, gg.in1);
                    if(!ok2) { return 65534u; }

                    if(cone.gate_count >= 256u) { ok2 = false; return 65534u; }
                    auto const gi = static_cast<::std::uint16_t>(cone.gate_count++);
                    cone.kind[gi] = enc_kind_tt(gg.k);
                    cone.in0[gi] = a;
                    cone.in1[gi] = b;
                    auto const out = static_cast<::std::uint16_t>(16u + gi);
                    out_index.emplace(n, out);
                    return out;
                };

                (void)dfs2(dfs2, root);
                if(!ok2 || cone.gate_count == 0u) { return false; }

                auto const U = static_cast<::std::size_t>(1u << var_count);
                auto const blocks = static_cast<::std::uint32_t>((U + 63u) / 64u);
                ::std::vector<::std::uint64_t> tt{};
                tt.assign(blocks, 0ull);
                if(opt.cuda_enable)
                {
                    if(!cuda_eval_tt_cones(opt.cuda_device_mask, __builtin_addressof(cone), 1u, blocks, tt.data()))
                    {
                        eval_tt_cone_cpu(cone, blocks, tt.data());
                    }
                }
                else { eval_tt_cone_cpu(cone, blocks, tt.data()); }

                on_out.reserve(U);
                for(::std::uint32_t bi{}; bi < blocks; ++bi)
                {
                    auto w = tt[bi];
                    while(w)
                    {
                        auto const b = static_cast<::std::uint32_t>(__builtin_ctzll(w));
                        auto const m = static_cast<::std::size_t>(bi * 64u + b);
                        if(m < U) { on_out.push_back(static_cast<::std::uint16_t>(m)); }
                        w &= (w - 1ull);
                    }
                }
                return true;
            };

            auto make_not = [&](::phy_engine::model::node_t* in) noexcept -> ::phy_engine::model::node_t*
            {
                if(in == nullptr) { return nullptr; }
                auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::NOT{});
                (void)pos;
                if(m == nullptr) { return nullptr; }
                auto& out_ref = ::phy_engine::netlist::create_node(nl);
                auto* out = __builtin_addressof(out_ref);
                if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *in) || !::phy_engine::netlist::add_to_node(nl, *m, 1, *out)) { return nullptr; }
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
                for(auto* p: from->pins) { pins_to_move.push_back(p); }
                for(auto* p: pins_to_move)
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
            for(auto const& kv: gate_by_out) { roots.push_back(kv.first); }

            // Multi-output sharing (bounded): for protected output nodes that share the same leaf set, choose covers jointly to
            // encourage shared product terms (beyond per-output independent minimization).
            //
            // This runs before the generic per-root loop below. We only target protected roots to keep the scope tight and safe.
            {
                struct cone
                {
                    ::phy_engine::model::node_t* root{};
                    ::std::vector<::phy_engine::netlist::model_pos> to_delete{};
                    ::std::vector<::phy_engine::model::node_t*> leaves{};  // canonical (sorted) leaf order
                    ::std::vector<::std::uint16_t> on{};
                    ::std::vector<::std::uint16_t> dc{};
                };

                struct group
                {
                    ::std::vector<::phy_engine::model::node_t*> leaves{};
                    ::std::vector<cone> cones{};
                };

                auto leaves_hash = [&](::std::vector<::phy_engine::model::node_t*> const& v) noexcept -> std::size_t
                {
                    auto mix = [](std::size_t h, std::size_t x) noexcept -> std::size_t { return (h ^ (x + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2))); };
                    std::size_t h{};
                    for(auto* p: v) { h = mix(h, reinterpret_cast<std::size_t>(p)); }
                    return h;
                };

                ::std::unordered_map<std::size_t, ::std::vector<std::size_t>> hash_to_groups{};
                hash_to_groups.reserve(256);
                ::std::vector<group> groups{};
                groups.reserve(128);

                auto find_group = [&](::std::vector<::phy_engine::model::node_t*> const& leaves) noexcept -> group&
                {
                    auto const h = leaves_hash(leaves);
                    auto& ids = hash_to_groups[h];
                    for(auto const gi: ids)
                    {
                        if(gi >= groups.size()) { continue; }
                        if(groups[gi].leaves == leaves) { return groups[gi]; }
                    }
                    groups.push_back(group{.leaves = leaves, .cones = {}});
                    ids.push_back(groups.size() - 1u);
                    return groups.back();
                };

                auto collect_cone = [&](::phy_engine::model::node_t* root, cone& out) noexcept -> bool
                {
                    out.root = root;
                    out.to_delete.clear();
                    out.leaves.clear();
                    out.on.clear();
                    out.dc.clear();

                    if(root == nullptr) { return false; }
                    if(!is_protected(root)) { return false; }
                    auto it_drv = fan.driver_count.find(root);
                    if(it_drv == fan.driver_count.end() || it_drv->second != 1) { return false; }
                    auto it_root = gate_by_out.find(root);
                    if(it_root == gate_by_out.end()) { return false; }

                    out.to_delete.reserve(max_gates);
                    out.leaves.reserve(max_vars);
                    ::std::unordered_map<::phy_engine::model::node_t*, bool> visited{};
                    visited.reserve(max_gates * 2u);
                    ::std::unordered_map<::phy_engine::model::node_t*, bool> leaf_seen{};
                    leaf_seen.reserve(max_vars * 2u);

                    bool ok{true};
                    auto dfs = [&](auto&& self, ::phy_engine::model::node_t* n) noexcept -> void
                    {
                        if(!ok || n == nullptr) { return; }
                        if(visited.contains(n)) { return; }
                        visited.emplace(n, true);

                        auto itg = gate_by_out.find(n);
                        if(itg == gate_by_out.end())
                        {
                            if(const_val.contains(n)) { return; }
                            if(!leaf_seen.contains(n))
                            {
                                if(out.leaves.size() >= max_vars)
                                {
                                    ok = false;
                                    return;
                                }
                                leaf_seen.emplace(n, true);
                                out.leaves.push_back(n);
                            }
                            return;
                        }

                        auto const& g = itg->second;
                        if(n != root)
                        {
                            if(is_protected(n))
                            {
                                ok = false;
                                return;
                            }
                            auto itc = fan.consumer_count.find(n);
                            if(itc == fan.consumer_count.end() || itc->second != 1)
                            {
                                ok = false;
                                return;
                            }
                            auto itd = fan.driver_count.find(n);
                            if(itd == fan.driver_count.end() || itd->second != 1)
                            {
                                ok = false;
                                return;
                            }
                        }

                        if(out.to_delete.size() >= max_gates)
                        {
                            ok = false;
                            return;
                        }
                        out.to_delete.push_back(g.pos);

                        self(self, g.in0);
                        if(g.k != gkind::not_gate) { self(self, g.in1); }
                    };

                    dfs(dfs, root);
                    if(!ok) { return false; }
                    if(out.leaves.empty()) { return false; }

                    // Canonical leaf order for consistent variable indexing across outputs.
                    ::std::sort(out.leaves.begin(),
                                out.leaves.end(),
                                [](auto* a, auto* b) noexcept { return reinterpret_cast<std::uintptr_t>(a) < reinterpret_cast<std::uintptr_t>(b); });

                    auto const var_count = out.leaves.size();
                    if(var_count == 0 || var_count > 16) { return false; }

                    if(!eval_binary_cone_on_dc(root, out.leaves, out.on, out.dc))
                    {
                        // Fallback: per-minterm 4-valued evaluation (preserves X/Z -> DC inference semantics).
                        out.on.reserve(1u << var_count);
                        for(::std::uint16_t m{}; m < static_cast<::std::uint16_t>(1u << var_count); ++m)
                        {
                            ::std::unordered_map<::phy_engine::model::node_t*, ::phy_engine::model::digital_node_statement_t> leaf_val{};
                            leaf_val.reserve(var_count * 2u);
                            for(std::size_t i{}; i < var_count; ++i)
                            {
                                auto const bit = ((m >> i) & 1u) != 0;
                                leaf_val.emplace(out.leaves[i],
                                                 bit ? ::phy_engine::model::digital_node_statement_t::true_state
                                                     : ::phy_engine::model::digital_node_statement_t::false_state);
                            }
                            ::std::unordered_map<::phy_engine::model::node_t*, ::phy_engine::model::digital_node_statement_t> memo{};
                            memo.reserve(out.to_delete.size() * 2u + 8u);
                            ::std::unordered_map<::phy_engine::model::node_t*, bool> visiting{};
                            visiting.reserve(out.to_delete.size() * 2u + 8u);
                            auto const r = eval_gate(eval_gate, root, leaf_val, memo, visiting);
                            if(r == ::phy_engine::model::digital_node_statement_t::true_state) { out.on.push_back(m); }
                            else if(r == ::phy_engine::model::digital_node_statement_t::false_state) { /* off */ }
                            else
                            {
                                if(opt.assume_binary_inputs && opt.infer_dc_from_xz) { out.dc.push_back(m); }
                                else { return false; }
                            }
                        }
                    }

                    if(opt.infer_dc_from_fsm)
                    {
                        auto const views = build_dc_group_views(out.leaves);
                        if(!views.empty()) { apply_dc_constraints(out.dc, out.on, out.leaves.size(), views); }
                    }

                    // Normalize deletion list and avoid duplicates.
                    ::std::sort(out.to_delete.begin(),
                                out.to_delete.end(),
                                [](auto const& a, auto const& b) noexcept
                                {
                                    if(a.chunk_pos != b.chunk_pos) { return a.chunk_pos > b.chunk_pos; }
                                    return a.vec_pos > b.vec_pos;
                                });
                    out.to_delete.erase(::std::unique(out.to_delete.begin(),
                                                      out.to_delete.end(),
                                                      [](auto const& a, auto const& b) noexcept
                                                      { return a.chunk_pos == b.chunk_pos && a.vec_pos == b.vec_pos; }),
                                        out.to_delete.end());

                    return true;
                };

                // Collect candidate protected cones into groups by leaf-set.
                for(auto* root: roots)
                {
                    cone c{};
                    if(!collect_cone(root, c)) { continue; }
                    auto& g = find_group(c.leaves);
                    g.cones.push_back(::std::move(c));
                }

                // Helper: per-output best cover (QM/Petrick/Espresso).
                auto best_single_cover = [&](::std::vector<::std::uint16_t> const& on,
                                             ::std::vector<::std::uint16_t> const& dc,
                                             std::size_t var_count) noexcept -> ::std::vector<qm_implicant>
                {
                    if(on.empty()) { return {}; }
                    if(on.size() == (1u << var_count))
                    {
                        return {
                            qm_implicant{0u, static_cast<::std::uint16_t>((var_count >= 16) ? 0xFFFFu : ((1u << var_count) - 1u)), false}
                        };
                    }

                    auto const primes = qm_prime_implicants(on, dc, var_count);
                    if(primes.empty()) { return {}; }
                    qm_solution qm_sol{};
                    if(var_count <= exact_vars) { qm_sol = qm_exact_minimum_cover(primes, on, var_count, opt); }
                    else
                    {
                        qm_sol = qm_greedy_cover(primes, on, var_count, opt);
                        if(primes.size() <= 64u && on.size() <= 64u)
                        {
                            auto const pet = qm_petrick_minimum_cover(primes, on, var_count, opt);
                            if(pet.cost != static_cast<std::size_t>(-1) && (qm_sol.cost == static_cast<std::size_t>(-1) || pet.cost < qm_sol.cost))
                            {
                                qm_sol = pet;
                            }
                        }
                    }
                    if(qm_sol.cost == static_cast<std::size_t>(-1)) { return {}; }

                    ::std::vector<qm_implicant> qm_cover{};
                    qm_cover.reserve(qm_sol.pick.size());
                    for(auto const idx: qm_sol.pick)
                    {
                        if(idx < primes.size()) { qm_cover.push_back(primes[idx]); }
                    }
                    auto qm_cost = two_level_cover_cost(qm_cover, var_count, opt);

                    if(var_count > exact_vars)
                    {
                        auto const esp = espresso_two_level_minimize_base(on, dc, var_count, opt);
                        if(esp.cost != static_cast<std::size_t>(-1) && esp.cost < qm_cost) { return esp.cover; }
                    }
                    return qm_cover;
                };

                for(auto& grp: groups)
                {
                    if(grp.cones.size() < 2u) { continue; }
                    auto const var_count = grp.leaves.size();
                    if(var_count == 0 || var_count > max_vars) { continue; }
                    if(opt.qm_max_minterms != 0u)
                    {
                        auto const universe = (var_count >= 63u) ? ::std::numeric_limits<std::size_t>::max() : (static_cast<std::size_t>(1u) << var_count);
                        if(universe > opt.qm_max_minterms) { continue; }
                    }

                    ::std::vector<::std::vector<::std::uint16_t>> on_list{};
                    ::std::vector<::std::vector<::std::uint16_t>> dc_list{};
                    on_list.reserve(grp.cones.size());
                    dc_list.reserve(grp.cones.size());
                    for(auto const& c: grp.cones)
                    {
                        on_list.push_back(c.on);
                        dc_list.push_back(c.dc);
                    }

                    // Baseline: independent best per output, but with shared-term accounting in the cost model.
                    ::std::vector<::std::vector<qm_implicant>> base_covers{};
                    base_covers.resize(grp.cones.size());
                    for(std::size_t oi{}; oi < grp.cones.size(); ++oi)
                    {
                        base_covers[oi] = best_single_cover(on_list[oi], dc_list[oi], var_count);
                        if(!on_list[oi].empty() && base_covers[oi].empty())
                        {
                            // Failed to find a cover for a non-constant-0 function: skip this group.
                            base_covers.clear();
                            break;
                        }
                    }
                    if(base_covers.empty()) { continue; }
                    auto const base_cost = multi_output_gate_cost(base_covers, var_count, opt);

                    // Multi-output greedy selection.
                    auto mo = multi_output_two_level_minimize(on_list, dc_list, var_count, opt);
                    ::std::vector<::std::vector<qm_implicant>> chosen = base_covers;
                    std::size_t chosen_cost = base_cost;
                    if(mo.cost != static_cast<std::size_t>(-1) && mo.cost < chosen_cost)
                    {
                        chosen = ::std::move(mo.covers);
                        chosen_cost = mo.cost;
                    }

                    // Partial sharing beyond identical cubes: extract a shared subcube (size 2..K) across distinct cubes.
                    struct lit
                    {
                        std::size_t v{};
                        bool neg{};
                    };

                    constexpr std::size_t max_shared_literals = 3u;
                    auto const var_mask = static_cast<std::uint16_t>((var_count >= 16) ? 0xFFFFu : ((1u << var_count) - 1u));
                    auto popcount16 = [](std::uint16_t x) noexcept -> std::size_t
                    { return static_cast<std::size_t>(__builtin_popcount(static_cast<unsigned>(x))); };

                    auto cube_lits = [&](qm_implicant const& imp) noexcept -> ::std::vector<lit>
                    {
                        ::std::vector<lit> lits{};
                        for(std::size_t v{}; v < var_count; ++v)
                        {
                            if((imp.mask >> v) & 1u) { continue; }
                            bool const bit_is_1 = ((imp.value >> v) & 1u) != 0;
                            lits.push_back(lit{v, !bit_is_1});
                        }
                        ::std::sort(lits.begin(),
                                    lits.end(),
                                    [](lit const& a, lit const& b) noexcept
                                    {
                                        if(a.v != b.v) { return a.v < b.v; }
                                        return a.neg < b.neg;
                                    });
                        return lits;
                    };

                    ::std::unordered_map<cube_key, qm_implicant, cube_key_hash, cube_key_eq> uniq_cubes{};
                    uniq_cubes.reserve(256);
                    for(auto const& cv: chosen)
                    {
                        for(auto const& imp: cv) { (void)uniq_cubes.emplace(to_cube_key(imp), imp); }
                    }

                    ::std::unordered_map<cube_key, std::size_t, cube_key_hash, cube_key_eq> subcube_count{};
                    subcube_count.reserve(512);
                    auto add_subcube = [&](lit const* lits, std::size_t count) noexcept
                    {
                        std::uint16_t mask = var_mask;
                        std::uint16_t value{};
                        for(std::size_t i{}; i < count; ++i)
                        {
                            auto const bit = static_cast<std::uint16_t>(1u << lits[i].v);
                            mask = static_cast<std::uint16_t>(mask & static_cast<std::uint16_t>(~bit));
                            if(!lits[i].neg) { value = static_cast<std::uint16_t>(value | bit); }
                        }
                        ++subcube_count[cube_key{mask, value}];
                    };

                    for(auto const& kv: uniq_cubes)
                    {
                        auto const& imp = kv.second;
                        auto lits = cube_lits(imp);
                        if(lits.size() < 2u) { continue; }
                        for(std::size_t i{}; i + 1 < lits.size(); ++i)
                        {
                            for(std::size_t j{i + 1}; j < lits.size(); ++j)
                            {
                                lit pair[2]{lits[i], lits[j]};
                                add_subcube(pair, 2u);
                            }
                        }
                        if(max_shared_literals >= 3u && lits.size() >= 3u)
                        {
                            for(std::size_t i{}; i + 2 < lits.size(); ++i)
                            {
                                for(std::size_t j{i + 1}; j + 1 < lits.size(); ++j)
                                {
                                    for(std::size_t k{j + 1}; k < lits.size(); ++k)
                                    {
                                        lit trip[3]{lits[i], lits[j], lits[k]};
                                        add_subcube(trip, 3u);
                                    }
                                }
                            }
                        }
                    }

                    cube_key best_shared{};
                    std::size_t best_gain{};
                    std::size_t best_cnt{};
                    std::size_t best_size{};
                    for(auto const& kv: subcube_count)
                    {
                        auto const cnt = kv.second;
                        if(cnt < 2u) { continue; }
                        auto const keep = static_cast<std::uint16_t>(~kv.first.mask & var_mask);
                        auto const size = popcount16(keep);
                        if(size < 2u || size > max_shared_literals) { continue; }

                        std::size_t gain{};
                        if(opt.two_level_cost == pe_synth_options::two_level_cost_model::literal_count) { gain = (cnt - 1u) * size; }
                        else
                        {
                            gain = (cnt - 1u) * (size - 1u) * static_cast<std::size_t>(opt.two_level_weights.and_w);
                        }
                        if(gain > best_gain || (gain == best_gain && (size > best_size || (size == best_size && cnt > best_cnt))))
                        {
                            best_gain = gain;
                            best_cnt = cnt;
                            best_size = size;
                            best_shared = kv.first;
                        }
                    }

                    bool use_shared_subcube = (best_gain > 0u);
                    if(use_shared_subcube) { chosen_cost = (chosen_cost > best_gain) ? (chosen_cost - best_gain) : 0u; }

                    // Include the required YES buffers for protected roots in the improvement check.
                    std::size_t old_models{};
                    for(auto const& c: grp.cones) { old_models += c.to_delete.size(); }
                    auto const est_new_models = chosen_cost + grp.cones.size();  // +YES per output root
                    if(est_new_models >= old_models) { continue; }

                    // Delete old cones.
                    for(auto const& c: grp.cones)
                    {
                        for(auto const& mp: c.to_delete) { (void)::phy_engine::netlist::delete_model(nl, mp); }
                    }

                    // Shared caches across outputs in the group.
                    auto* const0 = find_or_make_const_node(nl, ::phy_engine::model::digital_node_statement_t::false_state);
                    auto* const1 = find_or_make_const_node(nl, ::phy_engine::model::digital_node_statement_t::true_state);
                    if(const0 == nullptr || const1 == nullptr) { continue; }

                    ::std::vector<::phy_engine::model::node_t*> neg{};
                    neg.assign(var_count, nullptr);
                    ::std::unordered_map<cube_key, ::phy_engine::model::node_t*, cube_key_hash, cube_key_eq> term_cache{};
                    term_cache.reserve(256);

                    auto ensure_neg = [&](std::size_t v) noexcept -> ::phy_engine::model::node_t*
                    {
                        if(v >= var_count) { return nullptr; }
                        if(neg[v] != nullptr) { return neg[v]; }
                        neg[v] = make_not(grp.leaves[v]);
                        return neg[v];
                    };

                    auto const shared_keep = static_cast<std::uint16_t>(~best_shared.mask & var_mask);
                    auto const shared_value = static_cast<std::uint16_t>(best_shared.value & shared_keep);
                    auto const shared_size = popcount16(shared_keep);
                    ::phy_engine::model::node_t* shared_node = nullptr;

                    auto shared_in_cube = [&](qm_implicant const& imp) noexcept -> bool
                    {
                        if(!use_shared_subcube) { return false; }
                        if((imp.mask & shared_keep) != 0u) { return false; }
                        if(((imp.value ^ shared_value) & shared_keep) != 0u) { return false; }
                        return true;
                    };

                    auto build_shared_node = [&]() noexcept -> ::phy_engine::model::node_t*
                    {
                        if(!use_shared_subcube) { return nullptr; }
                        if(shared_node != nullptr) { return shared_node; }
                        ::phy_engine::model::node_t* term = nullptr;
                        for(std::size_t v{}; v < var_count; ++v)
                        {
                            if(((shared_keep >> v) & 1u) == 0u) { continue; }
                            bool const bit_is_1 = ((shared_value >> v) & 1u) != 0;
                            auto* lit = bit_is_1 ? grp.leaves[v] : ensure_neg(v);
                            if(lit == nullptr) { return nullptr; }
                            if(term == nullptr) { term = lit; }
                            else
                            {
                                term = make_and(term, lit);
                            }
                            if(term == nullptr) { return nullptr; }
                        }
                        shared_node = term;
                        return shared_node;
                    };

                    auto build_term = [&](qm_implicant const& imp) noexcept -> ::phy_engine::model::node_t*
                    {
                        auto const key = to_cube_key(imp);
                        if(auto it = term_cache.find(key); it != term_cache.end()) { return it->second; }

                        ::phy_engine::model::node_t* term = nullptr;
                        std::size_t lits{};
                        bool used_shared{};
                        if(shared_in_cube(imp))
                        {
                            term = build_shared_node();
                            if(term == nullptr) { return nullptr; }
                            used_shared = true;
                            lits = shared_size;
                        }
                        for(std::size_t v{}; v < var_count; ++v)
                        {
                            if((imp.mask >> v) & 1u) { continue; }
                            if(used_shared && ((shared_keep >> v) & 1u)) { continue; }
                            bool const bit_is_1 = ((imp.value >> v) & 1u) != 0;
                            auto* lit = bit_is_1 ? grp.leaves[v] : ensure_neg(v);
                            if(lit == nullptr) { return nullptr; }
                            ++lits;
                            if(term == nullptr) { term = lit; }
                            else
                            {
                                term = make_and(term, lit);
                            }
                            if(term == nullptr) { return nullptr; }
                        }
                        if(lits == 0 && !used_shared) { term = const1; }
                        if(term == nullptr) { return nullptr; }
                        term_cache.emplace(key, term);
                        return term;
                    };

                    auto build_cover_to = [&](::std::vector<qm_implicant> const& cover, ::phy_engine::model::node_t* root) noexcept -> bool
                    {
                        if(root == nullptr) { return false; }
                        if(cover.empty()) { return make_yes(const0, root); }
                        if(cover.size() == 1u && cube_literals(cover[0], var_count) == 0u) { return make_yes(const1, root); }

                        ::std::vector<::phy_engine::model::node_t*> terms{};
                        terms.reserve(cover.size());
                        for(auto const& imp: cover)
                        {
                            auto* t = build_term(imp);
                            if(t == nullptr) { return false; }
                            terms.push_back(t);
                        }
                        if(terms.empty()) { return make_yes(const0, root); }

                        ::phy_engine::model::node_t* out = terms[0];
                        for(std::size_t i{1}; i < terms.size(); ++i)
                        {
                            out = make_or(out, terms[i]);
                            if(out == nullptr) { return false; }
                        }
                        return make_yes(out, root);
                    };

                    for(std::size_t oi{}; oi < grp.cones.size() && oi < chosen.size(); ++oi) { (void)build_cover_to(chosen[oi], grp.cones[oi].root); }

                    changed = true;
                }
            }

            for(auto* root: roots)
            {
                if(root == nullptr) { continue; }
                auto it_drv = fan.driver_count.find(root);
                if(it_drv == fan.driver_count.end() || it_drv->second != 1) { continue; }
                auto it_root = gate_by_out.find(root);
                if(it_root == gate_by_out.end()) { continue; }

                // Ensure the root gate still exists (this pass deletes cones as it goes; we use a snapshot map).
                {
                    auto const& g = it_root->second;
                    auto* mb = ::phy_engine::netlist::get_model(nl, g.pos);
                    if(mb == nullptr || mb->type != ::phy_engine::model::model_type::normal || mb->ptr == nullptr) { continue; }
                    auto const nm = model_name_u8(*mb);
                    if((g.k == gkind::not_gate && nm != u8"NOT") || (g.k == gkind::and_gate && nm != u8"AND") || (g.k == gkind::or_gate && nm != u8"OR") ||
                       (g.k == gkind::xor_gate && nm != u8"XOR") || (g.k == gkind::xnor_gate && nm != u8"XNOR") ||
                       (g.k == gkind::nand_gate && nm != u8"NAND") || (g.k == gkind::nor_gate && nm != u8"NOR") || (g.k == gkind::imp_gate && nm != u8"IMP") ||
                       (g.k == gkind::nimp_gate && nm != u8"NIMP"))
                    {
                        continue;
                    }
                }

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
                        if(const_val.contains(n)) { return; }
                        if(!leaf_index.contains(n))
                        {
                            if(leaves.size() >= max_vars)
                            {
                                ok = false;
                                return;
                            }
                            leaf_index.emplace(n, leaves.size());
                            leaves.push_back(n);
                        }
                        return;
                    }

                    auto const& g = itg->second;
                    if(n != root)
                    {
                        if(is_protected(n))
                        {
                            ok = false;
                            return;
                        }
                        auto itc = fan.consumer_count.find(n);
                        if(itc == fan.consumer_count.end() || itc->second != 1)
                        {
                            ok = false;
                            return;
                        }
                        auto itd = fan.driver_count.find(n);
                        if(itd == fan.driver_count.end() || itd->second != 1)
                        {
                            ok = false;
                            return;
                        }
                    }

                    if(to_delete.size() >= max_gates)
                    {
                        ok = false;
                        return;
                    }
                    to_delete.push_back(g.pos);

                    self(self, g.in0);
                    if(g.k != gkind::not_gate) { self(self, g.in1); }
                };

                dfs(dfs, root);
                if(!ok) { continue; }
                if(leaves.empty()) { continue; }
                auto const odc_conditions = build_odc_conditions(root, leaf_index);

                // Build ON/DC sets by truth-table enumeration on leaf vars (binary only).
                ::std::vector<::std::uint16_t> on{};
                ::std::vector<::std::uint16_t> dc{};
                if(!eval_binary_cone_on_dc(root, leaves, on, dc))
                {
                    // Slow fallback: per-minterm evaluation with 4-valued semantics (preserves X/Z -> DC inference).
                    on.reserve(1u << leaves.size());
                    for(::std::uint16_t m{}; m < static_cast<::std::uint16_t>(1u << leaves.size()); ++m)
                    {
                        ::std::unordered_map<::phy_engine::model::node_t*, ::phy_engine::model::digital_node_statement_t> leaf_val{};
                        leaf_val.reserve(leaves.size() * 2u);
                        for(std::size_t i{}; i < leaves.size(); ++i)
                        {
                            auto const bit = ((m >> i) & 1u) != 0;
                            leaf_val.emplace(leaves[i],
                                             bit ? ::phy_engine::model::digital_node_statement_t::true_state
                                                 : ::phy_engine::model::digital_node_statement_t::false_state);
                        }
                        ::std::unordered_map<::phy_engine::model::node_t*, ::phy_engine::model::digital_node_statement_t> memo{};
                        memo.reserve(to_delete.size() * 2u + 8u);
                        ::std::unordered_map<::phy_engine::model::node_t*, bool> visiting{};
                        visiting.reserve(to_delete.size() * 2u + 8u);
                        auto const r = eval_gate(eval_gate, root, leaf_val, memo, visiting);
                        if(r == ::phy_engine::model::digital_node_statement_t::true_state) { on.push_back(m); }
                        else if(r == ::phy_engine::model::digital_node_statement_t::false_state) { /* off */ }
                        else
                        {
                            if(opt.assume_binary_inputs && opt.infer_dc_from_xz) { dc.push_back(m); }
                            else
                            {
                                ok = false;
                                break;
                            }
                        }
                    }
                    if(!ok) { continue; }
                }

                if(opt.infer_dc_from_fsm)
                {
                    auto const views = build_dc_group_views(leaves);
                    if(!views.empty()) { apply_dc_constraints(dc, on, leaves.size(), views); }
                }
                if(!odc_conditions.empty()) { apply_odc_conditions(on, dc, leaves.size(), odc_conditions); }

                auto const primes = qm_prime_implicants(on, dc, leaves.size());
                if(primes.size() > max_primes) { continue; }
                qm_solution qm_sol{};
                if(leaves.size() <= exact_vars) { qm_sol = qm_exact_minimum_cover(primes, on, leaves.size(), opt); }
                else
                {
                    qm_sol = qm_greedy_cover(primes, on, leaves.size(), opt);
                    if(primes.size() <= 64u && on.size() <= 64u)
                    {
                        auto const pet = qm_petrick_minimum_cover(primes, on, leaves.size(), opt);
                        if(pet.cost != static_cast<std::size_t>(-1) && (qm_sol.cost == static_cast<std::size_t>(-1) || pet.cost < qm_sol.cost))
                        {
                            qm_sol = pet;
                        }
                    }
                }
                if(qm_sol.cost == static_cast<std::size_t>(-1)) { continue; }

                enum class best_kind : std::uint8_t
                {
                    qm,
                    espresso,
                };
                best_kind best{best_kind::qm};
                std::size_t best_cost{qm_sol.cost};
                espresso_solution esp{};

                // Espresso-style heuristic (still bounded by enumeration). Skip tiny cases where QM exact cover is already optimal.
                if(leaves.size() > exact_vars)
                {
                    esp = espresso_two_level_minimize(on, dc, leaves.size(), opt);
                    if(esp.cost != static_cast<std::size_t>(-1) && esp.cost < best_cost)
                    {
                        best = best_kind::espresso;
                        best_cost = esp.cost;
                    }
                }

                // Normalize deletion list and only apply if we estimate a gate-count win.
                ::std::sort(to_delete.begin(),
                            to_delete.end(),
                            [](auto const& a, auto const& b) noexcept
                            {
                                if(a.chunk_pos != b.chunk_pos) { return a.chunk_pos > b.chunk_pos; }
                                return a.vec_pos > b.vec_pos;
                            });
                to_delete.erase(::std::unique(to_delete.begin(),
                                              to_delete.end(),
                                              [](auto const& a, auto const& b) noexcept { return a.chunk_pos == b.chunk_pos && a.vec_pos == b.vec_pos; }),
                                to_delete.end());

                if(best_cost >= to_delete.size()) { continue; }

                // Delete old cone (descending order).
                for(auto const& mp: to_delete) { (void)::phy_engine::netlist::delete_model(nl, mp); }

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
                ::std::vector<::phy_engine::model::node_t*> neg{};
                neg.assign(leaves.size(), nullptr);
                auto ensure_neg_for = [&](qm_implicant const& imp) noexcept
                {
                    for(std::size_t v{}; v < leaves.size(); ++v)
                    {
                        if((imp.mask >> v) & 1u) { continue; }
                        bool const bit_is_1 = ((imp.value >> v) & 1u) != 0;
                        if(bit_is_1) { continue; }
                        if(neg[v] == nullptr) { neg[v] = make_not(leaves[v]); }
                    }
                };
                if(best == best_kind::qm)
                {
                    for(auto const pi: qm_sol.pick)
                    {
                        if(pi >= primes.size()) { continue; }
                        ensure_neg_for(primes[pi]);
                    }
                }
                else
                {
                    for(auto const& imp: esp.cover) { ensure_neg_for(imp); }
                }

                ::std::vector<::phy_engine::model::node_t*> terms{};
                terms.reserve(best == best_kind::qm ? qm_sol.pick.size() : esp.cover.size());
                auto add_term_from = [&](qm_implicant const& imp) noexcept -> void
                {
                    ::phy_engine::model::node_t* term = nullptr;
                    std::size_t lits{};
                    for(std::size_t v{}; v < leaves.size(); ++v)
                    {
                        if((imp.mask >> v) & 1u) { continue; }
                        bool const bit_is_1 = ((imp.value >> v) & 1u) != 0;
                        auto* lit = bit_is_1 ? leaves[v] : neg[v];
                        if(lit == nullptr)
                        {
                            ok = false;
                            break;
                        }
                        ++lits;
                        if(term == nullptr) { term = lit; }
                        else
                        {
                            term = make_and(term, lit);
                        }
                    }
                    if(!ok) { return; }
                    if(lits == 0) { term = const1; }  // covers all
                    if(term != nullptr) { terms.push_back(term); }
                };
                if(best == best_kind::qm)
                {
                    for(auto const pi: qm_sol.pick)
                    {
                        if(pi >= primes.size()) { continue; }
                        add_term_from(primes[pi]);
                        if(!ok) { break; }
                    }
                }
                else
                {
                    for(auto const& imp: esp.cover)
                    {
                        add_term_from(imp);
                        if(!ok) { break; }
                    }
                }
                if(!ok || terms.empty()) { continue; }

                ::phy_engine::model::node_t* out = terms[0];
                for(std::size_t i{1}; i < terms.size(); ++i)
                {
                    out = make_or(out, terms[i]);
                    if(out == nullptr)
                    {
                        ok = false;
                        break;
                    }
                }
                if(!ok || out == nullptr) { continue; }

                if(best == best_kind::espresso && esp.complemented)
                {
                    auto* inv = make_not(out);
                    if(inv == nullptr) { continue; }
                    out = inv;
                }

                if(out == root)
                {
                    changed = true;
                    continue;
                }

                // Keep protected port nodes intact: use YES buffer if output is not directly driven by a newly created gate.
                // Otherwise, connect final output by a YES to ensure root has a driver.
                if(is_protected(root)) { (void)make_yes(out, root); }
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
            [[nodiscard]] ::phy_engine::model::node_t*
                expr_in_env(::std::size_t root, ::std::vector<::phy_engine::model::node_t*> const& env, ::std::vector<bool> const* use_env) noexcept;
        };

        struct synth_context
        {
            ::phy_engine::netlist::netlist& nl;
            pe_synth_options opt{};
            pe_synth_error* err{};
            bool failed{};
            dc_constraints dc{};

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
                    if(av == ::phy_engine::verilog::digital::logic_t::false_state) { return const_node(::phy_engine::verilog::digital::logic_t::false_state); }
                    if(av == ::phy_engine::verilog::digital::logic_t::true_state) { return b; }
                    if(bconst) { return const_node(::phy_engine::verilog::digital::logic_and(av, bv)); }
                }
                if(bconst)
                {
                    bv = ::phy_engine::verilog::digital::normalize_z_to_x(bv);
                    if(bv == ::phy_engine::verilog::digital::logic_t::false_state) { return const_node(::phy_engine::verilog::digital::logic_t::false_state); }
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
                if(aconst && bconst) { return const_node(::phy_engine::verilog::digital::logic_not(::phy_engine::verilog::digital::logic_xor(av, bv))); }
                if(aconst)
                {
                    av = ::phy_engine::verilog::digital::normalize_z_to_x(av);
                    if(av == ::phy_engine::verilog::digital::logic_t::false_state) { return gate_not(b); }
                    if(av == ::phy_engine::verilog::digital::logic_t::true_state) { return b; }
                    if(bconst) { return const_node(::phy_engine::verilog::digital::logic_not(::phy_engine::verilog::digital::logic_xor(av, bv))); }
                }
                if(bconst)
                {
                    bv = ::phy_engine::verilog::digital::normalize_z_to_x(bv);
                    if(bv == ::phy_engine::verilog::digital::logic_t::false_state) { return gate_not(a); }
                    if(bv == ::phy_engine::verilog::digital::logic_t::true_state) { return a; }
                    if(aconst) { return const_node(::phy_engine::verilog::digital::logic_not(::phy_engine::verilog::digital::logic_xor(av, bv))); }
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
                auto const key = (static_cast<::std::uint64_t>(ticks) << 32) ^ (static_cast<::std::uint64_t>(reinterpret_cast<::std::uintptr_t>(in)) >> 4);
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
            if(m == nullptr || root >= m->expr_nodes.size()) { return ctx.const_node(::phy_engine::verilog::digital::logic_t::indeterminate_state); }

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

        struct vector_assign_info
        {
            vector_desc const* desc{};
            bool bad{};
            ::std::vector<::std::uint16_t> patterns{};
        };

        inline void collect_vector_assignments(instance_builder& b,
                                               ::fast_io::vector<stmt_node> const& arena,
                                               ::std::size_t stmt_idx,
                                               ::std::vector<vector_assign_info>& vecs,
                                               ::std::vector<std::size_t> const& vec_by_signal) noexcept
        {
            if(stmt_idx >= arena.size()) { return; }
            auto const& n = arena.index_unchecked(stmt_idx);
            switch(n.k)
            {
                case stmt_node::kind::blocking_assign:
                case stmt_node::kind::nonblocking_assign:
                {
                    if(n.lhs_signal < vec_by_signal.size())
                    {
                        auto const vid = vec_by_signal[n.lhs_signal];
                        if(vid != SIZE_MAX && vid < vecs.size()) { vecs[vid].bad = true; }
                    }
                    return;
                }
                case stmt_node::kind::blocking_assign_vec:
                case stmt_node::kind::nonblocking_assign_vec:
                {
                    if(n.lhs_signals.size() != n.rhs_roots.size() || n.lhs_signals.empty()) { return; }
                    std::size_t vec_id = SIZE_MAX;
                    bool ok{true};
                    for(auto const sig: n.lhs_signals)
                    {
                        if(sig >= vec_by_signal.size())
                        {
                            ok = false;
                            break;
                        }
                        auto const vid = vec_by_signal[sig];
                        if(vid == SIZE_MAX)
                        {
                            ok = false;
                            break;
                        }
                        if(vec_id == SIZE_MAX) { vec_id = vid; }
                        else if(vec_id != vid)
                        {
                            ok = false;
                            break;
                        }
                    }
                    if(!ok || vec_id == SIZE_MAX || vec_id >= vecs.size())
                    {
                        for(auto const sig: n.lhs_signals)
                        {
                            if(sig < vec_by_signal.size())
                            {
                                auto const vid = vec_by_signal[sig];
                                if(vid != SIZE_MAX && vid < vecs.size()) { vecs[vid].bad = true; }
                            }
                        }
                        return;
                    }

                    auto* desc = vecs[vec_id].desc;
                    if(desc == nullptr || desc->bits.size() != n.lhs_signals.size())
                    {
                        vecs[vec_id].bad = true;
                        return;
                    }
                    for(std::size_t i{}; i < desc->bits.size(); ++i)
                    {
                        if(desc->bits.index_unchecked(i) != n.lhs_signals.index_unchecked(i))
                        {
                            vecs[vec_id].bad = true;
                            return;
                        }
                    }

                    ::std::uint16_t pattern{};
                    for(std::size_t i{}; i < n.rhs_roots.size(); ++i)
                    {
                        ::phy_engine::verilog::digital::logic_t v{};
                        if(!eval_const_expr_to_logic(b, n.rhs_roots.index_unchecked(i), v))
                        {
                            vecs[vec_id].bad = true;
                            return;
                        }
                        if(v == ::phy_engine::verilog::digital::logic_t::true_state) { pattern = static_cast<::std::uint16_t>(pattern | (1u << i)); }
                        else if(v != ::phy_engine::verilog::digital::logic_t::false_state)
                        {
                            vecs[vec_id].bad = true;
                            return;
                        }
                    }
                    vecs[vec_id].patterns.push_back(pattern);
                    return;
                }
                case stmt_node::kind::block:
                case stmt_node::kind::subprogram_block:
                {
                    for(auto const s: n.stmts) { collect_vector_assignments(b, arena, s, vecs, vec_by_signal); }
                    return;
                }
                case stmt_node::kind::if_stmt:
                {
                    for(auto const s: n.stmts) { collect_vector_assignments(b, arena, s, vecs, vec_by_signal); }
                    for(auto const s: n.else_stmts) { collect_vector_assignments(b, arena, s, vecs, vec_by_signal); }
                    return;
                }
                case stmt_node::kind::case_stmt:
                {
                    for(auto const& ci: n.case_items)
                    {
                        for(auto const s: ci.stmts) { collect_vector_assignments(b, arena, s, vecs, vec_by_signal); }
                    }
                    return;
                }
                case stmt_node::kind::for_stmt:
                {
                    if(n.init_stmt != SIZE_MAX) { collect_vector_assignments(b, arena, n.init_stmt, vecs, vec_by_signal); }
                    if(n.step_stmt != SIZE_MAX) { collect_vector_assignments(b, arena, n.step_stmt, vecs, vec_by_signal); }
                    if(n.body_stmt != SIZE_MAX) { collect_vector_assignments(b, arena, n.body_stmt, vecs, vec_by_signal); }
                    return;
                }
                case stmt_node::kind::while_stmt:
                case stmt_node::kind::do_while_stmt:
                {
                    if(n.body_stmt != SIZE_MAX) { collect_vector_assignments(b, arena, n.body_stmt, vecs, vec_by_signal); }
                    return;
                }
                default: return;
            }
        }

        inline void infer_one_hot_fsm_groups(instance_builder& b) noexcept
        {
            if(!b.ctx.opt.infer_dc_from_fsm || b.ctx.opt.dc_fsm_max_bits == 0) { return; }
            if(b.inst.mod == nullptr) { return; }
            auto const& m = *b.inst.mod;
            if(m.vectors.empty() || m.always_ffs.empty()) { return; }

            ::std::vector<vector_assign_info> vecs{};
            vecs.reserve(m.vectors.size());
            ::std::vector<std::size_t> vec_by_signal{};
            vec_by_signal.assign(m.signal_names.size(), SIZE_MAX);

            for(auto const& kv: m.vectors)
            {
                auto const& vd = kv.second;
                auto const w = vd.bits.size();
                if(w == 0 || w > b.ctx.opt.dc_fsm_max_bits || w > 16u) { continue; }
                auto const vid = vecs.size();
                vecs.push_back(vector_assign_info{.desc = __builtin_addressof(vd), .bad = false, .patterns = {}});
                for(auto const sig: vd.bits)
                {
                    if(sig < vec_by_signal.size()) { vec_by_signal[sig] = vid; }
                }
            }
            if(vecs.empty()) { return; }

            for(auto const& ff: m.always_ffs)
            {
                for(auto const root: ff.roots) { collect_vector_assignments(b, ff.stmt_nodes, root, vecs, vec_by_signal); }
            }

            auto add_group = [&](dc_group&& g) noexcept
            {
                for(auto const& existing: b.ctx.dc.groups)
                {
                    if(existing.nodes == g.nodes) { return; }
                }
                b.ctx.dc.groups.push_back(::std::move(g));
            };

            for(auto& v: vecs)
            {
                if(v.bad || v.patterns.empty() || v.desc == nullptr) { continue; }
                auto const w = v.desc->bits.size();
                if(w == 0 || w > 16u) { continue; }
                bool one_hot{true};
                for(auto const pat: v.patterns)
                {
                    if(__builtin_popcount(static_cast<unsigned>(pat)) != 1u)
                    {
                        one_hot = false;
                        break;
                    }
                }
                if(!one_hot) { continue; }

                ::std::sort(v.patterns.begin(), v.patterns.end());
                v.patterns.erase(::std::unique(v.patterns.begin(), v.patterns.end()), v.patterns.end());

                dc_group g{};
                g.nodes.reserve(w);
                bool ok{true};
                for(auto const sig: v.desc->bits)
                {
                    if(sig >= b.sig_nodes.size())
                    {
                        ok = false;
                        break;
                    }
                    auto* n = b.sig_nodes[sig];
                    if(n == nullptr)
                    {
                        ok = false;
                        break;
                    }
                    g.nodes.push_back(n);
                }
                if(!ok) { continue; }
                g.allowed = v.patterns;
                add_group(::std::move(g));
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
                case stmt_node::kind::subprogram_block:
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
                case stmt_node::kind::do_while_stmt:
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
                case stmt_node::kind::subprogram_block:
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
                    if(n.case_expr_roots.empty()) { key_bits[0] = b.ctx.const_node(::phy_engine::verilog::digital::logic_t::indeterminate_state); }
                    else
                    {
                        for(::std::size_t i{}; i < w; ++i) { key_bits[i] = b.expr_in_env(n.case_expr_roots.index_unchecked(i), base_cur, nullptr); }
                    }

                    auto* z = b.ctx.const_node(::phy_engine::verilog::digital::logic_t::high_impedence_state);

                    auto bit_match = [&](::phy_engine::model::node_t* a, ::phy_engine::model::node_t* bb) noexcept -> ::phy_engine::model::node_t*
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
                        else
                        {
                            items.push_back(__builtin_addressof(ci));
                        }
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
                case stmt_node::kind::do_while_stmt:
                {
                    auto const max_iter{b.ctx.opt.loop_unroll_limit};
                    if(max_iter == 0)
                    {
                        b.ctx.set_error("pe_synth: loop_unroll_limit is 0 (loops disabled)");
                        return false;
                    }

                    // Execute the body at least once.
                    if(n.body_stmt != SIZE_MAX)
                    {
                        if(!synth_stmt_ff(b, arena, n.body_stmt, cur, next, targets)) { return false; }
                    }

                    // Then behave like a bounded while loop.
                    for(::std::size_t iter{1}; iter < max_iter; ++iter)
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
                case stmt_node::kind::return_stmt:
                case stmt_node::kind::break_stmt:
                case stmt_node::kind::continue_stmt:
                {
                    // Best-effort: these control-flow statements are accepted, but full early-exit semantics
                    // are not modeled in the sequential subset.
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
                case stmt_node::kind::subprogram_block:
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
                    if(n.case_expr_roots.empty()) { key_bits[0] = b.ctx.const_node(::phy_engine::verilog::digital::logic_t::indeterminate_state); }
                    else
                    {
                        for(::std::size_t i{}; i < w; ++i) { key_bits[i] = b.expr_in_env(n.case_expr_roots.index_unchecked(i), base_value, nullptr); }
                    }

                    auto* z = b.ctx.const_node(::phy_engine::verilog::digital::logic_t::high_impedence_state);

                    auto bit_match = [&](::phy_engine::model::node_t* a, ::phy_engine::model::node_t* bb) noexcept -> ::phy_engine::model::node_t*
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
                        else
                        {
                            items.push_back(__builtin_addressof(ci));
                        }
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
                case stmt_node::kind::do_while_stmt:
                {
                    auto const max_iter{b.ctx.opt.loop_unroll_limit};
                    if(max_iter == 0)
                    {
                        b.ctx.set_error("pe_synth: loop_unroll_limit is 0 (loops disabled)");
                        return false;
                    }

                    // Execute the body at least once.
                    if(n.body_stmt != SIZE_MAX)
                    {
                        if(!synth_stmt_comb(b, arena, n.body_stmt, value, assigned_cond, targets)) { return false; }
                    }

                    for(::std::size_t iter{1}; iter < max_iter; ++iter)
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
                case stmt_node::kind::return_stmt:
                case stmt_node::kind::break_stmt:
                case stmt_node::kind::continue_stmt:
                {
                    // Best-effort: accept these statements so inlined SV functions using `subprogram_block`
                    // can synthesize. Full early-exit semantics are not modeled.
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

            infer_one_hot_fsm_groups(b);

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
            for(auto const& d: inst.output_drives)
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
                    if(d.src_is_literal) { src = const_node(d.literal); }
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

            // Heuristic: avoid materializing drivers (YES/DLATCH) for purely-local temporary signals that are:
            // - written in an always_comb, and
            // - only read within the same always block.
            //
            // This massively reduces netlist size for datapath-heavy SV that uses many block-scoped temporaries
            // (including the inlined function-call always_comb blocks generated by the frontend).
            //
            // Correctness note: we still model the temporaries internally via `value[...]` substitution when building
            // expression DAGs; we just avoid emitting external drivers for temps that are not observed elsewhere.
            ::std::vector<bool> port_out{};
            port_out.assign(m.signal_names.size(), false);
            for(auto const& p : m.ports)
            {
                if(p.signal >= port_out.size()) { continue; }
                if(p.dir == ::phy_engine::verilog::digital::port_dir::output || p.dir == ::phy_engine::verilog::digital::port_dir::inout) { port_out[p.signal] = true; }
            }

            ::std::vector<bool> assigns_read{};
            assigns_read.assign(m.signal_names.size(), false);

            ::std::vector<::std::uint32_t> read_block_count{};
            read_block_count.assign(m.signal_names.size(), 0u);

            ::std::vector<::std::uint32_t> expr_seen{};
            expr_seen.assign(m.expr_nodes.size(), 0u);
            ::std::vector<::std::uint32_t> sig_seen{};
            sig_seen.assign(m.signal_names.size(), 0u);
            ::std::uint32_t stamp{1u};

            auto collect_expr_reads = [&](::std::size_t root, ::std::uint32_t cur_stamp, ::std::vector<::std::size_t>& out_sigs) noexcept
            {
                if(root == SIZE_MAX || root >= m.expr_nodes.size()) { return; }
                ::std::vector<::std::size_t> st{};
                st.reserve(64);
                st.push_back(root);
                while(!st.empty())
                {
                    auto const r = st.back();
                    st.pop_back();
                    if(r == SIZE_MAX || r >= m.expr_nodes.size()) { continue; }
                    if(expr_seen[r] == cur_stamp) { continue; }
                    expr_seen[r] = cur_stamp;
                    auto const& en = m.expr_nodes.index_unchecked(r);
                    using ek = ::phy_engine::verilog::digital::expr_kind;
                    if(en.kind == ek::signal)
                    {
                        auto const s = en.signal;
                        if(s < sig_seen.size() && sig_seen[s] != cur_stamp)
                        {
                            sig_seen[s] = cur_stamp;
                            out_sigs.push_back(s);
                        }
                        continue;
                    }
                    if(en.a != SIZE_MAX) { st.push_back(en.a); }
                    if(en.b != SIZE_MAX) { st.push_back(en.b); }
                }
            };

            auto collect_stmt_reads = [&](auto&& self,
                                          ::fast_io::vector<stmt_node> const& arena,
                                          ::std::size_t stmt_idx,
                                          ::std::uint32_t cur_stamp,
                                          ::std::vector<::std::size_t>& out_sigs) noexcept -> void
            {
                if(stmt_idx == SIZE_MAX || stmt_idx >= arena.size()) { return; }
                auto const& sn = arena.index_unchecked(stmt_idx);
                switch(sn.k)
                {
                    case stmt_node::kind::blocking_assign:
                    case stmt_node::kind::nonblocking_assign: collect_expr_reads(sn.expr_root, cur_stamp, out_sigs); break;
                    case stmt_node::kind::blocking_assign_vec:
                    case stmt_node::kind::nonblocking_assign_vec:
                    {
                        for(auto const r : sn.rhs_roots) { collect_expr_reads(r, cur_stamp, out_sigs); }
                        break;
                    }
                    case stmt_node::kind::if_stmt:
                    {
                        collect_expr_reads(sn.expr_root, cur_stamp, out_sigs);
                        for(auto const s : sn.stmts) { self(self, arena, s, cur_stamp, out_sigs); }
                        for(auto const s : sn.else_stmts) { self(self, arena, s, cur_stamp, out_sigs); }
                        break;
                    }
                    case stmt_node::kind::case_stmt:
                    {
                        for(auto const r : sn.case_expr_roots) { collect_expr_reads(r, cur_stamp, out_sigs); }
                        for(auto const& ci : sn.case_items)
                        {
                            for(auto const r : ci.match_roots) { collect_expr_reads(r, cur_stamp, out_sigs); }
                            for(auto const s : ci.stmts) { self(self, arena, s, cur_stamp, out_sigs); }
                        }
                        break;
                    }
                    case stmt_node::kind::block:
                    case stmt_node::kind::subprogram_block:
                    {
                        for(auto const s : sn.stmts) { self(self, arena, s, cur_stamp, out_sigs); }
                        break;
                    }
                    case stmt_node::kind::for_stmt:
                    {
                        if(sn.init_stmt != SIZE_MAX) { self(self, arena, sn.init_stmt, cur_stamp, out_sigs); }
                        if(sn.step_stmt != SIZE_MAX) { self(self, arena, sn.step_stmt, cur_stamp, out_sigs); }
                        collect_expr_reads(sn.expr_root, cur_stamp, out_sigs);
                        if(sn.body_stmt != SIZE_MAX) { self(self, arena, sn.body_stmt, cur_stamp, out_sigs); }
                        break;
                    }
                    case stmt_node::kind::while_stmt:
                    case stmt_node::kind::do_while_stmt:
                    {
                        collect_expr_reads(sn.expr_root, cur_stamp, out_sigs);
                        if(sn.body_stmt != SIZE_MAX) { self(self, arena, sn.body_stmt, cur_stamp, out_sigs); }
                        break;
                    }
                    default: break;
                }
            };

            // Reads from continuous assigns (treated as a separate reader context).
            {
                ::std::vector<::std::size_t> sigs{};
                sigs.reserve(128);
                auto const cur_stamp = ++stamp;
                for(auto const& a : m.assigns)
                {
                    collect_expr_reads(a.expr_root, cur_stamp, sigs);
                    if(a.guard_root != SIZE_MAX) { collect_expr_reads(a.guard_root, cur_stamp, sigs); }
                }
                for(auto const s : sigs)
                {
                    if(s < assigns_read.size()) { assigns_read[s] = true; }
                }
            }

            // Reads from always blocks (counted per-block, not per-reference).
            auto add_reader_block = [&](::fast_io::vector<stmt_node> const& arena, ::fast_io::vector<::std::size_t> const& roots) noexcept
            {
                ::std::vector<::std::size_t> sigs{};
                sigs.reserve(256);
                auto const cur_stamp = ++stamp;
                for(auto const r : roots) { collect_stmt_reads(collect_stmt_reads, arena, r, cur_stamp, sigs); }
                for(auto const s : sigs)
                {
                    if(s < read_block_count.size()) { ++read_block_count[s]; }
                }
            };
            for(auto const& ac : m.always_combs) { add_reader_block(ac.stmt_nodes, ac.roots); }
            for(auto const& ff : m.always_ffs) { add_reader_block(ff.stmt_nodes, ff.roots); }
            for(auto const& ib : m.initials) { add_reader_block(ib.stmt_nodes, ib.roots); }

            // always_comb blocks (restricted subset).
            if(opt.support_always_comb)
            {
                for(auto const& comb: m.always_combs)
                {
                    (void)comb;
                    ::std::vector<bool> targets(m.signal_names.size(), false);
                    for(auto const root: comb.roots) { collect_assigned_signals(comb.stmt_nodes, root, targets); }

                    // Track which signals are read in this always block (for drive pruning).
                    auto const cur_stamp = ++stamp;
                    ::std::vector<::std::size_t> read_sigs{};
                    read_sigs.reserve(256);
                    for(auto const root: comb.roots) { collect_stmt_reads(collect_stmt_reads, comb.stmt_nodes, root, cur_stamp, read_sigs); }

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
                        bool const read_here = (sig < sig_seen.size()) && (sig_seen[sig] == cur_stamp);
                        bool const read_elsewhere =
                            (sig < assigns_read.size() && assigns_read[sig]) ||
                            (sig < read_block_count.size() && read_block_count[sig] > (read_here ? 1u : 0u));
                        bool const drive = (sig < port_out.size() && port_out[sig]) || read_elsewhere;
                        if(!drive) { continue; }
                        auto* lhs = b.sig_nodes[sig];
                        auto* rhs = values[sig];
                        if(lhs == nullptr || rhs == nullptr) { continue; }

                        ::phy_engine::verilog::digital::logic_t av{};
                        bool const always_assigned = try_get_const(assigned_cond[sig], av) && (av == ::phy_engine::verilog::digital::logic_t::true_state);

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
                                if(then_has_reset[sig] != else_has_reset[sig])
                                {
                                    same = false;
                                    break;
                                }
                                if(then_has_reset[sig] && then_reset_values[sig] != else_reset_values[sig])
                                {
                                    same = false;
                                    break;
                                }
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

        auto* rep = (opt.report_enable && opt.report != nullptr) ? opt.report : nullptr;
        if(rep != nullptr)
        {
            rep->passes.clear();
            rep->iter_gate_count.clear();
            rep->omax_best_gate_count.clear();
            rep->omax_best_cost.clear();
            rep->omax_summary.clear();
        }

        auto const raw_lvl = opt.opt_level;
        bool const is_omax = (raw_lvl >= 5u);
        auto const lvl = static_cast<::std::uint8_t>(raw_lvl > 4u ? 4u : raw_lvl);

        bool const do_wires = opt.optimize_wires || (lvl >= 1);
        bool const do_mul2 = opt.optimize_mul2 || (lvl >= 2);
        bool const do_adders = opt.optimize_adders || (lvl >= 2);

        bool const do_fuse_inverters = (lvl >= 1);
        bool const do_strash = (lvl >= 1);
        bool const do_dce = (lvl >= 1);
        bool const do_factoring = (lvl >= 2);
        bool const do_qm = (lvl >= 4);
        bool const do_input_inv_map = (lvl >= 2);
        bool const do_xor_rewrite = (lvl >= 2);
        bool const do_double_not = (lvl >= 1);
        bool const do_constprop = (lvl >= 1);
	        bool const do_absorption = (lvl >= 2);
	        bool const do_flatten = (lvl >= 2);
	        // Binary-only rewrites are not semantics-preserving under X/Z; keep them at O3+ even when
	        // `assume_binary_inputs` is enabled so the optimization levels stay meaningfully distinct.
	        bool const do_binary_simplify = (lvl >= 3) && opt.assume_binary_inputs;
	        bool const do_aig_rewrite = (lvl >= 3) && opt.assume_binary_inputs;
	        bool const do_resub = (lvl >= 3) && opt.assume_binary_inputs;
	        bool const do_sweep = (lvl >= 3) && opt.assume_binary_inputs;
	        bool const do_techmap = (lvl >= 4) && opt.assume_binary_inputs && opt.techmap_enable;
	        bool const do_decompose = (lvl >= 4) && opt.assume_binary_inputs && opt.decompose_large_functions;

        auto run_once = [&](::phy_engine::netlist::netlist& net, pe_synth_options const& opt_run, pe_synth_report* rep_run) noexcept -> void
        {
            bool node_budget_hit{};

            auto run_pass = [&](bool en, ::fast_io::u8string_view name, auto&& fn) noexcept -> void
            {
                if(!en) { return; }
                if(node_budget_hit) { return; }

                if(opt_run.max_total_nodes != 0u && details::count_total_nodes(net) > opt_run.max_total_nodes)
                {
                    node_budget_hit = true;
                    return;
                }

                bool const need_rollback = (opt_run.max_total_models != 0u) || (opt_run.max_total_logic_gates != 0u);
                ::std::optional<::phy_engine::netlist::netlist> snap{};
                if(need_rollback) { snap.emplace(net); }

                auto const before = details::count_logic_gates(net);

                fn();

                if(opt_run.max_total_nodes != 0u && details::count_total_nodes(net) > opt_run.max_total_nodes)
                {
                    // Can't reclaim node storage; stop running subsequent passes to prevent further growth.
                    node_budget_hit = true;
                }

                if(need_rollback)
                {
                    bool const too_many_models = (opt_run.max_total_models != 0u) && (details::count_total_models(net) > opt_run.max_total_models);
                    bool const too_many_gates = (opt_run.max_total_logic_gates != 0u) && (details::count_logic_gates(net) > opt_run.max_total_logic_gates);
                    if((too_many_models || too_many_gates) && snap.has_value()) { details::restore_from_snapshot(net, *snap); }
                }

                if(rep_run != nullptr)
                {
                    auto const after = details::count_logic_gates(net);
                    rep_run->passes.push_back(pe_synth_pass_stat{.pass = name, .before = before, .after = after});
                }
            };

            run_pass(do_wires, u8"wires", [&]() noexcept { details::optimize_eliminate_yes_buffers(net, top_port_nodes); });
            run_pass(do_mul2, u8"mul2", [&]() noexcept { details::optimize_mul2_in_pe_netlist(net); });
            run_pass(do_adders, u8"adders", [&]() noexcept { details::optimize_adders_in_pe_netlist(net); });

            run_pass(do_constprop, u8"constprop", [&]() noexcept { (void)details::optimize_constant_propagation_in_pe_netlist(net, top_port_nodes, opt_run); });
            run_pass(do_factoring, u8"factor", [&]() noexcept { (void)details::optimize_factor_common_terms_in_pe_netlist(net, top_port_nodes); });
            run_pass(do_absorption, u8"absorb", [&]() noexcept { (void)details::optimize_absorption_in_pe_netlist(net, top_port_nodes); });
            run_pass(do_xor_rewrite, u8"xor_rewrite", [&]() noexcept { (void)details::optimize_rewrite_xor_xnor_in_pe_netlist(net, top_port_nodes); });
            run_pass(do_flatten,
                     u8"flatten",
                     [&]() noexcept { (void)details::optimize_flatten_associative_and_or_in_pe_netlist(net, top_port_nodes, opt_run); });
            run_pass(do_binary_simplify,
                     u8"binary_simplify",
                     [&]() noexcept { (void)details::optimize_binary_complement_simplify_in_pe_netlist(net, top_port_nodes); });
            run_pass(do_binary_simplify,
                     u8"constprop2",
                     [&]() noexcept { (void)details::optimize_constant_propagation_in_pe_netlist(net, top_port_nodes, opt_run); });
            run_pass(do_aig_rewrite, u8"aig_rewrite", [&]() noexcept { (void)details::optimize_aig_rewrite_in_pe_netlist(net, top_port_nodes, opt_run); });
            run_pass(do_decompose,
                     u8"bdd_decompose",
                     [&]() noexcept { (void)details::optimize_bdd_decompose_large_cones_in_pe_netlist(net, top_port_nodes, opt_run); });
            run_pass(do_qm,
                     u8"qm_2lvl",
                     [&]() noexcept
                     { (void)details::optimize_qm_two_level_minimize_in_pe_netlist(net, top_port_nodes, opt_run, __builtin_addressof(ctx.dc)); });
            run_pass(do_input_inv_map, u8"push_inv", [&]() noexcept { (void)details::optimize_push_input_inverters_in_pe_netlist(net, top_port_nodes); });
            run_pass(do_double_not, u8"double_not", [&]() noexcept { (void)details::optimize_eliminate_double_not_in_pe_netlist(net, top_port_nodes); });
            run_pass(do_fuse_inverters, u8"fuse_inv", [&]() noexcept { (void)details::optimize_fuse_inverters_in_pe_netlist(net, top_port_nodes); });
            run_pass(do_strash, u8"strash", [&]() noexcept { (void)details::optimize_strash_in_pe_netlist(net, top_port_nodes); });
            run_pass(do_techmap, u8"techmap", [&]() noexcept { (void)details::optimize_cut_based_techmap_in_pe_netlist(net, top_port_nodes, opt_run); });
            run_pass(do_resub, u8"resub", [&]() noexcept { (void)details::optimize_bounded_resubstitute_in_pe_netlist(net, top_port_nodes, opt_run); });
            run_pass(do_sweep, u8"sweep", [&]() noexcept { (void)details::optimize_bounded_sweep_in_pe_netlist(net, top_port_nodes, opt_run); });
            run_pass(do_dce, u8"dce", [&]() noexcept { (void)details::optimize_dce_in_pe_netlist(net, top_port_nodes); });
        };

        auto run_o34_fixpoint = [&](::phy_engine::netlist::netlist& net, pe_synth_options const& opt_run, pe_synth_report* rep_run) noexcept
        {
            if(lvl >= 4)
            {
                // O4: best-effort fixpoint-ish pipeline for gate count reduction.
                for(::std::size_t iter{}; iter < 8u; ++iter)
                {
                    auto const before = details::count_logic_gates(net);
                    if(rep_run != nullptr) { rep_run->iter_gate_count.push_back(before); }
                    run_once(net, opt_run, rep_run);
                    auto const after = details::count_logic_gates(net);
                    if(rep_run != nullptr) { rep_run->iter_gate_count.push_back(after); }
                    if(after >= before) { break; }
                }
            }
            else
            {
                // O0..O3: single pass (O3 is the "fast medium-strength" tier).
                run_once(net, opt_run, rep_run);
            }
        };

        // Small O3 tuning: keep it predictably fast by default.
        pe_synth_options tuned_opt = opt;
        if(lvl == 3u)
        {
            // AIG rewrite can be the most expensive "fast tier" pass; cap it by default unless user already did.
            if(tuned_opt.rewrite_max_candidates == 0u) { tuned_opt.rewrite_max_candidates = 4096u; }
        }

        if(!is_omax) { run_o34_fixpoint(nl, tuned_opt, rep); }
        else
        {
            using clock = ::std::chrono::steady_clock;
            auto const t0 = clock::now();

            auto timed_out = [&]() noexcept -> bool
            {
                if(opt.omax_timeout_ms == 0u) { return false; }
                auto const ms = ::std::chrono::duration_cast<::std::chrono::milliseconds>(clock::now() - t0).count();
                return ms >= static_cast<::std::int64_t>(opt.omax_timeout_ms);
            };

            auto make_try_opt = [&](::std::size_t iter) noexcept -> pe_synth_options
            {
                pe_synth_options o = tuned_opt;
                // Force the O4 pipeline; Omax is the orchestration/search layer.
                if(o.opt_level > 4u) { o.opt_level = 4u; }

                // Iter 0 is the baseline "plain O4" run with exactly the user's knobs.
                // This guarantees -Omax/-Ocuda won't regress vs a single O4 fixpoint for the same options.
                if(iter == 0u) { return o; }

                // Gradually relax small internal heuristics where it is cheap and safe.
                // BDD decomposition can benefit from exploring more variable orders in Omax.
                {
                    std::size_t base = opt.decomp_var_order_tries;
                    if(base < 8u) { base = 8u; }
                    std::size_t target = base + iter * 4u;
                    if(target > 64u) { target = 64u; }
                    if(target > o.decomp_var_order_tries) { o.decomp_var_order_tries = target; }
                }

                // Per-try seed tweak (only affects passes that consult omax_randomize).
                o.omax_rand_seed = opt.omax_rand_seed ^ (0x9e3779b97f4a7c15ull + static_cast<::std::uint64_t>(iter));
                return o;
            };

            // Omax must not invalidate `top_port_nodes` pointers (they are owned by `nl` and expected to remain stable).
            // So we keep node storage in-place and restore models/connections from snapshots without clearing `nl.nodes`.
            auto restore_from_snapshot = [&](::phy_engine::netlist::netlist& dst, ::phy_engine::netlist::netlist const& src) noexcept
            { details::restore_from_snapshot(dst, src); };

            std::size_t tries = opt.omax_max_iter;
            if(tries == 0u) { tries = 1u; }

            ::std::size_t best_cost = details::compute_omax_cost(nl, opt);
            ::std::size_t best_gate = details::count_logic_gates(nl);
            ::phy_engine::netlist::netlist best_snapshot{nl};
            ::phy_engine::netlist::netlist ref_snapshot{nl};

            pe_synth_report best_rep{};
            bool have_best_rep{};

            for(std::size_t iter{}; iter < tries; ++iter)
            {
                if(timed_out()) { break; }

                // By default, always restart from the best-so-far snapshot.
                if(iter != 0u && !opt.omax_allow_regress) { restore_from_snapshot(nl, best_snapshot); }

                pe_synth_report cand_rep{};
                pe_synth_report* cand_rep_ptr = (rep != nullptr) ? __builtin_addressof(cand_rep) : nullptr;

                auto const opt_try = make_try_opt(iter);
                run_o34_fixpoint(nl, opt_try, cand_rep_ptr);

                if(opt.omax_verify)
                {
                    ::fast_io::u8string why{};
                    if(!details::omax_verify_equivalence(ref_snapshot, nl, opt, __builtin_addressof(why)))
                    {
                        if(err != nullptr) { err->message = why; }
                        return false;
                    }
                }

                auto const cand_cost = details::compute_omax_cost(nl, opt);
                auto const cand_gate = details::count_logic_gates(nl);
                if(cand_cost < best_cost)
                {
                    best_cost = cand_cost;
                    best_gate = cand_gate;
                    best_snapshot = nl;  // snapshot the new best (node pointers differ, but we'll restore by ordinal mapping)
                    if(opt.omax_dump_best_cb != nullptr) { opt.omax_dump_best_cb(best_snapshot, opt.omax_dump_best_user); }
                    if(cand_rep_ptr != nullptr)
                    {
                        best_rep = cand_rep;
                        have_best_rep = true;
                    }
                }
                if(rep != nullptr)
                {
                    rep->omax_best_gate_count.push_back(best_gate);
                    rep->omax_best_cost.push_back(best_cost);
                }
            }

            // Ensure the returned netlist matches the best snapshot, without invalidating `top_port_nodes`.
            restore_from_snapshot(nl, best_snapshot);

            if(rep != nullptr && have_best_rep)
            {
                rep->passes = ::std::move(best_rep.passes);
                rep->iter_gate_count = ::std::move(best_rep.iter_gate_count);
            }

            if(rep != nullptr)
            {
                auto append_num = [](::fast_io::u8string& s, ::std::uint64_t v) noexcept
                {
                    char buf[32];
                    auto [p, ec] = ::std::to_chars(buf, buf + 32, v);
                    (void)ec;
                    for(char* c = buf; c != p; ++c) { s.push_back(static_cast<char8_t>(*c)); }
                };

                ::fast_io::u8string sum{};
                sum.reserve(256);
                sum.append(u8"omax seed=");
                append_num(sum, opt.omax_rand_seed);
                sum.append(u8" tries=");
                append_num(sum, tries);
                sum.append(u8" timeout_ms=");
                append_num(sum, opt.omax_timeout_ms);
                sum.append(u8" randomize=");
                sum.push_back(static_cast<char8_t>(opt.omax_randomize ? '1' : '0'));
                sum.append(u8" allow_regress=");
                sum.push_back(static_cast<char8_t>(opt.omax_allow_regress ? '1' : '0'));
                sum.append(u8" verify=");
                sum.push_back(static_cast<char8_t>(opt.omax_verify ? '1' : '0'));
                sum.append(u8" best_gate=");
                append_num(sum, best_gate);
                sum.append(u8" best_cost=");
                append_num(sum, best_cost);
                rep->omax_summary = ::std::move(sum);
            }
        }
        return ctx.ok();
    }
}  // namespace phy_engine::verilog::digital
