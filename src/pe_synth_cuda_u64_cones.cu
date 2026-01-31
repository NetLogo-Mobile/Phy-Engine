// CUDA backend for pe_synth bounded truth-table batching (6 vars, <=64 gates).
//
// This file is compiled only when PHY_ENGINE_ENABLE_CUDA_PE_SYNTH is enabled.
// Intended build mode: Clang CUDA (no nvcc), see CMake wiring in src/CMakeLists.txt.

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <atomic>
#include <climits>
#include <deque>
#include <mutex>
#include <new>
#include <unordered_map>
#include <utility>
#include <vector>

#include <cuda_runtime.h>

#include "phy_engine/verilog/digital/pe_synth.h"

namespace phy_engine::verilog::digital::details
{
    namespace
    {
        __device__ __forceinline__ std::uint64_t leaf_pattern(unsigned var_idx) noexcept
        {
            switch(var_idx)
            {
                case 0u: return 0xAAAAAAAAAAAAAAAAull;
                case 1u: return 0xCCCCCCCCCCCCCCCCull;
                case 2u: return 0xF0F0F0F0F0F0F0F0ull;
                case 3u: return 0xFF00FF00FF00FF00ull;
                case 4u: return 0xFFFF0000FFFF0000ull;
                case 5u: return 0xFFFFFFFF00000000ull;
                default: return 0ull;
            }
        }

        __device__ __forceinline__ std::uint64_t leaf_word16(unsigned var_idx, unsigned word_idx) noexcept
        {
            if(var_idx < 6u) { return leaf_pattern(var_idx); }
            // word_idx corresponds to minterm base 64*word_idx, so ((base>>var)&1) == ((word_idx>>(var-6))&1) for var>=6.
            return (((word_idx >> (var_idx - 6u)) & 1u) != 0u) ? ~0ull : 0ull;
        }

        __global__ void eval_u64_cones_kernel(cuda_u64_cone_desc const* cones, std::size_t cone_count, std::uint64_t* out_masks) noexcept
        {
            auto const tid = static_cast<std::size_t>(blockIdx.x) * static_cast<std::size_t>(blockDim.x) + static_cast<std::size_t>(threadIdx.x);
            if(tid >= cone_count) { return; }

            auto const c = cones[tid];
            auto const vc = static_cast<unsigned>(c.var_count);
            if(vc > 6u) { out_masks[tid] = 0ull; return; }

            auto const U = (vc >= 6u) ? 64u : (1u << vc);
            auto const all_mask = (U == 64u) ? ~0ull : ((1ull << U) - 1ull);

            std::uint64_t val[6u + 64u];
            #pragma unroll
            for(unsigned i = 0u; i < 6u; ++i) { val[i] = 0ull; }
            #pragma unroll
            for(unsigned i = 0u; i < 64u; ++i) { val[6u + i] = 0ull; }

            #pragma unroll
            for(unsigned i = 0u; i < 6u; ++i)
            {
                if(i < vc) { val[i] = leaf_pattern(i) & all_mask; }
            }

            auto load = [&](std::uint8_t idx) noexcept -> std::uint64_t
            {
                if(idx == 254u) { return 0ull; }
                if(idx == 255u) { return all_mask; }
                if(idx < (6u + 64u)) { return val[idx]; }
                return 0ull;
            };

            auto const gc = static_cast<unsigned>(c.gate_count);
            if(gc == 0u || gc > 64u) { out_masks[tid] = 0ull; return; }

            for(unsigned gi = 0u; gi < gc; ++gi)
            {
                auto const a = load(c.in0[gi]);
                auto const b = load(c.in1[gi]);
                std::uint64_t r{};
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

            out_masks[tid] = val[6u + (gc - 1u)] & all_mask;
        }

        __global__ void eval_tt_cones_kernel(cuda_tt_cone_desc const* cones,
                                             std::size_t cone_count,
                                             std::uint32_t stride_blocks,
                                             std::uint64_t* out_blocks) noexcept
        {
            auto const cone_idx = static_cast<std::size_t>(blockIdx.x);
            auto const word_idx = static_cast<unsigned>(blockIdx.y) * static_cast<unsigned>(blockDim.x) + static_cast<unsigned>(threadIdx.x);
            if(cone_idx >= cone_count) { return; }
            if(word_idx >= stride_blocks) { return; }

            auto const c = cones[cone_idx];
            auto const vc = static_cast<unsigned>(c.var_count);
            if(vc > 16u)
            {
                out_blocks[cone_idx * static_cast<std::size_t>(stride_blocks) + word_idx] = 0ull;
                return;
            }

            auto const U = static_cast<unsigned>(1u) << vc;
            auto const blocks = static_cast<unsigned>((U + 63u) / 64u);
            if(blocks == 0u || blocks > stride_blocks)
            {
                out_blocks[cone_idx * static_cast<std::size_t>(stride_blocks) + word_idx] = 0ull;
                return;
            }
            if(word_idx >= blocks)
            {
                out_blocks[cone_idx * static_cast<std::size_t>(stride_blocks) + word_idx] = 0ull;
                return;
            }

            auto const rem = static_cast<unsigned>(U & 63u);
            auto const last_mask = (rem == 0u) ? ~0ull : ((1ull << rem) - 1ull);
            auto const word_mask = (word_idx + 1u == blocks) ? last_mask : ~0ull;

            std::uint64_t val[16u + 256u];
            #pragma unroll
            for(unsigned i = 0u; i < 16u + 256u; ++i) { val[i] = 0ull; }

            for(unsigned i = 0u; i < vc; ++i) { val[i] = leaf_word16(i, word_idx) & word_mask; }

            auto load = [&](std::uint16_t idx) noexcept -> std::uint64_t
            {
                if(idx == 65534u) { return 0ull; }
                if(idx == 65535u) { return word_mask; }
                if(idx < (16u + 256u)) { return val[idx] & word_mask; }
                return 0ull;
            };

            auto const gc = static_cast<unsigned>(c.gate_count);
            if(gc == 0u || gc > 256u)
            {
                out_blocks[cone_idx * static_cast<std::size_t>(stride_blocks) + word_idx] = 0ull;
                return;
            }

            for(unsigned gi = 0u; gi < gc; ++gi)
            {
                auto const a = load(c.in0[gi]);
                auto const b = load(c.in1[gi]);
                std::uint64_t r{};
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
                val[16u + gi] = r & word_mask;
            }

            out_blocks[cone_idx * static_cast<std::size_t>(stride_blocks) + word_idx] = val[16u + (gc - 1u)] & word_mask;
        }

        // Row-major u64 bitset matrix helpers (QM/Espresso cover scoring).
        // Layout: rows[row * stride_words + w]
        __global__ void bitset_row_and_popcount_kernel(std::uint64_t const* rows,
                                                       std::size_t row_count,
                                                       std::uint32_t stride_words,
                                                       std::uint64_t const* mask,
                                                       std::uint32_t* out_counts) noexcept
        {
            auto const row = static_cast<std::size_t>(blockIdx.x);
            if(row >= row_count) { return; }

            auto const w = static_cast<std::uint32_t>(blockIdx.y) * static_cast<std::uint32_t>(blockDim.x) +
                           static_cast<std::uint32_t>(threadIdx.x);
            std::uint32_t v{};
            if(w < stride_words)
            {
                auto const x = rows[row * static_cast<std::size_t>(stride_words) + w] & mask[w];
                v = static_cast<std::uint32_t>(__popcll(static_cast<unsigned long long>(x)));
            }

            extern __shared__ std::uint32_t sh[];
            sh[threadIdx.x] = v;
            __syncthreads();

            // Tree reduction (blockDim.x is power-of-2).
            for(unsigned s = blockDim.x >> 1u; s > 0u; s >>= 1u)
            {
                if(threadIdx.x < s) { sh[threadIdx.x] += sh[threadIdx.x + s]; }
                __syncthreads();
            }

            if(threadIdx.x == 0u) { atomicAdd(out_counts + row, sh[0]); }
        }

        __global__ void bitset_row_and_popcount_active_kernel(std::uint64_t const* rows,
                                                              std::size_t row_count,
                                                              std::uint32_t stride_words,
                                                              std::uint64_t const* mask,
                                                              std::uint32_t* out_counts,
                                                              std::uint8_t const* active) noexcept
        {
            auto const row = static_cast<std::size_t>(blockIdx.x);
            if(row >= row_count) { return; }
            if(active != nullptr && active[row] == 0u) { return; }

            auto const w = static_cast<std::uint32_t>(blockIdx.y) * static_cast<std::uint32_t>(blockDim.x) +
                           static_cast<std::uint32_t>(threadIdx.x);
            std::uint32_t v{};
            if(w < stride_words)
            {
                auto const x = rows[row * static_cast<std::size_t>(stride_words) + w] & mask[w];
                v = static_cast<std::uint32_t>(__popcll(static_cast<unsigned long long>(x)));
            }

            extern __shared__ std::uint32_t sh[];
            sh[threadIdx.x] = v;
            __syncthreads();

            for(unsigned s = blockDim.x >> 1u; s > 0u; s >>= 1u)
            {
                if(threadIdx.x < s) { sh[threadIdx.x] += sh[threadIdx.x + s]; }
                __syncthreads();
            }

            if(threadIdx.x == 0u) { atomicAdd(out_counts + row, sh[0]); }
        }

        struct best_item
        {
            std::int32_t score;
            std::uint32_t gain;
            std::uint32_t cost;
            std::uint32_t idx;
        };

        __device__ __forceinline__ bool best_item_better(best_item const& a, best_item const& b) noexcept
        {
            if(a.score != b.score) { return a.score > b.score; }
            if(a.gain != b.gain) { return a.gain > b.gain; }
            // Lower cost is better.
            if(a.cost != b.cost) { return a.cost < b.cost; }
            return a.idx < b.idx;
        }

        __global__ void bitset_best_stage1_kernel(std::uint32_t const* popc,
                                                  std::uint32_t const* cost,
                                                  std::uint32_t n,
                                                  std::uint32_t base_idx,
                                                  best_item* out_parts) noexcept
        {
            // Each block reduces up to 1024 rows (4 waves of 256 threads).
            constexpr std::uint32_t items_per_block = 1024u;
            auto const block_begin = static_cast<std::uint32_t>(blockIdx.x) * items_per_block;
            auto const block_end = min(n, block_begin + items_per_block);

            best_item local{};
            local.score = INT32_MIN;
            local.gain = 0u;
            local.cost = 0u;
            local.idx = 0xFFFFFFFFu;

            for(auto i = block_begin + static_cast<std::uint32_t>(threadIdx.x); i < block_end; i += static_cast<std::uint32_t>(blockDim.x))
            {
                auto const g = popc[i];
                if(g == 0u) { continue; }
                auto const c = (cost != nullptr) ? cost[i] : 0u;
                // score = gain*64 - cost (clamp to signed 32-bit)
                auto const s64 = static_cast<long long>(g) * 64ll - static_cast<long long>(c);
                std::int32_t s{};
                if(s64 > static_cast<long long>(INT32_MAX)) { s = INT32_MAX; }
                else if(s64 < static_cast<long long>(INT32_MIN)) { s = INT32_MIN; }
                else { s = static_cast<std::int32_t>(s64); }

                best_item cand{};
                cand.score = s;
                cand.gain = g;
                cand.cost = c;
                cand.idx = base_idx + i;
                if(best_item_better(cand, local)) { local = cand; }
            }

            __shared__ best_item sh[256];
            sh[threadIdx.x] = local;
            __syncthreads();

            for(unsigned s = blockDim.x >> 1u; s > 0u; s >>= 1u)
            {
                if(threadIdx.x < s)
                {
                    auto const rhs = sh[threadIdx.x + s];
                    if(best_item_better(rhs, sh[threadIdx.x])) { sh[threadIdx.x] = rhs; }
                }
                __syncthreads();
            }

            if(threadIdx.x == 0u) { out_parts[blockIdx.x] = sh[0]; }
        }

        __global__ void bitset_best_stage2_kernel(best_item const* parts, std::uint32_t parts_n, best_item* out_one) noexcept
        {
            best_item local{};
            local.score = INT32_MIN;
            local.gain = 0u;
            local.cost = 0u;
            local.idx = 0xFFFFFFFFu;

            for(auto i = static_cast<std::uint32_t>(threadIdx.x); i < parts_n; i += static_cast<std::uint32_t>(blockDim.x))
            {
                auto const cand = parts[i];
                if(best_item_better(cand, local)) { local = cand; }
            }

            __shared__ best_item sh[256];
            sh[threadIdx.x] = local;
            __syncthreads();

            for(unsigned s = blockDim.x >> 1u; s > 0u; s >>= 1u)
            {
                if(threadIdx.x < s)
                {
                    auto const rhs = sh[threadIdx.x + s];
                    if(best_item_better(rhs, sh[threadIdx.x])) { sh[threadIdx.x] = rhs; }
                }
                __syncthreads();
            }
            if(threadIdx.x == 0u) { out_one[0] = sh[0]; }
        }

        __global__ void qm_cov_fill_kernel(cuda_cube_desc const* cubes,
                                           std::uint32_t cube_count,
                                           std::uint16_t const* on_minterms,
                                           std::uint32_t on_count,
                                           std::uint16_t var_mask,
                                           std::uint32_t stride_words,
                                           std::uint64_t* out_rows) noexcept
        {
            auto const row = static_cast<std::uint32_t>(blockIdx.x);
            if(row >= cube_count) { return; }
            auto const w = static_cast<std::uint32_t>(blockIdx.y);
            if(w >= stride_words) { return; }
            // 64 threads, each thread computes one bit in this u64 word.
            auto const bit = static_cast<std::uint32_t>(threadIdx.x);
            if(bit >= 64u) { return; }

            auto const pos = w * 64u + bit;
            bool cover{};
            if(pos < on_count)
            {
                auto const m = static_cast<std::uint16_t>(on_minterms[pos] & var_mask);
                auto const c = cubes[row];
                auto const specified = static_cast<std::uint16_t>((~c.mask) & var_mask);
                cover = static_cast<std::uint16_t>((m ^ (c.value & var_mask)) & specified) == 0u;
            }

            // Build 64-bit word via two 32-bit warp ballots.
            __shared__ std::uint32_t sh_low;
            __shared__ std::uint32_t sh_high;

            auto const lane = static_cast<unsigned>(threadIdx.x) & 31u;
            auto const warp = static_cast<unsigned>(threadIdx.x) >> 5u;  // 0 or 1
            auto const mask = __ballot_sync(0xFFFFFFFFu, cover);
            if(warp == 0u && lane == 0u) { sh_low = static_cast<std::uint32_t>(mask); }
            if(warp == 1u && lane == 0u) { sh_high = static_cast<std::uint32_t>(mask); }
            __syncthreads();

            if(threadIdx.x == 0u)
            {
                out_rows[row * static_cast<std::size_t>(stride_words) + w] =
                    static_cast<std::uint64_t>(sh_low) | (static_cast<std::uint64_t>(sh_high) << 32u);
            }
        }

        __global__ void bitset_row_any_and_kernel(std::uint64_t const* rows,
                                                  std::size_t row_count,
                                                  std::uint32_t stride_words,
                                                  std::uint64_t const* mask,
                                                  std::uint32_t* out_any) noexcept
        {
            auto const row = static_cast<std::size_t>(blockIdx.x);
            if(row >= row_count) { return; }

            auto const w = static_cast<std::uint32_t>(blockIdx.y) * static_cast<std::uint32_t>(blockDim.x) +
                           static_cast<std::uint32_t>(threadIdx.x);
            std::uint8_t v{};
            if(w < stride_words)
            {
                auto const x = rows[row * static_cast<std::size_t>(stride_words) + w] & mask[w];
                v = static_cast<std::uint8_t>(x != 0ull);
            }

            __shared__ std::uint8_t sh8[256];
            sh8[threadIdx.x] = v;
            __syncthreads();

            for(unsigned s = blockDim.x >> 1u; s > 0u; s >>= 1u)
            {
                if(threadIdx.x < s) { sh8[threadIdx.x] = static_cast<std::uint8_t>(sh8[threadIdx.x] | sh8[threadIdx.x + s]); }
                __syncthreads();
            }

            if(threadIdx.x == 0u && sh8[0] != 0u) { atomicOr(out_any + row, 1u); }
        }

        __global__ void bitset_mask_andnot_kernel(std::uint64_t* mask, std::uint64_t const* row, std::uint32_t stride_words) noexcept
        {
            auto const i = static_cast<std::uint32_t>(blockIdx.x) * static_cast<std::uint32_t>(blockDim.x) +
                           static_cast<std::uint32_t>(threadIdx.x);
            if(i >= stride_words) { return; }
            mask[i] &= ~row[i];
        }

        __global__ void espresso_cube_hits_off_kernel(cuda_cube_desc const* cubes,
                                                      std::size_t cube_count,
                                                      std::uint32_t var_count,
                                                      std::uint64_t const* off_blocks,
                                                      std::uint32_t off_words,
                                                      std::uint32_t* out_hits) noexcept
        {
            auto const cube_idx = static_cast<std::size_t>(blockIdx.x);
            auto const word_idx = static_cast<std::uint32_t>(blockIdx.y) * static_cast<std::uint32_t>(blockDim.x) +
                                  static_cast<std::uint32_t>(threadIdx.x);
            if(cube_idx >= cube_count) { return; }
            if(word_idx >= off_words) { return; }
            if(var_count == 0u || var_count > 16u) { return; }

            auto const offw = off_blocks[word_idx];
            if(offw == 0ull) { return; }

            auto const c = cubes[cube_idx];
            std::uint64_t cov = ~0ull;
            #pragma unroll
            for(unsigned v = 0u; v < 16u; ++v)
            {
                if(v >= var_count) { break; }
                if(((c.mask >> v) & 1u) != 0u) { continue; }
                auto const pat = leaf_word16(v, word_idx);
                bool const bit_is_1 = ((c.value >> v) & 1u) != 0u;
                cov &= bit_is_1 ? pat : ~pat;
                if(cov == 0ull) { break; }
            }
            if((cov & offw) != 0ull) { atomicOr(out_hits + cube_idx, 1u); }
        }

        __device__ __forceinline__ bool cube_hits_off_warp(cuda_cube_desc c,
                                                           std::uint32_t var_count,
                                                           std::uint64_t const* off_blocks,
                                                           std::uint32_t off_words) noexcept
        {
            // Use one warp to scan the OFF bitset words. var_count<=16 and off_words<=1024.
            auto const lane = static_cast<unsigned>(threadIdx.x) & 31u;
            unsigned mask = 0xFFFFFFFFu;
            for(std::uint32_t base{}; base < off_words; base += 32u)
            {
                auto const word_idx = base + lane;
                bool hit{};
                if(word_idx < off_words)
                {
                    auto const offw = off_blocks[word_idx];
                    if(offw != 0ull)
                    {
                        std::uint64_t cov = ~0ull;
                        #pragma unroll
                        for(unsigned v = 0u; v < 16u; ++v)
                        {
                            if(v >= var_count) { break; }
                            if(((c.mask >> v) & 1u) != 0u) { continue; }
                            auto const pat = leaf_word16(v, word_idx);
                            bool const bit_is_1 = ((c.value >> v) & 1u) != 0u;
                            cov &= bit_is_1 ? pat : ~pat;
                            if(cov == 0ull) { break; }
                        }
                        hit = ((cov & offw) != 0ull);
                    }
                }
                if(__any_sync(mask, hit)) { return true; }
            }
            return false;
        }

        __global__ void espresso_cube_hits_off_warp4_kernel(cuda_cube_desc const* cubes,
                                                            std::size_t cube_count,
                                                            std::uint32_t var_count,
                                                            std::uint64_t const* off_blocks,
                                                            std::uint32_t off_words,
                                                            std::uint32_t* out_hits) noexcept
        {
            constexpr unsigned warps_per_block = 4u;
            auto const warp = static_cast<unsigned>(threadIdx.x) >> 5u;   // 0..3
            auto const lane = static_cast<unsigned>(threadIdx.x) & 31u;   // 0..31
            if(warp >= warps_per_block) { return; }
            auto const cube_idx = static_cast<std::size_t>(blockIdx.x) * warps_per_block + static_cast<std::size_t>(warp);
            if(cube_idx >= cube_count) { return; }
            if(var_count == 0u || var_count > 16u) { return; }
            if(off_words == 0u) { return; }

            auto const c = cubes[cube_idx];
            bool hit{};
            // Inline the warp scan to avoid extra registers from a function call.
            unsigned const full = 0xFFFFFFFFu;
            for(std::uint32_t base{}; base < off_words; base += 32u)
            {
                auto const word_idx = base + lane;
                bool lane_hit{};
                if(word_idx < off_words)
                {
                    auto const offw = off_blocks[word_idx];
                    if(offw != 0ull)
                    {
                        std::uint64_t cov = ~0ull;
                        #pragma unroll
                        for(unsigned v = 0u; v < 16u; ++v)
                        {
                            if(v >= var_count) { break; }
                            if(((c.mask >> v) & 1u) != 0u) { continue; }
                            auto const pat = leaf_word16(v, word_idx);
                            bool const bit_is_1 = ((c.value >> v) & 1u) != 0u;
                            cov &= bit_is_1 ? pat : ~pat;
                            if(cov == 0ull) { break; }
                        }
                        lane_hit = ((cov & offw) != 0ull);
                    }
                }
                if(__any_sync(full, lane_hit)) { hit = true; break; }
            }

            if(lane == 0u) { out_hits[cube_idx] = hit ? 1u : 0u; }
        }

        __global__ void espresso_off_expand_best_kernel(cuda_cube_desc* cubes,
                                                        std::size_t cube_count,
                                                        std::uint32_t var_count,
                                                        std::uint64_t const* off_blocks,
                                                        std::uint32_t off_words,
                                                        std::uint8_t const* cand_vars,
                                                        std::uint32_t cand_vars_count,
                                                        std::uint32_t max_rounds) noexcept
        {
            auto const cube_idx = static_cast<std::size_t>(blockIdx.x);
            if(cube_idx >= cube_count) { return; }
            if(var_count == 0u || var_count > 16u) { return; }
            if(off_words == 0u) { return; }
            if(cand_vars == nullptr || cand_vars_count == 0u) { return; }

            // One warp per cube.
            auto c = cubes[cube_idx];

            for(std::uint32_t round{}; round < max_rounds; ++round)
            {
                bool changed{};

                // Optional deeper search: try dropping 3 literals in one step (bounded).
                // This increases GPU work per cube (better utilization) and can improve cover quality
                // on small cones without additional host-side batching.
                bool const try_triples = (cand_vars_count <= 12u) && (off_words <= 32u);
                if(try_triples && !changed)
                {
                    for(std::uint32_t i{}; i < cand_vars_count && !changed; ++i)
                    {
                        auto const v1 = static_cast<unsigned>(cand_vars[i]);
                        if(v1 >= var_count) { continue; }
                        if(((c.mask >> v1) & 1u) != 0u) { continue; }
                        for(std::uint32_t j{i + 1u}; j < cand_vars_count && !changed; ++j)
                        {
                            auto const v2 = static_cast<unsigned>(cand_vars[j]);
                            if(v2 >= var_count) { continue; }
                            if(((c.mask >> v2) & 1u) != 0u) { continue; }
                            for(std::uint32_t k{j + 1u}; k < cand_vars_count; ++k)
                            {
                                auto const v3 = static_cast<unsigned>(cand_vars[k]);
                                if(v3 >= var_count) { continue; }
                                if(((c.mask >> v3) & 1u) != 0u) { continue; }

                                auto const b = static_cast<std::uint16_t>((1u << v1) | (1u << v2) | (1u << v3));
                                cuda_cube_desc cand = c;
                                cand.mask = static_cast<std::uint16_t>(cand.mask | b);
                                cand.value = static_cast<std::uint16_t>(cand.value & static_cast<std::uint16_t>(~b));
                                if(!cube_hits_off_warp(cand, var_count, off_blocks, off_words))
                                {
                                    c = cand;
                                    changed = true;
                                    break;
                                }
                            }
                        }
                    }
                }

                // Prefer dropping 2 literals first (matches CPU "best_drop_cnt" heuristic).
                for(std::uint32_t i{}; i < cand_vars_count && !changed; ++i)
                {
                    auto const v1 = static_cast<unsigned>(cand_vars[i]);
                    if(v1 >= var_count) { continue; }
                    if(((c.mask >> v1) & 1u) != 0u) { continue; }
                    for(std::uint32_t j{i + 1u}; j < cand_vars_count; ++j)
                    {
                        auto const v2 = static_cast<unsigned>(cand_vars[j]);
                        if(v2 >= var_count) { continue; }
                        if(((c.mask >> v2) & 1u) != 0u) { continue; }

                        cuda_cube_desc cand = c;
                        cand.mask = static_cast<std::uint16_t>(cand.mask | static_cast<std::uint16_t>((1u << v1) | (1u << v2)));
                        cand.value = static_cast<std::uint16_t>(cand.value & static_cast<std::uint16_t>(~((1u << v1) | (1u << v2))));

                        if(!cube_hits_off_warp(cand, var_count, off_blocks, off_words))
                        {
                            c = cand;
                            changed = true;
                            break;
                        }
                    }
                }

                // If no safe pair, try dropping one literal.
                if(!changed)
                {
                    for(std::uint32_t i{}; i < cand_vars_count; ++i)
                    {
                        auto const v = static_cast<unsigned>(cand_vars[i]);
                        if(v >= var_count) { continue; }
                        if(((c.mask >> v) & 1u) != 0u) { continue; }

                        cuda_cube_desc cand = c;
                        cand.mask = static_cast<std::uint16_t>(cand.mask | static_cast<std::uint16_t>(1u << v));
                        cand.value = static_cast<std::uint16_t>(cand.value & static_cast<std::uint16_t>(~(1u << v)));
                        if(!cube_hits_off_warp(cand, var_count, off_blocks, off_words))
                        {
                            c = cand;
                            changed = true;
                            break;
                        }
                    }
                }

                if(!changed) { break; }
            }

            // Thread 0 writes back (all lanes have the same final cube).
            if((static_cast<unsigned>(threadIdx.x) & 31u) == 0u) { cubes[cube_idx] = c; }
        }

        struct device_chunk
        {
            int device{};
            std::size_t begin{};
            std::size_t end{};
            bool ok{};
        };

        // Reuse per-device allocations/streams to avoid cudaMalloc/cudaFree overhead dominating small batches.
        // This significantly improves throughput and GPU utilization for the many small-ish launches typical in synthesis.
        struct eval_u64_ctx
        {
            int device{-1};
            std::mutex mu{};
            cudaStream_t stream{};
            cuda_u64_cone_desc* d_cones{};
            std::uint64_t* d_out{};
            std::size_t cap_cones{};
            // Optional pinned host staging to keep H2D/D2H asynchronous and reduce driver overhead.
            cuda_u64_cone_desc* h_cones_pinned{};
            std::uint64_t* h_out_pinned{};
            std::size_t cap_host_cones{};
            bool ok{};
        };

        struct eval_tt_ctx
        {
            int device{-1};
            std::mutex mu{};
            cudaStream_t stream{};
            cuda_tt_cone_desc* d_cones{};
            std::uint64_t* d_out{};
            std::size_t cap_cones{};
            std::uint32_t cap_stride_blocks{};
            cuda_tt_cone_desc* h_cones_pinned{};
            std::uint64_t* h_out_pinned{};
            std::size_t cap_host_cones{};
            std::uint32_t cap_host_stride_blocks{};
            bool ok{};
        };

        [[nodiscard]] inline eval_u64_ctx& get_u64_ctx(int device) noexcept
        {
            static std::mutex g_mu{};
            static std::deque<eval_u64_ctx> g_ctx{};
            std::scoped_lock lk(g_mu);
            for(auto& c: g_ctx)
            {
                if(c.device == device) { return c; }
            }
            // These ctx types are intentionally non-copyable/non-movable (they own mutex/streams).
            // Construct in-place to avoid deque push_back needing move/copy.
            g_ctx.emplace_back();
            g_ctx.back().device = device;
            return g_ctx.back();
        }

        [[nodiscard]] inline eval_tt_ctx& get_tt_ctx(int device) noexcept
        {
            static std::mutex g_mu{};
            static std::deque<eval_tt_ctx> g_ctx{};
            std::scoped_lock lk(g_mu);
            for(auto& c: g_ctx)
            {
                if(c.device == device) { return c; }
            }
            // Construct in-place (non-copyable/non-movable ctx).
            g_ctx.emplace_back();
            g_ctx.back().device = device;
            return g_ctx.back();
        }

        [[nodiscard]] inline bool ensure_u64_ctx_ready(eval_u64_ctx& ctx, std::size_t n) noexcept
        {
            ctx.ok = false;
            if(cudaSetDevice(ctx.device) != cudaSuccess) { return false; }
            if(ctx.stream == nullptr)
            {
                if(cudaStreamCreateWithFlags(&ctx.stream, cudaStreamNonBlocking) != cudaSuccess) { return false; }
            }
            if(n <= ctx.cap_cones)
            {
                ctx.ok = true;
                return true;
            }
            // Grow (free+malloc). Keep it simple and deterministic.
            if(ctx.d_out != nullptr) { (void)cudaFree(ctx.d_out); ctx.d_out = nullptr; }
            if(ctx.d_cones != nullptr) { (void)cudaFree(ctx.d_cones); ctx.d_cones = nullptr; }
            if(ctx.h_out_pinned != nullptr) { (void)cudaFreeHost(ctx.h_out_pinned); ctx.h_out_pinned = nullptr; }
            if(ctx.h_cones_pinned != nullptr) { (void)cudaFreeHost(ctx.h_cones_pinned); ctx.h_cones_pinned = nullptr; }
            ctx.cap_cones = 0u;
            ctx.cap_host_cones = 0u;

            if(cudaMalloc(reinterpret_cast<void**>(&ctx.d_cones), n * sizeof(cuda_u64_cone_desc)) != cudaSuccess) { return false; }
            if(cudaMalloc(reinterpret_cast<void**>(&ctx.d_out), n * sizeof(std::uint64_t)) != cudaSuccess) { return false; }
            ctx.cap_cones = n;

            // Best-effort pinned staging; if it fails we still work with pageable pointers.
            if(cudaHostAlloc(reinterpret_cast<void**>(&ctx.h_cones_pinned), n * sizeof(cuda_u64_cone_desc), cudaHostAllocPortable) == cudaSuccess &&
               cudaHostAlloc(reinterpret_cast<void**>(&ctx.h_out_pinned), n * sizeof(std::uint64_t), cudaHostAllocPortable) == cudaSuccess)
            {
                ctx.cap_host_cones = n;
            }
            else
            {
                if(ctx.h_out_pinned != nullptr) { (void)cudaFreeHost(ctx.h_out_pinned); ctx.h_out_pinned = nullptr; }
                if(ctx.h_cones_pinned != nullptr) { (void)cudaFreeHost(ctx.h_cones_pinned); ctx.h_cones_pinned = nullptr; }
                ctx.cap_host_cones = 0u;
            }
            ctx.ok = true;
            return true;
        }

        [[nodiscard]] inline bool ensure_tt_ctx_ready(eval_tt_ctx& ctx, std::size_t n, std::uint32_t stride_blocks) noexcept
        {
            ctx.ok = false;
            if(stride_blocks == 0u) { return false; }
            if(cudaSetDevice(ctx.device) != cudaSuccess) { return false; }
            if(ctx.stream == nullptr)
            {
                if(cudaStreamCreateWithFlags(&ctx.stream, cudaStreamNonBlocking) != cudaSuccess) { return false; }
            }
            if(n <= ctx.cap_cones && stride_blocks <= ctx.cap_stride_blocks)
            {
                ctx.ok = true;
                return true;
            }
            if(ctx.d_out != nullptr) { (void)cudaFree(ctx.d_out); ctx.d_out = nullptr; }
            if(ctx.d_cones != nullptr) { (void)cudaFree(ctx.d_cones); ctx.d_cones = nullptr; }
            if(ctx.h_out_pinned != nullptr) { (void)cudaFreeHost(ctx.h_out_pinned); ctx.h_out_pinned = nullptr; }
            if(ctx.h_cones_pinned != nullptr) { (void)cudaFreeHost(ctx.h_cones_pinned); ctx.h_cones_pinned = nullptr; }
            ctx.cap_cones = 0u;
            ctx.cap_stride_blocks = 0u;
            ctx.cap_host_cones = 0u;
            ctx.cap_host_stride_blocks = 0u;

            if(cudaMalloc(reinterpret_cast<void**>(&ctx.d_cones), n * sizeof(cuda_tt_cone_desc)) != cudaSuccess) { return false; }
            auto const out_words = n * static_cast<std::size_t>(stride_blocks);
            if(cudaMalloc(reinterpret_cast<void**>(&ctx.d_out), out_words * sizeof(std::uint64_t)) != cudaSuccess) { return false; }
            ctx.cap_cones = n;
            ctx.cap_stride_blocks = stride_blocks;

            if(cudaHostAlloc(reinterpret_cast<void**>(&ctx.h_cones_pinned), n * sizeof(cuda_tt_cone_desc), cudaHostAllocPortable) == cudaSuccess &&
               cudaHostAlloc(reinterpret_cast<void**>(&ctx.h_out_pinned), out_words * sizeof(std::uint64_t), cudaHostAllocPortable) == cudaSuccess)
            {
                ctx.cap_host_cones = n;
                ctx.cap_host_stride_blocks = stride_blocks;
            }
            else
            {
                if(ctx.h_out_pinned != nullptr) { (void)cudaFreeHost(ctx.h_out_pinned); ctx.h_out_pinned = nullptr; }
                if(ctx.h_cones_pinned != nullptr) { (void)cudaFreeHost(ctx.h_cones_pinned); ctx.h_cones_pinned = nullptr; }
                ctx.cap_host_cones = 0u;
                ctx.cap_host_stride_blocks = 0u;
            }
            ctx.ok = true;
            return true;
        }

        // Note: multi-GPU entrypoints should avoid per-call std::thread creation.
        // We enqueue work to each device's non-blocking stream from the caller thread, then synchronize streams.
        // This reduces CPU overhead dramatically while still allowing both GPUs to run concurrently.

        struct bitset_matrix_device
        {
            int device{};
            std::size_t begin{};
            std::size_t end{};
            std::uint32_t stride_words{};
            cudaStream_t stream{};

            std::uint64_t* d_rows{};
            std::uint64_t* d_mask{};
            std::uint64_t* d_row_tmp{};    // temp row buffer (stride_words u64) for resident-mask updates
            std::uint32_t* d_popc{};
            std::uint32_t* d_any{};
            std::uint32_t* d_cost{};       // optional row cost (u32 per row), for greedy argmax
            std::uint8_t* d_active{};      // optional row active flags (u8 per row), 1=active 0=disabled
            best_item* d_best_parts{};     // temporary reduction buffer
            best_item* d_best_one{};       // final best item (size 1)
            std::uint32_t cap_best_parts{};
            // Temporary buffers for GPU-built cover matrices (QM greedy cover).
            cuda_cube_desc* d_cubes_tmp{};
            std::uint16_t* d_on_tmp{};
            std::size_t cap_cubes_tmp{};
            std::size_t cap_on_tmp{};

            // Host staging buffer for row_any (u32 per row), reused across calls to avoid reallocations.
            std::vector<std::uint32_t> h_tmp{};
            best_item h_best{};  // host copy of d_best_one (best_row op)

            bool ok{};

            bitset_matrix_device() noexcept = default;
            bitset_matrix_device(bitset_matrix_device const&) = delete;
            bitset_matrix_device& operator=(bitset_matrix_device const&) = delete;
            bitset_matrix_device(bitset_matrix_device&& o) noexcept :
                device(o.device),
                begin(o.begin),
                end(o.end),
                stride_words(o.stride_words),
                stream(o.stream),
                d_rows(o.d_rows),
                d_mask(o.d_mask),
                d_row_tmp(o.d_row_tmp),
                d_popc(o.d_popc),
                d_any(o.d_any),
                d_cost(o.d_cost),
                d_active(o.d_active),
                d_best_parts(o.d_best_parts),
                d_best_one(o.d_best_one),
                cap_best_parts(o.cap_best_parts),
                d_cubes_tmp(o.d_cubes_tmp),
                d_on_tmp(o.d_on_tmp),
                cap_cubes_tmp(o.cap_cubes_tmp),
                cap_on_tmp(o.cap_on_tmp),
                h_tmp(std::move(o.h_tmp)),
                h_best(o.h_best),
                ok(o.ok)
            {
                o.device = 0;
                o.begin = 0;
                o.end = 0;
                o.stride_words = 0;
                o.stream = nullptr;
                o.d_rows = nullptr;
                o.d_mask = nullptr;
                o.d_row_tmp = nullptr;
                o.d_popc = nullptr;
                o.d_any = nullptr;
                o.d_cost = nullptr;
                o.d_active = nullptr;
                o.d_best_parts = nullptr;
                o.d_best_one = nullptr;
                o.cap_best_parts = 0u;
                o.d_cubes_tmp = nullptr;
                o.d_on_tmp = nullptr;
                o.cap_cubes_tmp = 0u;
                o.cap_on_tmp = 0u;
                o.ok = false;
            }
            bitset_matrix_device& operator=(bitset_matrix_device&& o) noexcept
            {
                if(this == &o) { return *this; }
                // Must be destroyed/reset by the owner before move-assigning in.
                device = o.device;
                begin = o.begin;
                end = o.end;
                stride_words = o.stride_words;
                stream = o.stream;
                d_rows = o.d_rows;
                d_mask = o.d_mask;
                d_row_tmp = o.d_row_tmp;
                d_popc = o.d_popc;
                d_any = o.d_any;
                d_cost = o.d_cost;
                d_active = o.d_active;
                d_best_parts = o.d_best_parts;
                d_best_one = o.d_best_one;
                cap_best_parts = o.cap_best_parts;
                d_cubes_tmp = o.d_cubes_tmp;
                d_on_tmp = o.d_on_tmp;
                cap_cubes_tmp = o.cap_cubes_tmp;
                cap_on_tmp = o.cap_on_tmp;
                h_tmp = std::move(o.h_tmp);
                h_best = o.h_best;
                ok = o.ok;
                o.device = 0;
                o.begin = 0;
                o.end = 0;
                o.stride_words = 0;
                o.stream = nullptr;
                o.d_rows = nullptr;
                o.d_mask = nullptr;
                o.d_row_tmp = nullptr;
                o.d_popc = nullptr;
                o.d_any = nullptr;
                o.d_cost = nullptr;
                o.d_active = nullptr;
                o.d_best_parts = nullptr;
                o.d_best_one = nullptr;
                o.cap_best_parts = 0u;
                o.d_cubes_tmp = nullptr;
                o.d_on_tmp = nullptr;
                o.cap_cubes_tmp = 0u;
                o.cap_on_tmp = 0u;
                o.ok = false;
                return *this;
            }
        };

        struct bitset_matrix_handle
        {
            std::uint32_t device_mask{};
            std::size_t row_count{};
            std::uint32_t stride_words{};
            std::vector<bitset_matrix_device> devs{};
        };

        struct espresso_hits_ctx
        {
            int device{-1};
            std::mutex mu{};
            cudaStream_t stream{};
            cuda_cube_desc* d_cubes{};
            std::uint64_t* d_off{};
            std::uint32_t* d_hits{};
            std::size_t cap_cubes{};
            std::uint32_t cap_words{};
            // Optional pinned host staging to reduce per-call allocations/driver overhead.
            cuda_cube_desc* h_cubes_pinned{};
            std::uint32_t* h_hits_pinned{};
            std::size_t cap_host_cubes{};
            std::vector<cuda_cube_desc> h_cubes{};
            std::vector<std::uint32_t> h_hits{};
            bool ok{};
        };

        [[nodiscard]] inline espresso_hits_ctx& get_espresso_ctx(int device) noexcept
        {
            static std::mutex g_mu{};
            static std::deque<espresso_hits_ctx> g_ctx{};
            std::scoped_lock lk(g_mu);
            for(auto& c: g_ctx)
            {
                if(c.device == device) { return c; }
            }
            // Construct in-place (non-copyable/non-movable ctx).
            g_ctx.emplace_back();
            g_ctx.back().device = device;
            return g_ctx.back();
        }

        [[nodiscard]] inline bool ensure_espresso_ctx_ready(espresso_hits_ctx& ctx, std::size_t cubes, std::uint32_t words) noexcept
        {
            ctx.ok = false;
            if(cubes == 0u || words == 0u) { return false; }
            if(cudaSetDevice(ctx.device) != cudaSuccess) { return false; }
            if(ctx.stream == nullptr)
            {
                if(cudaStreamCreateWithFlags(&ctx.stream, cudaStreamNonBlocking) != cudaSuccess) { return false; }
            }
            if(cubes <= ctx.cap_cubes && words <= ctx.cap_words)
            {
                ctx.ok = true;
                return true;
            }

            if(ctx.d_hits != nullptr) { (void)cudaFree(ctx.d_hits); ctx.d_hits = nullptr; }
            if(ctx.d_off != nullptr) { (void)cudaFree(ctx.d_off); ctx.d_off = nullptr; }
            if(ctx.d_cubes != nullptr) { (void)cudaFree(ctx.d_cubes); ctx.d_cubes = nullptr; }
            if(ctx.h_hits_pinned != nullptr) { (void)cudaFreeHost(ctx.h_hits_pinned); ctx.h_hits_pinned = nullptr; }
            if(ctx.h_cubes_pinned != nullptr) { (void)cudaFreeHost(ctx.h_cubes_pinned); ctx.h_cubes_pinned = nullptr; }
            ctx.cap_cubes = 0u;
            ctx.cap_words = 0u;
            ctx.cap_host_cubes = 0u;
            ctx.h_cubes.clear();
            ctx.h_hits.clear();

            if(cudaMalloc(reinterpret_cast<void**>(&ctx.d_cubes), cubes * sizeof(cuda_cube_desc)) != cudaSuccess) { return false; }
            if(cudaMalloc(reinterpret_cast<void**>(&ctx.d_hits), cubes * sizeof(std::uint32_t)) != cudaSuccess) { return false; }
            if(cudaMalloc(reinterpret_cast<void**>(&ctx.d_off), static_cast<std::size_t>(words) * sizeof(std::uint64_t)) != cudaSuccess) { return false; }
            ctx.cap_cubes = cubes;
            ctx.cap_words = words;

            // Best-effort pinned staging: helps when there are many small-ish calls.
            if(cudaHostAlloc(reinterpret_cast<void**>(&ctx.h_cubes_pinned), cubes * sizeof(cuda_cube_desc), cudaHostAllocPortable) == cudaSuccess &&
               cudaHostAlloc(reinterpret_cast<void**>(&ctx.h_hits_pinned), cubes * sizeof(std::uint32_t), cudaHostAllocPortable) == cudaSuccess)
            {
                ctx.cap_host_cubes = cubes;
            }
            else
            {
                if(ctx.h_hits_pinned != nullptr) { (void)cudaFreeHost(ctx.h_hits_pinned); ctx.h_hits_pinned = nullptr; }
                if(ctx.h_cubes_pinned != nullptr) { (void)cudaFreeHost(ctx.h_cubes_pinned); ctx.h_cubes_pinned = nullptr; }
                ctx.cap_host_cubes = 0u;
            }

            ctx.ok = true;
            return true;
        }

        [[nodiscard]] inline bool do_espresso_hits_off_on_device(int device,
                                                                 std::size_t begin,
                                                                 std::size_t end,
                                                                 cuda_cube_desc const* cubes,
                                                                 std::uint32_t var_count,
                                                                 std::uint64_t const* off_blocks,
                                                                 std::uint32_t off_words,
                                                                 std::uint8_t* out_hits) noexcept
        {
            if(end <= begin) { return true; }
            if(cubes == nullptr || off_blocks == nullptr || out_hits == nullptr) { return false; }

            auto const n = end - begin;
            auto& ctx = get_espresso_ctx(device);
            std::scoped_lock lk(ctx.mu);
            bool ok = ensure_espresso_ctx_ready(ctx, n, off_words);
            if(!ok) { ctx.ok = false; return false; }

            auto const cubes_bytes = n * sizeof(cuda_cube_desc);
            auto const off_bytes = static_cast<std::size_t>(off_words) * sizeof(std::uint64_t);
            auto const hits_bytes = n * sizeof(std::uint32_t);

            if(ctx.h_cubes_pinned != nullptr && ctx.cap_host_cubes >= n)
            {
                ::std::memcpy(ctx.h_cubes_pinned, cubes + begin, cubes_bytes);
                if(cudaMemcpyAsync(ctx.d_cubes, ctx.h_cubes_pinned, cubes_bytes, cudaMemcpyHostToDevice, ctx.stream) != cudaSuccess) { ok = false; }
            }
            else
            {
                if(cudaMemcpyAsync(ctx.d_cubes, cubes + begin, cubes_bytes, cudaMemcpyHostToDevice, ctx.stream) != cudaSuccess) { ok = false; }
            }
            if(ok && cudaMemcpyAsync(ctx.d_off, off_blocks, off_bytes, cudaMemcpyHostToDevice, ctx.stream) != cudaSuccess) { ok = false; }

            if(ok)
            {
                constexpr unsigned threads = 128u; // 4 warps per block, 4 cubes per block
                auto const blocks_x = static_cast<unsigned>((n + 4u - 1u) / 4u);
                espresso_cube_hits_off_warp4_kernel<<<blocks_x, threads, 0u, ctx.stream>>>(ctx.d_cubes, n, var_count, ctx.d_off, off_words, ctx.d_hits);
                if(cudaGetLastError() != cudaSuccess) { ok = false; }
            }

            std::uint32_t* h_hits = nullptr;
            if(ctx.h_hits_pinned != nullptr && ctx.cap_host_cubes >= n)
            {
                h_hits = ctx.h_hits_pinned;
                if(ok && cudaMemcpyAsync(h_hits, ctx.d_hits, hits_bytes, cudaMemcpyDeviceToHost, ctx.stream) != cudaSuccess) { ok = false; }
            }
            else
            {
                if(ctx.h_hits.size() < n) { ctx.h_hits.resize(n); }
                h_hits = ctx.h_hits.data();
                if(ok && cudaMemcpyAsync(h_hits, ctx.d_hits, hits_bytes, cudaMemcpyDeviceToHost, ctx.stream) != cudaSuccess) { ok = false; }
            }
            if(ok && cudaStreamSynchronize(ctx.stream) != cudaSuccess) { ok = false; }
            if(ok)
            {
                for(std::size_t i{}; i < n; ++i) { out_hits[begin + i] = static_cast<std::uint8_t>(h_hits[i] != 0u); }
            }

            ctx.ok = ok;
            return ok;
        }

        struct espresso_off_device
        {
            int device{-1};
            cudaStream_t stream{};
            std::uint64_t* d_off{};
            std::uint32_t cap_off_words{};
            cuda_cube_desc* d_cubes{};
            std::uint32_t* d_hits{};
            std::uint8_t* d_vars{};
            std::size_t cap_cubes{};
            // Optional pinned host staging to reduce driver overhead for the many small transfers.
            cuda_cube_desc* h_cubes_pinned{};
            std::uint32_t* h_hits_pinned{};
            std::vector<cuda_cube_desc> h_cubes{};
            std::vector<std::uint32_t> h_hits{};
            bool ok{};

            espresso_off_device() noexcept = default;
            espresso_off_device(espresso_off_device const&) = delete;
            espresso_off_device& operator=(espresso_off_device const&) = delete;
            espresso_off_device(espresso_off_device&& o) noexcept :
                device(o.device),
                stream(o.stream),
                d_off(o.d_off),
                cap_off_words(o.cap_off_words),
                d_cubes(o.d_cubes),
                d_hits(o.d_hits),
                d_vars(o.d_vars),
                cap_cubes(o.cap_cubes),
                h_cubes_pinned(o.h_cubes_pinned),
                h_hits_pinned(o.h_hits_pinned),
                h_cubes(std::move(o.h_cubes)),
                h_hits(std::move(o.h_hits)),
                ok(o.ok)
            {
                o.device = -1;
                o.stream = nullptr;
                o.d_off = nullptr;
                o.cap_off_words = 0u;
                o.d_cubes = nullptr;
                o.d_hits = nullptr;
                o.d_vars = nullptr;
                o.cap_cubes = 0u;
                o.h_cubes_pinned = nullptr;
                o.h_hits_pinned = nullptr;
                o.ok = false;
            }
            espresso_off_device& operator=(espresso_off_device&& o) noexcept
            {
                if(this == &o) { return *this; }
                // Must be destroyed/reset by the owner before move-assigning in.
                device = o.device;
                stream = o.stream;
                d_off = o.d_off;
                cap_off_words = o.cap_off_words;
                d_cubes = o.d_cubes;
                d_hits = o.d_hits;
                d_vars = o.d_vars;
                cap_cubes = o.cap_cubes;
                h_cubes_pinned = o.h_cubes_pinned;
                h_hits_pinned = o.h_hits_pinned;
                h_cubes = std::move(o.h_cubes);
                h_hits = std::move(o.h_hits);
                ok = o.ok;
                o.device = -1;
                o.stream = nullptr;
                o.d_off = nullptr;
                o.cap_off_words = 0u;
                o.d_cubes = nullptr;
                o.d_hits = nullptr;
                o.d_vars = nullptr;
                o.cap_cubes = 0u;
                o.h_cubes_pinned = nullptr;
                o.h_hits_pinned = nullptr;
                o.ok = false;
                return *this;
            }
        };

        struct espresso_off_handle
        {
            std::uint32_t var_count{};
            std::uint32_t off_words{};
            std::vector<espresso_off_device> devs{};
        };

        inline void destroy_espresso_off_device(espresso_off_device& d) noexcept
        {
            if(d.d_hits != nullptr) { (void)cudaFree(d.d_hits); }
            if(d.d_cubes != nullptr) { (void)cudaFree(d.d_cubes); }
            if(d.d_off != nullptr) { (void)cudaFree(d.d_off); }
            if(d.d_vars != nullptr) { (void)cudaFree(d.d_vars); }
            if(d.h_hits_pinned != nullptr) { (void)cudaFreeHost(d.h_hits_pinned); }
            if(d.h_cubes_pinned != nullptr) { (void)cudaFreeHost(d.h_cubes_pinned); }
            if(d.stream != nullptr) { (void)cudaStreamDestroy(d.stream); }
            d.device = -1;
            d.stream = nullptr;
            d.d_off = nullptr;
            d.cap_off_words = 0u;
            d.d_cubes = nullptr;
            d.d_hits = nullptr;
            d.d_vars = nullptr;
            d.cap_cubes = 0u;
            d.h_cubes_pinned = nullptr;
            d.h_hits_pinned = nullptr;
            d.h_cubes.clear();
            d.h_hits.clear();
            d.ok = false;
        }

        [[nodiscard]] inline bool init_espresso_off_device(espresso_off_device& d,
                                                           int device,
                                                           std::uint32_t var_count,
                                                           std::uint64_t const* off_blocks,
                                                           std::uint32_t off_words) noexcept
        {
            d.ok = false;
            d.device = device;
            if(cudaSetDevice(device) != cudaSuccess) { return false; }
            if(d.stream == nullptr)
            {
                if(cudaStreamCreateWithFlags(&d.stream, cudaStreamNonBlocking) != cudaSuccess) { return false; }
            }
            auto const off_bytes = static_cast<std::size_t>(off_words) * sizeof(std::uint64_t);
            if(d.d_off == nullptr || d.cap_off_words < off_words)
            {
                if(d.d_off != nullptr) { (void)cudaFree(d.d_off); d.d_off = nullptr; }
                d.cap_off_words = 0u;
                if(cudaMalloc(reinterpret_cast<void**>(&d.d_off), off_bytes) != cudaSuccess) { return false; }
                d.cap_off_words = off_words;
            }
            // Keep this async; subsequent kernels enqueued on the same stream are ordered after this memcpy.
            if(cudaMemcpyAsync(d.d_off, off_blocks, off_bytes, cudaMemcpyHostToDevice, d.stream) != cudaSuccess) { return false; }
            // Small per-device buffer for candidate variable indices (<=16).
            if(d.d_vars == nullptr)
            {
                if(cudaMalloc(reinterpret_cast<void**>(&d.d_vars), 16u * sizeof(std::uint8_t)) != cudaSuccess) { return false; }
            }
            d.ok = true;
            return true;
        }

        // Pool espresso_off_device objects to avoid per-cone cudaMalloc/cudaStreamCreate churn.
        // Espresso is invoked for many small cones, and repeated allocations dominate CPU time otherwise.
        inline std::mutex& espresso_off_pool_mutex() noexcept
        {
            static std::mutex mu{};
            return mu;
        }

        inline std::unordered_map<int, std::vector<espresso_off_device>>& espresso_off_pool() noexcept
        {
            static std::unordered_map<int, std::vector<espresso_off_device>> pool{};
            return pool;
        }

        [[nodiscard]] inline espresso_off_device take_espresso_off_device_from_pool(int device) noexcept
        {
            std::scoped_lock lk(espresso_off_pool_mutex());
            auto& pool = espresso_off_pool();
            auto it = pool.find(device);
            if(it == pool.end() || it->second.empty())
            {
                espresso_off_device d{};
                d.device = device;
                return d;
            }
            auto d = std::move(it->second.back());
            it->second.pop_back();
            return d;
        }

        inline void recycle_espresso_off_device_to_pool(espresso_off_device&& d) noexcept
        {
            if(d.device < 0) { return; }
            std::scoped_lock lk(espresso_off_pool_mutex());
            espresso_off_pool()[d.device].push_back(std::move(d));
        }

        [[nodiscard]] inline bool ensure_espresso_off_cubes(espresso_off_device& d, std::size_t n) noexcept
        {
            if(n <= d.cap_cubes) { return true; }
            if(cudaSetDevice(d.device) != cudaSuccess) { return false; }
            if(d.d_hits != nullptr) { (void)cudaFree(d.d_hits); d.d_hits = nullptr; }
            if(d.d_cubes != nullptr) { (void)cudaFree(d.d_cubes); d.d_cubes = nullptr; }
            if(d.h_hits_pinned != nullptr) { (void)cudaFreeHost(d.h_hits_pinned); d.h_hits_pinned = nullptr; }
            if(d.h_cubes_pinned != nullptr) { (void)cudaFreeHost(d.h_cubes_pinned); d.h_cubes_pinned = nullptr; }
            d.cap_cubes = 0u;
            if(cudaMalloc(reinterpret_cast<void**>(&d.d_cubes), n * sizeof(cuda_cube_desc)) != cudaSuccess) { return false; }
            if(cudaMalloc(reinterpret_cast<void**>(&d.d_hits), n * sizeof(std::uint32_t)) != cudaSuccess) { return false; }
            // Try pinned staging; if it fails, fall back to normal host vectors.
            bool pinned_ok = true;
            if(cudaHostAlloc(reinterpret_cast<void**>(&d.h_cubes_pinned), n * sizeof(cuda_cube_desc), cudaHostAllocPortable) != cudaSuccess) { pinned_ok = false; }
            if(pinned_ok && cudaHostAlloc(reinterpret_cast<void**>(&d.h_hits_pinned), n * sizeof(std::uint32_t), cudaHostAllocPortable) != cudaSuccess) { pinned_ok = false; }
            if(!pinned_ok)
            {
                if(d.h_hits_pinned != nullptr) { (void)cudaFreeHost(d.h_hits_pinned); d.h_hits_pinned = nullptr; }
                if(d.h_cubes_pinned != nullptr) { (void)cudaFreeHost(d.h_cubes_pinned); d.h_cubes_pinned = nullptr; }
                d.h_cubes.resize(n);
                d.h_hits.resize(n);
            }
            d.cap_cubes = n;
            return true;
        }

        [[nodiscard]] inline bool ensure_espresso_off_vars(espresso_off_device& d) noexcept
        {
            if(d.d_vars != nullptr) { return true; }
            if(cudaSetDevice(d.device) != cudaSuccess) { return false; }
            return cudaMalloc(reinterpret_cast<void**>(&d.d_vars), 16u * sizeof(std::uint8_t)) == cudaSuccess;
        }

        [[nodiscard]] inline bool enqueue_espresso_off_hits_on_device(espresso_off_device& d,
                                                                      std::size_t begin,
                                                                      std::size_t end,
                                                                      cuda_cube_desc const* cubes,
                                                                      std::uint32_t var_count,
                                                                      std::uint32_t off_words) noexcept
        {
            if(end <= begin) { d.ok = true; return true; }
            if(cubes == nullptr) { return false; }
            auto const n = end - begin;
            if(cudaSetDevice(d.device) != cudaSuccess) { return false; }
            if(!ensure_espresso_off_cubes(d, n)) { return false; }

            auto const cubes_bytes = n * sizeof(cuda_cube_desc);
            auto const hits_bytes = n * sizeof(std::uint32_t);

            bool ok = true;
            if(d.h_cubes_pinned != nullptr)
            {
                ::std::memcpy(d.h_cubes_pinned, cubes + begin, cubes_bytes);
                if(cudaMemcpyAsync(d.d_cubes, d.h_cubes_pinned, cubes_bytes, cudaMemcpyHostToDevice, d.stream) != cudaSuccess) { ok = false; }
            }
            else
            {
                // Pageable fallback: still works, just higher overhead.
                if(d.h_cubes.size() < n) { d.h_cubes.resize(n); }
                ::std::memcpy(d.h_cubes.data(), cubes + begin, cubes_bytes);
                if(cudaMemcpyAsync(d.d_cubes, d.h_cubes.data(), cubes_bytes, cudaMemcpyHostToDevice, d.stream) != cudaSuccess) { ok = false; }
            }

            if(ok)
            {
                constexpr unsigned threads = 128u; // 4 warps per block
                auto const blocks_x = static_cast<unsigned>((n + 4u - 1u) / 4u);
                espresso_cube_hits_off_warp4_kernel<<<blocks_x, threads, 0u, d.stream>>>(d.d_cubes, n, var_count, d.d_off, off_words, d.d_hits);
                if(cudaGetLastError() != cudaSuccess) { ok = false; }
            }

            if(ok)
            {
                if(d.h_hits_pinned != nullptr)
                {
                    if(cudaMemcpyAsync(d.h_hits_pinned, d.d_hits, hits_bytes, cudaMemcpyDeviceToHost, d.stream) != cudaSuccess) { ok = false; }
                }
                else
                {
                    if(d.h_hits.size() < n) { d.h_hits.resize(n); }
                    if(cudaMemcpyAsync(d.h_hits.data(), d.d_hits, hits_bytes, cudaMemcpyDeviceToHost, d.stream) != cudaSuccess) { ok = false; }
                }
            }
            d.ok = ok;
            return ok;
        }

        [[nodiscard]] inline bool enqueue_espresso_off_expand_best_on_device(espresso_off_device& d,
                                                                             std::size_t begin,
                                                                             std::size_t end,
                                                                             cuda_cube_desc* cubes,
                                                                             std::uint32_t var_count,
                                                                             std::uint32_t off_words,
                                                                             std::uint8_t const* cand_vars,
                                                                             std::uint32_t cand_vars_count,
                                                                             std::uint32_t max_rounds) noexcept
        {
            if(end <= begin) { d.ok = true; return true; }
            if(cubes == nullptr || cand_vars == nullptr || cand_vars_count == 0u) { return false; }
            if(cand_vars_count > 16u) { cand_vars_count = 16u; }
            auto const n = end - begin;
            if(cudaSetDevice(d.device) != cudaSuccess) { return false; }
            if(!ensure_espresso_off_cubes(d, n)) { return false; }
            if(!ensure_espresso_off_vars(d)) { return false; }

            auto const cubes_bytes = n * sizeof(cuda_cube_desc);

            bool ok = true;
            if(d.h_cubes_pinned != nullptr)
            {
                ::std::memcpy(d.h_cubes_pinned, cubes + begin, cubes_bytes);
                if(cudaMemcpyAsync(d.d_cubes, d.h_cubes_pinned, cubes_bytes, cudaMemcpyHostToDevice, d.stream) != cudaSuccess) { ok = false; }
            }
            else
            {
                if(d.h_cubes.size() < n) { d.h_cubes.resize(n); }
                ::std::memcpy(d.h_cubes.data(), cubes + begin, cubes_bytes);
                if(cudaMemcpyAsync(d.d_cubes, d.h_cubes.data(), cubes_bytes, cudaMemcpyHostToDevice, d.stream) != cudaSuccess) { ok = false; }
            }
            if(ok && cudaMemcpyAsync(d.d_vars, cand_vars, cand_vars_count * sizeof(std::uint8_t), cudaMemcpyHostToDevice, d.stream) != cudaSuccess) { ok = false; }

            if(ok)
            {
                dim3 grid(static_cast<unsigned>(n), 1u, 1u);
                constexpr unsigned threads = 32u;  // one warp per cube
                espresso_off_expand_best_kernel<<<grid, threads, 0u, d.stream>>>(d.d_cubes,
                                                                                 n,
                                                                                 var_count,
                                                                                 d.d_off,
                                                                                 off_words,
                                                                                 d.d_vars,
                                                                                 cand_vars_count,
                                                                                 max_rounds);
                if(cudaGetLastError() != cudaSuccess) { ok = false; }
            }

            if(ok)
            {
                if(d.h_cubes_pinned != nullptr)
                {
                    if(cudaMemcpyAsync(d.h_cubes_pinned, d.d_cubes, cubes_bytes, cudaMemcpyDeviceToHost, d.stream) != cudaSuccess) { ok = false; }
                }
                else
                {
                    if(d.h_cubes.size() < n) { d.h_cubes.resize(n); }
                    if(cudaMemcpyAsync(d.h_cubes.data(), d.d_cubes, cubes_bytes, cudaMemcpyDeviceToHost, d.stream) != cudaSuccess) { ok = false; }
                }
            }
            d.ok = ok;
            return ok;
        }

        [[nodiscard]] inline bool init_bitset_matrix_device(bitset_matrix_device& d,
                                                            std::uint64_t const* rows,
                                                            std::size_t row_count,
                                                            std::uint32_t stride_words) noexcept
        {
            d.ok = false;
            if(d.end <= d.begin) { d.ok = true; return true; }

            if(cudaSetDevice(d.device) != cudaSuccess) { return false; }
            if(cudaStreamCreateWithFlags(&d.stream, cudaStreamNonBlocking) != cudaSuccess) { return false; }

            auto const n = d.end - d.begin;
            auto const words = n * static_cast<std::size_t>(stride_words);
            auto const rows_bytes = words * sizeof(std::uint64_t);
            auto const mask_bytes = static_cast<std::size_t>(stride_words) * sizeof(std::uint64_t);
            auto const popc_bytes = n * sizeof(std::uint32_t);
            auto const any_bytes = n * sizeof(std::uint32_t);

            bool ok = true;
            if(cudaMalloc(reinterpret_cast<void**>(&d.d_rows), rows_bytes) != cudaSuccess) { ok = false; }
            if(ok && cudaMalloc(reinterpret_cast<void**>(&d.d_mask), mask_bytes) != cudaSuccess) { ok = false; }
            if(ok && cudaMalloc(reinterpret_cast<void**>(&d.d_popc), popc_bytes) != cudaSuccess) { ok = false; }
            if(ok && cudaMalloc(reinterpret_cast<void**>(&d.d_any), any_bytes) != cudaSuccess) { ok = false; }

            if(ok && cudaMemcpyAsync(d.d_rows,
                                     rows + d.begin * static_cast<std::size_t>(stride_words),
                                     rows_bytes,
                                     cudaMemcpyHostToDevice,
                                     d.stream) != cudaSuccess)
            {
                ok = false;
            }

            if(ok && cudaStreamSynchronize(d.stream) != cudaSuccess) { ok = false; }
            d.ok = ok;
            return ok;
        }

        [[nodiscard]] inline bool init_bitset_matrix_device_empty(bitset_matrix_device& d, std::uint32_t stride_words) noexcept
        {
            d.ok = false;
            if(d.end <= d.begin) { d.ok = true; return true; }

            if(cudaSetDevice(d.device) != cudaSuccess) { return false; }
            if(cudaStreamCreateWithFlags(&d.stream, cudaStreamNonBlocking) != cudaSuccess) { return false; }

            auto const n = d.end - d.begin;
            auto const words = n * static_cast<std::size_t>(stride_words);
            auto const rows_bytes = words * sizeof(std::uint64_t);
            auto const mask_bytes = static_cast<std::size_t>(stride_words) * sizeof(std::uint64_t);
            auto const popc_bytes = n * sizeof(std::uint32_t);
            auto const any_bytes = n * sizeof(std::uint32_t);

            bool ok = true;
            if(cudaMalloc(reinterpret_cast<void**>(&d.d_rows), rows_bytes) != cudaSuccess) { ok = false; }
            if(ok && cudaMalloc(reinterpret_cast<void**>(&d.d_mask), mask_bytes) != cudaSuccess) { ok = false; }
            if(ok && cudaMalloc(reinterpret_cast<void**>(&d.d_popc), popc_bytes) != cudaSuccess) { ok = false; }
            if(ok && cudaMalloc(reinterpret_cast<void**>(&d.d_any), any_bytes) != cudaSuccess) { ok = false; }
            if(ok && cudaMemsetAsync(d.d_rows, 0, rows_bytes, d.stream) != cudaSuccess) { ok = false; }

            if(ok && cudaStreamSynchronize(d.stream) != cudaSuccess) { ok = false; }
            d.ok = ok;
            return ok;
        }

        inline void destroy_bitset_matrix_device(bitset_matrix_device& d) noexcept
        {
            if(d.d_any != nullptr) { (void)cudaFree(d.d_any); }
            if(d.d_popc != nullptr) { (void)cudaFree(d.d_popc); }
            if(d.d_row_tmp != nullptr) { (void)cudaFree(d.d_row_tmp); }
            if(d.d_mask != nullptr) { (void)cudaFree(d.d_mask); }
            if(d.d_rows != nullptr) { (void)cudaFree(d.d_rows); }
            if(d.d_cost != nullptr) { (void)cudaFree(d.d_cost); }
            if(d.d_active != nullptr) { (void)cudaFree(d.d_active); }
            if(d.d_best_parts != nullptr) { (void)cudaFree(d.d_best_parts); }
            if(d.d_best_one != nullptr) { (void)cudaFree(d.d_best_one); }
            if(d.d_on_tmp != nullptr) { (void)cudaFree(d.d_on_tmp); }
            if(d.d_cubes_tmp != nullptr) { (void)cudaFree(d.d_cubes_tmp); }
            if(d.stream != nullptr) { (void)cudaStreamDestroy(d.stream); }
            d.device = 0;
            d.begin = 0;
            d.end = 0;
            d.stride_words = 0;
            d.stream = nullptr;
            d.d_rows = nullptr;
            d.d_mask = nullptr;
            d.d_row_tmp = nullptr;
            d.d_popc = nullptr;
            d.d_any = nullptr;
            d.d_cost = nullptr;
            d.d_active = nullptr;
            d.d_best_parts = nullptr;
            d.d_best_one = nullptr;
            d.cap_best_parts = 0u;
            d.d_cubes_tmp = nullptr;
            d.d_on_tmp = nullptr;
            d.cap_cubes_tmp = 0u;
            d.cap_on_tmp = 0u;
            d.h_tmp.clear();
            d.ok = false;
        }

        [[nodiscard]] inline bool ensure_bitset_matrix_best_buffers(bitset_matrix_device& d) noexcept
        {
            if(d.end <= d.begin) { return true; }
            if(cudaSetDevice(d.device) != cudaSuccess) { return false; }
            auto const n = static_cast<std::uint32_t>(d.end - d.begin);

            if(d.stream == nullptr)
            {
                if(cudaStreamCreateWithFlags(&d.stream, cudaStreamNonBlocking) != cudaSuccess) { return false; }
            }

            // Active flags (default: all active).
            if(d.d_active == nullptr)
            {
                if(cudaMalloc(reinterpret_cast<void**>(&d.d_active), static_cast<std::size_t>(n) * sizeof(std::uint8_t)) != cudaSuccess) { return false; }
                if(cudaMemsetAsync(d.d_active, 1, static_cast<std::size_t>(n) * sizeof(std::uint8_t), d.stream) != cudaSuccess) { return false; }
            }

            // Reduction buffers for best-row selection.
            constexpr std::uint32_t items_per_block = 1024u;
            auto const parts = (n + items_per_block - 1u) / items_per_block;
            if(d.d_best_one == nullptr)
            {
                if(cudaMalloc(reinterpret_cast<void**>(&d.d_best_one), sizeof(best_item)) != cudaSuccess) { return false; }
            }
            if(d.d_best_parts == nullptr || d.cap_best_parts < parts)
            {
                if(d.d_best_parts != nullptr) { (void)cudaFree(d.d_best_parts); d.d_best_parts = nullptr; }
                d.cap_best_parts = 0u;
                if(cudaMalloc(reinterpret_cast<void**>(&d.d_best_parts), static_cast<std::size_t>(parts) * sizeof(best_item)) != cudaSuccess) { return false; }
                d.cap_best_parts = parts;
            }
            return true;
        }

        [[nodiscard]] inline bool ensure_bitset_matrix_fill_buffers(bitset_matrix_device& d, std::size_t cube_count, std::size_t on_count) noexcept
        {
            if(cube_count == 0u || on_count == 0u) { return false; }
            if(cudaSetDevice(d.device) != cudaSuccess) { return false; }
            if(d.stream == nullptr)
            {
                if(cudaStreamCreateWithFlags(&d.stream, cudaStreamNonBlocking) != cudaSuccess) { return false; }
            }

            if(d.d_cubes_tmp == nullptr || d.cap_cubes_tmp < cube_count)
            {
                if(d.d_cubes_tmp != nullptr) { (void)cudaFree(d.d_cubes_tmp); d.d_cubes_tmp = nullptr; }
                d.cap_cubes_tmp = 0u;
                if(cudaMalloc(reinterpret_cast<void**>(&d.d_cubes_tmp), cube_count * sizeof(cuda_cube_desc)) != cudaSuccess) { return false; }
                d.cap_cubes_tmp = cube_count;
            }
            if(d.d_on_tmp == nullptr || d.cap_on_tmp < on_count)
            {
                if(d.d_on_tmp != nullptr) { (void)cudaFree(d.d_on_tmp); d.d_on_tmp = nullptr; }
                d.cap_on_tmp = 0u;
                if(cudaMalloc(reinterpret_cast<void**>(&d.d_on_tmp), on_count * sizeof(std::uint16_t)) != cudaSuccess) { return false; }
                d.cap_on_tmp = on_count;
            }
            return true;
        }

        [[nodiscard]] inline bool bitset_matrix_row_and_popcount_on_device(bitset_matrix_device& d,
                                                                           std::uint64_t const* mask,
                                                                           std::uint32_t mask_words,
                                                                           std::uint32_t* out_counts) noexcept
        {
            d.ok = false;
            if(d.end <= d.begin) { d.ok = true; return true; }
            if(mask == nullptr || out_counts == nullptr) { return false; }
            if(mask_words != d.stride_words) { return false; }

            if(cudaSetDevice(d.device) != cudaSuccess) { return false; }

            auto const n = d.end - d.begin;
            auto const mask_bytes = static_cast<std::size_t>(d.stride_words) * sizeof(std::uint64_t);
            auto const popc_bytes = n * sizeof(std::uint32_t);

            bool ok = true;
            if(cudaMemcpyAsync(d.d_mask, mask, mask_bytes, cudaMemcpyHostToDevice, d.stream) != cudaSuccess) { ok = false; }
            if(ok && cudaMemsetAsync(d.d_popc, 0, popc_bytes, d.stream) != cudaSuccess) { ok = false; }

            if(ok)
            {
                constexpr unsigned threads = 128u;
                auto const grid_y = static_cast<unsigned>((d.stride_words + threads - 1u) / threads);
                dim3 grid(static_cast<unsigned>(n), grid_y, 1u);
                bitset_row_and_popcount_kernel<<<grid, threads, threads * sizeof(std::uint32_t), d.stream>>>(
                    d.d_rows, n, d.stride_words, d.d_mask, d.d_popc);
                if(cudaGetLastError() != cudaSuccess) { ok = false; }
            }

            if(ok && cudaMemcpyAsync(out_counts + d.begin, d.d_popc, popc_bytes, cudaMemcpyDeviceToHost, d.stream) != cudaSuccess) { ok = false; }
            if(ok && cudaStreamSynchronize(d.stream) != cudaSuccess) { ok = false; }

            d.ok = ok;
            return ok;
        }

        [[nodiscard]] inline bool bitset_matrix_row_any_and_on_device(bitset_matrix_device& d,
                                                                      std::uint64_t const* mask,
                                                                      std::uint32_t mask_words,
                                                                      std::uint8_t* out_any) noexcept
        {
            d.ok = false;
            if(d.end <= d.begin) { d.ok = true; return true; }
            if(mask == nullptr || out_any == nullptr) { return false; }
            if(mask_words != d.stride_words) { return false; }

            if(cudaSetDevice(d.device) != cudaSuccess) { return false; }

            auto const n = d.end - d.begin;
            auto const mask_bytes = static_cast<std::size_t>(d.stride_words) * sizeof(std::uint64_t);
            auto const any_bytes = n * sizeof(std::uint32_t);

            bool ok = true;
            if(cudaMemcpyAsync(d.d_mask, mask, mask_bytes, cudaMemcpyHostToDevice, d.stream) != cudaSuccess) { ok = false; }
            if(ok && cudaMemsetAsync(d.d_any, 0, any_bytes, d.stream) != cudaSuccess) { ok = false; }

            if(ok)
            {
                constexpr unsigned threads = 256u;
                auto const grid_y = static_cast<unsigned>((d.stride_words + threads - 1u) / threads);
                dim3 grid(static_cast<unsigned>(n), grid_y, 1u);
                bitset_row_any_and_kernel<<<grid, threads, 0u, d.stream>>>(d.d_rows, n, d.stride_words, d.d_mask, d.d_any);
                if(cudaGetLastError() != cudaSuccess) { ok = false; }
            }

            std::vector<std::uint32_t> tmp{};
            tmp.resize(n);
            if(ok && cudaMemcpyAsync(tmp.data(), d.d_any, any_bytes, cudaMemcpyDeviceToHost, d.stream) != cudaSuccess) { ok = false; }
            if(ok && cudaStreamSynchronize(d.stream) != cudaSuccess) { ok = false; }
            if(ok)
            {
                for(std::size_t i{}; i < n; ++i) { out_any[d.begin + i] = static_cast<std::uint8_t>(tmp[i] != 0u); }
            }

            d.ok = ok;
            return ok;
        }
    }  // namespace

    extern "C" int phy_engine_pe_synth_cuda_get_device_count() noexcept
    {
        int n{};
        if(cudaGetDeviceCount(&n) != cudaSuccess) { return 0; }
        return n;
    }

    extern "C" bool phy_engine_pe_synth_cuda_eval_u64_cones(std::uint32_t device_mask,
                                                            cuda_u64_cone_desc const* cones,
                                                            std::size_t cone_count,
                                                            std::uint64_t* out_masks) noexcept
    {
        if(cones == nullptr || out_masks == nullptr || cone_count == 0u) { return false; }

        int dev_count{};
        if(cudaGetDeviceCount(&dev_count) != cudaSuccess || dev_count <= 0) { return false; }

        std::vector<int> devices{};
        devices.reserve(static_cast<std::size_t>(dev_count));
        if(device_mask == 0u)
        {
            for(int d = 0; d < dev_count; ++d) { devices.push_back(d); }
        }
        else
        {
            for(int d = 0; d < dev_count; ++d)
            {
                if((device_mask >> static_cast<unsigned>(d)) & 1u) { devices.push_back(d); }
            }
        }

        if(devices.empty()) { return false; }

        // Split cones across selected devices (contiguous chunks).
        std::vector<device_chunk> chunks{};
        chunks.reserve(devices.size());
        auto const k = devices.size();
        for(std::size_t i{}; i < k; ++i)
        {
            auto const begin = (cone_count * i) / k;
            auto const end = (cone_count * (i + 1u)) / k;
            chunks.push_back(device_chunk{devices[i], begin, end, false});
        }

        struct work
        {
            device_chunk* chunk{};
            eval_u64_ctx* ctx{};
            std::unique_lock<std::mutex> lk{};
        };

        std::vector<work> worklist{};
        worklist.reserve(chunks.size());

        // Enqueue on all selected devices first.
        for(auto& c: chunks)
        {
            c.ok = false;
            if(c.end <= c.begin) { c.ok = true; continue; }

            auto const n = c.end - c.begin;
            auto const cones_bytes = n * sizeof(cuda_u64_cone_desc);
            auto const out_bytes = n * sizeof(std::uint64_t);

            auto& ctx = get_u64_ctx(c.device);
            worklist.push_back(work{__builtin_addressof(c), __builtin_addressof(ctx), std::unique_lock<std::mutex>(ctx.mu)});

            bool ok = ensure_u64_ctx_ready(ctx, n);
            if(ok)
            {
                if(ctx.h_cones_pinned != nullptr && ctx.cap_host_cones >= n)
                {
                    ::std::memcpy(ctx.h_cones_pinned, cones + c.begin, cones_bytes);
                    if(cudaMemcpyAsync(ctx.d_cones, ctx.h_cones_pinned, cones_bytes, cudaMemcpyHostToDevice, ctx.stream) != cudaSuccess) { ok = false; }
                }
                else
                {
                    if(cudaMemcpyAsync(ctx.d_cones, cones + c.begin, cones_bytes, cudaMemcpyHostToDevice, ctx.stream) != cudaSuccess) { ok = false; }
                }
            }
            if(ok)
            {
                constexpr unsigned threads = 256u;
                auto const blocks = static_cast<unsigned>((n + threads - 1u) / threads);
                eval_u64_cones_kernel<<<blocks, threads, 0u, ctx.stream>>>(ctx.d_cones, n, ctx.d_out);
                if(cudaGetLastError() != cudaSuccess) { ok = false; }
            }
            if(ok)
            {
                if(ctx.h_out_pinned != nullptr && ctx.cap_host_cones >= n)
                {
                    if(cudaMemcpyAsync(ctx.h_out_pinned, ctx.d_out, out_bytes, cudaMemcpyDeviceToHost, ctx.stream) != cudaSuccess) { ok = false; }
                }
                else
                {
                    if(cudaMemcpyAsync(out_masks + c.begin, ctx.d_out, out_bytes, cudaMemcpyDeviceToHost, ctx.stream) != cudaSuccess) { ok = false; }
                }
            }
            ctx.ok = ok;
        }

        // Synchronize streams (still overlapped across devices).
        bool all_ok = true;
        for(auto& w: worklist)
        {
            auto& c = *w.chunk;
            auto& ctx = *w.ctx;
            bool ok = ctx.ok;
            if(ok && cudaStreamSynchronize(ctx.stream) != cudaSuccess) { ok = false; }
            if(ok && ctx.h_out_pinned != nullptr && c.end > c.begin && ctx.cap_host_cones >= (c.end - c.begin))
            {
                auto const n = c.end - c.begin;
                ::std::memcpy(out_masks + c.begin, ctx.h_out_pinned, n * sizeof(std::uint64_t));
            }
            c.ok = ok;
            ctx.ok = ok;
            if(!ok) { all_ok = false; }
            // unlock ctx.mu here via w.lk destructor when worklist is destroyed
        }

        if(!all_ok) { return false; }
        for(auto const& c: chunks)
        {
            if(!c.ok) { return false; }
        }
        return true;
    }

    extern "C" bool phy_engine_pe_synth_cuda_eval_tt_cones(std::uint32_t device_mask,
                                                           cuda_tt_cone_desc const* cones,
                                                           std::size_t cone_count,
                                                           std::uint32_t stride_blocks,
                                                           std::uint64_t* out_blocks) noexcept
    {
        if(cones == nullptr || out_blocks == nullptr || cone_count == 0u || stride_blocks == 0u) { return false; }

        int dev_count{};
        if(cudaGetDeviceCount(&dev_count) != cudaSuccess || dev_count <= 0) { return false; }

        std::vector<int> devices{};
        devices.reserve(static_cast<std::size_t>(dev_count));
        if(device_mask == 0u)
        {
            for(int d = 0; d < dev_count; ++d) { devices.push_back(d); }
        }
        else
        {
            for(int d = 0; d < dev_count; ++d)
            {
                if((device_mask >> static_cast<unsigned>(d)) & 1u) { devices.push_back(d); }
            }
        }

        if(devices.empty()) { return false; }

        // Split cones across selected devices (contiguous chunks).
        std::vector<device_chunk> chunks{};
        chunks.reserve(devices.size());
        auto const k = devices.size();
        for(std::size_t i{}; i < k; ++i)
        {
            auto const begin = (cone_count * i) / k;
            auto const end = (cone_count * (i + 1u)) / k;
            chunks.push_back(device_chunk{devices[i], begin, end, false});
        }

        struct work
        {
            device_chunk* chunk{};
            eval_tt_ctx* ctx{};
            std::unique_lock<std::mutex> lk{};
        };

        std::vector<work> worklist{};
        worklist.reserve(chunks.size());

        for(auto& c: chunks)
        {
            c.ok = false;
            if(c.end <= c.begin) { c.ok = true; continue; }

            auto const n = c.end - c.begin;
            auto const cones_bytes = n * sizeof(cuda_tt_cone_desc);
            auto const out_words = n * static_cast<std::size_t>(stride_blocks);
            auto const out_bytes = out_words * sizeof(std::uint64_t);

            auto& ctx = get_tt_ctx(c.device);
            worklist.push_back(work{__builtin_addressof(c), __builtin_addressof(ctx), std::unique_lock<std::mutex>(ctx.mu)});

            bool ok = ensure_tt_ctx_ready(ctx, n, stride_blocks);
            if(ok)
            {
                if(ctx.h_cones_pinned != nullptr && ctx.cap_host_cones >= n)
                {
                    ::std::memcpy(ctx.h_cones_pinned, cones + c.begin, cones_bytes);
                    if(cudaMemcpyAsync(ctx.d_cones, ctx.h_cones_pinned, cones_bytes, cudaMemcpyHostToDevice, ctx.stream) != cudaSuccess) { ok = false; }
                }
                else
                {
                    if(cudaMemcpyAsync(ctx.d_cones, cones + c.begin, cones_bytes, cudaMemcpyHostToDevice, ctx.stream) != cudaSuccess) { ok = false; }
                }
            }
            if(ok)
            {
                constexpr unsigned threads = 128u;
                auto const grid_y = static_cast<unsigned>((stride_blocks + threads - 1u) / threads);
                dim3 grid(static_cast<unsigned>(n), grid_y, 1u);
                eval_tt_cones_kernel<<<grid, threads, 0u, ctx.stream>>>(ctx.d_cones, n, stride_blocks, ctx.d_out);
                if(cudaGetLastError() != cudaSuccess) { ok = false; }
            }
            if(ok)
            {
                if(ctx.h_out_pinned != nullptr && ctx.cap_host_cones >= n && ctx.cap_host_stride_blocks >= stride_blocks)
                {
                    if(cudaMemcpyAsync(ctx.h_out_pinned, ctx.d_out, out_bytes, cudaMemcpyDeviceToHost, ctx.stream) != cudaSuccess) { ok = false; }
                }
                else
                {
                    if(cudaMemcpyAsync(out_blocks + c.begin * static_cast<std::size_t>(stride_blocks), ctx.d_out, out_bytes, cudaMemcpyDeviceToHost, ctx.stream) != cudaSuccess)
                    {
                        ok = false;
                    }
                }
            }
            ctx.ok = ok;
        }

        bool all_ok = true;
        for(auto& w: worklist)
        {
            auto& c = *w.chunk;
            auto& ctx = *w.ctx;
            bool ok = ctx.ok;
            if(ok && cudaStreamSynchronize(ctx.stream) != cudaSuccess) { ok = false; }
            if(ok && ctx.h_out_pinned != nullptr && c.end > c.begin && ctx.cap_host_cones >= (c.end - c.begin) && ctx.cap_host_stride_blocks >= stride_blocks)
            {
                auto const n = c.end - c.begin;
                auto const out_words = n * static_cast<std::size_t>(stride_blocks);
                ::std::memcpy(out_blocks + c.begin * static_cast<std::size_t>(stride_blocks), ctx.h_out_pinned, out_words * sizeof(std::uint64_t));
            }
            c.ok = ok;
            ctx.ok = ok;
            if(!ok) { all_ok = false; }
        }

        if(!all_ok) { return false; }
        for(auto const& c: chunks)
        {
            if(!c.ok) { return false; }
        }
        return true;
    }

    extern "C" void* phy_engine_pe_synth_cuda_bitset_matrix_create(std::uint32_t device_mask,
                                                                   std::uint64_t const* rows,
                                                                   std::size_t row_count,
                                                                   std::uint32_t stride_words) noexcept
    {
        if(rows == nullptr || row_count == 0u || stride_words == 0u) { return nullptr; }

        int dev_count{};
        if(cudaGetDeviceCount(&dev_count) != cudaSuccess || dev_count <= 0) { return nullptr; }

        std::vector<int> devices{};
        devices.reserve(static_cast<std::size_t>(dev_count));
        if(device_mask == 0u)
        {
            for(int d = 0; d < dev_count; ++d) { devices.push_back(d); }
        }
        else
        {
            for(int d = 0; d < dev_count; ++d)
            {
                if((device_mask >> static_cast<unsigned>(d)) & 1u) { devices.push_back(d); }
            }
        }

        if(devices.empty()) { return nullptr; }

        auto* h = new(std::nothrow) bitset_matrix_handle{};
        if(h == nullptr) { return nullptr; }

        h->device_mask = device_mask;
        h->row_count = row_count;
        h->stride_words = stride_words;

        // Split rows across selected devices (contiguous chunks).
        auto const k = devices.size();
        h->devs.reserve(k);
        for(std::size_t i{}; i < k; ++i)
        {
            auto const begin = (row_count * i) / k;
            auto const end = (row_count * (i + 1u)) / k;
            bitset_matrix_device d{};
            d.device = devices[i];
            d.begin = begin;
            d.end = end;
            d.stride_words = stride_words;
            h->devs.push_back(std::move(d));
        }

        bool ok = true;
        for(auto& d: h->devs)
        {
            if(!init_bitset_matrix_device(d, rows, row_count, stride_words))
            {
                ok = false;
                break;
            }
        }

        if(!ok)
        {
            for(auto& d: h->devs) { destroy_bitset_matrix_device(d); }
            delete h;
            return nullptr;
        }

        return reinterpret_cast<void*>(h);
    }

    extern "C" void* phy_engine_pe_synth_cuda_bitset_matrix_create_empty(std::uint32_t device_mask,
                                                                         std::size_t row_count,
                                                                         std::uint32_t stride_words) noexcept
    {
        if(row_count == 0u || stride_words == 0u) { return nullptr; }

        int dev_count{};
        if(cudaGetDeviceCount(&dev_count) != cudaSuccess || dev_count <= 0) { return nullptr; }

        std::vector<int> devices{};
        devices.reserve(static_cast<std::size_t>(dev_count));
        if(device_mask == 0u)
        {
            for(int d = 0; d < dev_count; ++d) { devices.push_back(d); }
        }
        else
        {
            for(int d = 0; d < dev_count; ++d)
            {
                if((device_mask >> static_cast<unsigned>(d)) & 1u) { devices.push_back(d); }
            }
        }

        if(devices.empty()) { return nullptr; }

        auto* h = new(std::nothrow) bitset_matrix_handle{};
        if(h == nullptr) { return nullptr; }

        h->device_mask = device_mask;
        h->row_count = row_count;
        h->stride_words = stride_words;

        auto const k = devices.size();
        h->devs.reserve(k);
        for(std::size_t i{}; i < k; ++i)
        {
            auto const begin = (row_count * i) / k;
            auto const end = (row_count * (i + 1u)) / k;
            bitset_matrix_device d{};
            d.device = devices[i];
            d.begin = begin;
            d.end = end;
            d.stride_words = stride_words;
            h->devs.push_back(std::move(d));
        }

        bool ok = true;
        for(auto& d: h->devs)
        {
            if(!init_bitset_matrix_device_empty(d, stride_words))
            {
                ok = false;
                break;
            }
        }

        if(!ok)
        {
            for(auto& d: h->devs) { destroy_bitset_matrix_device(d); }
            delete h;
            return nullptr;
        }

        return reinterpret_cast<void*>(h);
    }

    extern "C" void phy_engine_pe_synth_cuda_bitset_matrix_destroy(void* handle) noexcept
    {
        if(handle == nullptr) { return; }
        auto* h = reinterpret_cast<bitset_matrix_handle*>(handle);
        for(auto& d: h->devs) { destroy_bitset_matrix_device(d); }
        delete h;
    }

    extern "C" bool phy_engine_pe_synth_cuda_bitset_matrix_row_and_popcount(void* handle,
                                                                            std::uint64_t const* mask,
                                                                            std::uint32_t mask_words,
                                                                            std::uint32_t* out_counts) noexcept
    {
        if(handle == nullptr || mask == nullptr || out_counts == nullptr) { return false; }
        auto* h = reinterpret_cast<bitset_matrix_handle*>(handle);
        if(h->stride_words == 0u || h->row_count == 0u) { return false; }
        if(mask_words != h->stride_words) { return false; }

        // Enqueue work on each device stream (single host thread), then sync.
        for(auto& d: h->devs)
        {
            d.ok = false;
            if(d.end <= d.begin) { d.ok = true; continue; }
            if(cudaSetDevice(d.device) != cudaSuccess) { d.ok = false; continue; }

            auto const n = d.end - d.begin;
            auto const mask_bytes = static_cast<std::size_t>(d.stride_words) * sizeof(std::uint64_t);
            auto const popc_bytes = n * sizeof(std::uint32_t);

            bool ok = true;
            if(cudaMemcpyAsync(d.d_mask, mask, mask_bytes, cudaMemcpyHostToDevice, d.stream) != cudaSuccess) { ok = false; }
            if(ok && cudaMemsetAsync(d.d_popc, 0, popc_bytes, d.stream) != cudaSuccess) { ok = false; }
            if(ok)
            {
                constexpr unsigned threads = 128u;
                auto const grid_y = static_cast<unsigned>((d.stride_words + threads - 1u) / threads);
                dim3 grid(static_cast<unsigned>(n), grid_y, 1u);
                bitset_row_and_popcount_kernel<<<grid, threads, threads * sizeof(std::uint32_t), d.stream>>>(
                    d.d_rows, n, d.stride_words, d.d_mask, d.d_popc);
                if(cudaGetLastError() != cudaSuccess) { ok = false; }
            }
            if(ok && cudaMemcpyAsync(out_counts + d.begin, d.d_popc, popc_bytes, cudaMemcpyDeviceToHost, d.stream) != cudaSuccess) { ok = false; }
            d.ok = ok;
        }

        for(auto& d: h->devs)
        {
            if(!d.ok) { return false; }
            if(cudaSetDevice(d.device) != cudaSuccess) { return false; }
            if(cudaStreamSynchronize(d.stream) != cudaSuccess) { return false; }
        }
        return true;
    }

    extern "C" bool phy_engine_pe_synth_cuda_bitset_matrix_row_any_and(void* handle,
                                                                       std::uint64_t const* mask,
                                                                       std::uint32_t mask_words,
                                                                       std::uint8_t* out_any) noexcept
    {
        if(handle == nullptr || mask == nullptr || out_any == nullptr) { return false; }
        auto* h = reinterpret_cast<bitset_matrix_handle*>(handle);
        if(h->stride_words == 0u || h->row_count == 0u) { return false; }
        if(mask_words != h->stride_words) { return false; }

        // Enqueue work on each device stream (single host thread), then sync.
        // We use a per-device host staging buffer (std::vector<u32>) for the ANY result.
        for(auto& d: h->devs)
        {
            d.ok = false;
            if(d.end <= d.begin) { d.ok = true; continue; }
            if(cudaSetDevice(d.device) != cudaSuccess) { d.ok = false; continue; }

            auto const n = d.end - d.begin;
            auto const mask_bytes = static_cast<std::size_t>(d.stride_words) * sizeof(std::uint64_t);
            auto const any_bytes = n * sizeof(std::uint32_t);

            bool ok = true;
            if(cudaMemcpyAsync(d.d_mask, mask, mask_bytes, cudaMemcpyHostToDevice, d.stream) != cudaSuccess) { ok = false; }
            if(ok && cudaMemsetAsync(d.d_any, 0, any_bytes, d.stream) != cudaSuccess) { ok = false; }
            if(ok)
            {
                constexpr unsigned threads = 256u;
                auto const grid_y = static_cast<unsigned>((d.stride_words + threads - 1u) / threads);
                dim3 grid(static_cast<unsigned>(n), grid_y, 1u);
                bitset_row_any_and_kernel<<<grid, threads, 0u, d.stream>>>(d.d_rows, n, d.stride_words, d.d_mask, d.d_any);
                if(cudaGetLastError() != cudaSuccess) { ok = false; }
            }
            // Reuse d.h_tmp as staging; allocate on demand.
            if(d.h_tmp.size() < n) { d.h_tmp.resize(n); }
            if(ok && cudaMemcpyAsync(d.h_tmp.data(), d.d_any, any_bytes, cudaMemcpyDeviceToHost, d.stream) != cudaSuccess) { ok = false; }
            d.ok = ok;
        }

        for(auto& d: h->devs)
        {
            if(!d.ok) { return false; }
            if(d.end <= d.begin) { continue; }
            if(cudaSetDevice(d.device) != cudaSuccess) { return false; }
            if(cudaStreamSynchronize(d.stream) != cudaSuccess) { return false; }
            auto const n = d.end - d.begin;
            for(std::size_t i{}; i < n; ++i) { out_any[d.begin + i] = static_cast<std::uint8_t>(d.h_tmp[i] != 0u); }
        }

        return true;
    }

    extern "C" bool phy_engine_pe_synth_cuda_bitset_matrix_fill_qm_cov(void* handle,
                                                                       cuda_cube_desc const* cubes,
                                                                       std::size_t cube_count,
                                                                       std::uint16_t const* on_minterms,
                                                                       std::size_t on_count,
                                                                       std::uint32_t var_count,
                                                                       std::uint64_t* out_rows_host) noexcept
    {
        if(handle == nullptr || cubes == nullptr || on_minterms == nullptr) { return false; }
        if(cube_count == 0u || on_count == 0u) { return false; }
        if(var_count == 0u || var_count > 16u) { return false; }
        auto* h = reinterpret_cast<bitset_matrix_handle*>(handle);
        if(h->row_count != cube_count) { return false; }
        if(h->stride_words == 0u) { return false; }

        auto const stride_words = h->stride_words;
        auto const expected_stride = static_cast<std::size_t>((on_count + 63u) / 64u);
        if(static_cast<std::size_t>(stride_words) != expected_stride) { return false; }

        auto const var_mask = static_cast<std::uint16_t>((var_count >= 16u) ? 0xFFFFu : ((1u << var_count) - 1u));

        // Enqueue fill on each device.
        for(auto& d: h->devs)
        {
            d.ok = false;
            if(d.end <= d.begin) { d.ok = true; continue; }
            if(cudaSetDevice(d.device) != cudaSuccess) { d.ok = false; continue; }
            auto const n = d.end - d.begin;
            if(!ensure_bitset_matrix_fill_buffers(d, n, on_count)) { d.ok = false; continue; }

            bool ok = true;
            if(ok && cudaMemcpyAsync(d.d_cubes_tmp, cubes + d.begin, n * sizeof(cuda_cube_desc), cudaMemcpyHostToDevice, d.stream) != cudaSuccess) { ok = false; }
            if(ok && cudaMemcpyAsync(d.d_on_tmp, on_minterms, on_count * sizeof(std::uint16_t), cudaMemcpyHostToDevice, d.stream) != cudaSuccess) { ok = false; }

            if(ok)
            {
                dim3 grid(static_cast<unsigned>(n), static_cast<unsigned>(stride_words), 1u);
                qm_cov_fill_kernel<<<grid, 64u, 0u, d.stream>>>(d.d_cubes_tmp,
                                                               static_cast<std::uint32_t>(n),
                                                               d.d_on_tmp,
                                                               static_cast<std::uint32_t>(on_count),
                                                               var_mask,
                                                               stride_words,
                                                               d.d_rows);
                if(cudaGetLastError() != cudaSuccess) { ok = false; }
            }

            if(ok && out_rows_host != nullptr)
            {
                auto const bytes = n * static_cast<std::size_t>(stride_words) * sizeof(std::uint64_t);
                if(cudaMemcpyAsync(out_rows_host + d.begin * static_cast<std::size_t>(stride_words),
                                   d.d_rows,
                                   bytes,
                                   cudaMemcpyDeviceToHost,
                                   d.stream) != cudaSuccess)
                {
                    ok = false;
                }
            }

            d.ok = ok;
        }

        for(auto& d: h->devs)
        {
            if(!d.ok) { return false; }
            if(d.end <= d.begin) { continue; }
            if(cudaSetDevice(d.device) != cudaSuccess) { return false; }
            if(cudaStreamSynchronize(d.stream) != cudaSuccess) { return false; }
        }

        return true;
    }

    extern "C" bool phy_engine_pe_synth_cuda_bitset_matrix_set_row_cost_u32(void* handle,
                                                                            std::uint32_t const* costs,
                                                                            std::size_t cost_count) noexcept
    {
        if(handle == nullptr || costs == nullptr || cost_count == 0u) { return false; }
        auto* h = reinterpret_cast<bitset_matrix_handle*>(handle);
        if(h->row_count == 0u || h->stride_words == 0u) { return false; }
        if(cost_count != h->row_count) { return false; }

        for(auto& d: h->devs)
        {
            d.ok = false;
            if(d.end <= d.begin) { d.ok = true; continue; }
            if(cudaSetDevice(d.device) != cudaSuccess) { d.ok = false; continue; }
            if(!ensure_bitset_matrix_best_buffers(d)) { d.ok = false; continue; }

            auto const n = d.end - d.begin;
            auto const bytes = n * sizeof(std::uint32_t);
            if(d.d_cost == nullptr)
            {
                if(cudaMalloc(reinterpret_cast<void**>(&d.d_cost), bytes) != cudaSuccess) { d.ok = false; continue; }
            }
            // Reset active mask (new cone / new search).
            if(d.d_active != nullptr)
            {
                if(cudaMemsetAsync(d.d_active, 1, static_cast<std::size_t>(n) * sizeof(std::uint8_t), d.stream) != cudaSuccess) { d.ok = false; continue; }
            }
            bool ok = true;
            if(cudaMemcpyAsync(d.d_cost, costs + d.begin, bytes, cudaMemcpyHostToDevice, d.stream) != cudaSuccess) { ok = false; }
            d.ok = ok;
        }

        for(auto& d: h->devs)
        {
            if(!d.ok) { return false; }
            if(d.end <= d.begin) { continue; }
            if(cudaSetDevice(d.device) != cudaSuccess) { return false; }
            if(cudaStreamSynchronize(d.stream) != cudaSuccess) { return false; }
        }
        return true;
    }

    extern "C" bool phy_engine_pe_synth_cuda_bitset_matrix_disable_row(void* handle, std::size_t row_idx) noexcept
    {
        if(handle == nullptr) { return false; }
        auto* h = reinterpret_cast<bitset_matrix_handle*>(handle);
        if(h->row_count == 0u || row_idx >= h->row_count) { return false; }

        for(auto& d: h->devs)
        {
            if(row_idx < d.begin || row_idx >= d.end) { continue; }
            if(cudaSetDevice(d.device) != cudaSuccess) { return false; }
            if(!ensure_bitset_matrix_best_buffers(d)) { return false; }
            auto const local = row_idx - d.begin;
            if(d.d_active == nullptr) { return false; }
            // Use memset to avoid host staging lifetime issues.
            if(cudaMemsetAsync(d.d_active + local, 0, 1u, d.stream) != cudaSuccess) { return false; }
            return true;
        }
        return false;
    }

    extern "C" bool phy_engine_pe_synth_cuda_bitset_matrix_best_row(void* handle,
                                                                    std::uint64_t const* mask,
                                                                    std::uint32_t mask_words,
                                                                    std::uint32_t* out_best_row,
                                                                    std::uint32_t* out_best_gain,
                                                                    std::int32_t* out_best_score) noexcept
    {
        if(handle == nullptr || mask == nullptr || out_best_row == nullptr || out_best_gain == nullptr || out_best_score == nullptr) { return false; }
        auto* h = reinterpret_cast<bitset_matrix_handle*>(handle);
        if(h->row_count == 0u || h->stride_words == 0u) { return false; }
        if(mask_words != h->stride_words) { return false; }

        for(auto& d: h->devs)
        {
            d.ok = false;
            d.h_best = best_item{INT32_MIN, 0u, 0u, 0xFFFFFFFFu};
            if(d.end <= d.begin) { d.ok = true; continue; }
            if(cudaSetDevice(d.device) != cudaSuccess) { d.ok = false; continue; }
            if(!ensure_bitset_matrix_best_buffers(d)) { d.ok = false; continue; }

            auto const n = static_cast<std::uint32_t>(d.end - d.begin);
            auto const mask_bytes = static_cast<std::size_t>(d.stride_words) * sizeof(std::uint64_t);
            auto const popc_bytes = static_cast<std::size_t>(n) * sizeof(std::uint32_t);

            bool ok = true;
            if(cudaMemcpyAsync(d.d_mask, mask, mask_bytes, cudaMemcpyHostToDevice, d.stream) != cudaSuccess) { ok = false; }
            if(ok && cudaMemsetAsync(d.d_popc, 0, popc_bytes, d.stream) != cudaSuccess) { ok = false; }

            if(ok)
            {
                constexpr unsigned threads = 128u;
                auto const grid_y = static_cast<unsigned>((d.stride_words + threads - 1u) / threads);
                dim3 grid(static_cast<unsigned>(n), grid_y, 1u);
                bitset_row_and_popcount_active_kernel<<<grid, threads, threads * sizeof(std::uint32_t), d.stream>>>(
                    d.d_rows, n, d.stride_words, d.d_mask, d.d_popc, d.d_active);
                if(cudaGetLastError() != cudaSuccess) { ok = false; }
            }

            if(ok)
            {
                constexpr unsigned reduce_threads = 256u;
                constexpr std::uint32_t items_per_block = 1024u;
                auto const parts = (n + items_per_block - 1u) / items_per_block;
                bitset_best_stage1_kernel<<<parts, reduce_threads, 0u, d.stream>>>(
                    d.d_popc, d.d_cost, n, static_cast<std::uint32_t>(d.begin), d.d_best_parts);
                if(cudaGetLastError() != cudaSuccess) { ok = false; }
                if(ok)
                {
                    bitset_best_stage2_kernel<<<1u, reduce_threads, 0u, d.stream>>>(d.d_best_parts, parts, d.d_best_one);
                    if(cudaGetLastError() != cudaSuccess) { ok = false; }
                }
            }

            if(ok && cudaMemcpyAsync(&d.h_best, d.d_best_one, sizeof(best_item), cudaMemcpyDeviceToHost, d.stream) != cudaSuccess) { ok = false; }
            d.ok = ok;
        }

        for(auto& d: h->devs)
        {
            if(!d.ok) { return false; }
            if(d.end <= d.begin) { continue; }
            if(cudaSetDevice(d.device) != cudaSuccess) { return false; }
            if(cudaStreamSynchronize(d.stream) != cudaSuccess) { return false; }
        }

        // Host-side final reduce across devices.
        auto best = best_item{INT32_MIN, 0u, 0u, 0xFFFFFFFFu};
        auto better_host = [](best_item const& a, best_item const& b) noexcept -> bool
        {
            if(a.score != b.score) { return a.score > b.score; }
            if(a.gain != b.gain) { return a.gain > b.gain; }
            if(a.cost != b.cost) { return a.cost < b.cost; }
            return a.idx < b.idx;
        };
        for(auto const& d: h->devs)
        {
            if(d.h_best.gain == 0u) { continue; }
            if(better_host(d.h_best, best)) { best = d.h_best; }
        }

        if(best.gain == 0u || best.idx == 0xFFFFFFFFu) { return false; }
        *out_best_row = best.idx;
        *out_best_gain = best.gain;
        *out_best_score = best.score;
        return true;
    }

    extern "C" bool phy_engine_pe_synth_cuda_bitset_matrix_set_mask(void* handle,
                                                                    std::uint64_t const* mask,
                                                                    std::uint32_t mask_words) noexcept
    {
        if(handle == nullptr || mask == nullptr) { return false; }
        auto* h = reinterpret_cast<bitset_matrix_handle*>(handle);
        if(h->row_count == 0u || h->stride_words == 0u) { return false; }
        if(mask_words != h->stride_words) { return false; }

        auto const bytes = static_cast<std::size_t>(h->stride_words) * sizeof(std::uint64_t);
        for(auto& d: h->devs)
        {
            d.ok = false;
            if(d.end <= d.begin) { d.ok = true; continue; }
            if(cudaSetDevice(d.device) != cudaSuccess) { d.ok = false; continue; }
            // Ensure stream exists (best-effort); doesn't allocate any extra buffers besides active/best.
            if(!ensure_bitset_matrix_best_buffers(d)) { d.ok = false; continue; }
            bool ok = true;
            if(cudaMemcpyAsync(d.d_mask, mask, bytes, cudaMemcpyHostToDevice, d.stream) != cudaSuccess) { ok = false; }
            d.ok = ok;
        }
        for(auto& d: h->devs)
        {
            if(!d.ok) { return false; }
            if(d.end <= d.begin) { continue; }
            if(cudaSetDevice(d.device) != cudaSuccess) { return false; }
            if(cudaStreamSynchronize(d.stream) != cudaSuccess) { return false; }
        }
        return true;
    }

    extern "C" bool phy_engine_pe_synth_cuda_bitset_matrix_get_mask(void* handle,
                                                                    std::uint32_t mask_words,
                                                                    std::uint64_t* out_mask) noexcept
    {
        if(handle == nullptr || out_mask == nullptr) { return false; }
        auto* h = reinterpret_cast<bitset_matrix_handle*>(handle);
        if(h->row_count == 0u || h->stride_words == 0u) { return false; }
        if(mask_words != h->stride_words) { return false; }
        if(h->devs.empty()) { return false; }

        // The mask is replicated across devices; copy from the first device.
        auto& d = h->devs[0];
        if(d.end <= d.begin) { return false; }
        if(cudaSetDevice(d.device) != cudaSuccess) { return false; }
        if(!ensure_bitset_matrix_best_buffers(d)) { return false; }
        auto const bytes = static_cast<std::size_t>(h->stride_words) * sizeof(std::uint64_t);
        if(cudaMemcpyAsync(out_mask, d.d_mask, bytes, cudaMemcpyDeviceToHost, d.stream) != cudaSuccess) { return false; }
        if(cudaStreamSynchronize(d.stream) != cudaSuccess) { return false; }
        return true;
    }

    extern "C" bool phy_engine_pe_synth_cuda_bitset_matrix_best_row_resident_mask(void* handle,
                                                                                  std::uint32_t* out_best_row,
                                                                                  std::uint32_t* out_best_gain,
                                                                                  std::int32_t* out_best_score) noexcept
    {
        if(handle == nullptr || out_best_row == nullptr || out_best_gain == nullptr || out_best_score == nullptr) { return false; }
        auto* h = reinterpret_cast<bitset_matrix_handle*>(handle);
        if(h->row_count == 0u || h->stride_words == 0u) { return false; }

        for(auto& d: h->devs)
        {
            d.ok = false;
            d.h_best = best_item{INT32_MIN, 0u, 0u, 0xFFFFFFFFu};
            if(d.end <= d.begin) { d.ok = true; continue; }
            if(cudaSetDevice(d.device) != cudaSuccess) { d.ok = false; continue; }
            if(!ensure_bitset_matrix_best_buffers(d)) { d.ok = false; continue; }

            auto const n = static_cast<std::uint32_t>(d.end - d.begin);
            auto const popc_bytes = static_cast<std::size_t>(n) * sizeof(std::uint32_t);

            bool ok = true;
            if(cudaMemsetAsync(d.d_popc, 0, popc_bytes, d.stream) != cudaSuccess) { ok = false; }

            if(ok)
            {
                constexpr unsigned threads = 128u;
                auto const grid_y = static_cast<unsigned>((d.stride_words + threads - 1u) / threads);
                dim3 grid(static_cast<unsigned>(n), grid_y, 1u);
                bitset_row_and_popcount_active_kernel<<<grid, threads, threads * sizeof(std::uint32_t), d.stream>>>(
                    d.d_rows, n, d.stride_words, d.d_mask, d.d_popc, d.d_active);
                if(cudaGetLastError() != cudaSuccess) { ok = false; }
            }

            if(ok)
            {
                constexpr unsigned reduce_threads = 256u;
                constexpr std::uint32_t items_per_block = 1024u;
                auto const parts = (n + items_per_block - 1u) / items_per_block;
                bitset_best_stage1_kernel<<<parts, reduce_threads, 0u, d.stream>>>(
                    d.d_popc, d.d_cost, n, static_cast<std::uint32_t>(d.begin), d.d_best_parts);
                if(cudaGetLastError() != cudaSuccess) { ok = false; }
                if(ok)
                {
                    bitset_best_stage2_kernel<<<1u, reduce_threads, 0u, d.stream>>>(d.d_best_parts, parts, d.d_best_one);
                    if(cudaGetLastError() != cudaSuccess) { ok = false; }
                }
            }

            if(ok && cudaMemcpyAsync(&d.h_best, d.d_best_one, sizeof(best_item), cudaMemcpyDeviceToHost, d.stream) != cudaSuccess) { ok = false; }
            d.ok = ok;
        }

        for(auto& d: h->devs)
        {
            if(!d.ok) { return false; }
            if(d.end <= d.begin) { continue; }
            if(cudaSetDevice(d.device) != cudaSuccess) { return false; }
            if(cudaStreamSynchronize(d.stream) != cudaSuccess) { return false; }
        }

        auto best = best_item{INT32_MIN, 0u, 0u, 0xFFFFFFFFu};
        auto better_host = [](best_item const& a, best_item const& b) noexcept -> bool
        {
            if(a.score != b.score) { return a.score > b.score; }
            if(a.gain != b.gain) { return a.gain > b.gain; }
            if(a.cost != b.cost) { return a.cost < b.cost; }
            return a.idx < b.idx;
        };
        for(auto const& d: h->devs)
        {
            if(d.h_best.gain == 0u) { continue; }
            if(better_host(d.h_best, best)) { best = d.h_best; }
        }
        if(best.gain == 0u || best.idx == 0xFFFFFFFFu) { return false; }
        *out_best_row = best.idx;
        *out_best_gain = best.gain;
        *out_best_score = best.score;
        return true;
    }

    extern "C" bool phy_engine_pe_synth_cuda_bitset_matrix_mask_andnot_row(void* handle, std::size_t row_idx) noexcept
    {
        if(handle == nullptr) { return false; }
        auto* h = reinterpret_cast<bitset_matrix_handle*>(handle);
        if(h->row_count == 0u || h->stride_words == 0u) { return false; }
        if(row_idx >= h->row_count) { return false; }

        // Single-device fast path: keep everything on GPU (no row staging / broadcast).
        if(h->devs.size() == 1u)
        {
            auto& d = h->devs[0];
            if(row_idx < d.begin || row_idx >= d.end) { return false; }
            if(cudaSetDevice(d.device) != cudaSuccess) { return false; }
            if(!ensure_bitset_matrix_best_buffers(d)) { return false; }
            auto const local = row_idx - d.begin;
            auto const* row = d.d_rows + local * static_cast<std::size_t>(h->stride_words);
            constexpr unsigned threads = 256u;
            auto const blocks = static_cast<unsigned>((h->stride_words + threads - 1u) / threads);
            bitset_mask_andnot_kernel<<<blocks, threads, 0u, d.stream>>>(d.d_mask, row, h->stride_words);
            if(cudaGetLastError() != cudaSuccess) { return false; }
            if(cudaStreamSynchronize(d.stream) != cudaSuccess) { return false; }
            return true;
        }

        // D2H the selected row (from its owning device) once, then broadcast to all devices to update their resident masks.
        std::vector<std::uint64_t> h_row{};
        h_row.resize(static_cast<std::size_t>(h->stride_words));

        bool got_row{};
        for(auto& d: h->devs)
        {
            if(row_idx < d.begin || row_idx >= d.end) { continue; }
            if(cudaSetDevice(d.device) != cudaSuccess) { return false; }
            if(d.end <= d.begin) { return false; }
            auto const local = row_idx - d.begin;
            auto const bytes = static_cast<std::size_t>(h->stride_words) * sizeof(std::uint64_t);
            auto const* src = d.d_rows + local * static_cast<std::size_t>(h->stride_words);
            if(cudaMemcpyAsync(h_row.data(), src, bytes, cudaMemcpyDeviceToHost, d.stream) != cudaSuccess) { return false; }
            if(cudaStreamSynchronize(d.stream) != cudaSuccess) { return false; }
            got_row = true;
            break;
        }
        if(!got_row) { return false; }

        auto const bytes = static_cast<std::size_t>(h->stride_words) * sizeof(std::uint64_t);
        for(auto& d: h->devs)
        {
            d.ok = false;
            if(d.end <= d.begin) { d.ok = true; continue; }
            if(cudaSetDevice(d.device) != cudaSuccess) { d.ok = false; continue; }
            if(!ensure_bitset_matrix_best_buffers(d)) { d.ok = false; continue; }
            if(d.d_row_tmp == nullptr)
            {
                if(cudaMalloc(reinterpret_cast<void**>(&d.d_row_tmp), bytes) != cudaSuccess) { d.ok = false; continue; }
            }

            bool ok = true;
            if(cudaMemcpyAsync(d.d_row_tmp, h_row.data(), bytes, cudaMemcpyHostToDevice, d.stream) != cudaSuccess) { ok = false; }
            if(ok)
            {
                constexpr unsigned threads = 256u;
                auto const blocks = static_cast<unsigned>((h->stride_words + threads - 1u) / threads);
                bitset_mask_andnot_kernel<<<blocks, threads, 0u, d.stream>>>(d.d_mask, d.d_row_tmp, h->stride_words);
                if(cudaGetLastError() != cudaSuccess) { ok = false; }
            }
            d.ok = ok;
        }

        for(auto& d: h->devs)
        {
            if(!d.ok) { return false; }
            if(d.end <= d.begin) { continue; }
            if(cudaSetDevice(d.device) != cudaSuccess) { return false; }
            if(cudaStreamSynchronize(d.stream) != cudaSuccess) { return false; }
        }
        return true;
    }

    extern "C" bool phy_engine_pe_synth_cuda_bitset_matrix_get_row(void* handle,
                                                                   std::size_t row_idx,
                                                                   std::uint32_t row_words,
                                                                   std::uint64_t* out_row) noexcept
    {
        if(handle == nullptr || out_row == nullptr) { return false; }
        auto* h = reinterpret_cast<bitset_matrix_handle*>(handle);
        if(h->row_count == 0u || h->stride_words == 0u) { return false; }
        if(row_idx >= h->row_count) { return false; }
        if(row_words != h->stride_words) { return false; }

        for(auto& d: h->devs)
        {
            if(row_idx < d.begin || row_idx >= d.end) { continue; }
            if(cudaSetDevice(d.device) != cudaSuccess) { return false; }
            if(d.end <= d.begin) { return false; }
            auto const local = row_idx - d.begin;
            auto const bytes = static_cast<std::size_t>(h->stride_words) * sizeof(std::uint64_t);
            auto const* src = d.d_rows + local * static_cast<std::size_t>(h->stride_words);
            if(cudaMemcpyAsync(out_row, src, bytes, cudaMemcpyDeviceToHost, d.stream) != cudaSuccess) { return false; }
            if(cudaStreamSynchronize(d.stream) != cudaSuccess) { return false; }
            return true;
        }
        return false;
    }

    // Espresso OFF-set resident handle API (forward decls for older entrypoints).
    extern "C" void* phy_engine_pe_synth_cuda_espresso_off_create(std::uint32_t device_mask,
                                                                  std::uint32_t var_count,
                                                                  std::uint64_t const* off_blocks,
                                                                  std::uint32_t off_words) noexcept;
    extern "C" void phy_engine_pe_synth_cuda_espresso_off_destroy(void* handle) noexcept;
    extern "C" bool phy_engine_pe_synth_cuda_espresso_off_hits(void* handle,
                                                               cuda_cube_desc const* cubes,
                                                               std::size_t cube_count,
                                                               std::uint8_t* out_hits) noexcept;
    extern "C" bool phy_engine_pe_synth_cuda_espresso_off_expand_best(void* handle,
                                                                      cuda_cube_desc* cubes,
                                                                      std::size_t cube_count,
                                                                      std::uint8_t const* cand_vars,
                                                                      std::uint32_t cand_vars_count,
                                                                      std::uint32_t max_rounds) noexcept;

    extern "C" bool phy_engine_pe_synth_cuda_espresso_cube_hits_off(std::uint32_t device_mask,
                                                                    cuda_cube_desc const* cubes,
                                                                    std::size_t cube_count,
                                                                    std::uint32_t var_count,
                                                                    std::uint64_t const* off_blocks,
                                                                    std::uint32_t off_words,
                                                                    std::uint8_t* out_hits) noexcept
    {
        // Legacy helper: create a per-call OFF-handle and run one hits-off batch.
        // This avoids per-call thread spawning and keeps the implementation in one place.
        void* h = phy_engine_pe_synth_cuda_espresso_off_create(device_mask, var_count, off_blocks, off_words);
        if(h == nullptr) { return false; }
        bool const ok = phy_engine_pe_synth_cuda_espresso_off_hits(h, cubes, cube_count, out_hits);
        phy_engine_pe_synth_cuda_espresso_off_destroy(h);
        return ok;
    }

    extern "C" void* phy_engine_pe_synth_cuda_espresso_off_create(std::uint32_t device_mask,
                                                                  std::uint32_t var_count,
                                                                  std::uint64_t const* off_blocks,
                                                                  std::uint32_t off_words) noexcept
    {
        if(off_blocks == nullptr) { return nullptr; }
        if(off_words == 0u) { return nullptr; }
        if(var_count == 0u || var_count > 16u) { return nullptr; }

        int dev_count{};
        if(cudaGetDeviceCount(&dev_count) != cudaSuccess || dev_count <= 0) { return nullptr; }

        std::vector<int> devices{};
        devices.reserve(static_cast<std::size_t>(dev_count));
        if(device_mask == 0u)
        {
            for(int d = 0; d < dev_count; ++d) { devices.push_back(d); }
        }
        else
        {
            for(int d = 0; d < dev_count; ++d)
            {
                if((device_mask >> static_cast<unsigned>(d)) & 1u) { devices.push_back(d); }
            }
        }
        if(devices.empty()) { return nullptr; }

        auto* h = new(std::nothrow) espresso_off_handle{};
        if(h == nullptr) { return nullptr; }
        h->var_count = var_count;
        h->off_words = off_words;
        h->devs.reserve(devices.size());

        bool ok = true;
        for(auto const dev : devices)
        {
            espresso_off_device d = take_espresso_off_device_from_pool(dev);
            if(!init_espresso_off_device(d, dev, var_count, off_blocks, off_words))
            {
                ok = false;
                // If (re)initialization fails, fall back to fully destroying this device context.
                destroy_espresso_off_device(d);
                break;
            }
            h->devs.push_back(std::move(d));
        }

        if(!ok)
        {
            for(auto& d: h->devs) { destroy_espresso_off_device(d); }
            delete h;
            return nullptr;
        }
        return reinterpret_cast<void*>(h);
    }

    extern "C" void phy_engine_pe_synth_cuda_espresso_off_destroy(void* handle) noexcept
    {
        if(handle == nullptr) { return; }
        auto* h = reinterpret_cast<espresso_off_handle*>(handle);
        // Return per-device contexts to the pool to avoid repeated cudaMalloc/cudaStreamCreate churn.
        for(auto& d: h->devs) { recycle_espresso_off_device_to_pool(std::move(d)); }
        delete h;
    }

    extern "C" bool phy_engine_pe_synth_cuda_espresso_off_hits(void* handle,
                                                               cuda_cube_desc const* cubes,
                                                               std::size_t cube_count,
                                                               std::uint8_t* out_hits) noexcept
    {
        if(handle == nullptr || cubes == nullptr || out_hits == nullptr) { return false; }
        if(cube_count == 0u) { return false; }
        auto* h = reinterpret_cast<espresso_off_handle*>(handle);
        if(h->off_words == 0u || h->var_count == 0u) { return false; }
        if(h->devs.empty()) { return false; }

        struct chunk
        {
            std::size_t begin{};
            std::size_t end{};
        };

        // Heuristic: for very small batches, splitting across devices increases overhead (extra H2D/D2H and kernel launches).
        // Use a single device in a round-robin manner so multi-GPU users still see aggregate utilization over time.
        auto const k_all = h->devs.size();
        auto const total_work = cube_count * static_cast<std::size_t>(h->off_words);
        std::size_t k = k_all;
        std::size_t first_dev_idx{};
        if(k_all > 1u && total_work < 65536u)
        {
            static std::atomic<std::uint32_t> rr{};
            first_dev_idx = static_cast<std::size_t>(rr.fetch_add(1u, std::memory_order_relaxed) % static_cast<std::uint32_t>(k_all));
            k = 1u;
        }

        std::vector<chunk> chunks{};
        chunks.reserve(k);
        bool ok = true;
        for(std::size_t i{}; i < k; ++i)
        {
            auto const dev_idx = (k == 1u) ? first_dev_idx : i;
            auto const begin = (cube_count * i) / k;
            auto const end = (cube_count * (i + 1u)) / k;
            chunks.push_back(chunk{begin, end});
            auto& d = h->devs[dev_idx];
            if(!enqueue_espresso_off_hits_on_device(d, begin, end, cubes, h->var_count, h->off_words)) { ok = false; }
        }

        // Synchronize and scatter results.
        for(std::size_t i{}; i < k; ++i)
        {
            auto const dev_idx = (k == 1u) ? first_dev_idx : i;
            auto& d = h->devs[dev_idx];
            if(!d.ok) { ok = false; continue; }
            if(chunks[i].end <= chunks[i].begin) { continue; }
            auto const n = chunks[i].end - chunks[i].begin;
            if(cudaSetDevice(d.device) != cudaSuccess) { ok = false; d.ok = false; continue; }
            if(cudaStreamSynchronize(d.stream) != cudaSuccess) { ok = false; d.ok = false; continue; }
            for(std::size_t j{}; j < n; ++j)
            {
                auto const* src = (d.h_hits_pinned != nullptr) ? d.h_hits_pinned : d.h_hits.data();
                out_hits[chunks[i].begin + j] = static_cast<std::uint8_t>(src[j] != 0u);
            }
        }

        // When using a single device for a tiny batch, don't require all pooled devices to be ok.
        if(k == 1u)
        {
            if(!h->devs[first_dev_idx].ok) { return false; }
        }
        else
        {
            for(auto const& d: h->devs)
            {
                if(!d.ok) { return false; }
            }
        }
        return ok;
    }

    extern "C" bool phy_engine_pe_synth_cuda_espresso_off_expand_best(void* handle,
                                                                      cuda_cube_desc* cubes,
                                                                      std::size_t cube_count,
                                                                      std::uint8_t const* cand_vars,
                                                                      std::uint32_t cand_vars_count,
                                                                      std::uint32_t max_rounds) noexcept
    {
        if(handle == nullptr || cubes == nullptr || cand_vars == nullptr) { return false; }
        if(cube_count == 0u) { return false; }
        if(cand_vars_count == 0u) { return false; }
        if(max_rounds == 0u) { return true; }
        auto* h = reinterpret_cast<espresso_off_handle*>(handle);
        if(h->off_words == 0u || h->var_count == 0u) { return false; }
        if(h->devs.empty()) { return false; }

        struct chunk
        {
            std::size_t begin{};
            std::size_t end{};
        };

        auto const k_all = h->devs.size();
        auto const total_work = cube_count * static_cast<std::size_t>(h->off_words);
        std::size_t k = k_all;
        std::size_t first_dev_idx{};
        if(k_all > 1u && total_work < 65536u)
        {
            static std::atomic<std::uint32_t> rr{};
            first_dev_idx = static_cast<std::size_t>(rr.fetch_add(1u, std::memory_order_relaxed) % static_cast<std::uint32_t>(k_all));
            k = 1u;
        }

        std::vector<chunk> chunks{};
        chunks.reserve(k);
        bool ok = true;
        for(std::size_t i{}; i < k; ++i)
        {
            auto const begin = (cube_count * i) / k;
            auto const end = (cube_count * (i + 1u)) / k;
            chunks.push_back(chunk{begin, end});
            auto const dev_idx = (k == 1u) ? first_dev_idx : i;
            auto& d = h->devs[dev_idx];
            if(!enqueue_espresso_off_expand_best_on_device(d,
                                                          begin,
                                                          end,
                                                          cubes,
                                                          h->var_count,
                                                          h->off_words,
                                                          cand_vars,
                                                          cand_vars_count,
                                                          max_rounds))
            {
                ok = false;
            }
        }

        for(std::size_t i{}; i < k; ++i)
        {
            auto const dev_idx = (k == 1u) ? first_dev_idx : i;
            auto& d = h->devs[dev_idx];
            if(!d.ok) { ok = false; continue; }
            if(chunks[i].end <= chunks[i].begin) { continue; }
            if(cudaSetDevice(d.device) != cudaSuccess) { ok = false; d.ok = false; continue; }
            if(cudaStreamSynchronize(d.stream) != cudaSuccess) { ok = false; d.ok = false; continue; }
            // Scatter expanded cubes back to caller memory.
            auto const n = chunks[i].end - chunks[i].begin;
            auto const bytes = n * sizeof(cuda_cube_desc);
            if(d.h_cubes_pinned != nullptr) { ::std::memcpy(cubes + chunks[i].begin, d.h_cubes_pinned, bytes); }
            else { ::std::memcpy(cubes + chunks[i].begin, d.h_cubes.data(), bytes); }
        }

        if(k == 1u)
        {
            if(!h->devs[first_dev_idx].ok) { return false; }
        }
        else
        {
            for(auto const& d: h->devs)
            {
                if(!d.ok) { return false; }
            }
        }
        return ok;
    }
}  // namespace phy_engine::verilog::digital::details
