// CUDA backend for pe_synth bounded truth-table batching (6 vars, <=64 gates).
//
// This file is compiled only when PHY_ENGINE_ENABLE_CUDA_PE_SYNTH is enabled.
// Intended build mode: Clang CUDA (no nvcc), see CMake wiring in src/CMakeLists.txt.

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <new>
#include <thread>
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
            g_ctx.push_back(eval_u64_ctx{});
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
            g_ctx.push_back(eval_tt_ctx{});
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
            ctx.cap_cones = 0u;

            if(cudaMalloc(reinterpret_cast<void**>(&ctx.d_cones), n * sizeof(cuda_u64_cone_desc)) != cudaSuccess) { return false; }
            if(cudaMalloc(reinterpret_cast<void**>(&ctx.d_out), n * sizeof(std::uint64_t)) != cudaSuccess) { return false; }
            ctx.cap_cones = n;
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
            ctx.cap_cones = 0u;
            ctx.cap_stride_blocks = 0u;

            if(cudaMalloc(reinterpret_cast<void**>(&ctx.d_cones), n * sizeof(cuda_tt_cone_desc)) != cudaSuccess) { return false; }
            auto const out_words = n * static_cast<std::size_t>(stride_blocks);
            if(cudaMalloc(reinterpret_cast<void**>(&ctx.d_out), out_words * sizeof(std::uint64_t)) != cudaSuccess) { return false; }
            ctx.cap_cones = n;
            ctx.cap_stride_blocks = stride_blocks;
            ctx.ok = true;
            return true;
        }

        [[nodiscard]] inline bool do_eval_on_device(device_chunk& chunk,
                                                    cuda_u64_cone_desc const* cones,
                                                    std::uint64_t* out_masks) noexcept
        {
            chunk.ok = false;

            if(chunk.end <= chunk.begin) { chunk.ok = true; return true; }

            auto const n = chunk.end - chunk.begin;
            auto const cones_bytes = n * sizeof(cuda_u64_cone_desc);
            auto const out_bytes = n * sizeof(std::uint64_t);

            auto& ctx = get_u64_ctx(chunk.device);
            std::scoped_lock lk(ctx.mu);
            bool ok = ensure_u64_ctx_ready(ctx, n);
            if(ok && cudaMemcpyAsync(ctx.d_cones, cones + chunk.begin, cones_bytes, cudaMemcpyHostToDevice, ctx.stream) != cudaSuccess) { ok = false; }

            if(ok)
            {
                constexpr unsigned threads = 256u;
                auto const blocks = static_cast<unsigned>((n + threads - 1u) / threads);
                eval_u64_cones_kernel<<<blocks, threads, 0u, ctx.stream>>>(ctx.d_cones, n, ctx.d_out);
                if(cudaGetLastError() != cudaSuccess) { ok = false; }
            }

            if(ok && cudaMemcpyAsync(out_masks + chunk.begin, ctx.d_out, out_bytes, cudaMemcpyDeviceToHost, ctx.stream) != cudaSuccess) { ok = false; }
            if(ok && cudaStreamSynchronize(ctx.stream) != cudaSuccess) { ok = false; }

            chunk.ok = ok;
            return ok;
        }

        [[nodiscard]] inline bool do_eval_tt_on_device(device_chunk& chunk,
                                                       cuda_tt_cone_desc const* cones,
                                                       std::size_t cone_count,
                                                       std::uint32_t stride_blocks,
                                                       std::uint64_t* out_blocks) noexcept
        {
            chunk.ok = false;

            if(chunk.end <= chunk.begin) { chunk.ok = true; return true; }
            if(stride_blocks == 0u) { return false; }

            auto const n = chunk.end - chunk.begin;
            auto const cones_bytes = n * sizeof(cuda_tt_cone_desc);
            auto const out_words = n * static_cast<std::size_t>(stride_blocks);
            auto const out_bytes = out_words * sizeof(std::uint64_t);

            auto& ctx = get_tt_ctx(chunk.device);
            std::scoped_lock lk(ctx.mu);
            bool ok = ensure_tt_ctx_ready(ctx, n, stride_blocks);
            if(ok && cudaMemcpyAsync(ctx.d_cones, cones + chunk.begin, cones_bytes, cudaMemcpyHostToDevice, ctx.stream) != cudaSuccess) { ok = false; }

            if(ok)
            {
                constexpr unsigned threads = 128u;
                auto const grid_y = static_cast<unsigned>((stride_blocks + threads - 1u) / threads);
                dim3 grid(static_cast<unsigned>(n), grid_y, 1u);
                eval_tt_cones_kernel<<<grid, threads, 0u, ctx.stream>>>(ctx.d_cones, n, stride_blocks, ctx.d_out);
                if(cudaGetLastError() != cudaSuccess) { ok = false; }
            }

            if(ok && cudaMemcpyAsync(out_blocks + chunk.begin * static_cast<std::size_t>(stride_blocks),
                                     ctx.d_out,
                                     out_bytes,
                                     cudaMemcpyDeviceToHost,
                                     ctx.stream) != cudaSuccess)
            {
                ok = false;
            }

            if(ok && cudaStreamSynchronize(ctx.stream) != cudaSuccess) { ok = false; }

            chunk.ok = ok;
            return ok;
        }

        struct bitset_matrix_device
        {
            int device{};
            std::size_t begin{};
            std::size_t end{};
            std::uint32_t stride_words{};
            cudaStream_t stream{};

            std::uint64_t* d_rows{};
            std::uint64_t* d_mask{};
            std::uint32_t* d_popc{};
            std::uint32_t* d_any{};

            bool ok{};
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
            g_ctx.push_back(espresso_hits_ctx{});
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
            ctx.cap_cubes = 0u;
            ctx.cap_words = 0u;

            if(cudaMalloc(reinterpret_cast<void**>(&ctx.d_cubes), cubes * sizeof(cuda_cube_desc)) != cudaSuccess) { return false; }
            if(cudaMalloc(reinterpret_cast<void**>(&ctx.d_hits), cubes * sizeof(std::uint32_t)) != cudaSuccess) { return false; }
            if(cudaMalloc(reinterpret_cast<void**>(&ctx.d_off), static_cast<std::size_t>(words) * sizeof(std::uint64_t)) != cudaSuccess) { return false; }
            ctx.cap_cubes = cubes;
            ctx.cap_words = words;
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

            if(cudaMemcpyAsync(ctx.d_cubes, cubes + begin, cubes_bytes, cudaMemcpyHostToDevice, ctx.stream) != cudaSuccess) { ok = false; }
            if(ok && cudaMemcpyAsync(ctx.d_off, off_blocks, off_bytes, cudaMemcpyHostToDevice, ctx.stream) != cudaSuccess) { ok = false; }
            if(ok && cudaMemsetAsync(ctx.d_hits, 0, hits_bytes, ctx.stream) != cudaSuccess) { ok = false; }

            if(ok)
            {
                constexpr unsigned threads = 256u;
                auto const grid_y = static_cast<unsigned>((off_words + threads - 1u) / threads);
                dim3 grid(static_cast<unsigned>(n), grid_y, 1u);
                espresso_cube_hits_off_kernel<<<grid, threads, 0u, ctx.stream>>>(ctx.d_cubes, n, var_count, ctx.d_off, off_words, ctx.d_hits);
                if(cudaGetLastError() != cudaSuccess) { ok = false; }
            }

            std::vector<std::uint32_t> tmp{};
            tmp.resize(n);
            if(ok && cudaMemcpyAsync(tmp.data(), ctx.d_hits, hits_bytes, cudaMemcpyDeviceToHost, ctx.stream) != cudaSuccess) { ok = false; }
            if(ok && cudaStreamSynchronize(ctx.stream) != cudaSuccess) { ok = false; }
            if(ok)
            {
                for(std::size_t i{}; i < n; ++i) { out_hits[begin + i] = static_cast<std::uint8_t>(tmp[i] != 0u); }
            }

            ctx.ok = ok;
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

        inline void destroy_bitset_matrix_device(bitset_matrix_device& d) noexcept
        {
            if(d.d_any != nullptr) { (void)cudaFree(d.d_any); }
            if(d.d_popc != nullptr) { (void)cudaFree(d.d_popc); }
            if(d.d_mask != nullptr) { (void)cudaFree(d.d_mask); }
            if(d.d_rows != nullptr) { (void)cudaFree(d.d_rows); }
            if(d.stream != nullptr) { (void)cudaStreamDestroy(d.stream); }
            d = bitset_matrix_device{};
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

        std::vector<std::thread> threads{};
        threads.reserve(chunks.size());
        for(auto& c: chunks)
        {
            auto* cp = &c;
            threads.emplace_back([cp, cones, out_masks]() { (void)do_eval_on_device(*cp, cones, out_masks); });
        }
        for(auto& t: threads) { t.join(); }

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

        std::vector<std::thread> threads{};
        threads.reserve(chunks.size());
        for(auto& c: chunks)
        {
            auto* cp = &c;
            threads.emplace_back([cp, cones, cone_count, stride_blocks, out_blocks]()
                                 { (void)do_eval_tt_on_device(*cp, cones, cone_count, stride_blocks, out_blocks); });
        }
        for(auto& t: threads) { t.join(); }

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
            h->devs.push_back(d);
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

        std::vector<std::thread> threads{};
        threads.reserve(h->devs.size());
        for(auto& d: h->devs)
        {
            auto* dp = &d;
            threads.emplace_back([dp, mask, mask_words, out_counts]() { (void)bitset_matrix_row_and_popcount_on_device(*dp, mask, mask_words, out_counts); });
        }
        for(auto& t: threads) { t.join(); }

        for(auto const& d: h->devs)
        {
            if(!d.ok) { return false; }
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

        std::vector<std::thread> threads{};
        threads.reserve(h->devs.size());
        for(auto& d: h->devs)
        {
            auto* dp = &d;
            threads.emplace_back([dp, mask, mask_words, out_any]() { (void)bitset_matrix_row_any_and_on_device(*dp, mask, mask_words, out_any); });
        }
        for(auto& t: threads) { t.join(); }

        for(auto const& d: h->devs)
        {
            if(!d.ok) { return false; }
        }
        return true;
    }

    extern "C" bool phy_engine_pe_synth_cuda_espresso_cube_hits_off(std::uint32_t device_mask,
                                                                    cuda_cube_desc const* cubes,
                                                                    std::size_t cube_count,
                                                                    std::uint32_t var_count,
                                                                    std::uint64_t const* off_blocks,
                                                                    std::uint32_t off_words,
                                                                    std::uint8_t* out_hits) noexcept
    {
        if(cubes == nullptr || off_blocks == nullptr || out_hits == nullptr) { return false; }
        if(cube_count == 0u || off_words == 0u) { return false; }
        if(var_count == 0u || var_count > 16u) { return false; }

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

        // Split cubes across selected devices (contiguous chunks).
        std::vector<device_chunk> chunks{};
        chunks.reserve(devices.size());
        auto const k = devices.size();
        for(std::size_t i{}; i < k; ++i)
        {
            auto const begin = (cube_count * i) / k;
            auto const end = (cube_count * (i + 1u)) / k;
            chunks.push_back(device_chunk{devices[i], begin, end, false});
        }

        std::vector<std::thread> threads{};
        threads.reserve(chunks.size());
        for(auto& c: chunks)
        {
            auto* cp = &c;
            threads.emplace_back([cp, cubes, var_count, off_blocks, off_words, out_hits]()
                                 {
                                     bool ok = do_espresso_hits_off_on_device(cp->device,
                                                                             cp->begin,
                                                                             cp->end,
                                                                             cubes,
                                                                             var_count,
                                                                             off_blocks,
                                                                             off_words,
                                                                             out_hits);
                                     cp->ok = ok;
                                 });
        }
        for(auto& t: threads) { t.join(); }

        for(auto const& c: chunks)
        {
            if(!c.ok) { return false; }
        }
        return true;
    }
}  // namespace phy_engine::verilog::digital::details
