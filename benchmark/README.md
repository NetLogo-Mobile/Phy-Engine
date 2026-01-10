# CUDA build & benchmark guide

This repo can optionally accelerate large MNA solves with CUDA (cuSolverSP + cuSPARSE) when building with a CUDA-capable compiler (`nvcc` or `clang++ -x cuda`).

## Key knobs

- `circult::cuda_policy`
  - `auto_select`: use CUDA only when `node_counter >= cuda_node_threshold`
  - `force_cpu`: always use Eigen (CPU)
  - `force_cuda`: always use CUDA (if available)
- `circult::cuda_node_threshold` (default `100000`)
- Runtime env vars for CUDA solver internals:
  - `PHY_ENGINE_CUDA_PINNED=1` (use pinned host buffers)
  - `PHY_ENGINE_CUDA_CSR_CACHE=1` (reuse CSR pattern across solves; default enabled)
  - `PHY_ENGINE_CUDA_QR_REORDER=0/1` (cuSolver QR reordering; default 1)
  - `PHY_ENGINE_CUDA_SOLVER=qr|ilu0` (choose CUDA solver for real systems; default `qr`)
  - `PHY_ENGINE_CUDA_ITER_MAX=200` (BiCGSTAB max iterations for `ilu0`)
  - `PHY_ENGINE_CUDA_ITER_TOL=1e-10` (BiCGSTAB relative residual tolerance for `ilu0`)

## Build (CPU benchmarks)

```bash
cmake -S benchmark -B build_bench -DCMAKE_BUILD_TYPE=Release
cmake --build build_bench -j
```

Run:

```bash
./build_bench/bench_100000_random_links_cpu --nodes=100000 --links=10,100,1000,10000 --warmup=1 --iters=5
```

## Build (CUDA benchmarks with Clang, no nvcc)

Requires CUDA toolkit installed locally (no network fetch).

```bash
cmake -S benchmark -B build_bench_cuda \\
  -DCMAKE_BUILD_TYPE=Release \\
  -DCMAKE_CXX_COMPILER=clang++ \\
  -DPHY_ENGINE_CUDA_CLANG_BENCH=ON \\
  -DPHY_ENGINE_CUDA_PATH=/usr/local/cuda \\
  -DPHY_ENGINE_CUDA_BENCH_ARCH=sm_80
cmake --build build_bench_cuda -j
```

Run compare (prints CPU vs CUDA timing + max abs diffs):

```bash
PHY_ENGINE_CUDA_PINNED=1 PHY_ENGINE_CUDA_CSR_CACHE=1 \\
  ./build_bench_cuda/bench_100000_random_links_compare --nodes=100000 --links=10000 --warmup=1 --iters=5
```

Prefer the iterative solver for performance on large MNA (recommended):

```bash
PHY_ENGINE_CUDA_SOLVER=ilu0 PHY_ENGINE_CUDA_PINNED=1 PHY_ENGINE_CUDA_CSR_CACHE=1 \\
  ./build_bench_cuda/bench_100000_random_links_compare --nodes=200000 --links=500000 --warmup=2 --iters=20
```

Expected:
- `max_abs_diff_v` and `max_abs_diff_i` are small (default `--eps=1e-6`)
- `speedup` should be `> 1` for large enough cases (tune `--nodes/--links`, and ensure GPU has enough VRAM).

## Build (CUDA tests with Clang, no nvcc)

```bash
cmake -S test -B build_test_cuda \\
  -DCMAKE_BUILD_TYPE=Release \\
  -DCMAKE_CXX_COMPILER=clang++ \\
  -DPHY_ENGINE_CUDA_CLANG_TEST=ON \\
  -DPHY_ENGINE_CUDA_PATH=/usr/local/cuda \\
  -DPHY_ENGINE_CUDA_SMOKE_ARCH=sm_80
cmake --build build_test_cuda -j
ctest --test-dir build_test_cuda --output-on-failure
```

Notes:
- CUDA tests return code `77` when no GPU is visible; CTest marks these as skipped.

## Alternative: build a single benchmark with `nvcc` (no CMake)

Example (Linux):

```bash
CUDA_HOME=/usr/local/cuda
nvcc -O3 -std=c++20 -lineinfo \\
  -I./include -I"$CUDA_HOME/include" \\
  -o bench_100000_random_links_compare \\
  benchmark/0001.models/100000_random_links_compare.cu \\
  -L"$CUDA_HOME/lib64" -lcudart -lcusolver -lcusparse
./bench_100000_random_links_compare --nodes=100000 --links=10000 --warmup=1 --iters=5
```
