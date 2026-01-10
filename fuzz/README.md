# Fuzzing (LLVM libFuzzer)

This repo includes a libFuzzer target for the Verilog digital subset:

- `fuzz/verilog_digital_fuzzer.cpp`

It exercises `preprocess` (including `` `include ``), `compile`, `build_design`, `elaborate`, and a couple of `simulate` ticks.

## Build (Clang)

From repo root:

```sh
cmake -S test -B build_fuzz -DCMAKE_CXX_COMPILER=clang++ -DPHY_ENGINE_BUILD_FUZZERS=ON
cmake --build build_fuzz -j
```

The fuzzer binary will be at:

- `build_fuzz/fuzz/verilog_digital_fuzzer`

## Run

```sh
./build_fuzz/fuzz/verilog_digital_fuzzer \
  -max_len=4096 \
  -timeout=2 \
  -dict=fuzz/verilog.dict \
  -artifact_prefix=build_fuzz/fuzz/crash-
```

Note: the fuzzer supports a deterministic include pack layout to explore nested includes:

- `main\0a\0b\0c`

and resolves `` `include "a.vh" `` / `"b.vh"` / `"c.vh"` to the corresponding segments.

