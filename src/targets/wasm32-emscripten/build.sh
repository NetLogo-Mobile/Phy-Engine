#!/usr/bin/env bash
# Only for Dockerfile
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
ENGINE_DIR="$ROOT_DIR"
OUT_DIR="/wasm32-emscripten-pe-release"

mkdir -p "$OUT_DIR"
export EM_CACHE="${EM_CACHE:-$ROOT_DIR/.cache/emscripten}"
mkdir -p "$EM_CACHE"

em++ \
  -std=c++23 \
  -O3 \
  -I"$ENGINE_DIR/include" \
  "$ENGINE_DIR/src/dll_main.cpp" \
  -fno-rtti \
  -fno-exceptions \
  -fno-cxx-exceptions \
  -fno-unwind-tables \
  -s MODULARIZE=1 \
  -s EXPORT_ES6=1 \
  -s ENVIRONMENT=web \
  -s ALLOW_MEMORY_GROWTH=1 \
  -s FILESYSTEM=0 \
  -s EXPORTED_RUNTIME_METHODS='["cwrap"]' \
  -s EXPORTED_FUNCTIONS='["_malloc","_free","_create_circuit","_create_circuit_ex","_destroy_circuit","_circuit_set_analyze_type","_circuit_set_tr","_circuit_set_ac_omega","_circuit_analyze","_circuit_digital_clk","_circuit_sample_u8","_circuit_set_model_digital"]' \
  -o "$OUT_DIR/phy_engine.js"

# The generated ESModule does not expose HEAP views on the returned Module by default.
# Patch `updateMemoryViews()` so callers can access `Module.HEAPU8/HEAPU32/HEAPF64` and `Module.wasmMemory`.
node - <<'NODE'
import { readFileSync, writeFileSync } from "node:fs";
import path from "node:path";

const file = "/wasm32-emscripten-pe-release/phy_engine.js";
let s = readFileSync(file, "utf8");

const needle = "HEAPU64=new BigUint64Array(b)}";
if (!s.includes(needle)) {
  console.error("patch failed: updateMemoryViews() signature not found");
  process.exit(1);
}

const inject =
  "HEAPU64=new BigUint64Array(b);Module.HEAP8=HEAP8;Module.HEAPU8=HEAPU8;Module.HEAP32=HEAP32;Module.HEAPU32=HEAPU32;Module.HEAPF64=HEAPF64;Module.wasmMemory=wasmMemory}";

s = s.replace(needle, inject);
writeFileSync(file, s);
NODE

echo "Built:"
ls -la "$OUT_DIR/phy_engine.js" "$OUT_DIR/phy_engine.wasm"