# Verilog CLI Tools: `verilog2plsav` and `verilog2penl`

This document describes the command-line parameters for the two Verilog front-end tools shipped in `src/`:

- `verilog2plsav`: Verilog -> PE netlist -> PhysicsLab `.sav` (with IO auto-placement and auto-layout).
- `verilog2penl`: Verilog -> PE-NL export (`.penl`, LevelDB-backed), optionally synthesizing to primitives.

The built-in `--help` output is intentionally short. This README expands each option with usage guidance, interactions, and practical examples.

## Common Concepts (Both Tools)

### Input Verilog and Includes

`IN.v` is read as text. Verilog `include` directives are resolved relative to the directory of `IN.v` (filesystem include resolver).

### Selecting the Top Module: `--top`

If you omit `--top`, the tools select the top module using a heuristic:

1) Prefer a module that is not instantiated by any other module.
2) If there are multiple candidates, choose the one with the most ports.
3) If there are no candidates, fall back to the last parsed module.

Use `--top TOP_MODULE` to override this behavior explicitly.

### Optimization Levels: `-O*` / `--opt-level`

Both tools share the PE synthesis optimization pipeline and accept the same optimization levels:

- `-O0`: Minimal / baseline synthesis.
- `-O1`, `-O2`: Increasingly aggressive "cheap" optimization passes.
- `-O3`: Fast medium-strength tier (bounded rewrites/resub/sweep; no QM/techmap/decompose fixpoint). Intended as a ~seconds-level option between `O2` and `O4`.
- `-O4`: Strong tier (full fixpoint-ish pipeline; historically this was the old `O3`).
- `-Omax` / `-O5`: "Budgeted multi-start search" mode. Runs the `O4` pipeline repeatedly under budgets (timeout / iteration count / per-pass caps), keeping the best solution found.
- `-Ocuda`: Shorthand for `-Omax` (i.e. `-O5`) + enabling CUDA-assisted optimization. It may also increase some default bounded windows to improve quality (see CUDA sections).
  - If you want to keep the same optimization knobs as `-O5` but just enable CUDA acceleration, use `-O5 --cuda-opt`.
  - If you want `-Ocuda` to also expand some bounded windows by default, pass `--cuda-expand-windows`.

You can also specify `--opt-level N` where `N` is `0..5`. If present, `--opt-level` overrides `-O*`.

### Safety vs. Aggressiveness: `assume_binary_inputs`

`--assume-binary-inputs` changes how unknown values are treated during optimization:

- When enabled, optimizations may treat `X/Z` as "don’t care / absent" in certain contexts to unlock stronger simplifications.
- When disabled, the optimizer preserves X-propagation semantics more conservatively.

Important: For designs that rely on `X/Z` behavior, disabling `--assume-binary-inputs` is safer.

### Omax Search Knobs (apply when `-Omax/-Ocuda` is used)

These flags control the outer "try multiple improvements" loop:

- `--opt-timeout-ms MS`
  - Wall-clock time limit for Omax. `0` disables the time limit.
- `--opt-max-iter N`
  - Maximum number of Omax attempts / restarts. Set to `0` to disable the loop (effectively disables Omax).
- `--opt-randomize`
  - Enables randomized search variants (useful for escaping local minima). Without this flag, Omax is deterministic.
- `--opt-rand-seed SEED`
  - Seed for randomized variants. Use this to reproduce results.
- `--opt-allow-regress`
  - Allow non-improving candidates to be kept. Without this flag, Omax only accepts improvements by default.

### Omax Correctness Guardrails

Omax can optionally sanity-check candidates against a pre-optimization reference:

- `--opt-verify`
  - Enables best-effort verification (combinational-only).
- `--opt-verify-exact-max-inputs N`
  - If the input count is <= `N`, perform an exhaustive equivalence check.
- `--opt-verify-rand-vectors N`
  - If not exhaustive, run random vector regression (bounded confidence).
- `--opt-verify-seed SEED`
  - Seed for verification vectors.

Note: This is a guardrail, not a formal proof for large sequential designs.

### Pass Budgets (performance/quality controls)

These flags cap the expensive inner loops. They apply to `O3`/`O4` and therefore also affect `Omax/-Ocuda`.

- `--qm-max-vars N`
  - Two-level minimization window size. `0` disables QM/Espresso.
- `--qm-max-gates N`
  - Max internal gates per two-level cone.
- `--qm-max-primes N`
  - Max prime implicants to consider (best-effort).
- `--qm-max-minterms N`
  - If non-zero: skip cones where `2^vars > N`.

- `--resub-max-vars N`
  - Resubstitution window size. `0` disables resub.
  - CPU-only path supports up to 6 vars (u64 truth-table).
  - With CUDA enabled, values up to 16 are supported via bitset truth tables.
- `--resub-max-gates N`
  - Cap internal gates for resub truth-table cones.

- `--sweep-max-vars N`
  - Sweep window size. `0` disables sweep.
  - CPU-only path supports up to 6 vars (u64 truth-table).
  - With CUDA enabled, values up to 16 are supported via bitset truth tables.
- `--sweep-max-gates N`
  - Cap internal gates for sweep truth-table cones.

- `--rewrite-max-candidates N`
  - Limits the number of candidate roots considered by rewriting (0 = unlimited).

- `--max-total-nodes N`, `--max-total-models N`, `--max-total-logic-gates N`
  - Global "growth guards". When set, optimization stops/rolls back best-effort if the netlist would grow past the limit.

### Cost Model (Omax objective)

Omax needs an objective to decide what is "better":

- `--opt-cost gate|weighted`
  - `gate`: minimize primitive gate instance count (includes `YES`).
  - `weighted`: minimize weighted sum of primitive instances (approximate "area").
- `--opt-weight-<GATE> N`
  - Set weights for the weighted objective. Supported gate names:
    - `NOT`, `AND`, `OR`, `XOR`, `XNOR`, `NAND`, `NOR`, `IMP`, `NIMP`, `YES`, `CASE_EQ`, `IS_UNKNOWN`

### Reporting

- `--report`
  - Prints a per-pass / per-iteration report, plus an Omax summary line when `-Omax/-Ocuda` is used.
- `--cuda-trace`
  - Collects per-pass CUDA telemetry and prints it as part of `--report` output (batch sizes, time, and CPU fallback reasons).

### CUDA Acceleration (build-time + runtime)

CUDA support is optional and must be built in. At runtime, CUDA usage is best-effort:

- Build-time: compile the tools with `-DPHY_ENGINE_ENABLE_CUDA_PE_SYNTH=ON` (see `include/phy_engine/verilog/digital/README.md` for build examples).
- Runtime flags:
  - `--cuda-opt`: enable CUDA-assisted optimization.
  - `--cuda-device-mask MASK`: bitmask of devices to use (`0` = all). Example: `3` uses GPU0 and GPU1.
  - `--cuda-min-batch N`: minimum cone batch size before offloading. Higher values reduce GPU overhead for small designs.
  - `--cuda-trace`: print per-pass CUDA stats (requires `--report`).

`-Ocuda` is a convenience mode: it implies `-Omax` and enables CUDA optimization. It also adjusts a few defaults to improve optimization quality and GPU utilization (e.g. larger sweep/resub windows and smaller `--cuda-min-batch`).

## Tool: `verilog2plsav`

### Purpose

Compiles a Verilog source (subset), synthesizes it into PE primitives with optimizations, then exports a PhysicsLab `.sav`.

### Usage

```
verilog2plsav OUT.sav IN.v [--top TOP_MODULE]
```

### Options

#### Synthesis and Optimization

All "Common Concepts" options apply, with one key default:

- `--assume-binary-inputs` is **enabled by default** in `verilog2plsav`.

#### Export and Layout

These flags control the `.sav` export and auto-layout:

- `--layout fast|cluster|spectral|hier|force`
  - Selects the layout algorithm used for placing the synthesized elements.
  - `fast` is the default; use `spectral`/`force` for potentially better quality at higher runtime.

- `--layout3d xy|hier|force|pack`
  - Enables 3D placement. The Z step is fixed at `0.02`.
  - `xy`: 2D layout + incremental Z lift
  - `hier`: hierarchical layering in Z
  - `force`: 3D force-directed
  - `pack`: pack in XY + Z lift

- `--layout3d-z-base Z`
  - Base Z coordinate used by the 3D layout variants (default: `0`).

- `--no-wires`
  - Disable auto wire generation.

- `--layout-step STEP`
  - Grid step for layout (default: `0.01`).

- `--layout-min-step STEP`
  - Minimum grid step when auto-refinement is enabled (default: `0.001`).

- `--layout-fill FACTOR`
  - Capacity safety factor (must be `>= 1.0`, default: `1.25`). Higher values reduce density/overlaps.

- `--no-layout-refine`
  - Disables step auto-refinement (faster, potentially worse placement).

- `--cluster-max-nodes N`
  - Controls cluster macro size for cluster layout (default: `24`).

- `--cluster-channel-spacing N`
  - Controls spacing between clustered regions (default: `2`).

### Examples

1) Fast baseline export:
```
verilog2plsav out.sav design.v -O2
```

2) Omax quality search with a hard time budget:
```
verilog2plsav out.sav design.v -Omax --opt-timeout-ms 2000 --opt-max-iter 64 --report
```

3) CUDA-assisted Omax on two GPUs:
```
verilog2plsav out.sav design.v -Ocuda --cuda-device-mask 3 --report
```

4) Make optimization cheaper by disabling heavy passes:
```
verilog2plsav out.sav design.v -Omax --qm-max-vars 0 --resub-max-vars 0 --sweep-max-vars 0
```

## Tool: `verilog2penl`

### Purpose

Exports a PE netlist in PE-NL format (LevelDB-backed). This tool supports two workflows:

1) Default: store the Verilog module as a `VERILOG_MODULE` block in a PE netlist, optionally with IO wrappers.
2) `--synth`: synthesize to PE primitives (no `VERILOG_MODULE`) before exporting.

### Usage

```
verilog2penl OUT.penl IN.v [--top TOP_MODULE]
```

Note: `verilog2penl` is built only when LevelDB support is enabled in the build configuration.

### Options

#### Output Layout and Export Mode

- `--layout file|dir`
  - `file` (default): output a single `.penl` file.
  - `dir`: output as a directory layout (useful for very large designs / different storage constraints).

- `--mode full|structure|checkpoint`
  - `full` (default): export structure + runtime state.
  - `structure`: export structure only.
  - `checkpoint`: export runtime/checkpoint only.

- `--overwrite`
  - Overwrite existing output.

#### IO Modeling and Semantics

- `--no-io`
  - Do not generate `INPUT/OUTPUT` models for the module ports.

- `--allow-inout`
  - Allow inout ports. Requires `--no-io` (because IO wrapper generation assumes pure in/out).

- `--allow-multi-driver`
  - Allow multi-driver digital nets (unsafe for many designs unless you know what you are doing).

#### Synthesis Mode

- `--synth`
  - Actually run PE synthesis and export primitives.
  - When `--synth` is NOT set, optimization flags (`-O*`, budgets, etc.) are parsed but synthesis is not performed.

#### Optimization Defaults

All "Common Concepts" options apply, with one key default:

- `--assume-binary-inputs` is **disabled by default** in `verilog2penl`.

### Examples

1) Export structure only without synthesis:
```
verilog2penl out.penl design.v --mode structure
```

2) Synthesize and export primitives with conservative X/Z behavior:
```
verilog2penl out.penl design.v --synth -O3 --no-assume-binary-inputs --report
```

3) CUDA-assisted, higher-quality synthesis to primitives:
```
verilog2penl out.penl design.v --synth -Ocuda --cuda-device-mask 3 --report
```

## Practical Tuning Tips

- If `-Omax` is too slow: start by lowering `--opt-max-iter`, disabling expensive passes (`--qm-max-vars 0`), and/or shrinking window sizes (`--resub-max-vars`, `--sweep-max-vars`).
- If `-Ocuda` does not appear to use the GPU: reduce `--cuda-min-batch`, and make sure you built with `PHY_ENGINE_ENABLE_CUDA_PE_SYNTH=ON`.
- If results look "too aggressive" for X/Z-heavy designs: disable `--assume-binary-inputs`.
