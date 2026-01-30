# Verilog Digital Subset (Phy-Engine) — TODO

This folder contains the single-header Verilog parser/compiler/simulator used as a **digital “device”** inside Phy-Engine:
- `digital.h`

It intentionally implements a **small synthesizable subset** (not a full Verilog/SystemVerilog front-end).

## Current Capabilities (high level)

- Modules: `module` / `endmodule`
- Simple preprocessor: `define`/`undef`/`ifdef`/`ifndef`/`else`/`endif` + macro expansion
- Decls: `input`/`output`/`inout`, `wire`/`reg`/`logic`/`bit`, vectors `[msb:lsb]`, and packed integer types `byte`/`shortint`/`int`/`integer`/`longint` (treated as packed vectors in this subset)
- Continuous assign: `assign lhs = expr;` (lhs may include dynamic selects)
- Always blocks:
  - `always @*` / `always @(*)` / `always_comb` combinational (`if`/`case`/begin-end; blocking `=` or nonblocking `<=`) (runs once at t=0)
  - `always_latch` (treated like combinational `always @*` in this subset)
  - `always @(a or b or c)` event control lists (level-sensitive) for scheduling
  - `always @(posedge/negedge clk)` / `always_ff @(posedge clk or negedge rst_n)` sequential (nonblocking `<=`)
  - `initial` blocks (run once per instance; time origin = first `simulate()` tick observed; supports blocking `#delay` for straight-line assignment-only blocks)
- Expressions:
  - Bitwise: `~ & ^ |`
  - Logical: `&& || !`
  - Compare: `== != < <= > >=`
  - Arithmetic/shift: `+ - * / << >>`
  - Ternary: `?:`
  - Concatenation/replication: `{...}` and `{N{...}}` (rep-count must be constant)
  - Selects:
    - Bit-select: `expr[idx]` (idx may be dynamic)
    - Part-select: `expr[msb:lsb]` (msb/lsb must be constant)
    - Indexed part-select: `expr[start +: W]`, `expr[start -: W]` (W must be constant; start may be dynamic)
  - System functions (subset): `$clog2(expr)` (const only), `$bits(expr)`, `$signed(expr)`, `$unsigned(expr)`, `$urandom`/`$random`
  - Numeric literal note: unsized decimal numbers like `0`, `1`, `123` are treated as 32-bit signed (Verilog/SV behavior)
- Instances: named/positional connections; connections support identifiers, literals, concat/replication, and constant slices

## TODO / Not Yet Supported

### Preprocessor / Compilation Units
- [x] `` `include `` (via `preprocess_options::include_resolver`; no include search paths)
- [x] Macro arguments (function-like macros) (basic: `` `NAME(...) ``; no stringify/paste/line-continuation)
- [x] Better `line`/column mapping through preprocessing (for diagnostics)

### SystemVerilog / Elaboration
- [x] `parameter` / `localparam` and parameterized instantiation `#(...)` (subset: const-int params only; accepts `int unsigned` / `logic [..]` syntax; params modeled as `[31:0]`)
- [x] `generate` / `genvar` / `for-generate` (subset: for-generate that instantiates modules; genvar treated as compile-time constant)
- [x] Hierarchical names (subset: `inst.port` resolves to the parent-side connection expression for named port connections)
- [x] `function` / `task` definitions and calls (subset: procedural bodies are inlined at call sites)
- [x] SystemVerilog cast syntax `type'(expr)` with width/signedness and packed ranges (subset)
- [x] Procedural block-scoped declarations (`logic`/`bit`/`int`/`byte`/`shortint`/`longint`) with real scoping (subset: implemented via name-mangled internal signals)
- [x] Unsized fill literals `'0/'1/'x/'z` with context-width semantics (subset: parsed as 1-bit signed fill; sign-extension replicates)
- [x] Procedural function bodies via call-site inlining (subset: local decls + procedural statements; return via `<fname> = ...;`)
- [x] Full task bodies with procedural statements (subset: inlined, scalar/whole-vector output/inout args only)
- [ ] True 2-state semantics for `bit`/integral types (currently treated as 4-state `logic_t` everywhere)

### Expressions (coverage & semantics)
- [x] Modulo `%`
- [x] Power `**`
- [x] Case equality `===` / `!==`
- [x] Reduction operators (`&a`, `|a`, `^a`, `~&a`, `~|a`, `~^a`)
- [x] Signed arithmetic rules (`signed` / `unsigned`, sign extension)
- [x] Better 4-state semantics for arithmetic (full Verilog rules for `x/z`)
- [x] More numeric literal formats (unsized based literals, underscores everywhere, etc.)

### LHS / Assignments
- [x] Continuous assignment with dynamic LHS selects (e.g. `assign a[idx] = ...;`)
- [x] Dynamic part-select `[msb:lsb]` where `msb/lsb` are expressions (not constants)
- [x] Multiple drivers / net resolution (basic 0/1/X/Z; no strength modeling)

### Statements / Procedural Features
- [x] `for` / `while` / `repeat` (repeat count must be constant in this subset)
- [x] `do ... while (...) ;` (subset)
- [x] `casez` / `casex`
- [x] `always_comb` / `always_ff` keywords
- [x] `always_latch` keyword (treated like `always @*` in this subset)
- [x] `initial` blocks (subset)
- [x] Event control support (multi-item `@( ... or ... )`, posedge/negedge + level-sensitive lists)
- [x] Improved blocking/nonblocking timing (delta-cycle + NBA ordering)
- [x] `return;` in procedural function/task bodies (subset: inlined; `return expr;` lowered to assignment + `return;`)
- [x] `break;` / `continue;` inside `for`/`while` (subset)
- [x] Statement `++/--` (prefix/postfix) on scalar/whole-vector lvalues (subset)
- [x] Compound assignments (`+= -= *= /= %= <<= >>= |= &= ^=`) (subset; dynamic part-select compound is rejected)
- [x] Procedural declaration initializers (`int i = 0;`, `logic [3:0] x = 4'ha;`) (subset)
- [x] `for (int i = 0; ... )` loop-local declarations (subset)
- [x] `unique` / `unique0` / `priority` qualifiers before `if`/`case` (accepted/ignored in this subset)
- [x] Named begin/end blocks: `begin : label ... end : label` (labels ignored)

### Port Connections
- [x] General expression connections (e.g. `.a(a + b)`), not just name/literal/concat/slice
- [x] Width coercion rules (sign/zero extension, truncation)

### Performance / Robustness
- [x] More aggressive expression DAG sharing and simplification (constant folding, common subexpr)
- [x] Explicit compile-time limits and better error messages for huge expressions
- [x] More tests for edge cases (out-of-range selects, X/Z propagation, width corner cases)

## Notes

- This implementation is meant to be “good enough” for embedded digital behavior in circuits, not a full HDL toolchain.
- If you add features, prefer adding a focused test under `test/0007.verilog/` that demonstrates the expected behavior.

## PE Synthesis Optimizations (`pe_synth.h`) — Roadmap / TODO

This project also contains a Verilog→PE netlist synthesizer with **gate-count driven** post-synthesis optimizations.
The optimization pipeline supports LLVM/GCC-like levels via `pe_synth_options::opt_level`:
- `O0`: no extra gate-count passes (baseline; only enabled flags run)
- `O1`: “cheap & safe” local/structural passes
- `O2`: stronger multi-level rewrites + mapping-like passes
- `O3`: iterative pipeline + bounded two-level minimization on small cones

### Implemented (current)
- [x] Structural hashing / strash (AND/OR/XOR/XNOR/NAND/NOR/NOT, plus IMP/NIMP)
- [x] Dead code elimination (unused subgraphs)
- [x] Inverter optimization (double-NOT, inverter fusion into NAND/NOR/XNOR/…)
- [x] Input inverter push / “primitive selection” mapping (IMP/NIMP/NAND/NOR/XNOR, etc.)
- [x] Constant propagation (safe 4-valued identities; `assume_binary_inputs` enables extra identities)
- [x] AND/OR tree flattening (associativity), duplicate removal (idempotence), and constant folding (binary-only `x|~x` / `x&~x` when `assume_binary_inputs`)
- [x] Multi-term factoring on OR-trees-of-ANDs and AND-trees-of-ORs
- [x] Absorption / redundant-level elimination (`a&(a|b)->a`, `a|(a&b)->a`, etc.)
- [x] Local XOR/XNOR rewriting from SOP patterns
- [x] Two-level minimization on small, exclusive cones:
  - [x] Quine–McCluskey prime implicants + exact cover (very small) / greedy cover (moderate)
  - [x] Espresso-style heuristic minimization with ON/DC/OFF (bounded truth-table cones)
    - [x] “full” loop components: EXPAND + IRREDUNDANT + REDUCE + (bounded) “last gasp” perturbation
  - [x] Better PI selection for bounded cases (Petrick-style exact cover when feasible)
  - [x] Multi-output sharing (bounded): joint cover selection to encourage shared product terms across multiple outputs
  - [x] Stronger cost model: gate-count vs literal-count, plus optional primitive weights for 2-level selection
- [x] Tests validating correctness + gate-count improvement under `test/0015.verilog_compile/`

### TODO (not yet implemented)
#### `-Omax` / “budgeted multi-start” mode (beyond `O4`)
- [x] Add an `Omax` mode conceptually defined as: run the existing `O4` optimization pipeline in a loop until a **budget** is exhausted, trying to monotonically reduce gate count (or at least never regress).
- [x] Budgets must be explicit and user-configurable (examples; pick a minimal, coherent set):
  - [x] Global wall-clock timeout (e.g. `--opt-timeout-ms`)
  - [x] Max outer iterations / restarts (e.g. `--opt-max-iter`)
  - [x] Per-pass budgets (e.g. `--qm-max-vars`, `--qm-max-minterms`, `--resub-max-window`, `--rewrite-max-candidates`)
  - [x] Max memory / node growth limits (best-effort: model/gate rollback + node-growth early stop)
- [x] Make `Omax` deterministic by default (stable seeds / stable traversal order), with an opt-in randomized mode for “search” (e.g. `--opt-rand-seed`).
- [x] Define an “acceptance policy”:
  - [x] Default: only accept transformations that strictly reduce a cost metric (currently: `gate_count`), or accept equal-cost only if it enables later reductions (requires bookkeeping).
  - [x] Always prevent regressions unless explicitly allowed (e.g. `--opt-allow-regress` for research).
- [x] Add a cost model abstraction:
  - [x] `gate_count` (default)
  - [x] `literal_count` (optional; useful for 2-level)
  - [x] weighted costs per primitive (optional; to approximate “area”)
- [x] Add correctness strategy for `Omax` (must be testable and fast):
  - [x] For small cones: exact truth-table equivalence check after local rewrite/minimize. (Implemented as exhaustive IO check when input-count is small.)
  - [x] For larger graphs: bounded/random vector simulation as a regression guardrail (still keep per-pass soundness where possible).
- [x] Add reporting/telemetry:
  - [x] Per-iteration delta (models/gates) and per-pass deltas
  - [x] Optional dump of “best-so-far” netlist (via callback)
  - [x] A reproducible summary line (seed, budgets, final cost)
- [ ] (Optional research) GPU acceleration is primarily useful for high-throughput **evaluation/search** (e.g. many cone truth-tables / candidate scoring), but it does not change worst-case complexity; budgets remain mandatory.
  - [x] CUDA support (best-effort) + multi-GPU (device bitmask), used to accelerate bounded truth-table batching in `resub`/`sweep` when enabled at runtime (`--cuda-opt`).
  - Build example (Clang CUDA, no nvcc):
    - Recommended optimized build settings (important for `-Omax/-Ocuda` / `-O5` runtime):
      - `-DCMAKE_BUILD_TYPE=Release` (or `RelWithDebInfo` for profiling)
      - `-DCMAKE_INTERPROCEDURAL_OPTIMIZATION=ON` (LTO, if supported by your toolchain)
    - `cmake -S src -B build_cuda -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -DPHY_ENGINE_ENABLE_CUDA_PE_SYNTH=ON -DPHY_ENGINE_CUDA_PATH=/usr/local/cuda -DPHY_ENGINE_CUDA_PE_SYNTH_ARCH=sm_70 -DCMAKE_EXE_LINKER_FLAGS="-fuse-ld=lld" -DCMAKE_SHARED_LINKER_FLAGS="-fuse-ld=lld" `
    - `cmake --build build_cuda -j`
  - Run example (2x V100 -> mask 3):
    - `./build_cuda/verilog2plsav out.sav in.v -Ocuda --cuda-device-mask 3`
    - If you want to understand GPU utilization and CPU fallbacks:
      - `./build_cuda/verilog2plsav out.sav in.v -Ocuda --cuda-device-mask 3 --report --cuda-trace`
  - Notes:
    - `-Ocuda` is shorthand for `-Omax` (aka `-O5`) + enabling CUDA-assisted optimization.
    - To also increase some bounded windows by default (e.g. sweep/resub vars), pass `--cuda-expand-windows`.
    - `--cuda-trace` collects per-pass CUDA telemetry and prints it as part of `--report` output.

#### Two-level minimization (Espresso / full cover)
- [x] Espresso “industrial-strength” loop (additional heuristics beyond EXPAND/REDUCE/IRREDUNDANT, e.g. cube ordering + bounded last-gasp)
- [x] Multi-output sharing beyond exact cube identity (bounded partial sharing via common literal-pair extraction)
- [x] More advanced Espresso heuristics (binate variable ordering + complementation-based improvement via output inversion)
- [x] Richer multi-output sharing (kernel extraction / partial sharing across larger subcubes)

#### Don’t-care (DC-set) inference & exploitation
- [x] Derive DC from X/Z semantics and `assume_binary_inputs` (explicit DC-set plumbing in cone minimization; gated by `infer_dc_from_xz`)
- [x] Observability DC / controllability DC (ODC/CDC) propagation (bounded: local gate-mask ODC + CDC from FSM constraints with bounded state width)
- [x] FSM unreachable state detection (when sequential elements are present) → DC constraints (one-hot constant assignment inference)
- [x] Mutual-exclusion / predicate-based DC inference from conditionals (via one-hot FSM inference from constant `if/case` assignments)

#### AIG-style rewriting / resubstitution (beyond local patterns)
- [x] More rewrite templates (consensus-style AIG rewrites; AOI/OAI-like factoring using existing primitive library)
- [x] Resubstitution using existing nodes in the DAG (bounded truth-table matching with complemented reuse)
- [x] Sweeping / redundancy removal beyond DCE (SAT-free bounded cone truth-table hashing)
- [x] Iterative “area recovery” scheduling (rewrite/strash/resub/sweep inside O4 fixpoint loop)

#### Technology mapping (general)
- [x] Cost-driven DAG covering (“subject graph” → library patterns) for minimum gate count (bounded cones, template-based)
- [x] Cut-based DP mapper (bounded cut size) with area recovery (O3 loop, best-effort)
- [x] Optional support for a richer primitive library (AOI/OAI/etc.) and mapping into it (templates lowered to primitives)

#### Functional decomposition (large functions)
- [x] Decompose large cones into smaller sub-functions (BDD-based, bounded truth-table window) to reduce total gates (2-valued only; gated by `assume_binary_inputs` + `decompose_large_functions`)
- [x] Heuristics to decide when to decompose vs keep SOP/AIG form (gate-count acceptance; small variable-order tries; cofactor early-stop; mux/XOR shortcuts)

#### Engineering / UX
- [x] More regression tests targeting each pass + cross-pass interactions (especially `optimize_adders` vs mapping)
- [x] Better reporting (per-pass gate count deltas + O3 iteration gate-count trace via `pe_synth_report`)
