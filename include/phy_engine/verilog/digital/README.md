# Verilog Digital Subset (Phy-Engine) — TODO

This folder contains the single-header Verilog parser/compiler/simulator used as a **digital “device”** inside Phy-Engine:
- `digital.h`

It intentionally implements a **small synthesizable subset** (not a full Verilog/SystemVerilog front-end).

## Current Capabilities (high level)

- Modules: `module` / `endmodule`
- Simple preprocessor: `define`/`undef`/`ifdef`/`ifndef`/`else`/`endif` + macro expansion
- Decls: `input`/`output`/`inout`, `wire`/`reg`, vectors `[msb:lsb]`, `output reg`
- Continuous assign: `assign lhs = expr;` (lhs must be static)
- Always blocks:
  - `always @*` / `always @(*)` combinational (`if`/`case`/begin-end; blocking `=`)
  - `always @(posedge/negedge clk)` sequential (nonblocking `<=`)
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
- Instances: named/positional connections; connections support identifiers, literals, concat/replication, and constant slices

## TODO / Not Yet Supported

### Preprocessor / Compilation Units
- [ ] `` `include `` (file loading, include search paths)
- [x] Macro arguments (function-like macros) (basic: `` `NAME(...) ``; no stringify/paste/line-continuation)
- [x] Better `line`/column mapping through preprocessing (for diagnostics)

### SystemVerilog / Elaboration
- [x] `parameter` / `localparam` and parameterized instantiation `#(...)` (subset: const-int params only; params modeled as `[31:0]`)
- [x] `generate` / `genvar` / `for-generate` (subset: for-generate that instantiates modules; genvar treated as compile-time constant)
- [x] Hierarchical names (subset: `inst.port` resolves to the parent-side connection expression for named port connections)
- [x] `function` / `task` definitions and calls (subset: single-expression functions + single-assignment tasks)

### Expressions (coverage & semantics)
- [x] Modulo `%`
- [x] Power `**`
- [x] Case equality `===` / `!==`
- [x] Reduction operators (`&a`, `|a`, `^a`, `~&a`, `~|a`, `~^a`)
- [x] Signed arithmetic rules (`signed` / `unsigned`, sign extension)
- [x] Better 4-state semantics for arithmetic (full Verilog rules for `x/z`)
- [x] More numeric literal formats (unsized based literals, underscores everywhere, etc.)

### LHS / Assignments
- [ ] Continuous assignment with dynamic LHS selects (e.g. `assign a[idx] = ...;`)
- [ ] Dynamic part-select `[msb:lsb]` where `msb/lsb` are expressions (not constants)
- [ ] Multiple drivers / proper net resolution (wire strength, etc.)

### Statements / Procedural Features
- [ ] `for` / `while` / `repeat`
- [ ] `casez` / `casex`
- [ ] `always_comb` / `always_ff` keywords (current parser uses `always`)
- [ ] Full event control support (more complex sensitivity/event expressions)
- [ ] Blocking/nonblocking timing semantics beyond the current simplified scheduler

### Port Connections
- [ ] General expression connections (e.g. `.a(a + b)`), not just name/literal/concat/slice
- [ ] Width coercion rules matching real Verilog (sign/zero extension, truncation details)

### Performance / Robustness
- [ ] More aggressive expression DAG sharing and simplification (constant folding, common subexpr)
- [ ] Explicit compile-time limits and better error messages for huge expressions
- [ ] More tests for edge cases (out-of-range selects, X/Z propagation, width corner cases)

## Notes

- This implementation is meant to be “good enough” for embedded digital behavior in circuits, not a full HDL toolchain.
- If you add features, prefer adding a focused test under `test/0007.verilog/` that demonstrates the expected behavior.
