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
- [ ] Macro arguments (function-like macros)
- [ ] Better `line`/column mapping through preprocessing (for diagnostics)

### SystemVerilog / Elaboration
- [ ] `parameter` / `localparam` and parameterized instantiation `#(...)`
- [ ] `generate` / `genvar` / `for-generate`
- [ ] Hierarchical names (e.g. `u0.sig`, `top.u1.sub.x`)
- [ ] `function` / `task` definitions and calls

### Expressions (coverage & semantics)
- [ ] Modulo `%`
- [ ] Power `**`
- [ ] Case equality `===` / `!==`
- [ ] Reduction operators (`&a`, `|a`, `^a`, `~&a`, `~|a`, `~^a`)
- [ ] Signed arithmetic rules (`signed` / `unsigned`, sign extension)
- [ ] Better 4-state semantics for arithmetic (full Verilog rules for `x/z`)
- [ ] More numeric literal formats (unsized based literals, underscores everywhere, etc.)

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

