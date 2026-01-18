# 0026.multi-module

This test demonstrates composing **multiple Verilog modules** into one PE circuit by:

- Converting each module independently into a `VERILOG_MODULE` (one top module per conversion).
- Wiring module ports together as shared PE nodes (pin-linking between “sub-circuits”).

The example is a tiny **16-bit toy CPU** (not a full x86/8086 implementation) built from:

- `pc8`: program counter register
- `rom256x16`: instruction ROM
- `ir16`: instruction register (latches ROM output)
- `decode16`: instruction field decode
- `control16`: control unit (PC / reg write / ALU op)
- `imm_ext8_to_16`: sign-extend immediate
- `regfile4x16`: 4×16 register file
- `alu16`: 16-bit ALU (split into `alu16_*` op blocks + `alu16_select`)
- `flag1`: 1-bit Z flag register

The C++ test runs a small ROM program and asserts final register values and `halt`.

## Pinout (for `.sav` export)

The exporter `test/0026.multi-module/x86_16_multi_module_export_plsav.cc` generates `x86_16_multi_module_3d.sav` with:

- Internal layout: 3D stacked layers, each laid out in `(-1, -1/3, z) .. (1, +1/3, z)` with `z += 0.1`.
- IO placement: inputs in the top third, outputs in the bottom third.
- Bit ordering: **LSB on the right**, MSB on the left (`[0]` at `x=+1`).
- IO element type: all pins are exported as **Logic Output** elements (inputs and outputs both use the same PL element).
- Note: PhysicsLab stores positions as the string `"x,z,y"` (Z is the middle value).

Inputs:
- `clk`
- `rst_n`

Outputs:
- `halt`
- `dbg_r0[15:0]`
- `dbg_r1[15:0]`
- `dbg_r2[15:0]`
- `dbg_r3[15:0]`

## Debug/Trace Environment Variables

This directory includes two debugging helpers:

- Exporter: `test/0026.8086/x86_16_multi_module_export_plsav.cc`
- PE trace sim: `test/0026.8086/x86_16_multi_module_plsav_trace.cc`

Set the following environment variables to enable extra debug features.

### Exporter (`x86_16_multi_module_export_plsav.cc`)

- `PHY_ENGINE_TRACE_0026_EXPORT_MINIMAL`
  - When set (to any value), exports only the minimal pin set (no extra internal debug signals).
  - Adds groups such as `pc_next[*]`, `rom_data[*]`, `ir[*]`, decoded fields (`opcode[*]`, `reg_dst[*]`, `reg_src[*]`), control signals (`pc_we`, `reg_we`, `alu_b_sel`, `flags_we_*`), regfile ports, and ALU ports/flags.

- `PHY_ENGINE_TRACE_0026_EXPORT_OUTPUT_COLS`
  - Integer (default `1`). Controls how many columns the *output pin groups* are split into in the PhysicsLab IO region.
  - Useful when `PHY_ENGINE_TRACE_0026_EXPORT_DEBUG_PINS` is enabled and you need more “columns/rows” visible for debugging.

- `PHY_ENGINE_TRACE_0026_EXPORT_SKIP_PINS`
  - When set, skips calling `add_pin_outputs()` (no exported IO pins are created as `Logic Output` elements).
  - Intended only for A/B testing whether the pin export itself affects file size/layout.

- `PHY_ENGINE_TRACE_0026_EXPORT_VALIDATE_PINS`
  - When set, validates that every expected pin label (inputs + outputs) appears exactly once as a `Logic Output` element in the generated `.sav`.
  - Fails fast with an exception if any label is missing or duplicated.

- `PHY_ENGINE_TRACE_0026_EXPORT_LAYERS`
  - When set, prints a short summary of synthesized PE “layers” (module name and model count), capped to the first 10 layers.
  - Helps correlate “which module/layer” might be responsible for PE vs PL behavior differences.

- `PHY_ENGINE_TRACE_0026_EXPORT_VALIDATE_LAYOUT`
  - When set, performs a basic post-export layout sanity check on the generated `.sav` content:
    - pins have valid `Position` strings,
    - pins are on the IO plane (`z == 0`),
    - inputs are in the top band and outputs in the bottom band,
    - non-pin elements have non-trivial Z layering (not collapsed at the origin).

### PE Trace (`x86_16_multi_module_plsav_trace.cc`)

- `PHY_ENGINE_TRACE_0026_PLSAV`
  - When set, prints a per-step trace including `clk/rst_n/halt/pc/ir/dbg_r0/dbg_r1` and timing (`sim_ns` and `t_ns`).

- `PHY_ENGINE_TRACE_0026_PLSAV_LAYERS`
  - When set (and `PHY_ENGINE_TRACE_0026_PLSAV` is set), prints one extra “layer summary” line per step.
  - The summary is intentionally compact (≤10 groups) and includes key inter-module nets and control signals:
    - `pc_next/pc_we`, `rom_data`, decoded fields, control signals, regfile ports, mux/ALU ports, and latched flags.

## Implemented ISA (toy, 16-bit)

Instruction word is `instr[15:0]`:

- `instr[15:12]` = `opcode`
- `instr[11:10]` = `reg_dst` (register index 0..3)
- `instr[9:8]` = `reg_src` (register index 0..3)
- `instr[7:0]` = `imm8` / `addr8`

Registers:

- 4 general registers: `R0..R3` (16-bit)
- Flags: `Z` (zero), `C` (carry/borrow), `S` (sign)

Immediate rules:

- `imm8` is sign-extended to 16-bit (`imm16 = {{8{imm8[7]}}, imm8}`)
- Used as ALU operand for `*_I` instructions, and as absolute address for `JMP/JZ/JNZ`

Opcodes:

- `0x0 EXT/NOP` : extended ops; `imm8[7:4]==0` is `NOP`, otherwise:
  - `0x01s SHLI dst, shamt4` : `R[dst] <- R[dst] << shamt4`, updates `Z/C/S`
  - `0x02s SHRI dst, shamt4` : `R[dst] <- R[dst] >> shamt4`, updates `Z/C/S`
  - `0x03? SHLR dst, src` : `R[dst] <- R[dst] << R[src][3:0]`, updates `Z/C/S`
  - `0x04? SHRR dst, src` : `R[dst] <- R[dst] >> R[src][3:0]`, updates `Z/C/S`
- `0x1 MOVI dst, imm8` : `R[dst] <- imm16`
- `0x2 ADDI dst, imm8` : `R[dst] <- R[dst] + imm16`, updates `Z/C/S`
- `0x3 XORI dst, imm8` : `R[dst] <- R[dst] ^ imm16`, updates `Z/C/S`
- `0x4 JMP addr8` : `PC <- addr8`
- `0x5 JZ addr8` : if `Z==1` then `PC <- addr8` else fall-through
- `0x6 MOVR dst, src` : `R[dst] <- R[src]`
- `0x7 ADDR dst, src` : `R[dst] <- R[dst] + R[src]`, updates `Z/C/S`
- `0x8 SUBR dst, src` : `R[dst] <- R[dst] - R[src]`, updates `Z/C/S`
- `0x9 ANDR dst, src` : `R[dst] <- R[dst] & R[src]`, updates `Z/C/S`
- `0xA ORR dst, src` : `R[dst] <- R[dst] | R[src]`, updates `Z/C/S`
- `0xB XORR dst, src` : `R[dst] <- R[dst] ^ R[src]`, updates `Z/C/S`
- `0xC CMPR dst, src` : updates `Z/C/S` from `R[dst] - R[src]` (no register write)
- `0xD JNZ addr8` : if `Z==0` then `PC <- addr8` else fall-through
- `0xE SUBI dst, imm8` : `R[dst] <- R[dst] - imm16`, updates `Z/C/S`
- `0xF HLT` : assert `halt`, stop PC update

Flag semantics:

- Flags are latched from `alu16` outputs when the corresponding `flags_we_*==1`.
- `JZ/JNZ` use the previously latched `Z` (from the last flag-updating instruction).
