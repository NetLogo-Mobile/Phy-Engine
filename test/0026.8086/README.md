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
