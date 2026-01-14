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
- `alu16`: 16-bit ALU
- `flag1`: 1-bit Z flag register

The C++ test runs a small ROM program and asserts final register values and `halt`.

## Pinout (for `.sav` export)

The exporter `test/0026.multi-module/x86_16_multi_module_export_plsav.cc` generates `x86_16_multi_module_3d.sav` with:

- Internal layout: 3D stacked layers, each laid out in `(-1, -1/3, z) .. (1, +1/3, z)` with `z += 0.1`.
- IO placement: inputs in the top third, outputs in the bottom third.
- Bit ordering: **LSB on the right**, MSB on the left (`[0]` at `x=+1`).
- IO element type: all pins are exported as **Logic Output** elements (inputs and outputs both use the same PL element).

Inputs:
- `clk`
- `rst_n`

Outputs:
- `halt`
- `dbg_r0[15:0]`
- `dbg_r1[15:0]`
