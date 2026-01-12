// Included by "risc-v.v" to define the instruction ROM contents.
// Default demo program:
//   x1=10; x2=20; x3=x1+x2=30;
//   sw x3,0(x0); lw x4,0(x0);
//   if (x4==30) x10=1 else x10=0;
//   loop forever.

case(pc[6:2])
  5'd0: instr = 32'h00a00093; // addi x1, x0, 10
  5'd1: instr = 32'h01400113; // addi x2, x0, 20
  5'd2: instr = 32'h002081b3; // add  x3, x1, x2
  5'd3: instr = 32'h00302023; // sw   x3, 0(x0)
  5'd4: instr = 32'h00002203; // lw   x4, 0(x0)
  5'd5: instr = 32'h01e00293; // addi x5, x0, 30
  5'd6: instr = 32'h00520663; // beq  x4, x5, +12 (to index 9)
  5'd7: instr = 32'h00000513; // addi x10, x0, 0
  5'd8: instr = 32'h0000006f; // jal  x0, 0  (loop)
  5'd9: instr = 32'h00100513; // addi x10, x0, 1
  5'd10: instr = 32'h0000006f; // jal x0, 0
  default: instr = 32'h00000013;
endcase
