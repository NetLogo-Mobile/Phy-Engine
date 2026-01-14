module rom256x16(
    input  wire [7:0]  addr,
    output reg  [15:0] data
);
    always @(*) begin
        case (addr)
            // Encoding: [15:12]=opcode, [9:8]=reg_sel, [7:0]=imm8/addr8
            8'h00: data = 16'h1005; // MOVI R0, 5
            8'h01: data = 16'h2007; // ADDI R0, 7
            8'h02: data = 16'h300F; // XORI R0, 0x0F
            8'h03: data = 16'h20FD; // ADDI R0, -3  (0xFD)
            8'h04: data = 16'h5006; // JZ 0x06
            8'h05: data = 16'h1155; // MOVI R1, 0x55 (should be skipped)
            8'h06: data = 16'hF000; // HLT
            default: data = 16'h0000;
        endcase
    end
endmodule

