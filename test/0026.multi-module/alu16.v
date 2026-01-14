module alu16(
    input  wire [2:0]  op,
    input  wire [15:0] a,
    input  wire [15:0] b,
    output reg  [15:0] y,
    output wire        zf
);
    always @(*) begin
        case (op)
            3'd0: y = a + b;      // ADD
            3'd1: y = a - b;      // SUB
            3'd2: y = a & b;      // AND
            3'd3: y = a | b;      // OR
            3'd4: y = a ^ b;      // XOR
            3'd5: y = b;          // MOV (pass B)
            default: y = 16'd0;
        endcase
    end

    assign zf = (y == 16'd0);
endmodule

