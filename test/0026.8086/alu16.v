module alu16(
    input  wire [2:0]  op,
    input  wire [15:0] a,
    input  wire [15:0] b,
    output reg  [15:0] y,
    output wire        zf,
    output reg         cf,
    output wire        sf
);
    reg [16:0] tmp;
    reg [3:0] sh;

    always @(*) begin
        cf = 1'b0;
        tmp = 17'd0;
        sh = b[3:0];
        case (op)
            3'd0: begin  // ADD
                tmp = {1'b0, a} + {1'b0, b};
                y = tmp[15:0];
                cf = tmp[16];
            end
            3'd1: begin  // SUB (cf=borrow)
                tmp = {1'b0, a} - {1'b0, b};
                y = tmp[15:0];
                cf = tmp[16];
            end
            3'd2: y = a & b;  // AND
            3'd3: y = a | b;  // OR
            3'd4: y = a ^ b;  // XOR
            3'd5: y = b;      // MOV (pass B)
            3'd6: begin  // SHL a, b[3:0]
                y = a << sh;
                if(sh != 0) cf = a[16 - sh];
            end
            3'd7: begin  // SHR a, b[3:0]
                y = a >> sh;
                if(sh != 0) cf = a[sh - 1];
            end
            default: y = 16'd0;
        endcase
    end

    assign zf = (y == 16'd0);
    assign sf = y[15];
endmodule
