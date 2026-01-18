module alu16_select(
    input  wire [2:0]  op,
    input  wire [15:0] y_addsub,
    input  wire        cf_addsub,
    input  wire [15:0] y_and,
    input  wire [15:0] y_or,
    input  wire [15:0] y_xor,
    input  wire [15:0] y_mov,
    input  wire [15:0] y_shl,
    input  wire        cf_shl,
    input  wire [15:0] y_shr,
    input  wire        cf_shr,
    output reg  [15:0] y,
    output wire        zf,
    output reg         cf,
    output wire        sf
);
    always @(*) begin
        y = 16'd0;
        cf = 1'b0;
        case (op)
            3'd0: begin y = y_addsub; cf = cf_addsub; end
            3'd1: begin y = y_addsub; cf = cf_addsub; end
            3'd2: begin y = y_and; cf = 1'b0; end
            3'd3: begin y = y_or;  cf = 1'b0; end
            3'd4: begin y = y_xor; cf = 1'b0; end
            3'd5: begin y = y_mov; cf = 1'b0; end
            3'd6: begin y = y_shl; cf = cf_shl; end
            3'd7: begin y = y_shr; cf = cf_shr; end
            default: begin y = 16'd0; cf = 1'b0; end
        endcase
    end

    assign zf = (y == 16'd0);
    assign sf = y[15];
endmodule
