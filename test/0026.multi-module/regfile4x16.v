module regfile4x16(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        we,
    input  wire [1:0]  addr,
    input  wire [15:0] wdata,
    output reg  [15:0] rdata,
    output wire [15:0] dbg_r0,
    output wire [15:0] dbg_r1,
    output wire [15:0] dbg_r2,
    output wire [15:0] dbg_r3
);
    reg [15:0] r0;
    reg [15:0] r1;
    reg [15:0] r2;
    reg [15:0] r3;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            r0 <= 16'd0;
            r1 <= 16'd0;
            r2 <= 16'd0;
            r3 <= 16'd0;
        end else if(we) begin
            case (addr)
                2'd0: r0 <= wdata;
                2'd1: r1 <= wdata;
                2'd2: r2 <= wdata;
                2'd3: r3 <= wdata;
                default: begin end
            endcase
        end
    end

    always @(*) begin
        case (addr)
            2'd0: rdata = r0;
            2'd1: rdata = r1;
            2'd2: rdata = r2;
            2'd3: rdata = r3;
            default: rdata = 16'd0;
        endcase
    end

    assign dbg_r0 = r0;
    assign dbg_r1 = r1;
    assign dbg_r2 = r2;
    assign dbg_r3 = r3;
endmodule

