module regfile4x16(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        we,
    input  wire [1:0]  waddr,
    input  wire [15:0] wdata,
    input  wire [1:0]  raddr_a,
    input  wire [1:0]  raddr_b,
    output reg  [15:0] rdata_a,
    output reg  [15:0] rdata_b,
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
            case (waddr)
                2'd0: r0 <= wdata;
                2'd1: r1 <= wdata;
                2'd2: r2 <= wdata;
                2'd3: r3 <= wdata;
                default: begin end
            endcase
        end
    end

    always @(*) begin
        case (raddr_a)
            2'd0: rdata_a = r0;
            2'd1: rdata_a = r1;
            2'd2: rdata_a = r2;
            2'd3: rdata_a = r3;
            default: rdata_a = 16'd0;
        endcase

        case (raddr_b)
            2'd0: rdata_b = r0;
            2'd1: rdata_b = r1;
            2'd2: rdata_b = r2;
            2'd3: rdata_b = r3;
            default: rdata_b = 16'd0;
        endcase
    end

    assign dbg_r0 = r0;
    assign dbg_r1 = r1;
    assign dbg_r2 = r2;
    assign dbg_r3 = r3;
endmodule
