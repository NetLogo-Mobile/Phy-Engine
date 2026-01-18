module alu16(
    input  wire [2:0]  op,
    input  wire [15:0] a,
    input  wire [15:0] b,
    output reg  [15:0] y,
    output wire        zf,
    output reg         cf,
    output wire        sf
);
    wire        sub;
    wire [15:0] y_addsub;
    wire        cf_addsub;
    wire [15:0] y_and;
    wire [15:0] y_or;
    wire [15:0] y_xor;
    wire [15:0] y_mov;
    wire [15:0] y_shl;
    wire        cf_shl;
    wire [15:0] y_shr;
    wire        cf_shr;

    alu16_sub_decode u_sub_dec(.op(op), .sub(sub));
    alu16_addsub u_addsub(.sub(sub), .a(a), .b(b), .y(y_addsub), .cf(cf_addsub));
    alu16_and    u_and(.a(a), .b(b), .y(y_and));
    alu16_or     u_or(.a(a), .b(b), .y(y_or));
    alu16_xor    u_xor(.a(a), .b(b), .y(y_xor));
    alu16_mov    u_mov(.b(b), .y(y_mov));
    alu16_shl    u_shl(.a(a), .b(b), .y(y_shl), .cf(cf_shl));
    alu16_shr    u_shr(.a(a), .b(b), .y(y_shr), .cf(cf_shr));

    wire [15:0] y_sel;
    wire        cf_sel;
    alu16_select u_sel(
        .op(op),
        .y_addsub(y_addsub), .cf_addsub(cf_addsub),
        .y_and(y_and),
        .y_or(y_or),
        .y_xor(y_xor),
        .y_mov(y_mov),
        .y_shl(y_shl), .cf_shl(cf_shl),
        .y_shr(y_shr), .cf_shr(cf_shr),
        .y(y_sel), .zf(zf), .cf(cf_sel), .sf(sf)
    );

    always @(*) begin
        y = y_sel;
        cf = cf_sel;
    end
endmodule
