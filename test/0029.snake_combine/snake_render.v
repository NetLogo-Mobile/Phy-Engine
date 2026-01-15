module snake_render(
    input  wire [5:0] idx_head,
    input  wire [5:0] idx0,
    input  wire [5:0] idx1,
    input  wire [5:0] idx2,
    input  wire [5:0] idx_food,
    input  wire       game_over,
    output wire [63:0] pix
);
    // Safe 64-bit one-hot for synthesis: avoid large variable shifts.
    function automatic [63:0] onehot64(input [5:0] idx);
        begin
            onehot64 = (idx < 6'd32) ? (64'h1 << idx) : ((64'h1 << 6'd32) << (idx - 6'd32));
        end
    endfunction

    wire [63:0] p_head;
    wire [63:0] p0;
    wire [63:0] p1;
    wire [63:0] p2;
    wire [63:0] p_food;
    assign p_head = onehot64(idx_head);
    assign p0 = onehot64(idx0);
    assign p1 = onehot64(idx1);
    assign p2 = onehot64(idx2);
    assign p_food = onehot64(idx_food);

    // "FAIL" in 8x8, 2 columns per letter (F A I L). Bits use idx = y*8 + x.
    // Byte order is row7..row0 (each byte is x=7..0).
    wire [63:0] FAIL_PIX;
    assign FAIL_PIX = 64'hC39ADA9E9A9A9A83;

    assign pix = game_over ? FAIL_PIX : (p_head | p0 | p1 | p2 | p_food);
endmodule
