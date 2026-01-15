module snake_dir(
    input  wire       btn_up,
    input  wire       btn_down,
    input  wire       btn_left,
    input  wire       btn_right,
    input  wire [1:0] dir,
    output wire [1:0] next_dir
);
    // Direction encoding:
    // 00: right, 01: down, 10: left, 11: up
    wire [1:0] raw_dir;
    assign raw_dir = btn_up    ? 2'b11 :
                     btn_down  ? 2'b01 :
                     btn_left  ? 2'b10 :
                     btn_right ? 2'b00 :
                                 dir;

    // Prevent immediate 180-degree reversal (for length=4).
    assign next_dir =
        ((dir == 2'b00) && (raw_dir == 2'b10)) ? dir :
        ((dir == 2'b10) && (raw_dir == 2'b00)) ? dir :
        ((dir == 2'b01) && (raw_dir == 2'b11)) ? dir :
        ((dir == 2'b11) && (raw_dir == 2'b01)) ? dir :
        raw_dir;
endmodule

