module tetris6x6 (
    input  clk,
    input  rst,
    input  left,
    input  right,
    input  rotate,

    output reg game_over,
    output [5:0] row0,
    output [5:0] row1,
    output [5:0] row2,
    output [5:0] row3,
    output [5:0] row4,
    output [5:0] row5
);

    // ============================
    // 6x6 locked board
    // ============================
    reg [5:0] b0, b1, b2, b3, b4, b5;

    // ============================
    // active piece meta
    //  ptype: 0=I 1=O 2=T 3=L
    //  prot : 0..3 (rotation)
    //  px/py: top-left origin for the piece's local bounding box
    // ============================
    reg [1:0] ptype;
    reg [1:0] prot;
    reg [2:0] px;
    reg [2:0] py;
    reg [1:0] next_type;

    // For pe_synth's internal $random lowering: provide an active-low reset signal name.
    wire rst_n;
    assign rst_n = ~rst;
    wire [1:0] rand2;
    assign rand2 = $random;

    // FSM: 0=PLAY, 1=CLEAR, 2=SPAWN
    reg [1:0] state;
    reg [2:0] scan;

    wire [5:0] FULL;
    assign FULL = 6'b111111;

    // ============================
    // current piece shape (4 local rows, LSB is left)
    // ============================
    reg [3:0] sh0, sh1, sh2, sh3;
    reg [2:0] sh_w, sh_h;

    always @(*) begin
        sh0 = 4'b0000; sh1 = 4'b0000; sh2 = 4'b0000; sh3 = 4'b0000;
        sh_w = 3'd0; sh_h = 3'd0;
        if (ptype == 2'd0) begin
            // I
            if (prot == 2'd0 || prot == 2'd2) begin
                sh0 = 4'b1111; sh_w = 3'd4; sh_h = 3'd1;
            end else begin
                sh0 = 4'b0001; sh1 = 4'b0001; sh2 = 4'b0001; sh3 = 4'b0001;
                sh_w = 3'd1; sh_h = 3'd4;
            end
        end else if (ptype == 2'd1) begin
            // O
            sh0 = 4'b0011; sh1 = 4'b0011;
            sh_w = 3'd2; sh_h = 3'd2;
        end else if (ptype == 2'd2) begin
            // T
            if (prot == 2'd0) begin
                sh0 = 4'b0111; sh1 = 4'b0010; sh_w = 3'd3; sh_h = 3'd2;
            end else if (prot == 2'd1) begin
                sh0 = 4'b0010; sh1 = 4'b0011; sh2 = 4'b0010; sh_w = 3'd2; sh_h = 3'd3;
            end else if (prot == 2'd2) begin
                sh0 = 4'b0010; sh1 = 4'b0111; sh_w = 3'd3; sh_h = 3'd2;
            end else begin
                sh0 = 4'b0001; sh1 = 4'b0011; sh2 = 4'b0001; sh_w = 3'd2; sh_h = 3'd3;
            end
        end else begin
            // L
            if (prot == 2'd0) begin
                sh0 = 4'b0111; sh1 = 4'b0100; sh_w = 3'd3; sh_h = 3'd2;
            end else if (prot == 2'd1) begin
                sh0 = 4'b0001; sh1 = 4'b0001; sh2 = 4'b0011; sh_w = 3'd2; sh_h = 3'd3;
            end else if (prot == 2'd2) begin
                sh0 = 4'b0001; sh1 = 4'b0111; sh_w = 3'd3; sh_h = 3'd2;
            end else begin
                sh0 = 4'b0011; sh1 = 4'b0001; sh2 = 4'b0001; sh_w = 3'd2; sh_h = 3'd3;
            end
        end
    end

    // ============================
    // active piece masks in board coordinates
    // ============================
    wire active_en;
    assign active_en = (state == 2'd0);

    wire [5:0] s0, s1, s2, s3;
    assign s0 = ({2'b00, sh0} << px);
    assign s1 = ({2'b00, sh1} << px);
    assign s2 = ({2'b00, sh2} << px);
    assign s3 = ({2'b00, sh3} << px);

    wire [5:0] ar0, ar1, ar2, ar3, ar4, ar5;
    assign ar0 = active_en ? ((py == 3'd0) ? s0 :
                             (py == 3'd1) ? 6'd0 :
                             (py == 3'd2) ? 6'd0 :
                             (py == 3'd3) ? 6'd0 :
                             (py == 3'd4) ? 6'd0 : 6'd0) : 6'd0;
    assign ar1 = active_en ? ((py == 3'd0) ? s1 :
                             (py == 3'd1) ? s0 :
                             (py == 3'd2) ? 6'd0 :
                             (py == 3'd3) ? 6'd0 :
                             (py == 3'd4) ? 6'd0 : 6'd0) : 6'd0;
    assign ar2 = active_en ? ((py == 3'd0) ? s2 :
                             (py == 3'd1) ? s1 :
                             (py == 3'd2) ? s0 :
                             (py == 3'd3) ? 6'd0 :
                             (py == 3'd4) ? 6'd0 : 6'd0) : 6'd0;
    assign ar3 = active_en ? ((py == 3'd0) ? s3 :
                             (py == 3'd1) ? s2 :
                             (py == 3'd2) ? s1 :
                             (py == 3'd3) ? s0 :
                             (py == 3'd4) ? 6'd0 : 6'd0) : 6'd0;
    assign ar4 = active_en ? ((py == 3'd0) ? 6'd0 :
                             (py == 3'd1) ? s3 :
                             (py == 3'd2) ? s2 :
                             (py == 3'd3) ? s1 :
                             (py == 3'd4) ? s0 : 6'd0) : 6'd0;
    assign ar5 = active_en ? ((py == 3'd0) ? 6'd0 :
                             (py == 3'd1) ? 6'd0 :
                             (py == 3'd2) ? s3 :
                             (py == 3'd3) ? s2 :
                             (py == 3'd4) ? s1 :
                             (py == 3'd5) ? s0 : 6'd0) : 6'd0;

    // ============================
    // collision helpers
    // ============================
    wire [5:0] ar_or;
    assign ar_or = ar0 | ar1 | ar2 | ar3 | ar4 | ar5;

    wire left_edge, right_edge;
    assign left_edge  = ((ar_or & 6'b000001) != 0);
    assign right_edge = ((ar_or & 6'b100000) != 0);

    wire collide_left, collide_right;
    assign collide_left =
        ((((ar0>>1)&b0)!=0) | (((ar1>>1)&b1)!=0) | (((ar2>>1)&b2)!=0) |
         (((ar3>>1)&b3)!=0) | (((ar4>>1)&b4)!=0) | (((ar5>>1)&b5)!=0));

    assign collide_right =
        ((((ar0<<1)&b0)!=0) | (((ar1<<1)&b1)!=0) | (((ar2<<1)&b2)!=0) |
         (((ar3<<1)&b3)!=0) | (((ar4<<1)&b4)!=0) | (((ar5<<1)&b5)!=0));

    wire can_left, can_right;
    assign can_left  = (~left_edge)  & (~collide_left);
    assign can_right = (~right_edge) & (~collide_right);

    wire hit_bottom;
    assign hit_bottom = (ar5 != 0);

    wire collide_down;
    assign collide_down =
        ((ar4 & b5)!=0) | ((ar3 & b4)!=0) | ((ar2 & b3)!=0) |
        ((ar1 & b2)!=0) | ((ar0 & b1)!=0);

    wire can_down;
    assign can_down = (~hit_bottom) & (~collide_down);

    // ============================
    // rotate candidate
    // ============================
    wire [1:0] prot_rot;
    assign prot_rot = prot + 2'd1;

    reg [3:0] rsh0, rsh1, rsh2, rsh3;
    reg [2:0] rsh_w, rsh_h;
    always @(*) begin
        rsh0 = 4'b0000; rsh1 = 4'b0000; rsh2 = 4'b0000; rsh3 = 4'b0000;
        rsh_w = 3'd0; rsh_h = 3'd0;
        if (ptype == 2'd0) begin
            if (prot_rot == 2'd0 || prot_rot == 2'd2) begin
                rsh0 = 4'b1111; rsh_w = 3'd4; rsh_h = 3'd1;
            end else begin
                rsh0 = 4'b0001; rsh1 = 4'b0001; rsh2 = 4'b0001; rsh3 = 4'b0001;
                rsh_w = 3'd1; rsh_h = 3'd4;
            end
        end else if (ptype == 2'd1) begin
            rsh0 = 4'b0011; rsh1 = 4'b0011;
            rsh_w = 3'd2; rsh_h = 3'd2;
        end else if (ptype == 2'd2) begin
            if (prot_rot == 2'd0) begin
                rsh0 = 4'b0111; rsh1 = 4'b0010; rsh_w = 3'd3; rsh_h = 3'd2;
            end else if (prot_rot == 2'd1) begin
                rsh0 = 4'b0010; rsh1 = 4'b0011; rsh2 = 4'b0010; rsh_w = 3'd2; rsh_h = 3'd3;
            end else if (prot_rot == 2'd2) begin
                rsh0 = 4'b0010; rsh1 = 4'b0111; rsh_w = 3'd3; rsh_h = 3'd2;
            end else begin
                rsh0 = 4'b0001; rsh1 = 4'b0011; rsh2 = 4'b0001; rsh_w = 3'd2; rsh_h = 3'd3;
            end
        end else begin
            if (prot_rot == 2'd0) begin
                rsh0 = 4'b0111; rsh1 = 4'b0100; rsh_w = 3'd3; rsh_h = 3'd2;
            end else if (prot_rot == 2'd1) begin
                rsh0 = 4'b0001; rsh1 = 4'b0001; rsh2 = 4'b0011; rsh_w = 3'd2; rsh_h = 3'd3;
            end else if (prot_rot == 2'd2) begin
                rsh0 = 4'b0001; rsh1 = 4'b0111; rsh_w = 3'd3; rsh_h = 3'd2;
            end else begin
                rsh0 = 4'b0011; rsh1 = 4'b0001; rsh2 = 4'b0001; rsh_w = 3'd2; rsh_h = 3'd3;
            end
        end
    end

    wire [2:0] px_rot;
    assign px_rot = (px > (3'd6 - rsh_w)) ? (3'd6 - rsh_w) : px;

    wire [5:0] rs0, rs1, rs2, rs3;
    assign rs0 = ({2'b00, rsh0} << px_rot);
    assign rs1 = ({2'b00, rsh1} << px_rot);
    assign rs2 = ({2'b00, rsh2} << px_rot);
    assign rs3 = ({2'b00, rsh3} << px_rot);

    wire [5:0] rr0, rr1, rr2, rr3, rr4, rr5;
    assign rr0 = (py == 3'd0) ? rs0 :
                 (py == 3'd1) ? 6'd0 :
                 (py == 3'd2) ? 6'd0 :
                 (py == 3'd3) ? 6'd0 :
                 (py == 3'd4) ? 6'd0 : 6'd0;
    assign rr1 = (py == 3'd0) ? rs1 :
                 (py == 3'd1) ? rs0 :
                 (py == 3'd2) ? 6'd0 :
                 (py == 3'd3) ? 6'd0 :
                 (py == 3'd4) ? 6'd0 : 6'd0;
    assign rr2 = (py == 3'd0) ? rs2 :
                 (py == 3'd1) ? rs1 :
                 (py == 3'd2) ? rs0 :
                 (py == 3'd3) ? 6'd0 :
                 (py == 3'd4) ? 6'd0 : 6'd0;
    assign rr3 = (py == 3'd0) ? rs3 :
                 (py == 3'd1) ? rs2 :
                 (py == 3'd2) ? rs1 :
                 (py == 3'd3) ? rs0 :
                 (py == 3'd4) ? 6'd0 : 6'd0;
    assign rr4 = (py == 3'd0) ? 6'd0 :
                 (py == 3'd1) ? rs3 :
                 (py == 3'd2) ? rs2 :
                 (py == 3'd3) ? rs1 :
                 (py == 3'd4) ? rs0 : 6'd0;
    assign rr5 = (py == 3'd0) ? 6'd0 :
                 (py == 3'd1) ? 6'd0 :
                 (py == 3'd2) ? rs3 :
                 (py == 3'd3) ? rs2 :
                 (py == 3'd4) ? rs1 :
                 (py == 3'd5) ? rs0 : 6'd0;

    wire rotate_in_bounds;
    assign rotate_in_bounds = (py + rsh_h) <= 3'd6;

    wire rotate_collide;
    assign rotate_collide =
        ((rr0 & b0)!=0) | ((rr1 & b1)!=0) | ((rr2 & b2)!=0) | ((rr3 & b3)!=0) | ((rr4 & b4)!=0) | ((rr5 & b5)!=0);

    wire can_rotate;
    assign can_rotate = active_en & rotate_in_bounds & (~rotate_collide);

    // ============================
    // spawn (fixed center)
    // ============================
    reg [2:0] spx;
    always @(*) begin
        if (next_type == 2'd0) spx = 3'd1;       // I (width 4)
        else if (next_type == 2'd1) spx = 3'd2;  // O (width 2)
        else spx = 3'd1;                         // T/L (width 3)
    end

    wire [5:0] sp0, sp1;
    assign sp0 = (next_type == 2'd0) ? (6'b001111 << spx) :
                 (next_type == 2'd1) ? (6'b000011 << spx) :
                 (next_type == 2'd2) ? (6'b000111 << spx) : (6'b000111 << spx);
    assign sp1 = (next_type == 2'd1) ? (6'b000011 << spx) :
                 (next_type == 2'd2) ? (6'b000010 << spx) :
                 (next_type == 2'd3) ? (6'b000100 << spx) : 6'd0;

    wire spawn_collide;
    assign spawn_collide = ((sp0 & b0)!=0) | ((sp1 & b1)!=0);

    // ============================
    // main sequential
    // ============================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            b0 <= 0; b1 <= 0; b2 <= 0; b3 <= 0; b4 <= 0; b5 <= 0;
            ptype <= 2'd0;
            prot <= 2'd0;
            px <= 3'd1;
            py <= 3'd0;
            next_type <= 2'd0;  // deterministic first spawn
            game_over <= 1'b0;
            state <= 2'd2;  // SPAWN
            scan <= 3'd5;
        end else if (!game_over) begin
            // CLEAR
            if (state == 2'd1) begin
                if (scan == 3'd5) begin
                    if (b5 == FULL) begin b5 <= b4; b4 <= b3; b3 <= b2; b2 <= b1; b1 <= b0; b0 <= 0; end
                    else scan <= 3'd4;
                end else if (scan == 3'd4) begin
                    if (b4 == FULL) begin b4 <= b3; b3 <= b2; b2 <= b1; b1 <= b0; b0 <= 0; end
                    else scan <= 3'd3;
                end else if (scan == 3'd3) begin
                    if (b3 == FULL) begin b3 <= b2; b2 <= b1; b1 <= b0; b0 <= 0; end
                    else scan <= 3'd2;
                end else if (scan == 3'd2) begin
                    if (b2 == FULL) begin b2 <= b1; b1 <= b0; b0 <= 0; end
                    else scan <= 3'd1;
                end else if (scan == 3'd1) begin
                    if (b1 == FULL) begin b1 <= b0; b0 <= 0; end
                    else scan <= 3'd0;
                end else begin
                    if (b0 == FULL) b0 <= 0;
                    state <= 2'd2;  // SPAWN
                end
            end

            // SPAWN
            else if (state == 2'd2) begin
                if (spawn_collide) begin
                    game_over <= 1'b1;
                end else begin
                    ptype <= next_type;
                    prot <= 2'd0;
                    px <= spx;
                    py <= 3'd0;
                    next_type <= rand2;
                    state <= 2'd0;  // PLAY
                end
            end

            // PLAY
            else begin
                if (rotate && can_rotate) begin
                    prot <= prot_rot;
                    px <= px_rot;
                end else if (left && can_left) begin
                    px <= px - 3'd1;
                end else if (right && can_right) begin
                    px <= px + 3'd1;
                end else if (can_down) begin
                    py <= py + 3'd1;
                end else begin
                    // lock
                    b0 <= b0 | ar0;
                    b1 <= b1 | ar1;
                    b2 <= b2 | ar2;
                    b3 <= b3 | ar3;
                    b4 <= b4 | ar4;
                    b5 <= b5 | ar5;
                    state <= 2'd1;
                    scan <= 3'd5;
                end
            end
        end
    end

    // ============================
    // output
    // ============================
    assign row0 = game_over ? 6'b100001 : (b0 | ar0);
    assign row1 = game_over ? 6'b010010 : (b1 | ar1);
    assign row2 = game_over ? 6'b001100 : (b2 | ar2);
    assign row3 = game_over ? 6'b001100 : (b3 | ar3);
    assign row4 = game_over ? 6'b010010 : (b4 | ar4);
    assign row5 = game_over ? 6'b100001 : (b5 | ar5);

endmodule
