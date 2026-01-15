module rng4_pl(
    input  wire       clk,
    input  wire       rst_n,
    output reg  [3:0] rnd
);
    // 4-bit LFSR-like generator (PL-style): feedback = q3 XOR q2.
    // Resets to a non-zero value to avoid lock-up.
    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            rnd <= 4'hA;
        end else begin
            rnd <= {rnd[2:0], (rnd[3] ^ rnd[2])};
        end
    end
endmodule
