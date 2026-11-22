// ============================================================================
// OpenNear v4.0 Route B — Synthesizable L1 Fixed-Point Q4.4 Polar Decoder (N=128, K=48)
//
// Features:
//   - Zero-multiplier, fixed-point Q4.4 (10-bit signed [-512..+511]) min-sum arithmetic
//   - Compact 255-node flat binary tree (STAGE_OFF(st) = (1<<st)-1, 75% RAM savings)
//   - Bit-exact match with core/src/on_polar.c 3GPP TS 38.212 48-channel mask
//   - Integrated hardware CRC-8 (poly 0x07) integrity check over decoded 40-bit message
// ============================================================================
`timescale 1ns / 1ps

module opennear_polar_decoder_n128 (
    input  wire          clk,
    input  wire          rst_n,
    input  wire          start,
    input  wire [1279:0] llr_in_flat,  // 128 x 10-bit signed Q4.4 channel LLRs
    output reg           done,
    output reg           crc_ok,
    output reg  [39:0]   msg_out,
    output reg  [7:0]    crc_rx_out
);

    // 128-bit mask of active 5G NR info bit-channels (K=48, natural order)
    localparam [127:0] INFO_MASK =
        (128'd1 << 31)  | (128'd1 << 47)  | (128'd1 << 55)  | (128'd1 << 58)  |
        (128'd1 << 59)  | (128'd1 << 60)  | (128'd1 << 61)  | (128'd1 << 62)  |
        (128'd1 << 63)  | (128'd1 << 78)  | (128'd1 << 79)  | (128'd1 << 85)  |
        (128'd1 << 86)  | (128'd1 << 87)  | (128'd1 << 89)  | (128'd1 << 90)  |
        (128'd1 << 91)  | (128'd1 << 92)  | (128'd1 << 93)  | (128'd1 << 94)  |
        (128'd1 << 95)  | (128'd1 << 99)  | (128'd1 << 101) | (128'd1 << 102) |
        (128'd1 << 103) | (128'd1 << 105) | (128'd1 << 106) | (128'd1 << 107) |
        (128'd1 << 108) | (128'd1 << 109) | (128'd1 << 110) | (128'd1 << 111) |
        (128'd1 << 112) | (128'd1 << 113) | (128'd1 << 114) | (128'd1 << 115) |
        (128'd1 << 116) | (128'd1 << 117) | (128'd1 << 118) | (128'd1 << 119) |
        (128'd1 << 120) | (128'd1 << 121) | (128'd1 << 122) | (128'd1 << 123) |
        (128'd1 << 124) | (128'd1 << 125) | (128'd1 << 126) | (128'd1 << 127);

    // Compact 255-node flat binary tree (stages 0..7)
    reg signed [9:0] llr_tree [0:254];
    reg              beta_l   [0:254];
    reg              beta_r   [0:254];
    reg              u_bits   [0:127];

    // Q4.4 Saturation helper [-512, +511]
    function automatic signed [9:0] sat_q4(input signed [11:0] v);
        begin
            if (v > 12'sd511)       sat_q4 = 10'sd511;
            else if (v < -12'sd512) sat_q4 = -10'sd512;
            else                    sat_q4 = v[9:0];
        end
    endfunction

    // Hardware min-sum f(a, b) = sign(a)*sign(b)*min(|a|, |b|)
    function automatic signed [9:0] llr_f_q4(input signed [9:0] a, input signed [9:0] b);
        reg signed [11:0] aa, ab, m;
        begin
            aa = (a < 0) ? -a : a;
            ab = (b < 0) ? -b : b;
            m  = (aa < ab) ? aa : ab;
            llr_f_q4 = ((a < 0) ^ (b < 0)) ? sat_q4(-m) : sat_q4(m);
        end
    endfunction

    // Hardware g(a, b, u_left) = u_left ? (b - a) : (b + a)
    function automatic signed [9:0] llr_g_q4(
        input signed [9:0] a,
        input signed [9:0] b,
        input              u_left
    );
        reg signed [11:0] res;
        begin
            res = u_left ? ($signed({{2{b[9]}}, b}) - $signed({{2{a[9]}}, a}))
                         : ($signed({{2{b[9]}}, b}) + $signed({{2{a[9]}}, a}));
            llr_g_q4 = sat_q4(res);
        end
    endfunction

    // CRC-8 (poly 0x07) verification over 40 decoded message bits
    function automatic [7:0] calc_crc8(input [39:0] d);
        reg [7:0] c;
        integer k;
        begin
            c = 8'h00;
            for (k = 39; k >= 0; k = k - 1) begin
                if (c[7] ^ d[k])
                    c = {c[6:0], 1'b0} ^ 8'h07;
                else
                    c = {c[6:0], 1'b0};
            end
            calc_crc8 = c;
        end
    endfunction

    localparam [1:0] S_IDLE   = 2'd0,
                     S_DECODE = 2'd1,
                     S_PACK   = 2'd2;

    reg [1:0] state;
    reg [7:0] phi;

    integer i, s, j, st, half, off_s, off_p, cur_phi;
    reg u_hat;
    reg next_is_right;
    reg [47:0] extracted_info;
    integer bit_pos;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            phi        <= 8'd0;
            done       <= 1'b0;
            crc_ok     <= 1'b0;
            msg_out    <= 40'd0;
            crc_rx_out <= 8'd0;
        end else begin
            done <= 1'b0;
            case (state)
                S_IDLE: begin
                    if (start) begin
                        for (i = 0; i < 128; i = i + 1) begin
                            llr_tree[127 + i] <= llr_in_flat[i*10 +: 10];
                            u_bits[i]         <= 1'b0;
                        end
                        phi   <= 8'd0;
                        state <= S_DECODE;
                    end
                end

                S_DECODE: begin
                    // 1. Top-down LLR tree update for bit index phi
                    st = 0;
                    while (st < 7 && (((phi >> st) & 1) == 0)) begin
                        st = st + 1;
                    end
                    for (s = st; s >= 1; s = s - 1) begin
                        half  = (1 << (s - 1));
                        off_s = (1 << s) - 1;
                        off_p = (1 << (s - 1)) - 1;
                        if (((phi >> (s - 1)) & 1) == 0) begin
                            for (j = 0; j < half; j = j + 1) begin
                                llr_tree[off_p + j] = llr_f_q4(
                                    llr_tree[off_s + j],
                                    llr_tree[off_s + j + half]
                                );
                            end
                        end else begin
                            for (j = 0; j < half; j = j + 1) begin
                                llr_tree[off_p + j] = llr_g_q4(
                                    llr_tree[off_s + j],
                                    llr_tree[off_s + j + half],
                                    beta_l[off_p + j]
                                );
                            end
                        end
                    end

                    // 2. Leaf decision at stage 0 (index 0)
                    if (INFO_MASK[phi])
                        u_hat = (llr_tree[0] < 0) ? 1'b1 : 1'b0;
                    else
                        u_hat = 1'b0;

                    u_bits[phi] = u_hat;

                    // 3. Bottom-up partial-sum (beta) butterfly propagation
                    if ((phi & 1) == 0) begin
                        beta_l[0] = u_hat;
                    end else begin
                        beta_r[0] = u_hat;
                        s = 0;
                        cur_phi = phi;
                        while (s < 7 && ((cur_phi & 1) == 1)) begin
                            half  = (1 << s);
                            off_p = (1 << s) - 1;
                            off_s = (1 << (s + 1)) - 1;
                            next_is_right = (((cur_phi >> 1) & 1) != 0);
                            for (j = 0; j < half; j = j + 1) begin
                                if (next_is_right) begin
                                    beta_r[off_s + j]        = beta_l[off_p + j] ^ beta_r[off_p + j];
                                    beta_r[off_s + j + half] = beta_r[off_p + j];
                                end else begin
                                    beta_l[off_s + j]        = beta_l[off_p + j] ^ beta_r[off_p + j];
                                    beta_l[off_s + j + half] = beta_r[off_p + j];
                                end
                            end
                            cur_phi = cur_phi >> 1;
                            s = s + 1;
                        end
                    end

                    if (phi == 8'd127)
                        state <= S_PACK;
                    else
                        phi <= phi + 8'd1;
                end

                S_PACK: begin
                    extracted_info = 48'd0;
                    bit_pos = 47;
                    for (i = 0; i < 128; i = i + 1) begin
                        if (INFO_MASK[i]) begin
                            extracted_info[bit_pos] = u_bits[i];
                            bit_pos = bit_pos - 1;
                        end
                    end
                    msg_out    <= extracted_info[47:8];
                    crc_rx_out <= extracted_info[7:0];
                    crc_ok     <= (calc_crc8(extracted_info[47:8]) == extracted_info[7:0]);
                    done       <= 1'b1;
                    state      <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
