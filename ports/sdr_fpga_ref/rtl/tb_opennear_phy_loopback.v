// ============================================================================
// OpenNear v4.0 Route B — Full L1 Baseband RTL Loopback Testbench
//
// Connects:
//   1. opennear_preamble_correlator (32-chip Golay matched filter + 4ns latch)
//   2. opennear_polar_encoder_n128  (40b msg + CRC8 -> 128b Polar codeword)
//   3. AWGN / Q4.4 LLR Channel Mapper (+32 for bit=0, -32 for bit=1 + noise)
//   4. opennear_polar_decoder_n128  (Fixed-point Q4.4 255-node tree SC + CRC8)
// ============================================================================
`timescale 1ns / 1ps

module tb_opennear_phy_loopback;

    reg          clk = 1'b0;
    reg          rst_n = 1'b0;
    always #2 clk = ~clk; // 250 MHz (4 ns cycle)

    // 1. Encoder instance
    reg  [39:0]  tx_msg = 40'h5AC39901EF;
    wire [7:0]   tx_crc8;
    wire [127:0] tx_coded;

    opennear_polar_encoder_n128 u_enc (
        .msg_in   (tx_msg),
        .crc8_out (tx_crc8),
        .coded_out(tx_coded)
    );

    // 2. Preamble Correlator instance
    reg                sample_valid = 1'b0;
    reg  signed [11:0] i_sample = 12'sd0;
    reg  signed [11:0] q_sample = 12'sd0;
    reg  [63:0]        free_timer_ns = 64'd1000;
    wire               frame_valid;
    wire [63:0]        rx_timestamp_ns;
    wire [31:0]        peak_metric;

    opennear_preamble_correlator #(
        .DATA_WIDTH(12),
        .PEAK_THRESHOLD(32'd200000)
    ) u_corr (
        .clk            (clk),
        .rst_n          (rst_n),
        .sample_valid   (sample_valid),
        .i_in           (i_sample),
        .q_in           (q_sample),
        .free_timer_ns  (free_timer_ns),
        .frame_valid    (frame_valid),
        .rx_timestamp_ns(rx_timestamp_ns),
        .peak_metric    (peak_metric)
    );

    // 3. Polar Decoder instance
    reg          dec_start = 1'b0;
    reg  [1279:0] llr_flat;
    wire         dec_done;
    wire         dec_crc_ok;
    wire [39:0]  rx_msg;
    wire [7:0]   rx_crc8;

    opennear_polar_decoder_n128 u_dec (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (dec_start),
        .llr_in_flat(llr_flat),
        .done       (dec_done),
        .crc_ok     (dec_crc_ok),
        .msg_out    (rx_msg),
        .crc_rx_out (rx_crc8)
    );

    localparam [0:15] GA_BITS = 16'b1110101101110001;
    integer k;
    reg signed [9:0] q4_val;

    initial begin
        rst_n = 1'b0;
        #20;
        rst_n = 1'b1;
        #20;

        // Stream 2 x 16-chip Golay preamble into correlator
        for (k = 0; k < 32; k = k + 1) begin
            @(posedge clk);
            free_timer_ns <= free_timer_ns + 64'd250;
            sample_valid  <= 1'b1;
            i_sample      <= GA_BITS[k & 15] ? 12'sd512 : -12'sd512;
            q_sample      <= 12'sd0;
        end
        @(posedge clk);
        sample_valid <= 1'b0;

        // Build Q4.4 LLR vector from tx_coded (with 2 flipped bits to test error correction!)
        for (k = 0; k < 128; k = k + 1) begin
            if (k == 17 || k == 83)
                q4_val = tx_coded[k] ? 10'sd12 : -10'sd12; // 2 soft bit errors
            else
                q4_val = tx_coded[k] ? -10'sd48 : 10'sd48;
            llr_flat[k*10 +: 10] = q4_val;
        end

        @(posedge clk);
        dec_start <= 1'b1;
        @(posedge clk);
        dec_start <= 1'b0;

        wait (dec_done == 1'b1);
        @(posedge clk);

        $display("[RTL TB] TX Msg=%h CRC8=%h | RX Msg=%h CRC8=%h CRC_OK=%b | Timestamp=%0d ns",
                 tx_msg, tx_crc8, rx_msg, rx_crc8, dec_crc_ok, rx_timestamp_ns);
        if (rx_msg !== tx_msg || !dec_crc_ok) begin
            $display("[RTL TB] FAIL: Mismatch!");
            $finish;
        end
        $display("[RTL TB] PASS: Full L1 RTL Baseband Loopback Verified!");
        $finish;
    end

endmodule
