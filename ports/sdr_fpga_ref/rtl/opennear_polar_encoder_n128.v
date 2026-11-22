// ============================================================================
// OpenNear v3.2 Route B — Synthesizable L1 CA-Polar (N=128, K=48) Hardware Encoder
//
// Features:
//   - Matches core/src/on_polar.c bit-for-bit (verified by TEST 16 golden vectors)
//   - Computes 8-bit CRC-8 (poly 0x07) over 40-bit control message -> 48 info bits
//   - Maps 48 info bits onto top-48 5G NR beta-expansion reliable bit channels
//   - Executes 7-stage combinational Arikan butterfly transform x = u * G_128
// ============================================================================
`timescale 1ns / 1ps

module opennear_polar_encoder_n128 (
    input  wire [39:0] msg_in,       // 5-byte control message (MSB first: msg[0]..msg[4])
    output wire [7:0]  crc8_out,     // Computed CRC-8 (poly 0x07)
    output wire [127:0] coded_out    // 128-bit Polar codeword (bit 0 = index 0)
);

    // 1. Combinational CRC-8 (poly 0x07, init 0x00) over 40 message bits
    function automatic [7:0] calc_crc8(input [39:0] d);
        reg [7:0] c;
        integer i;
        begin
            c = 8'h00;
            for (i = 39; i >= 0; i = i - 1) begin
                if (c[7] ^ d[i])
                    c = {c[6:0], 1'b0} ^ 8'h07;
                else
                    c = {c[6:0], 1'b0};
            end
            calc_crc8 = c;
        end
    endfunction

    assign crc8_out = calc_crc8(msg_in);
    wire [47:0] info_bits = {msg_in, crc8_out};

    // 2. Map 48 info bits (MSB first: info_bits[47] -> k=0 .. info_bits[0] -> k=47)
    //    onto the exact 48 most-reliable 5G NR beta-expansion channels in natural order:
    reg [127:0] u;
    always @(*) begin
        u = 128'd0;
        u[31]  = info_bits[47]; u[47]  = info_bits[46]; u[55]  = info_bits[45]; u[58]  = info_bits[44];
        u[59]  = info_bits[43]; u[60]  = info_bits[42]; u[61]  = info_bits[41]; u[62]  = info_bits[40];
        u[63]  = info_bits[39]; u[78]  = info_bits[38]; u[79]  = info_bits[37]; u[85]  = info_bits[36];
        u[86]  = info_bits[35]; u[87]  = info_bits[34]; u[89]  = info_bits[33]; u[90]  = info_bits[32];
        u[91]  = info_bits[31]; u[92]  = info_bits[30]; u[93]  = info_bits[29]; u[94]  = info_bits[28];
        u[95]  = info_bits[27]; u[99]  = info_bits[26]; u[101] = info_bits[25]; u[102] = info_bits[24];
        u[103] = info_bits[23]; u[105] = info_bits[22]; u[106] = info_bits[21]; u[107] = info_bits[20];
        u[108] = info_bits[19]; u[109] = info_bits[18]; u[110] = info_bits[17]; u[111] = info_bits[16];
        u[112] = info_bits[15]; u[113] = info_bits[14]; u[114] = info_bits[13]; u[115] = info_bits[12];
        u[116] = info_bits[11]; u[117] = info_bits[10]; u[118] = info_bits[9];  u[119] = info_bits[8];
        u[120] = info_bits[7];  u[121] = info_bits[6];  u[122] = info_bits[5];  u[123] = info_bits[4];
        u[124] = info_bits[3];  u[125] = info_bits[2];  u[126] = info_bits[1];  u[127] = info_bits[0];
    end

    // 3. 7-stage combinational Arikan butterfly network x = u * G_128
    reg [127:0] x;
    integer step, base, j;
    always @(*) begin
        x = u;
        for (step = 1; step < 128; step = step << 1) begin
            for (base = 0; base < 128; base = base + (step << 1)) begin
                for (j = 0; j < step; j = j + 1) begin
                    x[base + j] = x[base + j] ^ x[base + j + step];
                end
            end
        end
    end

    assign coded_out = x;

endmodule
