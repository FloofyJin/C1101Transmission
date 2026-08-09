`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// tb_rx_points  --  milestone-14 testbench: point packets into PointRam
//
// Injects frames into radio B's model and checks that RxSeq parses them and
// commits the right coordinates to the right PointRam addresses.
//
// The cases that matter are the rejections, not the happy path. A receiver that
// accepts everything looks identical to a correct one until the link degrades:
//   1. a small packet lands at the right indices          (basic parse)
//   2. a FULL 30-point / 65-byte packet                   (the burst-read limit)
//   3. CRC bad          -> nothing is written             (staging works)
//   4. CRC good, length inconsistent -> bad_fmt           (sender bug, not RF)
//   5. start_index + count past the end -> rejected       (no RAM overrun)
//   6. a second packet at a different index leaves the first intact
//
// Case 3 is the reason RxSeq stages the payload instead of writing it straight
// through: CRC_OK is the LAST byte of the burst, so by the time it is known,
// every coordinate has already gone past.
//------------------------------------------------------------------------------
module tb_rx_points;

    logic sysclk = 0;
    always #4 sysclk = ~sysclk;      // 125 MHz

    logic rst = 1, tx_btn = 0, cfg_btn = 0, sw_auto = 0;

    wire a_sclk, a_mosi, a_csn, a_miso, a_gdo0;
    wire b_sclk, b_mosi, b_csn, b_miso, b_gdo0;
    wire led_cfg, led_tx, led_err, led_rx;

    topRF #(.CLK_HZ(125_000_000), .POWERUP_US(1)) dut (
        .sysclk(sysclk), .rst(rst),
        .sw_sender_enable(1'b0), .sw_receiver_enable(1'b1),
        .tx_btn(tx_btn), .cfg_btn(cfg_btn), .sw_auto(sw_auto),
        .cc_sclk(a_sclk), .cc_mosi(a_mosi), .cc_miso(a_miso),
        .cc_csn(a_csn),   .cc_gdo2(a_gdo0),
        .cc_b_sclk(b_sclk), .cc_b_mosi(b_mosi), .cc_b_miso(b_miso),
        .cc_b_csn(b_csn),   .cc_b_gdo2(b_gdo0),
        .led_cfg(led_cfg), .led_tx(led_tx), .led_err(led_err), .led_rx(led_rx)
    );

    cc1101_model radio_a (
        .cs_n(a_csn), .sclk(a_sclk), .mosi(a_mosi), .miso(a_miso), .gdo0(a_gdo0)
    );
    cc1101_model radio_b (
        .cs_n(b_csn), .sclk(b_sclk), .mosi(b_mosi), .miso(b_miso), .gdo0(b_gdo0)
    );

    integer errors = 0;

    task check(input [255:0] what, input integer got, input integer exp);
        begin
            if (got !== exp) begin
                $display("  FAIL  %0s: got 0x%0h, expected 0x%0h", what, got, exp);
                errors = errors + 1;
            end else begin
                $display("  ok    %0s = 0x%0h", what, got);
            end
        end
    endtask

    // PointRam entry is {x[7:0], y[7:0], dwell[1:0]}.
    task check_point(input [7:0] idx, input [7:0] ex, input [7:0] ey);
        reg [17:0] e;
        begin
            e = {ex, ey, 2'b11};        // DWELL_DEFAULT
            if (dut.pram.mem[idx] !== e) begin
                $display("  FAIL  PointRam[%0d]: got %04h, expected %04h (x=%0d y=%0d)",
                         idx, dut.pram.mem[idx], e, ex, ey);
                errors = errors + 1;
            end else begin
                $display("  ok    PointRam[%0d] = x %0d, y %0d", idx, ex, ey);
            end
        end
    endtask

    // RxSeq drops busy only in R_LISTEN, so this is a precise "armed and
    // waiting" test -- no guessed delays.
    task wait_listening;
        begin
            wait (dut.b_config_ok);
            @(posedge sysclk);
            wait (dut.b_rx_busy == 1'b1);   // arming: SIDLE/SFRX/SRX
            wait (dut.b_rx_busy == 1'b0);   // listening
        end
    endtask

    // Build [len][start_index][count][x0][y0]...[RSSI][LQI|CRC_OK<<7] into the
    // model's RX FIFO and pulse GDO2. Coordinates are generated as
    // x = xbase + i, y = ybase + i so every point is distinguishable -- a
    // constant payload would hide an off-by-one in the index arithmetic.
    task send_points(input [7:0] start_index, input [7:0] count,
                     input [7:0] xbase, input [7:0] ybase, input crc_good);
        integer i;
        reg [7:0] n;
        begin
            radio_b.rx_fifo[0] = 8'd2 + count*2;      // len
            radio_b.rx_fifo[1] = start_index;
            radio_b.rx_fifo[2] = count;
            for (i = 0; i < count; i = i + 1) begin
                radio_b.rx_fifo[3 + i*2] = xbase + i[7:0];
                radio_b.rx_fifo[4 + i*2] = ybase + i[7:0];
            end
            n = 8'd3 + count*2;                        // len + payload
            radio_b.rx_fifo[n]     = 8'h50;            // RSSI
            radio_b.rx_fifo[n + 1] = crc_good ? 8'h92 : 8'h12;  // CRC_OK<<7 | LQI
            radio_b.rx_deliver(n + 2);
        end
    endtask

    // Same, but with a deliberately wrong length byte.
    task send_bad_len(input [7:0] start_index, input [7:0] count);
        integer i;
        reg [7:0] n;
        begin
            radio_b.rx_fifo[0] = 8'd2 + count*2 + 8'd4;   // inconsistent
            radio_b.rx_fifo[1] = start_index;
            radio_b.rx_fifo[2] = count;
            for (i = 0; i < count; i = i + 1) begin
                radio_b.rx_fifo[3 + i*2] = 8'hEE;
                radio_b.rx_fifo[4 + i*2] = 8'hEE;
            end
            n = 8'd3 + count*2;
            radio_b.rx_fifo[n]     = 8'h50;
            radio_b.rx_fifo[n + 1] = 8'h92;               // CRC good
            radio_b.rx_deliver(n + 2);
        end
    endtask

    integer k;

    initial begin
        $display("\n=== milestone 14: point packets into PointRam ===\n");

        repeat (10) @(posedge sysclk);
        rst = 0;

        wait_listening();
        $display("--- radio B configured and listening ---");
        check("b_config_ok", dut.b_config_ok, 1);

        // ---------- 1. a small packet ----------
        $display("\n=== 1. three points at index 100 ===");
        send_points(8'd100, 8'd3, 8'd100, 8'd200, 1);
        wait (dut.b_rx_done);
        @(posedge sysclk);

        check("crc_ok",      dut.b_crc_ok,      1);
        check("fmt_ok",      dut.b_fmt_ok,      1);
        check("pkt_ok",      dut.b_pkt_ok,      1);
        check("len_byte",    dut.b_len_byte,    8'd8);
        check("start_index", dut.b_start_index, 8'd100);
        check("count",       dut.b_count,       8'd3);
        check("rxbytes",     dut.b_rxbytes,     8'd11);
        check("pt_writes",   dut.b_pt_writes,   16'd3);
        check("rx_count",    dut.b_rx_count,    16'd1);
        check("led_rx",      led_rx,            1);

        check_point(8'd100, 8'd100, 8'd200);
        check_point(8'd101, 8'd101, 8'd201);
        check_point(8'd102, 8'd102, 8'd202);

        // ---------- 2. a full 30-point packet ----------
        // 1 len + 62 payload + 2 status = 65 bytes, the RX_MAX case. This is the
        // only test that exercises the burst read at full length, which is where
        // an off-by-one in rx_idx would hide.
        $display("\n=== 2. full 30-point packet at index 0 ===");
        wait_listening();
        send_points(8'd0, 8'd30, 8'd1, 8'd51, 1);
        wait (dut.b_rx_done);
        @(posedge sysclk);

        check("pkt_ok",    dut.b_pkt_ok,    1);
        check("len_byte",  dut.b_len_byte,  8'd62);
        check("count",     dut.b_count,     8'd30);
        check("rxbytes",   dut.b_rxbytes,   8'd65);
        check("pt_writes", dut.b_pt_writes, 16'd33);   // 3 + 30

        check_point(8'd0,  8'd1,  8'd51);    // first
        check_point(8'd15, 8'd16, 8'd66);    // middle
        check_point(8'd29, 8'd30, 8'd80);    // last -- the off-by-one canary

        // the earlier packet is untouched
        check_point(8'd100, 8'd100, 8'd200);

        // ---------- 3. a CRC failure must write nothing ----------
        $display("\n=== 3. CRC bad -> PointRam unchanged ===");
        wait_listening();
        send_points(8'd100, 8'd3, 8'd7, 8'd7, 0);  // would overwrite 100-102
        wait (dut.b_rx_done);
        @(posedge sysclk);

        check("crc_ok",    dut.b_crc_ok,    0);
        check("pkt_ok",    dut.b_pkt_ok,    0);
        check("bad_fmt",   dut.b_bad_fmt,   0);    // format was fine; CRC was not
        check("pt_writes", dut.b_pt_writes, 16'd33);   // unchanged
        check_point(8'd100, 8'd100, 8'd200);            // original survives
        check_point(8'd101, 8'd101, 8'd201);

        // ---------- 4. CRC good but length inconsistent ----------
        $display("\n=== 4. length != 2 + 2*count -> bad_fmt ===");
        wait_listening();
        send_bad_len(8'd100, 8'd3);
        wait (dut.b_rx_done);
        @(posedge sysclk);

        check("crc_ok",    dut.b_crc_ok,    1);    // RF was fine
        check("fmt_ok",    dut.b_fmt_ok,    0);    // the SENDER is wrong
        check("pkt_ok",    dut.b_pkt_ok,    0);
        check("bad_fmt",   dut.b_bad_fmt,   1);
        check("pt_writes", dut.b_pt_writes, 16'd33);
        check_point(8'd100, 8'd100, 8'd200);

        // ---------- 5. start_index + count past the end ----------
        $display("\n=== 5. index range past PointRam -> rejected ===");
        wait_listening();
        send_points(8'd250, 8'd30, 8'd9, 8'd9, 1);   // 250 + 30 = 280 > 255
        wait (dut.b_rx_done);
        @(posedge sysclk);

        check("crc_ok",    dut.b_crc_ok,    1);
        check("fmt_ok",    dut.b_fmt_ok,    0);
        check("pkt_ok",    dut.b_pkt_ok,    0);
        check("pt_writes", dut.b_pt_writes, 16'd33);

        // ---------- 6. a packet at the very top still fits ----------
        $display("\n=== 6. index 225 + 30 = 255, the exact boundary ===");
        wait_listening();
        send_points(8'd225, 8'd30, 8'd3, 8'd4, 1);
        wait (dut.b_rx_done);
        @(posedge sysclk);

        check("pkt_ok",    dut.b_pkt_ok,    1);
        check("pt_writes", dut.b_pt_writes, 16'd63);   // 33 + 30
        check_point(8'd225, 8'd3,  8'd4);
        check_point(8'd254, 8'd32, 8'd33);             // last legal entry

        $display("\n=====================================");
        if (errors == 0)
            $display("PASS -- point packets parse and land correctly");
        else
            $display("FAIL -- %0d check(s) failed", errors);
        $display("=====================================\n");

        $finish;
    end

    // Global watchdog: a wedged FSM must not hang the sim forever.
    initial begin
        #50_000_000;
        $display("\nFAIL -- testbench timed out (FSM wedged?)");
        $finish;
    end

endmodule
