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

    // PointRam entry is {x[7:0], y[7:0], blank, reserved}.
    //
    // Reads the bank RxSeq is WRITING (not the one scanout reads) -- these
    // tests are about the receive path, and the display side is exercised by
    // tb_scanout. The bank offset matters: bank 0 holds the seeded triangle,
    // so reading it would compare against the wrong data entirely.
    localparam int PT_N = 1024;
    task check_point(input [10:0] idx, input [7:0] ex, input [7:0] ey);
        reg [17:0] e;
        int a;
        begin
            a = (dut.rx_b.wr_bank ? PT_N : 0) + idx;
            e = {ex, ey, 2'b00};        // BLANK_DEFAULT
            if (dut.pram.mem[a] !== e) begin
                $display("  FAIL  PointRam[bank%0d][%0d]: got %05h, expected %05h (x=%0d y=%0d)",
                         dut.rx_b.wr_bank, idx, dut.pram.mem[a], e, ex, ey);
                errors = errors + 1;
            end else begin
                $display("  ok    PointRam[bank%0d][%0d] = x %0d, y %0d",
                         dut.rx_b.wr_bank, idx, ex, ey);
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
    // payload = idx_hi, idx_lo, count, then 2 B per point.
    // count[7] set on the last packet of a frame.
    task send_points(input [15:0] start_index, input [7:0] count,
                     input [7:0] xbase, input [7:0] ybase,
                     input crc_good, input eof);
        integer i;
        reg [7:0] n;
        begin
            radio_b.rx_fifo[0] = 8'd3 + count*2;             // len
            radio_b.rx_fifo[1] = start_index[15:8];
            radio_b.rx_fifo[2] = start_index[7:0];
            radio_b.rx_fifo[3] = count | (eof ? 8'h80 : 8'h00);
            for (i = 0; i < count; i = i + 1) begin
                radio_b.rx_fifo[4 + i*2] = xbase + i[7:0];
                radio_b.rx_fifo[5 + i*2] = ybase + i[7:0];
            end
            n = 8'd4 + count*2;                              // len + payload
            radio_b.rx_fifo[n]     = 8'h50;                  // RSSI
            radio_b.rx_fifo[n + 1] = crc_good ? 8'h92 : 8'h12;
            radio_b.rx_deliver(n + 2);
        end
    endtask

    // Same, but with a deliberately wrong length byte.
    task send_bad_len(input [15:0] start_index, input [7:0] count);
        integer i;
        reg [7:0] n;
        begin
            radio_b.rx_fifo[0] = 8'd3 + count*2 + 8'd4;   // inconsistent
            radio_b.rx_fifo[1] = start_index[15:8];
            radio_b.rx_fifo[2] = start_index[7:0];
            radio_b.rx_fifo[3] = count;
            for (i = 0; i < count; i = i + 1) begin
                radio_b.rx_fifo[4 + i*2] = 8'hEE;
                radio_b.rx_fifo[5 + i*2] = 8'hEE;
            end
            n = 8'd4 + count*2;
            radio_b.rx_fifo[n]     = 8'h50;
            radio_b.rx_fifo[n + 1] = 8'h92;               // CRC good
            radio_b.rx_deliver(n + 2);
        end
    endtask

    integer k;
    bit bank_before;

    initial begin
        $display("\n=== milestone 14: point packets into PointRam ===\n");

        repeat (10) @(posedge sysclk);
        rst = 0;

        wait_listening();
        $display("--- radio B configured and listening ---");
        check("b_config_ok", dut.b_config_ok, 1);

        // ---------- 1. a small packet ----------
        $display("\n=== 1. three points at index 100 ===");
        send_points(16'd100, 8'd4, 8'd100, 8'd200, 1, 0);
        wait (dut.b_rx_done);
        @(posedge sysclk);

        check("crc_ok",      dut.b_crc_ok,      1);
        check("fmt_ok",      dut.b_fmt_ok,      1);
        check("pkt_ok",      dut.b_pkt_ok,      1);
        check("len_byte",    dut.b_len_byte,    8'd11);
        check("start_index", dut.b_start_index, 16'd100);
        check("count",       dut.b_count,       8'd4);
        check("rxbytes",     dut.b_rxbytes,     8'd14);
        check("pt_writes",   dut.b_pt_writes,   16'd4);
        check("rx_count",    dut.b_rx_count,    16'd1);
        check("led_rx",      led_rx,            1);

        check_point(8'd100, 8'd100, 8'd200);
        check_point(8'd101, 8'd101, 8'd201);
        check_point(8'd102, 8'd102, 8'd202);
        check_point(8'd103, 8'd103, 8'd203);

        // ---------- 2. a full 30-point packet ----------
        // 1 len + 62 payload + 2 status = 65 bytes, the RX_MAX case. This is the
        // only test that exercises the burst read at full length, which is where
        // an off-by-one in rx_idx would hide.
        $display("\n=== 2. full 30-point packet at index 0 ===");
        wait_listening();
        send_points(16'd0, 8'd28, 8'd1, 8'd51, 1, 0);
        wait (dut.b_rx_done);
        @(posedge sysclk);

        check("pkt_ok",    dut.b_pkt_ok,    1);
        check("len_byte",  dut.b_len_byte,  8'd59);
        check("count",     dut.b_count,     8'd28);
        check("rxbytes",   dut.b_rxbytes,   8'd62);
        check("pt_writes", dut.b_pt_writes, 16'd32);   // 4 + 28

        check_point(8'd0,  8'd1,  8'd51);    // first
        check_point(8'd15, 8'd16, 8'd66);    // middle
        check_point(8'd27, 8'd28, 8'd78);    // last -- the off-by-one canary

        // the earlier packet is untouched
        check_point(8'd100, 8'd100, 8'd200);

        // ---------- 3. a CRC failure must write nothing ----------
        $display("\n=== 3. CRC bad -> PointRam unchanged ===");
        wait_listening();
        send_points(16'd100, 8'd4, 8'd7, 8'd7, 0, 0);  // would overwrite 100-103
        wait (dut.b_rx_done);
        @(posedge sysclk);

        check("crc_ok",    dut.b_crc_ok,    0);
        check("pkt_ok",    dut.b_pkt_ok,    0);
        check("bad_fmt",   dut.b_bad_fmt,   0);    // format was fine; CRC was not
        check("pt_writes", dut.b_pt_writes, 16'd32);   // unchanged
        check_point(8'd100, 8'd100, 8'd200);            // original survives
        check_point(8'd101, 8'd101, 8'd201);

        // ---------- 4. CRC good but length inconsistent ----------
        $display("\n=== 4. length != 2 + 2*count -> bad_fmt ===");
        wait_listening();
        send_bad_len(16'd100, 8'd4);
        wait (dut.b_rx_done);
        @(posedge sysclk);

        check("crc_ok",    dut.b_crc_ok,    1);    // RF was fine
        check("fmt_ok",    dut.b_fmt_ok,    0);    // the SENDER is wrong
        check("pkt_ok",    dut.b_pkt_ok,    0);
        check("bad_fmt",   dut.b_bad_fmt,   1);
        check("pt_writes", dut.b_pt_writes, 16'd32);
        check_point(8'd100, 8'd100, 8'd200);

        // ---------- 5. start_index + count past the end ----------
        $display("\n=== 5. index range past PointRam -> rejected ===");
        wait_listening();
        send_points(16'd1020, 8'd28, 8'd9, 8'd9, 1, 0);  // 1020 + 28 > 1024
        wait (dut.b_rx_done);
        @(posedge sysclk);

        check("crc_ok",    dut.b_crc_ok,    1);
        check("fmt_ok",    dut.b_fmt_ok,    0);
        check("pkt_ok",    dut.b_pkt_ok,    0);
        check("pt_writes", dut.b_pt_writes, 16'd32);

        // ---------- 6. a packet at the very top still fits ----------
        $display("\n=== 6. index 996 + 28 = 1024, the exact boundary ===");
        wait_listening();
        send_points(16'd996, 8'd28, 8'd3, 8'd4, 1, 0);
        wait (dut.b_rx_done);
        @(posedge sysclk);

        check("pkt_ok",    dut.b_pkt_ok,    1);
        check("pt_writes", dut.b_pt_writes, 16'd60);   // 32 + 28
        check_point(11'd996, 8'd3,  8'd4);
        check_point(11'd1023, 8'd30, 8'd31);          // last legal entry

        // ---------- 7. end-of-frame swaps the display buffer ----------
        // Nothing above set count[7], so no swap has happened yet and every
        // test so far has been reading one stable bank. One EOF packet should
        // flip which bank the scanout reads, and publish n_active from
        // max(start_index + count).
        $display("");
        $display("=== 7. end-of-frame swaps banks ===");
        // Reset first. frame_points accumulates max(start_index+count) until a
        // frame is taken, and tests 1-6 sent no EOF -- so as far as RxSeq is
        // concerned they are all one frame still in progress, and it would
        // publish n_active = 1024 from test 6's boundary case.
        rst = 1; repeat (10) @(posedge sysclk); rst = 0;

        wait_listening();
        bank_before = dut.rd_bank;
        send_points(16'd0, 8'd10, 8'd11, 8'd22, 1, 1);   // EOF set
        wait (dut.b_rx_done);
        @(posedge sysclk);
        check("frame_ready raised", int'(dut.b_frame_ready), 1);

        // The swap lands at the SCANOUT's wrap, not immediately -- flipping
        // mid-pass would move the tear rather than remove it.
        fork
            begin wait (dut.rd_bank != bank_before); end
            // One scanout pass over the seeded 32-point triangle is ~3.5 ms
            // through the real Mcp4922Driver, so allow generously.
            begin #20_000_000;
                  $display("  FAIL  banks never swapped"); errors = errors + 1; end
        join_any
        disable fork;
        @(posedge sysclk);

        if (dut.rd_bank != bank_before)
            $display("  ok    scanout bank swapped %0d -> %0d", bank_before, dut.rd_bank);
        check("frame_ready cleared", int'(dut.b_frame_ready), 0);
        check("n_active latched",    int'(dut.n_active_reg), 10);

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
        #150_000_000;
        $display("\nFAIL -- testbench timed out (FSM wedged?)");
        $finish;
    end

endmodule
