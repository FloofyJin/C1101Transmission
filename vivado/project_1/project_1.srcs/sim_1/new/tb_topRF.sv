`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// tb_topRF  --  milestone-6 testbench
//
// Runs the real topRF against two cc1101_model stubs. Checks that:
//   1. both radios configure and read back OK (regression on milestone 5)
//   2. pressing tx_btn emits EXACTLY the milestone-6 transmit sequence:
//        SIDLE(0x36) SFTX(0x3B) 0x7F 0x02 0xAA 0x55 TXBYTES(0xFA) STX(0x35)
//        MARCSTATE(0xF5) SIDLE(0x36)
//   3. the diagnostics read back what they should (TXBYTES=3, MARCSTATE=0x13)
//   4. GDO0 rise + fall is seen and pkt_count increments
//
// POWERUP_US is cut to 1 us so the sim finishes quickly -- on hardware it stays
// at 50 ms to let the chip's supply and crystal settle.
//------------------------------------------------------------------------------
module tb_topRF;

    logic sysclk = 0;
    always #4 sysclk = ~sysclk;      // 125 MHz

    logic rst = 1, tx_btn = 0, cfg_btn = 0, sw_auto = 0;

    wire a_sclk, a_mosi, a_csn, a_miso, a_gdo0;
    wire b_sclk, b_mosi, b_csn, b_miso, b_gdo0;
    wire led_cfg, led_tx, led_err;

    topRF #(.CLK_HZ(125_000_000), .POWERUP_US(1)) dut (
        .sysclk(sysclk), .rst(rst),
        .tx_btn(tx_btn), .cfg_btn(cfg_btn), .sw_auto(sw_auto),
        .cc_sclk(a_sclk), .cc_mosi(a_mosi), .cc_miso(a_miso),
        .cc_csn(a_csn),   .cc_gdo0(a_gdo0),
        .cc_b_sclk(b_sclk), .cc_b_mosi(b_mosi), .cc_b_miso(b_miso),
        .cc_b_csn(b_csn),   .cc_b_gdo0(b_gdo0),
        .led_cfg(led_cfg), .led_tx(led_tx), .led_err(led_err)
    );

    cc1101_model radio_a (
        .cs_n(a_csn), .sclk(a_sclk), .mosi(a_mosi), .miso(a_miso), .gdo0(a_gdo0)
    );
    cc1101_model radio_b (
        .cs_n(b_csn), .sclk(b_sclk), .mosi(b_mosi), .miso(b_miso), .gdo0(b_gdo0)
    );

    integer errors = 0;
    integer tx_base;

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

    task expect_byte(input integer offset, input [7:0] exp, input [255:0] what);
        begin
            if (radio_a.seen[tx_base + offset] !== exp) begin
                $display("  FAIL  tx byte %0d (%0s): got 0x%02h, expected 0x%02h",
                         offset, what, radio_a.seen[tx_base + offset], exp);
                errors = errors + 1;
            end else begin
                $display("  ok    tx byte %0d = 0x%02h  (%0s)", offset, exp, what);
            end
        end
    endtask

    initial begin
        $dumpfile("tb_topRF.vcd");
        $dumpvars(0, tb_topRF);

        repeat (20) @(posedge sysclk);
        rst = 0;

        // ---------- 1. wait for both configs to finish ----------
        $display("\n=== configuration ===");
        wait (dut.a_cfg_done);
        wait (dut.b_cfg_done);
        @(posedge sysclk);
        check("a_config_ok", dut.a_config_ok, 1);
        check("b_config_ok", dut.b_config_ok, 1);
        check("led_cfg",     led_cfg,         1);
        check("PKTCTRL0 in chip A", radio_a.regs[8'h08], 8'h05);
        check("MDMCFG2  in chip A", radio_a.regs[8'h12], 8'h13);
        check("IOCFG0   in chip A", radio_a.regs[8'h02], 8'h06);
        check("PATABLE  in chip A", radio_a.regs[8'h3E], 8'h12);

        // ---------- 2. transmit one packet ----------
        $display("\n=== transmit ===");
        tx_base = radio_a.n_seen;      // ignore every byte the config sent
        @(posedge sysclk);
        tx_btn = 1;
        repeat (10) @(posedge sysclk);
        tx_btn = 0;

        fork
            begin
                wait (dut.a_tx_done);
            end
            begin
                #5_000_000;            // 5 ms guard -- the FSM must not wedge
                $display("  FAIL  timed out waiting for a_tx_done");
                errors = errors + 1;
            end
        join_any
        disable fork;
        @(posedge sysclk);

        expect_byte(0, 8'h36, "SIDLE strobe");
        expect_byte(1, 8'h3B, "SFTX strobe");
        expect_byte(2, 8'h7F, "TX FIFO burst-write header");
        expect_byte(3, 8'h02, "length byte");
        expect_byte(4, 8'hAA, "payload[0]");
        expect_byte(5, 8'h55, "payload[1]");
        expect_byte(6, 8'hFA, "TXBYTES status read");
        expect_byte(8, 8'h35, "STX strobe");
        expect_byte(9, 8'hF5, "MARCSTATE status read");
        expect_byte(11, 8'h36, "SIDLE strobe");

        check("a_txbytes",   dut.a_txbytes,   8'h03);
        check("a_marcstate", dut.a_marcstate, 8'h13);
        check("a_sync_seen", dut.a_sync_seen, 1);
        check("a_sent_ok",   dut.a_sent_ok,   1);
        check("a_timeout",   dut.a_timeout,   0);
        check("a_pkt_count", dut.a_pkt_count, 1);
        check("led_tx",      led_tx,          1);
        check("led_err",     led_err,         0);

        // the model saw the payload land in its FIFO in order
        check("chip A fifo[0] (len)", radio_a.fifo[0], 8'h02);
        check("chip A fifo[1]",       radio_a.fifo[1], 8'hAA);
        check("chip A fifo[2]",       radio_a.fifo[2], 8'h55);

        // ---------- 3. a second packet still works ----------
        $display("\n=== second packet ===");
        @(posedge sysclk);
        tx_btn = 1;
        repeat (10) @(posedge sysclk);
        tx_btn = 0;
        wait (dut.a_tx_done);
        @(posedge sysclk);
        check("a_pkt_count after 2", dut.a_pkt_count, 2);

        $display("\n=====================================");
        if (errors == 0) $display("PASS -- milestone 6 sequence is correct");
        else             $display("FAIL -- %0d check(s) failed", errors);
        $display("=====================================\n");
        $finish;
    end

    // global watchdog
    initial begin
        #50_000_000;
        $display("FAIL -- global timeout, the design is stuck");
        $finish;
    end
endmodule
