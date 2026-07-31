`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// tb_topRF_timeout  --  the failure path
//
// Same design, but the chip model never pulses GDO0 (GDO0_DELAY pushed past the
// timeout). This is what a miswired GDO0 pin, a wrong IOCFG0, or a chip that
// refuses to leave IDLE actually looks like on the bench.
//
// The FSM must NOT wedge: it has to give up, raise `timeout`, light LD2, put the
// chip back in IDLE and return to accepting commands. A hung state machine is
// the worst outcome because the ILA then shows you nothing at all.
//------------------------------------------------------------------------------
module tb_topRF_timeout;

    logic sysclk = 0;
    always #4 sysclk = ~sysclk;      // 125 MHz

    logic rst = 1, tx_btn = 0, cfg_btn = 0, sw_auto = 0;

    wire a_sclk, a_mosi, a_csn, a_miso, a_gdo0;
    wire b_sclk, b_mosi, b_csn, b_miso, b_gdo0;
    wire led_cfg, led_tx, led_err;

    topRF #(.CLK_HZ(125_000_000), .POWERUP_US(1), .TX_TIMEOUT_MS(1)) dut (
        .sysclk(sysclk), .rst(rst),
        .tx_btn(tx_btn), .cfg_btn(cfg_btn), .sw_auto(sw_auto),
        .cc_sclk(a_sclk), .cc_mosi(a_mosi), .cc_miso(a_miso),
        .cc_csn(a_csn),   .cc_gdo0(a_gdo0),
        .cc_b_sclk(b_sclk), .cc_b_mosi(b_mosi), .cc_b_miso(b_miso),
        .cc_b_csn(b_csn),   .cc_b_gdo0(b_gdo0),
        .led_cfg(led_cfg), .led_tx(led_tx), .led_err(led_err)
    );

    // GDO0 never arrives within the 1 ms timeout
    cc1101_model #(.GDO0_DELAY(100_000_000)) radio_a (
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
            end else $display("  ok    %0s = 0x%0h", what, got);
        end
    endtask

    initial begin
        repeat (20) @(posedge sysclk);
        rst = 0;

        wait (dut.a_cfg_done);
        wait (dut.b_cfg_done);
        @(posedge sysclk);

        $display("\n=== transmit with GDO0 dead ===");
        tx_btn = 1;
        repeat (10) @(posedge sysclk);
        tx_btn = 0;

        fork
            wait (dut.a_tx_done);
            begin
                #10_000_000;    // 10 ms -- ten times the 1 ms timeout
                $display("  FAIL  FSM wedged: a_tx_done never pulsed");
                errors = errors + 1;
            end
        join_any
        disable fork;
        @(posedge sysclk);

        check("a_timeout",   dut.a_timeout,   1);
        check("a_sent_ok",   dut.a_sent_ok,   0);
        check("a_pkt_count", dut.a_pkt_count, 0);
        check("led_err",     led_err,         1);
        check("led_tx",      led_tx,          0);
        // the chip was still parked back in IDLE on the way out
        check("chip A marcstate", radio_a.marcstate, 8'h01);
        // and the sequencer released the bus
        check("a_tx_busy",   dut.a_tx_busy,   0);

        $display("\n=====================================");
        if (errors == 0) $display("PASS -- timeout path recovers cleanly");
        else             $display("FAIL -- %0d check(s) failed", errors);
        $display("=====================================\n");
        $finish;
    end

    initial begin
        #100_000_000;
        $display("FAIL -- global timeout");
        $finish;
    end
endmodule
