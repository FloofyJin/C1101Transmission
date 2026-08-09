`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// tb_mcp4922  --  milestone-10 testbench for Mcp4922Driver
//
// Reconstructs both 16-bit words by sampling SDI on SCK rising edges, exactly
// as the real chip does, and checks them against {CFG, data}. Then checks every
// AC timing minimum from the MCP4902/4912/4922 datasheet (DS22250A page 7)
// against the actual waveform.
//
// The timing checks are the point. A word that is right but violates tHD works
// on the bench and fails on someone else's board -- that class of bug is
// invisible to a DMM and invisible to eyeballing the RTL.
//
//   tHI    15 ns   clock high time
//   tLO    15 ns   clock low time
//   tSU    15 ns   data setup before SCK rise
//   tHD    10 ns   data hold after SCK rise
//   tCSH   15 ns   CS high time between words
//   tIDLE  40 ns   SCK idle before CS falls
//   tLD   100 ns   LDAC low pulse width
//   tLS    40 ns   LDAC setup: CS rise -> LDAC fall
//
// Run:  iverilog -g2012 -o tb.vvp -s tb_mcp4922 Mcp4922Driver.sv tb_mcp4922.sv
//------------------------------------------------------------------------------
module tb_mcp4922;

    // Datasheet minimums, in ns.
    localparam real T_HI    = 15.0;
    localparam real T_LO    = 15.0;
    localparam real T_SU    = 15.0;
    localparam real T_HD    = 10.0;
    localparam real T_CSH   = 15.0;
    localparam real T_IDLE  = 40.0;
    localparam real T_LD    = 100.0;
    localparam real T_LS    = 40.0;

    logic clk = 0;
    always #4 clk = ~clk;              // 125 MHz

    logic        rst = 1;
    logic        start = 0;
    logic [11:0] x, y;
    logic        done, busy;
    logic        dac_cs, dac_ldac, dac_sclk, dac_sdi;

    Mcp4922Driver dut (
        .clk(clk), .rst(rst), .start(start), .x(x), .y(y),
        .done(done), .busy(busy),
        .dac_cs(dac_cs), .dac_ldac(dac_ldac),
        .dac_sclk(dac_sclk), .dac_sdi(dac_sdi)
    );

    integer errors = 0;

    task fail(input [255:0] what);
        begin
            $display("  FAIL  %0s", what);
            errors = errors + 1;
        end
    endtask

    task check(input [255:0] what, input integer got, input integer exp);
        begin
            if (got !== exp) begin
                $display("  FAIL  %0s: got 0x%0h, expected 0x%0h", what, got, exp);
                errors = errors + 1;
            end else $display("  ok    %0s = 0x%0h", what, got);
        end
    endtask

    task check_t(input [255:0] what, input real got, input real min);
        begin
            if (got < min) begin
                $display("  FAIL  %0s: %0.1f ns, minimum %0.1f ns", what, got, min);
                errors = errors + 1;
            end else $display("  ok    %0s = %0.1f ns (min %0.1f)", what, got, min);
        end
    endtask

    // ---- reconstruct the words exactly as the chip would ----
    logic [15:0] word     [0:3];       // captured words, in order
    integer      n_words  = 0;
    logic [15:0] shifter  = 0;
    integer      n_bits   = 0;
    integer      bits_in_word [0:3];

    // ---- timing observation ----
    real t_sck_rise = -1, t_sck_fall = -1;
    real t_sdi_chg  = -1;
    real t_cs_rise  = -1, t_cs_fall = -1;
    real t_ldac_fall = -1;
    real worst_hi = 1e9, worst_lo = 1e9, worst_su = 1e9, worst_hd = 1e9;
    real meas_csh = -1, meas_idle = -1, meas_ld = -1, meas_ls = -1;

    // SDI transitions: check hold against the previous SCK rise.
    always @(dac_sdi) begin
        if (!rst && !dac_cs && t_sck_rise >= 0) begin
            if ($realtime - t_sck_rise < worst_hd) worst_hd = $realtime - t_sck_rise;
        end
        t_sdi_chg = $realtime;
    end

    always @(posedge dac_sclk) if (!rst && !dac_cs) begin
        // setup: how long has SDI been stable before this sampling edge?
        if (t_sdi_chg >= 0 && $realtime - t_sdi_chg < worst_su)
            worst_su = $realtime - t_sdi_chg;
        // low time just ended
        if (t_sck_fall >= 0 && $realtime - t_sck_fall < worst_lo)
            worst_lo = $realtime - t_sck_fall;
        t_sck_rise = $realtime;

        // the chip samples here
        shifter = {shifter[14:0], dac_sdi};
        n_bits  = n_bits + 1;
    end

    always @(negedge dac_sclk) if (!rst && !dac_cs) begin
        if (t_sck_rise >= 0 && $realtime - t_sck_rise < worst_hi)
            worst_hi = $realtime - t_sck_rise;
        t_sck_fall = $realtime;
    end

    always @(posedge dac_cs) if (!rst) begin
        // CS rise commits the word
        if (n_words < 4) begin
            word[n_words]         = shifter;
            bits_in_word[n_words] = n_bits;
            n_words               = n_words + 1;
        end
        shifter    = 0;
        n_bits     = 0;
        t_cs_rise  = $realtime;
        t_sck_rise = -1;
        t_sck_fall = -1;
    end

    always @(negedge dac_cs) if (!rst) begin
        if (t_cs_rise >= 0) meas_csh  = $realtime - t_cs_rise;   // CS high time
        if (t_sck_fall >= 0) meas_idle = $realtime - t_sck_fall;
        else if (t_cs_rise >= 0) meas_idle = $realtime - t_cs_rise;
        t_cs_fall = $realtime;
    end

    always @(negedge dac_ldac) if (!rst) begin
        t_ldac_fall = $realtime;
        if (t_cs_rise >= 0) meas_ls = $realtime - t_cs_rise;      // LDAC setup
    end

    always @(posedge dac_ldac) if (!rst) begin
        if (t_ldac_fall >= 0) meas_ld = $realtime - t_ldac_fall;  // LDAC pulse
    end

    initial begin
        $display("\n=== milestone 10: Mcp4922Driver ===\n");

        x = 12'h5A3;
        y = 12'h2C7;

        repeat (10) @(posedge clk);
        rst = 0;
        repeat (5) @(posedge clk);

        @(posedge clk); start <= 1;
        @(posedge clk); start <= 0;

        fork
            begin
                wait (done);
                @(posedge clk);
            end
            begin
                #20_000;
                fail("timed out waiting for done -- FSM wedged");
                $display("\n=====================================");
                $display("FAIL -- %0d check(s) failed", errors);
                $display("=====================================\n");
                $finish;
            end
        join_any
        disable fork;

        $display("--- word framing ---");
        check("words written",     n_words, 2);
        if (n_words >= 1) check("bits in word A", bits_in_word[0], 16);
        if (n_words >= 2) check("bits in word B", bits_in_word[1], 16);

        $display("\n--- word contents ---");
        // A/B=0, BUF=0, GA=1, SHDN=1 -> 0x3 ; A/B=1 -> 0xB
        if (n_words >= 1) check("word A", word[0], {4'b0011, x});
        if (n_words >= 2) check("word B", word[1], {4'b1011, y});

        $display("\n--- SPI timing (datasheet DS22250A p.7) ---");
        check_t("tHI  clock high",        worst_hi,  T_HI);
        check_t("tLO  clock low",         worst_lo,  T_LO);
        check_t("tSU  data setup",        worst_su,  T_SU);
        check_t("tHD  data hold",         worst_hd,  T_HD);
        check_t("tCSH CS high between",   meas_csh,  T_CSH);
        check_t("tIDLE SCK idle pre-CS",  meas_idle, T_IDLE);

        $display("\n--- LDAC ---");
        if (meas_ls < 0) fail("LDAC never went low -- outputs would never update");
        else begin
            check_t("tLS  CS rise -> LDAC fall", meas_ls, T_LS);
            check_t("tLD  LDAC low pulse",       meas_ld, T_LD);
        end

        $display("\n--- handshake ---");
        check("busy low after done", busy, 0);
        check("ldac high at rest",   dac_ldac, 1);
        check("cs high at rest",     dac_cs, 1);

        // ---- a second point must behave identically ----
        $display("\n=== second point ===");
        n_words = 0;
        x = 12'hFFF; y = 12'h000;
        @(posedge clk); start <= 1;
        @(posedge clk); start <= 0;

        fork
            begin wait (done); @(posedge clk); end
            begin #20_000; fail("second point timed out"); end
        join_any
        disable fork;

        check("words written",  n_words, 2);
        if (n_words >= 1) check("word A (0xFFF)", word[0], {4'b0011, 12'hFFF});
        if (n_words >= 2) check("word B (0x000)", word[1], {4'b1011, 12'h000});

        $display("\n=====================================");
        if (errors == 0) $display("PASS -- words and timing are correct");
        else             $display("FAIL -- %0d check(s) failed", errors);
        $display("=====================================\n");

        $finish;
    end

endmodule
