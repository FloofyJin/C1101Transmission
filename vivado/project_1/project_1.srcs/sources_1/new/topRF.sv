`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// topRF  --  Milestone 6: configure both radios, then transmit from radio A
//
// Per radio: SPIMaster + CC1101Driver + ConfigSeq. Radio A additionally gets a
// TxSeq that sends a hardcoded {0xAA, 0x55} packet and watches gdo2.
//
// ConfigSeq and TxSeq both drive radio A's single command port, so they are
// muxed on `a_tx_busy`. They never overlap: TxSeq only starts once config is
// finished, and the config re-run button is ignored while TxSeq is busy.
//
// Radio B is configured but otherwise idle -- it becomes the receiver at
// milestone 7. Its gdo2 is wired and probed now so the pin is proven early.
//
// Controls:  btn0 = reset          btn1 = send one packet on A
//            btn2 = re-run config on both radios
//            sw0  = auto-transmit (a packet every ~200 ms)
// LEDs:      LD0 = both radios configured OK
//            LD1 = last transmit completed (gdo2 rose and fell)
//            LD2 = gdo2 never moved -> transmit timed out (sticky)
//
// NOTE: net set changed -> clear debug_ila.xdc and re-run "Set Up Debug".
//------------------------------------------------------------------------------
module topRF #(
    parameter int CLK_HZ        = 125_000_000,
    parameter int POWERUP_US    = 50_000,
    parameter int TX_TIMEOUT_MS = 200        // lowered by the testbench
)(
    input  logic sysclk,
    input  logic rst,          // btn0
    input  logic tx_btn,       // btn1 -- send one packet on radio A
    input  logic cfg_btn,      // btn2 -- re-run config on both radios
    input  logic sw_auto,      // sw0  -- continuous transmit

    // CC1101 radio A (Pmod JC)
    output logic cc_sclk,
    output logic cc_mosi,
    input  logic cc_miso,
    output logic cc_csn,
    input  logic cc_gdo2,

    // CC1101 radio B (Pmod JE)
    output logic cc_b_sclk,
    output logic cc_b_mosi,
    input  logic cc_b_miso,
    output logic cc_b_csn,
    input  logic cc_b_gdo2,

    output logic led_cfg,      // LD0: both radios configured OK
    output logic led_tx,       // LD1: last packet transmitted OK
    output logic led_err       // LD2: gdo2 timeout
);
    localparam int SPI_DIV = CLK_HZ / (2 * 1_000_000);   // ~1 MHz SCLK

    // ================= CDC synchronizers =================
    // Every asynchronous chip input gets two flops. gdo2 is the new one here:
    // it is driven by the CC1101's own clock domain, so sampling it raw would
    // eventually latch a metastable value into the TX state machine.
    (* mark_debug = "true" *) logic miso_s, gdo2_s;
    logic miso_m, gdo2_m;
    logic txb_m, txb_s, txb_d;
    logic cfgb_m, cfgb_s, cfgb_d;
    logic auto_m, auto_s;

    (* mark_debug = "true" *) logic b_miso_s, b_gdo2_s;
    logic b_miso_m, b_gdo2_m;

    always_ff @(posedge sysclk) begin
        miso_m   <= cc_miso;    miso_s   <= miso_m;
        gdo2_m   <= cc_gdo2;    gdo2_s   <= gdo2_m;
        b_miso_m <= cc_b_miso;  b_miso_s <= b_miso_m;
        b_gdo2_m <= cc_b_gdo2;  b_gdo2_s <= b_gdo2_m;

        txb_m    <= tx_btn;     txb_s    <= txb_m;   txb_d  <= txb_s;
        cfgb_m   <= cfg_btn;    cfgb_s   <= cfgb_m;  cfgb_d <= cfgb_s;
        auto_m   <= sw_auto;    auto_s   <= auto_m;
    end
    wire tx_edge  = txb_s  & ~txb_d;
    wire cfg_edge = cfgb_s & ~cfgb_d;

    // CSn mirrored into the ILA: a register read must hold CSn low across all 16
    // SCLK cycles (header + data). If it lifts in between, the chip reads byte 2
    // as a new header and answers with the status byte instead of register data.
    // NOTE: this samples at sysclk, so it only catches glitches wider than 8 ns --
    // a scope on JC1 is still the authority.
    (* mark_debug = "true" *) logic csn_dbg, b_csn_dbg;
    assign csn_dbg   = cc_csn;
    assign b_csn_dbg = cc_b_csn;

    // ================= RADIO A =================
    (* mark_debug = "true" *) logic       spi_start, spi_hold, spi_busy, spi_done;
    (* mark_debug = "true" *) logic [7:0] spi_tx, spi_rx;

    SPIMaster #(.CLK_DIV(SPI_DIV)) spi_a (
        .clk(sysclk), .rst(rst),
        .start(spi_start), .tx_data(spi_tx), .hold_cs(spi_hold),
        .rx_data(spi_rx), .busy(spi_busy), .done(spi_done),
        .sclk(cc_sclk), .mosi(cc_mosi), .miso(miso_s), .cs_n(cc_csn)
    );

    // --- command bus into the driver (muxed between ConfigSeq and TxSeq) ---
    (* mark_debug = "true" *) logic [2:0] cmd;
    logic       cmd_valid;
    (* mark_debug = "true" *) logic [7:0] cmd_addr;
    logic [7:0] cmd_data;
    (* mark_debug = "true" *) logic [7:0] rd_data;
    logic       drv_done, drv_busy;
    logic [7:0] pl_idx, pl_byte, pl_len;

    CC1101Driver #(.CLK_HZ(CLK_HZ)) drv_a (
        .clk(sysclk), .rst(rst),
        .cmd_valid(cmd_valid), .cmd(cmd), .cmd_addr(cmd_addr), .cmd_data(cmd_data),
        .rd_data(rd_data), .done(drv_done), .busy(drv_busy),
        .pl_idx(pl_idx), .pl_byte(pl_byte), .pl_len(pl_len),
        .spi_start(spi_start), .spi_tx(spi_tx), .spi_hold(spi_hold),
        .spi_done(spi_done), .spi_rx(spi_rx)
    );

    // --- transmit-mission status ---
    // Declared ahead of cfg_a because its start guard reads a_tx_busy; a forward
    // reference would quietly become an implicit wire instead.
    (* mark_debug = "true" *) logic        a_tx_busy, a_tx_done, a_sync_seen;
    (* mark_debug = "true" *) logic        a_sent_ok, a_timeout;
    (* mark_debug = "true" *) logic [7:0]  a_txbytes, a_marcstate;
    (* mark_debug = "true" *) logic [15:0] a_pkt_count;

    // --- config mission ---
    // a_config_ok is only valid while a_cfg_busy is low -- ConfigSeq publishes it
    // at C_DONE, never mid-sequence. a_cfg_timeout means the driver handshake
    // stalled and the run was aborted.
    (* mark_debug = "true" *) logic       a_config_ok, a_cfg_done, a_cfg_busy;
    (* mark_debug = "true" *) logic       a_cfg_timeout;
    (* mark_debug = "true" *) logic [7:0] a_fail_addr, a_fail_got;
    logic       cfg_cmd_valid;
    logic [2:0] cfg_cmd;
    logic [7:0] cfg_cmd_addr, cfg_cmd_data;

    ConfigSeq #(.CLK_HZ(CLK_HZ), .POWERUP_US(POWERUP_US)) cfg_a (
        .clk(sysclk), .rst(rst), .start(cfg_edge & ~a_tx_busy),
        .cmd_valid(cfg_cmd_valid), .cmd(cfg_cmd),
        .cmd_addr(cfg_cmd_addr), .cmd_data(cfg_cmd_data),
        .drv_done(drv_done), .rd_data(rd_data),
        .config_ok(a_config_ok), .done(a_cfg_done), .busy(a_cfg_busy),
        .fail_addr(a_fail_addr), .fail_got(a_fail_got),
        .timeout(a_cfg_timeout)
    );

    // --- transmit mission ---
    logic       tx_cmd_valid;
    logic [2:0] tx_cmd;
    logic [7:0] tx_cmd_addr, tx_cmd_data;

    // Only transmit once the chip is actually configured -- an unconfigured
    // CC1101 will happily accept STX and put nothing useful on the air.
    wire tx_go = tx_edge & a_config_ok & ~a_cfg_busy;

    TxSeq #(.CLK_HZ(CLK_HZ), .TIMEOUT_MS(TX_TIMEOUT_MS)) tx_a (
        .clk(sysclk), .rst(rst),
        .start(tx_go), .auto_tx(auto_s & a_config_ok & ~a_cfg_busy),
        .cmd_valid(tx_cmd_valid), .cmd(tx_cmd),
        .cmd_addr(tx_cmd_addr), .cmd_data(tx_cmd_data),
        .drv_done(drv_done), .rd_data(rd_data),
        .pl_idx(pl_idx), .pl_byte(pl_byte), .pl_len(pl_len),
        .gdo2(gdo2_s),
        .busy(a_tx_busy), .done(a_tx_done),
        .sync_seen(a_sync_seen), .sent_ok(a_sent_ok), .timeout(a_timeout),
        .txbytes(a_txbytes), .marcstate(a_marcstate), .pkt_count(a_pkt_count)
    );

    // --- who owns the command bus ---
    always_comb begin
        if (a_tx_busy) begin
            cmd_valid = tx_cmd_valid;
            cmd       = tx_cmd;
            cmd_addr  = tx_cmd_addr;
            cmd_data  = tx_cmd_data;
        end else begin
            cmd_valid = cfg_cmd_valid;
            cmd       = cfg_cmd;
            cmd_addr  = cfg_cmd_addr;
            cmd_data  = cfg_cmd_data;
        end
    end

    // ================= RADIO B =================
    (* mark_debug = "true" *) logic       b_spi_start, b_spi_hold, b_spi_busy, b_spi_done;
    (* mark_debug = "true" *) logic [7:0] b_spi_tx, b_spi_rx;

    SPIMaster #(.CLK_DIV(SPI_DIV)) spi_b (
        .clk(sysclk), .rst(rst),
        .start(b_spi_start), .tx_data(b_spi_tx), .hold_cs(b_spi_hold),
        .rx_data(b_spi_rx), .busy(b_spi_busy), .done(b_spi_done),
        .sclk(cc_b_sclk), .mosi(cc_b_mosi), .miso(b_miso_s), .cs_n(cc_b_csn)
    );

    (* mark_debug = "true" *) logic [2:0] b_cmd;
    logic       b_cmd_valid;
    logic [7:0] b_cmd_addr, b_cmd_data;
    (* mark_debug = "true" *) logic [7:0] b_rd_data;
    logic       b_drv_done, b_drv_busy;
    logic [7:0] b_pl_idx;

    // Radio B never gets CMD_WRITE_FIFO in this build (it receives at M7).
    CC1101Driver #(.CLK_HZ(CLK_HZ)) drv_b (
        .clk(sysclk), .rst(rst),
        .cmd_valid(b_cmd_valid), .cmd(b_cmd), .cmd_addr(b_cmd_addr), .cmd_data(b_cmd_data),
        .rd_data(b_rd_data), .done(b_drv_done), .busy(b_drv_busy),
        .pl_idx(b_pl_idx), .pl_byte(8'h00), .pl_len(8'h00),
        .spi_start(b_spi_start), .spi_tx(b_spi_tx), .spi_hold(b_spi_hold),
        .spi_done(b_spi_done), .spi_rx(b_spi_rx)
    );

    (* mark_debug = "true" *) logic       b_config_ok, b_cfg_done, b_cfg_busy;
    (* mark_debug = "true" *) logic       b_cfg_timeout;
    (* mark_debug = "true" *) logic [7:0] b_fail_addr, b_fail_got;

    ConfigSeq #(.CLK_HZ(CLK_HZ), .POWERUP_US(POWERUP_US)) cfg_b (
        .clk(sysclk), .rst(rst), .start(cfg_edge),
        .cmd_valid(b_cmd_valid), .cmd(b_cmd), .cmd_addr(b_cmd_addr), .cmd_data(b_cmd_data),
        .drv_done(b_drv_done), .rd_data(b_rd_data),
        .config_ok(b_config_ok), .done(b_cfg_done), .busy(b_cfg_busy),
        .fail_addr(b_fail_addr), .fail_got(b_fail_got),
        .timeout(b_cfg_timeout)
    );

    // ================= LEDs =================
    assign led_cfg = a_config_ok & b_config_ok;
    assign led_tx  = a_sent_ok;
    assign led_err = a_timeout;

endmodule
