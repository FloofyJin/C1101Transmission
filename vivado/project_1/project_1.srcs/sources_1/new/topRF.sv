`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// topRF  --  Milestone 14: receive point packets into PointRam
//
// Per radio: SPIMaster + CC1101Driver + ConfigSeq. Radio A additionally gets a
// TxSeq (the phase-1 transmitter, kept for regression); radio B gets RxSeq,
// which now parses the phase-2 point packet and loads PointRam.
//
// Phase 2 reverses the link: the STM32 transmits and the Zybo receives, so the
// live path is radio B. Radio A's TxSeq still works and is worth keeping -- it
// is the only way to test the receiver without the STM32 in the loop.
//
// ConfigSeq and TxSeq both drive radio A's single command port, so they are
// muxed on `a_tx_busy`. They never overlap: TxSeq only starts once config is
// finished, and the config re-run button is ignored while TxSeq is busy. Radio
// B's bus is shared the same way between ConfigSeq and RxSeq.
//
// Controls:  btn0 = reset          btn1 = send one packet on A
//            btn2 = re-run config on both radios
//            sw0  = auto-transmit (a packet every ~200 ms)
//            sw1  = sender enable (radio A)
//            sw2  = receiver enable (radio B)
// LEDs:      LD0 = enabled radios configured OK
//            LD1 = last transmit completed (gdo2 rose and fell)
//            LD2 = gdo2 never moved -> transmit timed out (sticky)
//            LD3 = a point packet was received and committed (sticky)
//
// NOTE: net set changed -> clear debug_ila.xdc and re-run "Set Up Debug".
//------------------------------------------------------------------------------
module topRF #(
    parameter int CLK_HZ        = 125_000_000,
    parameter int POWERUP_US    = 50_000,
    parameter int TX_TIMEOUT_MS = 200        // lowered by the testbench
)(
    input  logic sysclk,
    input  logic rst,           // btn0
    input  logic tx_btn,        // btn1 -- send one packet on radio A
    input  logic cfg_btn,       // btn2 -- re-run config on both radios
    input  logic sw_auto,       // sw0  -- continuous transmit
    input  logic sw_sender_enable,   // sw1
    input  logic sw_receiver_enable,// sw2  -- receive only

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

    // MCP4922 (Pmod JD)
    output logic dac_cs,
    output logic dac_sdi,
    output logic dac_ldac,
    output logic dac_sclk,

    // Oscilloscope Z axis (Pmod JD7). 1 = beam off.
    // Goes straight to the scope's rear EXT Z AXIS IN -- NOT to the DAC. No
    // series resistor: 3.3 V into the ~500 ohm input is 6.6 mA, inside Zynq
    // limits, and overdriving past the 2 V p-p "full range" only blanks harder.
    output logic z_blank,

    output logic led_cfg,      // LD0: both radios configured OK
    output logic led_tx,       // LD1: last packet transmitted OK
    output logic led_err,      // LD2: gdo2 timeout
    output logic led_rx        // LD3: radio B received the expected packet (M7)
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
    logic sender_enable_m, sender_enable_s;
    logic receiver_enable_m, receiver_enable_s;

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
        sender_enable_m <= sw_sender_enable;  sender_enable_s <= sender_enable_m;
        receiver_enable_m <= sw_receiver_enable;  receiver_enable_s <= receiver_enable_m;
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
        // Radio A transmits only -- it never issues CMD_READ_FIFO, so the RX
        // burst-read interface is tied off rather than left dangling.
        .rx_len(8'h00), .rx_idx(), .rx_byte(), .rx_strobe(),
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
    (* mark_debug = "true" *) logic [7:0] a_fail_addr, a_fail_value;
    logic       cfg_cmd_valid;
    logic [2:0] cfg_cmd;
    logic [7:0] cfg_cmd_addr, cfg_cmd_data;

    ConfigSeq #(.CLK_HZ(CLK_HZ), .POWERUP_US(POWERUP_US)) cfg_a (
        .clk(sysclk), .rst(rst), .start(cfg_edge & ~a_tx_busy),  .enable(sender_enable_s),
        .cmd_valid(cfg_cmd_valid), .cmd(cfg_cmd),
        .cmd_addr(cfg_cmd_addr), .cmd_data(cfg_cmd_data),
        .drv_done(drv_done), .rd_data(rd_data),
        .config_ok(a_config_ok), .done(a_cfg_done), .busy(a_cfg_busy),
        .fail_addr(a_fail_addr), .fail_value(a_fail_value),
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
    (* mark_debug = "true" *) logic [7:0] b_cmd_addr;
    logic [7:0] b_cmd_data;
    (* mark_debug = "true" *) logic [7:0] b_rd_data;
    logic       b_drv_done, b_drv_busy;
    logic [7:0] b_pl_idx;
    logic [7:0] b_rx_len, b_rx_idx, b_rx_byte;
    logic       b_rx_strobe;

    // Radio B never gets CMD_WRITE_FIFO in this build -- it only receives.
    CC1101Driver #(.CLK_HZ(CLK_HZ)) drv_b (
        .clk(sysclk), .rst(rst),
        .cmd_valid(b_cmd_valid), .cmd(b_cmd), .cmd_addr(b_cmd_addr), .cmd_data(b_cmd_data),
        .rd_data(b_rd_data), .done(b_drv_done), .busy(b_drv_busy),
        .pl_idx(b_pl_idx), .pl_byte(8'h00), .pl_len(8'h00),
        .rx_len(b_rx_len), .rx_idx(b_rx_idx), .rx_byte(b_rx_byte),
        .rx_strobe(b_rx_strobe),
        .spi_start(b_spi_start), .spi_tx(b_spi_tx), .spi_hold(b_spi_hold),
        .spi_done(b_spi_done), .spi_rx(b_spi_rx)
    );

    // --- receive-mission status (declared ahead of cfg_b, whose start guard
    //     reads b_rx_busy) ---
    (* mark_debug = "true" *) logic        b_rx_busy, b_rx_done;
    (* mark_debug = "true" *) logic        b_pkt_ok, b_crc_ok, b_overflow, b_rx_timeout;
    (* mark_debug = "true" *) logic        b_fmt_ok, b_bad_fmt;
    (* mark_debug = "true" *) logic [7:0]  b_rxbytes, b_len_byte;
    (* mark_debug = "true" *) logic [15:0] b_start_index;
    (* mark_debug = "true" *) logic [7:0]  b_count, b_rssi;
    (* mark_debug = "true" *) logic [6:0]  b_lqi;
    (* mark_debug = "true" *) logic [15:0] b_rx_count, b_err_count, b_pt_writes;

    // ---- PointRam write port, driven by RxSeq ----
    localparam int PT_N     = 1024;              // points per bank
    localparam int PT_IDX_W = $clog2(PT_N);      // 10

    (* mark_debug = "true" *) logic                pt_we;
    (* mark_debug = "true" *) logic [PT_IDX_W:0]   pt_addr;
    (* mark_debug = "true" *) logic [17:0]         pt_data;

    // ---- double-buffer handshake ----
    (* mark_debug = "true" *) logic                b_frame_ready;
    (* mark_debug = "true" *) logic [PT_IDX_W:0]   b_frame_points;
    (* mark_debug = "true" *) logic                sc_wrap;
    (* mark_debug = "true" *) logic                rd_bank;
    (* mark_debug = "true" *) logic [PT_IDX_W:0]   n_active_reg;

    // Swap ONLY at the scanout's wrap. Flipping mid-pass would move the tear
    // rather than remove it -- same reason a framebuffer swaps on vsync.
    wire do_swap = sc_wrap & b_frame_ready;

    always_ff @(posedge sysclk) begin
        if (rst) begin
            rd_bank      <= 1'b0;
            n_active_reg <= (PT_IDX_W+1)'(1024); // the seeded checkerboard:
                                                  // 128 rows x 4 spans x 2 points
        end else if (do_swap) begin
            rd_bank      <= ~rd_bank;
            n_active_reg <= b_frame_points;
        end
    end

    // --- config mission ---
    (* mark_debug = "true" *) logic       b_config_ok, b_cfg_done, b_cfg_busy;
    (* mark_debug = "true" *) logic       b_cfg_timeout;
    (* mark_debug = "true" *) logic [7:0] b_fail_addr, b_fail_value;
    logic       b_cfg_cmd_valid;
    logic [2:0] b_cfg_cmd;
    logic [7:0] b_cfg_cmd_addr, b_cfg_cmd_data;

    ConfigSeq #(.CLK_HZ(CLK_HZ), .POWERUP_US(POWERUP_US)) cfg_b (
        .clk(sysclk), .rst(rst), .start(cfg_edge & ~b_rx_busy), .enable(receiver_enable_s),
        .cmd_valid(b_cfg_cmd_valid), .cmd(b_cfg_cmd),
        .cmd_addr(b_cfg_cmd_addr), .cmd_data(b_cfg_cmd_data),
        .drv_done(b_drv_done), .rd_data(b_rd_data),
        .config_ok(b_config_ok), .done(b_cfg_done), .busy(b_cfg_busy),
        .fail_addr(b_fail_addr), .fail_value(b_fail_value),
        .timeout(b_cfg_timeout)
    );

    logic       b_rx_cmd_valid;
    logic [2:0] b_rx_cmd;
    logic [7:0] b_rx_cmd_addr, b_rx_cmd_data;

    RxSeq #(.CLK_HZ(CLK_HZ)) rx_b (
        .clk(sysclk), .rst(rst),
        .enable(b_config_ok), .cfg_busy(b_cfg_busy),
        .cmd_valid(b_rx_cmd_valid), .cmd(b_rx_cmd),
        .cmd_addr(b_rx_cmd_addr), .cmd_data(b_rx_cmd_data),
        .drv_done(b_drv_done), .rd_data(b_rd_data),
        .rx_len(b_rx_len), .rx_idx(b_rx_idx), .rx_byte(b_rx_byte),
        .rx_strobe(b_rx_strobe),
        .gdo2(b_gdo2_s),
        .pt_we(pt_we), .pt_addr(pt_addr), .pt_data(pt_data),
        .frame_ready(b_frame_ready), .frame_points(b_frame_points),
        .frame_ack(do_swap),
        .busy(b_rx_busy), .done(b_rx_done),
        .pkt_ok(b_pkt_ok), .crc_ok(b_crc_ok),
        .fmt_ok(b_fmt_ok), .bad_fmt(b_bad_fmt),
        .overflow(b_overflow), .timeout(b_rx_timeout),
        .rxbytes(b_rxbytes), .len_byte(b_len_byte),
        .start_index(b_start_index), .count(b_count),
        .rssi(b_rssi), .lqi(b_lqi),
        .rx_count(b_rx_count), .err_count(b_err_count),
        .pt_writes(b_pt_writes)
    );

    // ================= POINT RAM =================
    // The coordinate list. RxSeq writes it; ScanoutEngine will read it at M11.
    //
    // The read port belongs to ScanoutEngine, which walks it forever. Probing
    // pt_raddr/pt_rdata on the ILA shows the corners actually being drawn.
    //
    // (Before ScanoutEngine existed, a free-running counter drove raddr purely
    // so the RAM had a consumer -- an unread BRAM is optimised away entirely.
    // That counter also dumped all 255 entries into one ILA capture, which was
    // how M14 was verified. Restore it temporarily if you need that view back.)
    (* mark_debug = "true" *) logic [PT_IDX_W:0] pt_raddr;
    (* mark_debug = "true" *) logic [17:0] pt_rdata;

    PointRam #(.N_POINTS(PT_N), .DWELL_BITS(2)) pram (
        .clk(sysclk),
        .we(pt_we), .waddr(pt_addr), .wdata(pt_data),
        .raddr(pt_raddr), .rdata(pt_rdata)
    );

    // --- who owns radio B's command bus ---
    // RxSeq drops b_rx_busy while merely listening, so ConfigSeq can still take
    // the bus for a re-run; RxSeq stands down and re-arms when that finishes.
    always_comb begin
        if (b_rx_busy) begin
            b_cmd_valid = b_rx_cmd_valid;
            b_cmd       = b_rx_cmd;
            b_cmd_addr  = b_rx_cmd_addr;
            b_cmd_data  = b_rx_cmd_data;
        end else begin
            b_cmd_valid = b_cfg_cmd_valid;
            b_cmd       = b_cfg_cmd;
            b_cmd_addr  = b_cfg_cmd_addr;
            b_cmd_data  = b_cfg_cmd_data;
        end
    end

    // ================= DISPLAY =================
    // ScanoutEngine walks PointRam and interpolates; Mcp4922Driver turns each
    // point into two SPI words and an LDAC pulse. Free-running -- the radio
    // never gates it, which is the whole reason a ~36 fps upload can coexist
    // with a >=60 Hz refresh.
    //
    // n_active is the number of points drawn, latched from RxSeq's
    // max(start_index + count) at each buffer swap -- so the vectorizer picks
    // the fps/detail trade per frame with no RTL change. It resets to 1024 to
    // match the 128 x 8 checkerboard seeded into BOTH PointRam banks (128 rows
    // x 4 spans x 2 points), which is what is on screen before any RF arrives.
    //
    // SPACING is the distance between consecutive DAC points, in 8-bit
    // coordinate units. It MUST be a power of two -- the step count rounds up
    // to 2^k so the divide is a shift -- which makes the real choice 4 or 8.
    // Lower it if spans break into visible dots; raise it if the sustained
    // point rate cannot keep up at 60 Hz.
    localparam int SPACING  = 4;       // coordinate units between DAC points

    logic        draw_point, draw_point_done;
    (* mark_debug = "true" *) logic [11:0] draw_x, draw_y;
    (* mark_debug = "true" *) logic [PT_IDX_W-1:0] sc_corner;
    (* mark_debug = "true" *) logic [15:0] sc_frames;

    ScanoutEngine #(.N_POINTS(PT_N), .SPACING(SPACING)) scan (
        .clk(sysclk), .rst(rst),
        .n_active(n_active_reg),
        .rd_bank(rd_bank), .restart(do_swap),
        .raddr(pt_raddr), .rdata(pt_rdata),
        .start(draw_point), .x(draw_x), .y(draw_y), .done(draw_point_done),
        .z_blank(z_blank),
        .corner_idx(sc_corner), .wrap(sc_wrap), .frame_count(sc_frames)
    );

    // --- MCP4922 on Pmod JD ---
    Mcp4922Driver #() dacDriver (
        .clk(sysclk), .rst(rst), .start(draw_point),
        .x(draw_x), .y(draw_y), .done(draw_point_done),
        .busy(), .dac_cs(dac_cs), .dac_ldac(dac_ldac),
        .dac_sclk(dac_sclk), .dac_sdi(dac_sdi)
    );

    // ================= LEDs =================
    // In send-only mode radio B's ConfigSeq is held disabled and never publishes
    // b_config_ok, so B must be excused rather than required: the term is
    // "B is not needed OR B passed", not "B is needed AND B passed".
    assign led_cfg = (a_config_ok | !sender_enable_s) & (b_config_ok | !receiver_enable_s);
    assign led_tx  = a_sent_ok;
    assign led_err = a_timeout;
    // LD3: a point packet was received and committed to PointRam. Driven from
    // the COUNTER, not from b_pkt_ok -- RxSeq clears pkt_ok when it re-arms,
    // roughly 30 us after each packet, so b_pkt_ok is a per-packet verdict for
    // the ILA to sample at b_rx_done, not something an eye can see. rx_count is
    // sticky: first good packet lights it and it stays lit.
    assign led_rx  = |b_rx_count;

endmodule
