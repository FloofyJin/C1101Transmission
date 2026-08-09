`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// RxSeq  --  Milestone 14: receive a point packet and load it into PointRam
//
// Drives one CC1101Driver's command interface:
//
//   SIDLE -> SFRX -> SRX -> listen -> gdo2 rise -> gdo2 fall
//         -> read RXBYTES (twice, must agree) -> burst-read the FIFO
//         -> check CRC + format -> commit to PointRam -> SFRX -> re-arm
//
// ---- what is actually in the FIFO ----
// With PKTCTRL0 = 0x05 (variable length) and PKTCTRL1 = 0x04 (append status):
//
//     [len] [start_index] [count] [x0][y0] ... [RSSI] [LQI | CRC_OK<<7]
//      ^^^   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^   ^^^^^^^^^^^^^^^^^^^^^^
//      chip           len bytes of payload           chip-appended
//
// Note the sender's 0x7F burst-write header is NOT here and never was -- that
// byte only ever existed on the transmitter's own SPI bus. Likewise this side's
// 0xFF read header is consumed by the driver.
//
// So a 30-point packet is 1 + 62 + 2 = 65 bytes, which is why RX_MAX is 65.
//
// ---- why the payload is staged instead of written straight through ----
// CRC_OK arrives in the LAST byte of the burst, after every payload byte has
// already streamed past. Writing points into PointRam as they arrive would
// therefore commit a corrupted packet before there is any way to know it was
// corrupted -- and since the CRC covers the whole payload, a bad packet can
// have a wrong `start_index`, scattering garbage at a random location.
//
// The format is self-healing against LOSS, not against CORRUPTION: a lost
// packet leaves stale but valid coordinates that the next pass repairs, while a
// corrupted one writes points that were never in the drawing. So the payload
// lands in stage_x/stage_y first and is replayed into PointRam only once CRC
// and format both pass. 60 bytes of staging is the price.
//
// ---- why RXBYTES is read twice ----
// A status register read can be corrupted if it lands while the register is
// updating. RXBYTES decides how many bytes we clock out of the FIFO, so a wrong
// value desynchronises the whole burst and every byte after it is garbage.
// Reading until two consecutive reads agree costs two SPI transactions and
// removes the entire failure class.
//
// ---- bus sharing with ConfigSeq ----
// A receiver listens indefinitely, so RxSeq must NOT hold the command bus while
// waiting. It drops `busy` in R_LISTEN (where it issues no commands) and picks
// it back up the moment gdo2 rises. If a config re-run starts while listening,
// RxSeq stands down and re-arms afterwards -- config leaves the chip in IDLE,
// so RX has to be re-armed anyway.
//------------------------------------------------------------------------------
module RxSeq #(
    parameter int CLK_HZ     = 125_000_000,
    parameter int MAX_POINTS = 30,          // (63 - 2) / 2, the TX FIFO ceiling
    parameter int N_POINTS   = 255,         // PointRam depth
    parameter int RX_MAX     = 65,          // 1 len + 60 payload + 1 starting index + count + 2 status (CRC/LQI + RSSI)
    parameter int END_TIMEOUT_MS = 200,     // gdo2 stuck high -> give up
    // Dwell is not transmitted -- every committed point gets this value. The
    // encoding is defined by ScanoutEngine (M12); until then it is inert.
    // Make it a register there rather than a literal, so M12 can sweep it live
    // over the ILA to find the value that lights lines evenly.
    parameter logic [1:0] DWELL_DEFAULT = 2'b11
)(
    input  logic       clk,
    input  logic       rst,
    input  logic       enable,         // level: listen (tie to config_ok)
    input  logic       cfg_busy,       // stand down while ConfigSeq owns the chip

    // to CC1101Driver command interface
    output logic       cmd_valid,
    output logic [2:0] cmd,
    output logic [7:0] cmd_addr,
    output logic [7:0] cmd_data,
    input  logic       drv_done,
    input  logic [7:0] rd_data,

    // RX FIFO burst-read interface from the driver
    output logic [7:0] rx_len,        // how many bytes to clock out
    input  logic [7:0] rx_idx,
    input  logic [7:0] rx_byte,
    input  logic       rx_strobe,

    input  logic       gdo2,          // MUST already be synchronized to clk

    // ---- PointRam write port ----
    output logic        pt_we,
    output logic [7:0]  pt_addr,
    output logic [17:0] pt_data,      // {x, y, dwell}

    // status / ILA
    output logic        busy,
    output logic        done,          // 1-cycle pulse per completed reception
    output logic        pkt_ok,        // CRC good AND format valid
    output logic        crc_ok,        // hardware CRC flag from the status byte
    output logic        fmt_ok,        // format check alone -- see note below
    output logic        bad_fmt,       // sticky: CRC passed but format did not
    output logic        overflow,      // RXBYTES bit7: RX FIFO overflowed (sticky)
    output logic        timeout,       // gdo2 rose but never fell (sticky)
    output logic [7:0]  rxbytes,       // what RXBYTES reported
    output logic [7:0]  len_byte,      // first FIFO byte = payload length
    output logic [7:0]  start_index,   // payload byte 0
    output logic [7:0]  count,         // payload byte 1
    output logic [7:0]  rssi,          // status byte 1
    output logic [6:0]  lqi,           // status byte 2, bits 6:0
    output logic [15:0] rx_count,      // packets that passed pkt_ok
    output logic [15:0] err_count,     // packets that did not
    output logic [15:0] pt_writes      // total points committed to PointRam
);
    import cc1101_pkg::*;

    localparam int TO_CYCLES = (CLK_HZ / 1000) * END_TIMEOUT_MS;
    localparam logic [7:0] MAX_PTS8 = MAX_POINTS[7:0];

    typedef enum logic [4:0] {
        R_IDLE,
        R_SIDLE_I, R_SIDLE_W,
        R_FRX_I,   R_FRX_W,
        R_SRX_I,   R_SRX_W,
        R_LISTEN,
        R_END,
        R_RXB_I,   R_RXB_W,
        R_RXB2_I,  R_RXB2_W,
        R_FIFO_I,  R_FIFO_W,
        R_CHECK,
        R_STORE,
        R_DONE
    } rst_t;
    rst_t rstate;

    logic [31:0] tmr;
    logic [7:0]  rxb_prev;    // previous RXBYTES sample, for the agree-twice test
    logic [7:0]  n_read;      // clamped byte count actually clocked out
    logic        gdo2_d;      // for edge detection
    logic [7:0]  st_i;        // replay index during R_STORE

    // Payload staging. Split into two arrays so a point's x and y can be read
    // in the SAME cycle during the replay -- one array would need two read
    // ports or two cycles per point.
    logic [7:0] stage_x [0:MAX_POINTS-1];
    logic [7:0] stage_y [0:MAX_POINTS-1];

    assign rx_len = n_read;

    // FIFO layout by rx_idx:  0 = len, 1 = start_index, 2 = count,
    // So we ignore the first 3 idx. Then since theres x and y, we divide by 2
    wire [7:0] pt_k  = (rx_idx - 8'd3) >>1; // coordinate count
    wire       is_x  = rx_idx[0];      // 3,5,7,... are x; 4,6,8,... are y

    // Format verdict. Every field is cross-checked against every other, because
    // a CRC-good packet can still be malformed if the SENDER is wrong -- and
    // that failure looks identical to an RF problem unless it is separated out.
    // len_byte must equal 2 + 2*count, or the packet is internally inconsistent.
    wire [8:0] end_index = {1'b0, start_index} + {1'b0, count};
    wire fmt_ok_w = (count != 8'd0)
                 && (count <= MAX_PTS8)
                 && (len_byte == (8'd2 + {count[6:0], 1'b0}))
                 && (end_index <= N_POINTS[8:0]);

    always_ff @(posedge clk) begin
        if (rst) begin
            rstate    <= R_IDLE;
            cmd_valid <= 1'b0; cmd <= 3'd0; cmd_addr <= 8'h00; cmd_data <= 8'h00;
            busy      <= 1'b0; done <= 1'b0;
            pkt_ok    <= 1'b0; crc_ok <= 1'b0; fmt_ok <= 1'b0; bad_fmt <= 1'b0;
            overflow  <= 1'b0; timeout <= 1'b0;
            rxbytes   <= 8'h00; len_byte <= 8'h00;
            start_index <= 8'h00; count <= 8'h00;
            rssi      <= 8'h00; lqi <= 7'h00;
            rx_count  <= 16'd0; err_count <= 16'd0; pt_writes <= 16'd0;
            tmr       <= 32'd0; rxb_prev <= 8'h00; n_read <= 8'h00;
            gdo2_d    <= 1'b0; st_i <= 8'd0;
            pt_we     <= 1'b0; pt_addr <= 8'h00; pt_data <= 18'd0;
        end else begin
            cmd_valid <= 1'b0;      // one-cycle-pulse defaults
            done      <= 1'b0;
            pt_we     <= 1'b0;
            gdo2_d    <= gdo2;

            // ---- latch FIFO bytes as the driver hands them up ----
            // The status bytes are tested BEFORE the payload branches, so a
            // short or malformed packet can never have a payload byte
            // overwrite the CRC flag.
            if (rx_strobe) begin
                if (rx_idx == 8'd0) begin // First: length
                    len_byte <= rx_byte;
                end else if (rx_idx == n_read - 8'd1) begin
                    lqi    <= rx_byte[6:0];
                    crc_ok <= rx_byte[7];
                end else if (rx_idx == n_read - 8'd2) begin
                    rssi <= rx_byte;
                end else if (rx_idx == 8'd1) begin // Second: starting index
                    start_index <= rx_byte;
                end else if (rx_idx == 8'd2) begin // Third: How many xy coords there are
                    count <= rx_byte;
                end else if (pt_k < MAX_PTS8) begin // Coordinates
                    if (is_x) stage_x[pt_k[4:0]] <= rx_byte;
                    else      stage_y[pt_k[4:0]] <= rx_byte;
                end
            end

            case (rstate)
                // Wait until the radio is configured and ConfigSeq is done with
                // the chip, then arm the receiver.
                R_IDLE: if (enable && !cfg_busy) begin
                    busy   <= 1'b1;
                    rstate <= R_SIDLE_I;
                end

                // ---- park, flush, then enter RX ----
                R_SIDLE_I: begin
                    cmd_valid <= 1'b1; cmd <= CMD_STROBE; cmd_addr <= SIDLE;
                    rstate <= R_SIDLE_W;
                end
                R_SIDLE_W: if (drv_done) rstate <= R_FRX_I;

                // A leftover partial packet corrupts the next good one, so the
                // flush is not optional -- it runs on every re-arm.
                R_FRX_I: begin
                    cmd_valid <= 1'b1; cmd <= CMD_STROBE; cmd_addr <= SFRX;
                    rstate <= R_FRX_W;
                end
                R_FRX_W: if (drv_done) rstate <= R_SRX_I;

                R_SRX_I: begin
                    cmd_valid <= 1'b1; cmd <= CMD_STROBE; cmd_addr <= SRX;
                    rstate <= R_SRX_W;
                end
                R_SRX_W: if (drv_done) begin
                    pkt_ok <= 1'b0;     // verdict applies to the packet we are about to get
                    rstate <= R_LISTEN;
                end

                // ---- listening: hold no bus, wait for sync ----
                R_LISTEN: begin
                    busy <= 1'b0;
                    if (cfg_busy) begin
                        // config re-run takes the chip; stand down and re-arm after
                        rstate <= R_IDLE;
                    end else if (gdo2 && !gdo2_d) begin
                        busy   <= 1'b1;
                        tmr    <= 32'd0;
                        rstate <= R_END;
                    end
                end

                // ---- gdo2 fall: the packet is complete and in the FIFO ----
                R_END: begin
                    if (!gdo2) begin
                        rstate <= R_RXB_I;
                    end else if (tmr == TO_CYCLES-1) begin
                        timeout   <= 1'b1;
                        err_count <= err_count + 16'd1;
                        rstate    <= R_DONE;
                    end else tmr <= tmr + 32'd1;
                end

                // ---- RXBYTES, twice, until two consecutive reads agree ----
                R_RXB_I: begin
                    cmd_valid <= 1'b1; cmd <= CMD_READ_REG; cmd_addr <= RXBYTES_ADDR;
                    rstate <= R_RXB_W;
                end
                R_RXB_W: if (drv_done) begin
                    rxb_prev <= rd_data;
                    rstate   <= R_RXB2_I;
                end

                R_RXB2_I: begin
                    cmd_valid <= 1'b1; cmd <= CMD_READ_REG; cmd_addr <= RXBYTES_ADDR;
                    rstate <= R_RXB2_W;
                end
                R_RXB2_W: if (drv_done) begin
                    if (rd_data == rxb_prev) begin
                        rxbytes <= rd_data;
                        if (rd_data[7]) begin
                            // RXFIFO_OVERFLOW -- the FIFO is unusable, just flush
                            overflow  <= 1'b1;
                            err_count <= err_count + 16'd1;
                            rstate    <= R_DONE;
                        end else if (rd_data[6:0] == 7'd0) begin
                            // GDO2 pulsed but nothing landed (packet discarded)
                            err_count <= err_count + 16'd1;
                            rstate    <= R_DONE;
                        end else begin
                            n_read <= (rd_data[6:0] > RX_MAX[7:0]) ? RX_MAX[7:0]
                                                                  : rd_data[6:0];
                            rstate <= R_FIFO_I;
                        end
                    end else begin
                        // disagreed -- keep the newer sample and compare again
                        rxb_prev <= rd_data;
                        rstate   <= R_RXB2_I;
                    end
                end

                // ---- burst-read the FIFO ----
                R_FIFO_I: begin
                    cmd_valid <= 1'b1; cmd <= CMD_READ_FIFO;
                    rstate <= R_FIFO_W;
                end
                R_FIFO_W: if (drv_done) rstate <= R_CHECK;

                // One cycle for the last rx_strobe capture to settle before the
                // verdict is taken.
                R_CHECK: begin
                    fmt_ok <= fmt_ok_w;
                    if (crc_ok && fmt_ok_w) begin
                        pkt_ok   <= 1'b1;
                        rx_count <= rx_count + 16'd1;
                        st_i     <= 8'd0;
                        rstate   <= R_STORE;
                    end else begin
                        // Separating these two is the point of bad_fmt: a CRC
                        // failure is an RF problem, a format failure with a good
                        // CRC is a SENDER problem. They need different fixes and
                        // otherwise look identical from the ILA.
                        if (crc_ok) bad_fmt <= 1'b1;
                        err_count <= err_count + 16'd1;
                        rstate    <= R_DONE;
                    end
                end

                // ---- commit: replay the staged payload into PointRam ----
                // One point per cycle, so a full 30-point packet costs 30 cycles
                // (240 ns). The radio cannot deliver the next packet for at
                // least ~3 ms, so this is free.
                R_STORE: begin
                    if (st_i < count) begin
                        pt_we     <= 1'b1;
                        pt_addr   <= start_index + st_i;
                        pt_data   <= {stage_x[st_i[4:0]], stage_y[st_i[4:0]],
                                      DWELL_DEFAULT};
                        st_i      <= st_i + 8'd1;
                        pt_writes <= pt_writes + 16'd1;
                    end else begin
                        rstate <= R_DONE;
                    end
                end

                // Always flush before going back to RX, then re-arm.
                R_DONE: begin
                    done   <= 1'b1;
                    rstate <= R_IDLE;
                end

                default: rstate <= R_IDLE;
            endcase
        end
    end
endmodule
