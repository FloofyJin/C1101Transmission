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
//     [len] [idx_hi][idx_lo][count] [x0][y0] ... [RSSI] [LQI | CRC_OK<<7]
//      ^^^   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^   ^^^^^^^^^^^^^^^^^^^^^^
//      chip            len bytes of payload             chip-appended
//
// Note the sender's 0x7F burst-write header is NOT here and never was -- that
// byte only ever existed on the transmitter's own SPI bus. Likewise this side's
// 0xFF read header is consumed by the driver.
//
// So a 28-point packet is 1 + 59 + 2 = 62 bytes, which is why RX_MAX is 62.
//
// ---- why 28 points, and why the RX FIFO is the binding limit ----
// Both FIFOs are 64 bytes, but the RECEIVING chip appends two status bytes
// into its own FIFO (datasheet 15.3.3), so:
//
//     TX:  1 len + payload            <= 64  ->  payload <= 63
//     RX:  1 len + payload + 2 status <= 64  ->  payload <= 61   <- binds
//
// And the count must be EVEN, because scanline fill sends points in PAIRS. An
// odd count splits a span across a packet boundary; lose that packet and one
// endpoint comes from the new frame while its partner is two frames stale,
// which draws a line straight across the image. An even count means a lost
// packet costs a whole span -- one missing row, nearly invisible.
//
// ---- double buffering ----
// count[7] marks the last packet of a frame. RxSeq raises frame_ready and
// publishes frame_points; topRF swaps banks at the SCANOUT's wrap, never
// mid-pass (that would move the tear rather than remove it), then pulses
// frame_ack.
//
// If the end-of-frame packet is lost, no swap happens and the previous frame
// stays up one extra period -- a far better failure than half a frame.
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
    parameter int MAX_POINTS = 28,          // EVEN (spans are pairs), and <= (61-3)/2
    parameter int N_POINTS   = 1024,        // PointRam depth PER BANK
    parameter int RX_MAX     = 62,          // 1 len + 59 payload + 2 status
    parameter int END_TIMEOUT_MS = 200,     // gdo2 stuck high -> give up
    // Blanking is DERIVED in ScanoutEngine (it needs both endpoints of a
    // segment, which RxSeq never has at once). These two bits are reserved as
    // a future explicit override and are written as zero.
    parameter logic [1:0] SPARE_DEFAULT = 2'b00
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
    output logic             pt_we,
    output logic [IDX_W:0]   pt_addr,     // {bank, index}
    output logic [17:0]      pt_data,     // {x, y, spare[1:0]}

    // ---- double buffering ----
    output logic             frame_ready,   // level: a complete frame has landed
    // IDX_W+1 bits, not IDX_W: a frame using every entry has 1024 points, and
    // 1024 does not fit in 10 bits. Truncating it to 0 would silently blank
    // the display on exactly the largest frame.
    output logic [IDX_W:0]   frame_points,  // max(start_index+count) of that frame
    input  logic             frame_ack,     // topRF took it; clears frame_ready

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
    output logic [15:0] start_index,   // payload bytes 0-1, big-endian
    output logic [7:0]  count,         // payload byte 2 (bit 7 = end of frame)
    output logic [7:0]  rssi,          // status byte 1
    output logic [6:0]  lqi,           // status byte 2, bits 6:0
    output logic [15:0] rx_count,      // packets that passed pkt_ok
    output logic [15:0] err_count,     // packets that did not
    output logic [15:0] pt_writes      // total points committed to PointRam
);
    import cc1101_pkg::*;

    localparam int IDX_W = $clog2(N_POINTS); // 10

    localparam int TO_CYCLES = (CLK_HZ / 1000) * END_TIMEOUT_MS;
    localparam logic [7:0] MAX_PTS8 = MAX_POINTS[7:0];
    logic wr_bank;                       // RxSeq writes ~scanout's bank

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
    logic [IDX_W-1:0] st_i;   // replay index during R_STORE (address-width)

    // Payload staging. Split into two arrays so a point's x and y can be read
    // in the SAME cycle during the replay -- one array would need two read
    // ports or two cycles per point.
    logic [7:0] stage_x [0:MAX_POINTS-1];
    logic [7:0] stage_y [0:MAX_POINTS-1];

    assign rx_len = n_read;

    // FIFO layout by rx_idx:
    //   0 = len, 1 = idx_hi, 2 = idx_lo, 3 = count, 4.. = coordinate pairs
    //
    // Coordinates begin at rx_idx 4, two bytes per point:
    //     rx_idx  4  5   6  7   8  9
    //     point   0  0   1  1   2  2
    //     field   x  y   x  y   x  y
    //
    // Note this flipped when the index went from 1 to 2 bytes -- x used to sit
    // on ODD rx_idx and now sits on EVEN.
    wire [7:0] pt_k  = (rx_idx - 8'd4) >> 1;
    wire       is_x  = ~rx_idx[0];     // 4,6,8,... are x; 5,7,9,... are y

    // Format verdict. Every field is cross-checked against every other, because
    // a CRC-good packet can still be malformed if the SENDER is wrong -- and
    // that failure looks identical to an RF problem unless it is separated out.
    // len_byte must equal 3 + 2*count (2-byte start_index + 1-byte count, then
    // two bytes per point), or the packet is internally inconsistent.
    // count[7] is the end-of-frame flag; the point count is count[6:0].
    wire [7:0]  n_pts     = {1'b0, count[6:0]};
    wire        eof_flag  = count[7];
    wire [16:0] end_index = {1'b0, start_index} + {9'd0, n_pts};

    wire fmt_ok_w = (n_pts != 8'd0)
                 && (n_pts <= MAX_PTS8)
                 && (n_pts[0] == 1'b0)                       // spans are PAIRS
                 && (len_byte == (8'd3 + {n_pts[6:0], 1'b0}))
                 && (end_index <= {1'b0, N_POINTS[15:0]});

    always_ff @(posedge clk) begin
        if (rst) begin
            rstate    <= R_IDLE;
            cmd_valid <= 1'b0; cmd <= 3'd0; cmd_addr <= 8'h00; cmd_data <= 8'h00;
            busy      <= 1'b0; done <= 1'b0;
            pkt_ok    <= 1'b0; crc_ok <= 1'b0; fmt_ok <= 1'b0; bad_fmt <= 1'b0;
            overflow  <= 1'b0; timeout <= 1'b0;
            rxbytes   <= 8'h00; len_byte <= 8'h00;
            start_index <= 16'h0000; count <= 8'h00;
            wr_bank   <= 1'b1;          // scanout starts on bank 0
            frame_ready  <= 1'b0;
            frame_points <= '0;
            rssi      <= 8'h00; lqi <= 7'h00;
            rx_count  <= 16'd0; err_count <= 16'd0; pt_writes <= 16'd0;
            tmr       <= 32'd0; rxb_prev <= 8'h00; n_read <= 8'h00;
            gdo2_d    <= 1'b0; st_i <= '0;
            pt_we     <= 1'b0; pt_addr <= 8'h00; pt_data <= 18'd0;
        end else begin
            cmd_valid <= 1'b0;      // one-cycle-pulse defaults
            done      <= 1'b0;
            pt_we     <= 1'b0;
            gdo2_d    <= gdo2;

            // topRF took the frame: flip to the other bank and start measuring
            // the next frame's extent from scratch.
            if (frame_ack) begin
                frame_ready  <= 1'b0;
                wr_bank      <= ~wr_bank;
                frame_points <= '0;
            end

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
                end else if (rx_idx == 8'd1) begin      // index, big-endian
                    start_index[15:8] <= rx_byte;
                end else if (rx_idx == 8'd2) begin
                    start_index[7:0]  <= rx_byte;
                end else if (rx_idx == 8'd3) begin      // count, bit 7 = end of frame
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

                // ---- RXBYTES, twice, until two consecutive reads agree. This reads the size of the data received ----
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
                    if (rd_data == rxb_prev) begin // Both reads are the same. GOOD
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
                        // TODO: Could add a counter to return back to IDLE if it keeps mismatching
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
                        st_i     <= '0;
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
                    if (st_i < {{(IDX_W-8){1'b0}}, n_pts}) begin
                        pt_we     <= 1'b1;
                        // Written into the bank the scanout is NOT reading.
                        pt_addr   <= {wr_bank, (start_index[IDX_W-1:0] + st_i)};
                        pt_data   <= {stage_x[st_i[4:0]], stage_y[st_i[4:0]],
                                      SPARE_DEFAULT};
                        st_i      <= st_i + 1'b1;
                        pt_writes <= pt_writes + 16'd1;
                    end else begin
                        // end_index is the number of points we should have received
                        // after this payload. count_begin + # points received
                        if (end_index[IDX_W:0] > frame_points)
                            frame_points <= end_index[IDX_W:0];
                        // Last packet of the frame -- offer it to the display.
                        if (eof_flag) frame_ready <= 1'b1;
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
