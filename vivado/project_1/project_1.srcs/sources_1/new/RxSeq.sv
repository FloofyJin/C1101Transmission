`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// RxSeq  --  Milestone 7: receive a packet on radio B and check it
//
// The mirror of TxSeq. Drives one CC1101Driver's command interface:
//
//   SIDLE -> SFRX -> SRX -> listen -> gdo2 rise -> gdo2 fall
//         -> read RXBYTES (twice, must agree) -> burst-read the FIFO
//         -> check CRC + payload -> SFRX -> re-arm
//
// With PKTCTRL0 = 0x05 (variable length) and PKTCTRL1 = 0x04 (append status),
// the RX FIFO holds:
//
//     [len] [payload 0..len-1] [RSSI] [LQI | CRC_OK<<7]
//
// so a 2-byte payload reads back as 5 bytes total. `pkt_ok` is the milestone-7
// verdict: CRC good AND the payload is exactly the {0xAA,0x55} TxSeq sends.
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
    parameter int RX_MAX     = 16,          // clamp: never clock out more than this
    parameter int END_TIMEOUT_MS = 200,     // gdo2 stuck high -> give up
    // The payload TxSeq sends. Kept as parameters so the check is not buried
    // in the FSM and M8 can repoint it at PacketGenerator.
    // Payload is {0xAA, 0x55, seq}. Only the two markers are checked -- the
    // third byte is the sequence number and changes every packet by design.
    parameter logic [7:0] EXP_LEN = 8'd3,
    parameter logic [7:0] EXP_B0  = 8'hAA,
    parameter logic [7:0] EXP_B1  = 8'h55
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

    // status / ILA
    output logic        busy,
    output logic        done,          // 1-cycle pulse per completed reception
    output logic        pkt_ok,        // CRC good AND payload matched -- M7 PASS
    output logic        crc_ok,        // hardware CRC flag from the status byte
    output logic        overflow,      // RXBYTES bit7: RX FIFO overflowed (sticky)
    output logic        timeout,       // gdo2 rose but never fell (sticky)
    output logic [7:0]  rxbytes,       // what RXBYTES reported
    output logic [7:0]  len_byte,      // first FIFO byte = payload length
    output logic [7:0]  b0, b1,        // marker bytes, expect 0xAA / 0x55
    output logic [7:0]  b2,            // sequence number -- gaps here = lost packets
    output logic [7:0]  last_seq,      // previous packet's seq, for spotting gaps
    output logic        seq_gap,       // sticky: b2 != last_seq+1 on a good packet
    output logic [7:0]  rssi,          // status byte 1
    output logic [6:0]  lqi,           // status byte 2, bits 6:0
    output logic [15:0] rx_count,      // receptions that passed pkt_ok
    output logic [15:0] err_count      // receptions that did not
);
    import cc1101_pkg::*;

    localparam int TO_CYCLES = (CLK_HZ / 1000) * END_TIMEOUT_MS;

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
        R_DONE
    } rst_t;
    rst_t rstate;

    logic [31:0] tmr;
    logic [7:0]  rxb_prev;    // previous RXBYTES sample, for the agree-twice test
    logic [7:0]  n_read;      // clamped byte count actually clocked out
    logic        gdo2_d;      // for edge detection

    assign rx_len = n_read;

    // Payload verdict, evaluated once the whole burst has landed.
    wire payload_match = (len_byte == EXP_LEN) && (b0 == EXP_B0) && (b1 == EXP_B1);

    always_ff @(posedge clk) begin
        if (rst) begin
            rstate    <= R_IDLE;
            cmd_valid <= 1'b0; cmd <= 3'd0; cmd_addr <= 8'h00; cmd_data <= 8'h00;
            busy      <= 1'b0; done <= 1'b0;
            pkt_ok    <= 1'b0; crc_ok <= 1'b0;
            overflow  <= 1'b0; timeout <= 1'b0;
            rxbytes   <= 8'h00; len_byte <= 8'h00;
            b0        <= 8'h00; b1 <= 8'h00; b2 <= 8'h00;
            last_seq  <= 8'h00; seq_gap <= 1'b0;
            rssi      <= 8'h00; lqi <= 7'h00;
            rx_count  <= 16'd0; err_count <= 16'd0;
            tmr       <= 32'd0; rxb_prev <= 8'h00; n_read <= 8'h00;
            gdo2_d    <= 1'b0;
        end else begin
            cmd_valid <= 1'b0;      // one-cycle-pulse defaults
            done      <= 1'b0;
            gdo2_d    <= gdo2;

            // Latch FIFO bytes as the driver hands them up. Indices are fixed by
            // the frame layout: 0 is the length byte, 1.. are payload, and the
            // last two are always the appended status bytes.
            if (rx_strobe) begin // This means last communication using Driver was a read
                if (rx_idx == 8'd0) len_byte <= rx_byte;
                if (rx_idx == 8'd1) b0       <= rx_byte;
                if (rx_idx == 8'd2) b1       <= rx_byte;
                if (rx_idx == 8'd3) b2       <= rx_byte;
                if (rx_idx == n_read - 8'd2) rssi <= rx_byte;
                if (rx_idx == n_read - 8'd1) begin
                    lqi    <= rx_byte[6:0];
                    crc_ok <= rx_byte[7];
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
                    if (crc_ok && payload_match) begin
                        pkt_ok   <= 1'b1;
                        rx_count <= rx_count + 16'd1;
                        // Loss detection. Only meaningful from the second good
                        // packet on -- there is nothing to compare the first to.
                        // 8-bit wrap is fine: 0xFF -> 0x00 is still +1.
                        if (rx_count != 16'd0 && b2 != (last_seq + 8'd1))
                            seq_gap <= 1'b1;
                        last_seq <= b2;
                    end else begin
                        err_count <= err_count + 16'd1;
                    end
                    rstate <= R_DONE;
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
