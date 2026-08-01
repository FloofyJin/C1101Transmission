`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// TxSeq  --  Milestone 6: transmit a hardcoded packet and watch gdo2
//
// Drives one CC1101Driver's command interface, exactly like ConfigSeq does, but
// with the transmit mission instead of the config mission:
//
//   SIDLE -> SFTX -> write TX FIFO (len + payload) -> read TXBYTES
//         -> STX -> read MARCSTATE -> wait gdo2 rise -> wait gdo2 fall -> SIDLE
//
// The two register reads are pure diagnostics and are the point of this module.
// If the packet never goes out you need to know WHICH step failed:
//   txbytes   should read 0x03  (1 length byte + 2 payload bytes)
//             0x00 -> the FIFO write never landed (SPI/burst problem)
//             bit7 set -> TX FIFO underflow
//   marcstate should read 0x13 (TX), or 0x08-0x0C if caught mid-calibration.
//             0x01 (IDLE) means STX did not take -- the chip never left idle.
//
// gdo2 with IOCFG0 = 0x06 rises when the sync word has gone out and falls at
// the end of the packet, so rise-then-fall is the chip telling you the whole
// frame was transmitted. Both waits are bounded by TIMEOUT_MS: a missing gdo2
// must not wedge the FSM, or the ILA shows you nothing at all.
//
// PacketGenerator is deliberately NOT used yet -- milestone 6 wants a KNOWN
// constant payload so a wrong byte on the receiver is unambiguous.
//------------------------------------------------------------------------------
module TxSeq #(
    parameter int CLK_HZ     = 125_000_000,
    parameter int TIMEOUT_MS = 200,          // give up waiting on gdo2
    parameter int REPEAT_MS  = 200           // gap between packets in auto mode
)(
    input  logic       clk,
    input  logic       rst,
    input  logic       start,          // 1-cycle pulse: send one packet
    input  logic       auto_tx,        // level: keep sending every REPEAT_MS

    // to CC1101Driver command interface
    output logic       cmd_valid,
    output logic [2:0] cmd,
    output logic [7:0] cmd_addr,
    output logic [7:0] cmd_data,
    input  logic       drv_done,
    input  logic [7:0] rd_data,

    // payload source for CMD_WRITE_FIFO (driver drives pl_idx, we answer)
    input  logic [7:0] pl_idx,
    output logic [7:0] pl_byte,
    output logic [7:0] pl_len,

    // gdo2 from the chip -- MUST already be synchronized to clk
    input  logic       gdo2,

    // status / ILA
    output logic       busy,
    output logic       done,           // 1-cycle pulse per packet attempt
    output logic       sync_seen,      // gdo2 rose  -> sync word went on air
    output logic       sent_ok,        // rose AND fell -> whole packet sent
    output logic       timeout,        // gdo2 never moved (latched, sticky)
    output logic [7:0] txbytes,        // TXBYTES read back after the FIFO write
    output logic [7:0] marcstate,      // MARCSTATE read back after STX
    output logic [15:0] pkt_count      // packets that completed with sent_ok
);
    import cc1101_pkg::*;

    localparam int TO_CYCLES  = (CLK_HZ / 1000) * TIMEOUT_MS;
    localparam int GAP_CYCLES = (CLK_HZ / 1000) * REPEAT_MS;

    // ---- the packet ----
    // Flat constant, byte 0 leftmost (same reason as ConfigSeq's ROM).
    localparam int PL_N = 2;
    localparam logic [PL_N*8-1:0] PAYLOAD = {8'hAA, 8'h55};

    assign pl_len  = PL_N[7:0];
    assign pl_byte = (pl_idx < PL_N[7:0]) ? PAYLOAD[(PL_N-1-pl_idx)*8 +: 8]
                                          : 8'h00;

    typedef enum logic [4:0] {
        T_IDLE,
        T_SIDLE_I, T_SIDLE_W,
        T_FTX_I,   T_FTX_W,
        T_FIFO_I,  T_FIFO_W,
        T_TXB_I,   T_TXB_W,
        T_STX_I,   T_STX_W,
        T_MARC_I,  T_MARC_W,
        T_RISE,    T_FALL,
        T_END_I,   T_END_W,
        T_DONE,    T_GAP
    } tst_t;
    tst_t tstate;

    logic [31:0] tmr;
    logic        gdo2_hi_latch;   // gdo2 went high at some point since STX

    always_ff @(posedge clk) begin
        if (rst) begin
            tstate    <= T_IDLE;
            cmd_valid <= 1'b0; cmd <= 3'd0; cmd_addr <= 8'h00; cmd_data <= 8'h00;
            busy      <= 1'b0; done <= 1'b0;
            sync_seen <= 1'b0; sent_ok <= 1'b0; timeout <= 1'b0;
            txbytes   <= 8'h00; marcstate <= 8'h00;
            pkt_count <= 16'd0;
            tmr       <= 32'd0;
            gdo2_hi_latch <= 1'b0;
        end else begin
            cmd_valid <= 1'b0;      // one-cycle-pulse defaults
            done      <= 1'b0;

            // Latch gdo2 high from the moment we strobe STX. At high data rates
            // the whole pulse can fit inside one SPI read, and a pulse you did
            // not catch live still proves the packet went out.
            if (tstate != T_IDLE && tstate != T_GAP && gdo2) gdo2_hi_latch <= 1'b1;

            case (tstate)
                T_IDLE: if (start || auto_tx) begin
                    busy      <= 1'b1;
                    sync_seen <= 1'b0;
                    sent_ok   <= 1'b0;
                    gdo2_hi_latch <= 1'b0;
                    tstate    <= T_SIDLE_I;
                end

                // ---- park the chip in a known state ----
                T_SIDLE_I: begin
                    cmd_valid <= 1'b1; cmd <= CMD_STROBE; cmd_addr <= SIDLE;
                    tstate <= T_SIDLE_W;
                end
                T_SIDLE_W: if (drv_done) tstate <= T_FTX_I;

                // ---- flush TX FIFO: a leftover byte corrupts this packet ----
                T_FTX_I: begin
                    cmd_valid <= 1'b1; cmd <= CMD_STROBE; cmd_addr <= SFTX;
                    tstate <= T_FTX_W;
                end
                T_FTX_W: if (drv_done) tstate <= T_FIFO_I;

                // ---- length byte + payload, one CSn window (burst) ----
                T_FIFO_I: begin
                    cmd_valid <= 1'b1; cmd <= CMD_WRITE_FIFO;
                    tstate <= T_FIFO_W;
                end
                T_FIFO_W: if (drv_done) tstate <= T_TXB_I;

                // ---- diagnostic: did the bytes actually reach the FIFO? ----
                T_TXB_I: begin
                    cmd_valid <= 1'b1; cmd <= CMD_READ_REG; cmd_addr <= TXBYTES_ADDR;
                    tstate <= T_TXB_W;
                end
                T_TXB_W: if (drv_done) begin
                    txbytes <= rd_data;
                    tstate  <= T_STX_I;
                end

                // ---- go ----
                T_STX_I: begin
                    cmd_valid <= 1'b1; cmd <= CMD_STROBE; cmd_addr <= STX;
                    tstate <= T_STX_W;
                end
                T_STX_W: if (drv_done) tstate <= T_MARC_I;

                // ---- diagnostic: did the chip actually leave IDLE? ----
                T_MARC_I: begin
                    cmd_valid <= 1'b1; cmd <= CMD_READ_REG; cmd_addr <= MARCSTATE_ADDR;
                    tstate <= T_MARC_W;
                end
                T_MARC_W: if (drv_done) begin
                    marcstate <= rd_data;
                    tmr       <= 32'd0;
                    tstate    <= T_RISE;
                end

                // ---- gdo2 rise: sync word is on the air ----
                T_RISE: begin
                    if (gdo2 || gdo2_hi_latch) begin
                        sync_seen <= 1'b1;
                        tmr       <= 32'd0;
                        tstate    <= T_FALL;
                    end else if (tmr == TO_CYCLES-1) begin
                        timeout <= 1'b1;
                        tstate  <= T_END_I;
                    end else tmr <= tmr + 32'd1;
                end

                // ---- gdo2 fall: end of packet ----
                T_FALL: begin
                    if (!gdo2) begin
                        sent_ok   <= 1'b1;
                        pkt_count <= pkt_count + 16'd1;
                        tstate    <= T_END_I;
                    end else if (tmr == TO_CYCLES-1) begin
                        timeout <= 1'b1;
                        tstate  <= T_END_I;
                    end else tmr <= tmr + 32'd1;
                end

                // ---- back to idle either way ----
                T_END_I: begin
                    cmd_valid <= 1'b1; cmd <= CMD_STROBE; cmd_addr <= SIDLE;
                    tstate <= T_END_W;
                end
                T_END_W: if (drv_done) tstate <= T_DONE;

                T_DONE: begin
                    done <= 1'b1;
                    tmr  <= 32'd0;
                    if (auto_tx) tstate <= T_GAP;
                    else begin busy <= 1'b0; tstate <= T_IDLE; end
                end

                T_GAP: begin
                    if (tmr == GAP_CYCLES-1) begin
                        if (auto_tx) begin
                            sync_seen <= 1'b0; sent_ok <= 1'b0;
                            gdo2_hi_latch <= 1'b0;
                            tstate <= T_SIDLE_I;
                        end else begin
                            busy <= 1'b0; tstate <= T_IDLE;
                        end
                    end else tmr <= tmr + 32'd1;
                end

                default: tstate <= T_IDLE;
            endcase
        end
    end
endmodule
