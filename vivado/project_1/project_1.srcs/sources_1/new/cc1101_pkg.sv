`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// cc1101_pkg  --  shared CC1101 constants
//
// Single source of truth for the driver command opcodes, CC1101 command strobes,
// and common status-register addresses. Import into any module that needs them:
//     import cc1101_pkg::*;
// (Compile this file before the modules that use it -- Vivado handles the order
//  automatically in project mode.)
//------------------------------------------------------------------------------
package cc1101_pkg;

    // CC1101Driver command opcodes (the `cmd` field)
    localparam logic [2:0]
        CMD_RESET      = 3'd0,
        CMD_WRITE_REG  = 3'd1,
        CMD_READ_REG   = 3'd2,
        CMD_STROBE     = 3'd3,
        CMD_WRITE_FIFO = 3'd4,   // reserved (milestone 6)
        CMD_READ_FIFO  = 3'd5;   // reserved (milestone 7)

    // CC1101 command strobes (single-byte fire-and-forget commands)
    localparam logic [7:0]
        SRES  = 8'h30,   // reset
        SRX   = 8'h34,   // enter RX
        STX   = 8'h35,   // enter TX
        SIDLE = 8'h36,   // go idle
        SFRX  = 8'h3A,   // flush RX FIFO
        SFTX  = 8'h3B,   // flush TX FIFO
        SNOP  = 8'h3D;   // no-op (just fetch the status byte)

    // Common status-register addresses (driver adds the burst bit on read).
    // NOTE these collide with the strobes above on purpose -- same address, the
    // burst bit is what makes it a read instead of an action. e.g. 0x3B is SFTX
    // as a strobe and RXBYTES as a status read.
    localparam logic [7:0]
        PARTNUM_ADDR   = 8'h30,
        VERSION_ADDR   = 8'h31,
        MARCSTATE_ADDR = 8'h35,   // chip's own state machine -- best TX/RX debug net
        TXBYTES_ADDR   = 8'h3A,
        RXBYTES_ADDR   = 8'h3B;

    // TX FIFO burst-write header: R/W=0, burst=1, addr=0x3F
    localparam logic [7:0] TXFIFO_BURST = 8'h7F;

    // MARCSTATE values worth recognising (bits [4:0])
    localparam logic [4:0]
        MARC_IDLE = 5'h01,
        MARC_RX   = 5'h0D,
        MARC_TX   = 5'h13;

endpackage
