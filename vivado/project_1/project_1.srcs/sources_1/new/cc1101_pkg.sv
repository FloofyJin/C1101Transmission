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
        SFTX  = 8'h3B;   // flush TX FIFO

    // Common status-register addresses (driver adds the burst bit on read)
    localparam logic [7:0]
        PARTNUM_ADDR = 8'h30,
        VERSION_ADDR = 8'h31;

endpackage
