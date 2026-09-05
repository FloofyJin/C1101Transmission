`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// CC1101Driver  --  command-driven chip primitives
//
// Issue ONE command per `cmd_valid` pulse; the driver runs it and pulses `done`
// when finished. It owns the chip's SPI conventions (header-byte format, the
// burst bit for status registers, holding CSn across header+data). It does NOT
// know about payloads or missions -- a controller above it sequences commands
// and supplies content. This is the seam that later bridges cleanly to the PS
// over AXI (command = AXI-Lite reg, payload = AXI-Stream).
//
//   CMD_RESET      SRES strobe + settle
//   CMD_WRITE_REG  write cmd_data to register cmd_addr
//   CMD_READ_REG   read register cmd_addr -> rd_data (burst bit auto-added for
//                  status regs 0x30-0x3D)
//   CMD_STROBE     issue the strobe byte in cmd_addr (SRX/STX/SIDLE/SFRX/SFTX...)
//   CMD_WRITE_FIFO burst-write the TX FIFO: header 0x7F, then the length byte,
//                  then pl_len payload bytes pulled from the controller
//   CMD_READ_FIFO  -- reserved (milestone 7)
//
// Milestone-6 scope: RESET / WRITE_REG / READ_REG / STROBE / WRITE_FIFO.
//
// Payload interface (CMD_WRITE_FIFO only) -- deliberately NOT a valid/ready
// handshake. The driver drives `pl_idx` and the controller answers
// combinationally on `pl_byte`, like addressing a small ROM. No handshake means
// no timing race to get wrong, and the driver still never sees the content as
// anything but opaque bytes.
//------------------------------------------------------------------------------
module CC1101Driver #(
    parameter int CLK_HZ     = 125_000_000,
    parameter int POSTRES_US = 1_000        // settle after SRES (1 ms)
)(
    input logic clk,
    input logic rst,

    input logic [2:0] cmd,
    input logic cmd_valid,
    input logic [7:0] cmd_addr,
    input logic [7:0] cmd_data,
    output logic [7:0] rd_data,

    // payload source for CMD_WRITE_FIFO (see note above)
    output logic [7:0] pl_idx,      // which payload byte the driver wants
    input  logic [7:0] pl_byte,     // must be combinational from pl_idx
    input  logic [7:0] pl_len,      // payload length -> becomes the CC1101 length byte

    // payload sink for CMD_READ_FIFO -- the mirror image of the write side.
    // The driver burst-reads rx_len bytes and hands each one up as it arrives;
    // the controller latches whichever ones it cares about. Again no handshake:
    // rx_strobe is a 1-cycle pulse and rx_idx/rx_byte are valid alongside it.
    input  logic [7:0] rx_len,      // how many bytes to burst-read
    output logic [7:0] rx_idx,      // index of the byte on rx_byte
    output logic [7:0] rx_byte,     // the byte just read out of the RX FIFO
    output logic       rx_strobe,   // 1-cycle: rx_idx/rx_byte are valid

    input logic [7:0] spi_rx, //data read from spi
    input logic spi_done,
    output logic spi_start,
    output logic[7:0] spi_tx, // data writing to spi
    output logic spi_hold, // tells spi if theres more bytes to write/read
    output logic busy,
    output logic done
);

import cc1101_pkg::*;
localparam POST_RST_CYCLE = (CLK_HZ /1_000_000) * POSTRES_US;

typedef enum logic [3:0] {
    IDLE, RST_S, COMMUNICATE_S, COMMUNICATING_S, RW_DATA_S, WR_DATA_S,
    FIFO_LEN_S, FIFO_DATA_S, FIFO_RD_S, DONE
} state_t;

state_t state;
state_t rtn_state;

logic is_receiving;
logic fifo_reading;      // capture spi_rx into rx_byte (RX FIFO data phase only)
logic [7:0] rx_ptr;      // next RX FIFO byte index to fetch

logic [7:0] saved_cmd_addr, saved_cmd_data;

logic [31:0] dly;

function automatic logic [7:0] rd_reader(input logic[7:0] addr);
    rd_reader =8'h80 | (((addr >= 8'h30) && (addr <= 8'h3D)) ? 8'h40 : 8'h00) | addr;
    /*
    * first part: 0'h80 means READ
    * second part: ternary is for burst bit. So if burst bit is 1, that means addr points to status registers.
    * Status registers are read-only info like PARTNUM, VERSION, MARCSTATE, RXBYTES, and RSSI. 
    * We want to use burst on if the address is between 0x30 and 0x3D
    * Addresses outside of that registers are command strobe like SRE, SRX, STX, SIDKE, and SFRX/SFTX.
    * strobe is an action
    * status register is a read
    * config write:         R/W=0  burst=0   addr=0x00-0x2E (BURST=0 means register access)(BURST=1 means read/write block, auto incrementing)
    * config read:          R/W=1  burst=0   addr=0x00-0x2E (BURST=0 means register access)(BURST=1 means read/write block, auto incrementing)
    * command strobe:       R/W=0  burst=0   addr=0x30-0x3D (BURST=0 meaning strobe)(BURST=1 means status register AKA read)
    * status register read: R/W=1  burst=1   addr=0x30-0x3D (BURST=0 meaning strobe)(BURST=1 means status register AKA read)
    * TX FIFO write:        R/W=0  burst=0/1 addr=0x3F      (BURST=0 means single byte)(BURST=1 means multi byte)
    * RX FIFO read:         R/W=1  burst=0/1 addr=0x3F      (BURST=0 means single byte)(BURST=1 means multi byte)
    */
endfunction

always @(posedge clk) begin
    if(rst)begin
        busy <= 1'b0;
        done <= 1'b0;
        saved_cmd_addr <= '0;
        saved_cmd_data <= '0;
        spi_hold <= 0;
        spi_tx <= '0;
        spi_start <= 0;
        rd_data <= '0;
        dly <= '0;
        is_receiving <= 0;
        pl_idx <= '0;
        fifo_reading <= 0;
        rx_ptr <= '0;
        rx_idx <= '0;
        rx_byte <= '0;
        rx_strobe <= 1'b0;
        state <= IDLE;
    end else begin
        spi_start <= 1'b0;
        rx_strobe <= 1'b0;
        case(state)
            IDLE:
            begin
                done <= 1'b0;
                if(cmd_valid) begin
                    busy <= 1'b1;
                    saved_cmd_addr <= cmd_addr;
                    saved_cmd_data <= cmd_data;
                    // Only CMD_READ_FIFO's data phase captures into rx_byte.
                    // Clearing here stops a previous FIFO read from strobing
                    // during an unrelated register access.
                    fifo_reading <= 1'b0;

                    case(cmd)
                        CMD_RESET: begin
                            spi_hold <= 1'b0;
                            dly <= 1'b0;
                            spi_tx <= SRES;
                            state <= COMMUNICATE_S;
                            rtn_state <= RST_S;
                        end

                        CMD_WRITE_REG: begin
                            spi_tx <= cmd_addr;
                            spi_hold <= 1'b1; // 1'b1 will  pull down cs line
                            is_receiving <= 1'b0;
                            state <= COMMUNICATE_S;
                            rtn_state <= WR_DATA_S;
                        end

                        CMD_READ_REG:
                        begin
                            spi_tx <= rd_reader(cmd_addr);
                            spi_hold <= 1'b1;
                            is_receiving <= 1'b1;
                            state <= COMMUNICATE_S;
                            rtn_state <= RW_DATA_S;
                        end

                        CMD_STROBE:
                        begin
                            // A strobe is one byte and nothing else. CSn must
                            // RISE for the chip to execute it, so hold is 0.
                            spi_tx <= cmd_addr;
                            spi_hold <= 1'b0;
                            is_receiving <= 1'b0;
                            state <= COMMUNICATE_S;
                            rtn_state <= DONE;
                        end

                        CMD_WRITE_FIFO:
                        begin
                            // header 0x7F, then length byte, then the payload --
                            // all inside ONE CSn-low window.
                            spi_tx <= TXFIFO_BURST;
                            spi_hold <= 1'b1;
                            is_receiving <= 1'b0;
                            pl_idx <= 8'd0;
                            state <= COMMUNICATE_S;
                            rtn_state <= FIFO_LEN_S;
                        end

                        CMD_READ_FIFO:
                        begin
                            // header 0xFF, then rx_len dummy bytes clocked out to
                            // shift the FIFO contents in -- one CSn-low window.
                            // is_receiving stays 0 here: the byte returned during
                            // the header is the chip status byte, not FIFO data.
                            spi_tx <= RXFIFO_BURST;
                            spi_hold <= 1'b1;
                            is_receiving <= 1'b0;
                            rx_ptr <= 8'd0;
                            state <= COMMUNICATE_S;
                            rtn_state <= FIFO_RD_S;
                        end

                        default:
                        begin
                            // unimplemented opcode: finish instead of hanging
                            // with busy stuck high
                            state <= DONE;
                        end
                    endcase
                end
            end

            RST_S:
            begin
                if(dly != POST_RST_CYCLE-1) dly <= dly + 1;
                else begin 
                    state <= DONE;
                    dly <= '0;
                end
            end

            WR_DATA_S:
            begin
                spi_tx <= saved_cmd_data;
                spi_hold <= 1'b0;
                state <= COMMUNICATE_S;
                rtn_state <= DONE;
            end

            RW_DATA_S:
            begin
                spi_tx <= 8'h00;
                spi_hold <= 1'b0;
                state <= COMMUNICATE_S;
                rtn_state <= DONE;
            end

            // In variable-length mode (PKTCTRL0 = 0x05) the first byte in the
            // TX FIFO is the payload length, NOT counting itself.
            FIFO_LEN_S:
            begin
                spi_tx <= pl_len;
                spi_hold <= (pl_len != 8'd0);
                state <= COMMUNICATE_S;
                rtn_state <= (pl_len == 8'd0) ? DONE : FIFO_DATA_S;
            end

            FIFO_DATA_S:
            begin
                spi_tx <= pl_byte;                     // combinational from pl_idx
                spi_hold  <= (pl_idx == pl_len-8'd1) ? 1'b0 : 1'b1;
                rtn_state <= (pl_idx == pl_len-8'd1) ? DONE : FIFO_DATA_S;
                pl_idx <= pl_idx + 8'd1;
                state <= COMMUNICATE_S;
            end

            // Clock out one dummy byte per RX FIFO byte. rx_len is guaranteed
            // non-zero by the controller (it only issues this after RXBYTES
            // reads non-zero), so no zero-length guard is needed here.
            FIFO_RD_S:
            begin
                spi_tx    <= 8'h00;
                spi_hold  <= (rx_ptr == rx_len-8'd1) ? 1'b0 : 1'b1;
                rtn_state <= (rx_ptr == rx_len-8'd1) ? DONE : FIFO_RD_S;
                fifo_reading <= 1'b1;
                state <= COMMUNICATE_S;
            end

            COMMUNICATE_S:
            begin
                spi_start <= 1'b1;
                state <= COMMUNICATING_S;
            end

            COMMUNICATING_S:
            begin
                if(spi_done)begin
                    if(is_receiving) begin
                        rd_data <= spi_rx;
                    end
                    if(fifo_reading) begin
                        // rx_idx carries the index of THIS byte, not the next --
                        // rx_ptr is the lookahead the FIFO_RD_S test uses.
                        rx_byte   <= spi_rx;
                        rx_idx    <= rx_ptr;
                        rx_strobe <= 1'b1;
                        rx_ptr    <= rx_ptr + 8'd1;
                    end
                    state <= rtn_state;
                end
            end

            DONE:
            begin
                done <= 1'b1;
                busy <= 1'b0;
                state <= IDLE;
            end

        endcase
    end
end

endmodule