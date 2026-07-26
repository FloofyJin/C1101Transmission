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
//   CMD_WRITE_FIFO / CMD_READ_FIFO  -- reserved (milestones 6/7; will attach a
//                  ByteStream for the payload)
//
// Milestone-4 scope: RESET / WRITE_REG / READ_REG / STROBE implemented.
//------------------------------------------------------------------------------
module CC1101Driver #(
    parameter int CLK_HZ     = 125_000_000,
    parameter int POSTRES_US = 1_000        // settle after SRES (1 ms)
)(
    input  logic       clk,
    input  logic       rst,

    // command interface (one command per cmd_valid pulse; issue only when !busy)
    input  logic       cmd_valid,
    input  logic [2:0] cmd,
    input  logic [7:0] cmd_addr,
    input  logic [7:0] cmd_data,

    // results / status
    output logic [7:0] rd_data,   // last CMD_READ_REG result
    output logic       done,      // 1-cycle pulse when a command completes
    output logic       busy,

    // SPIMaster handshake
    output logic       spi_start,
    output logic [7:0] spi_tx,
    output logic       spi_hold,
    input  logic       spi_done,
    input  logic [7:0] spi_rx
);
    localparam int POST_CYCLES = (CLK_HZ / 1_000_000) * POSTRES_US;

    localparam logic [7:0] SRES_STROBE = 8'h30;

    localparam logic [2:0]
        CMD_RESET      = 3'd0,
        CMD_WRITE_REG  = 3'd1,
        CMD_READ_REG   = 3'd2,
        CMD_STROBE     = 3'd3,
        CMD_WRITE_FIFO = 3'd4,   // reserved (milestone 6)
        CMD_READ_FIFO  = 3'd5;   // reserved (milestone 7)

    typedef enum logic [3:0] {
        S_IDLE,
        S_DISPATCH,       // decode the latched command
        S_RST_SETTLE,     // wait after SRES
        S_WR_DATA,        // write: 2nd byte (data)
        S_RD_DUMMY,       // read: 2nd byte (dummy, capture)
        S_XI, S_XW,       // generic byte transfer: issue, wait-done
        S_DONE
    } st_t;

    st_t         state, ret_state;
    logic [2:0]  cur_cmd;
    logic [7:0]  cur_addr, cur_data;
    logic        cap;              // capture next transfer's rx into rd_data
    logic [31:0] dly;

    // Read header = read bit (0x80) + burst bit (0x40) ONLY for status regs
    // (0x30-0x3D, where the burst bit selects the status reg over the strobe).
    function automatic logic [7:0] rd_header(input logic [7:0] a);
        rd_header = 8'h80 | (((a >= 8'h30) && (a <= 8'h3D)) ? 8'h40 : 8'h00) | a;
    endfunction

    always_ff @(posedge clk) begin
        if (rst) begin
            state     <= S_IDLE;
            ret_state <= S_IDLE;
            cur_cmd   <= 3'd0;
            cur_addr  <= 8'h00;
            cur_data  <= 8'h00;
            cap       <= 1'b0;
            dly       <= 0;
            spi_start <= 1'b0;
            spi_tx    <= 8'h00;
            spi_hold  <= 1'b0;
            rd_data   <= 8'h00;
            done      <= 1'b0;
            busy      <= 1'b0;
        end else begin
            // one-cycle-pulse defaults
            spi_start <= 1'b0;
            done      <= 1'b0;

            case (state)
                S_IDLE: begin
                    busy <= 1'b0;
                    if (cmd_valid) begin
                        cur_cmd  <= cmd;
                        cur_addr <= cmd_addr; 
                        cur_data <= cmd_data;
                        busy     <= 1'b1;
                        state    <= S_DISPATCH;
                    end
                end

                // decode the latched command; set up its first SPI byte
                S_DISPATCH: begin
                    cap <= 1'b0;
                    case (cur_cmd)
                        CMD_RESET: begin
                            spi_tx <= SRES_STROBE; spi_hold <= 1'b0; dly <= 0;
                            ret_state <= S_RST_SETTLE; state <= S_XI;
                        end
                        CMD_WRITE_REG: begin
                            spi_tx <= cur_addr;    // R/W=0, burst=0 -> just the address
                            spi_hold <= 1'b1;      // hold CSn for the data byte
                            ret_state <= S_WR_DATA; state <= S_XI;
                        end
                        CMD_READ_REG: begin
                            spi_tx <= rd_header(cur_addr);
                            spi_hold <= 1'b1;      // hold CSn for the dummy byte
                            ret_state <= S_RD_DUMMY; state <= S_XI;
                        end
                        CMD_STROBE: begin
                            spi_tx <= cur_addr;    // the strobe byte itself
                            spi_hold <= 1'b0;
                            ret_state <= S_DONE; state <= S_XI;
                        end
                        default: state <= S_DONE;  // FIFO / unknown: no-op for now
                    endcase
                end

                S_RST_SETTLE: if (dly == POST_CYCLES-1) begin
                    dly <= 0; state <= S_DONE;
                end else dly <= dly + 1;

                // write: second byte is the data, release CSn afterwards
                S_WR_DATA: begin
                    spi_tx <= cur_data; spi_hold <= 1'b0;
                    ret_state <= S_DONE; state <= S_XI;
                end

                // read: second byte clocks the value in; capture it, release CSn
                S_RD_DUMMY: begin
                    spi_tx <= 8'h00; spi_hold <= 1'b0; cap <= 1'b1;
                    ret_state <= S_DONE; state <= S_XI;
                end

                // ---- generic byte transfer ----
                S_XI: begin
                    spi_start <= 1'b1;    // one-cycle kick; SPIMaster samples start & !busy
                    state     <= S_XW;
                end
                S_XW: if (spi_done) begin
                    if (cap) rd_data <= spi_rx;
                    state <= ret_state;
                end

                S_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
