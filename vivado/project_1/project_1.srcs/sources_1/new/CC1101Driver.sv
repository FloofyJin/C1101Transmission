`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// CC1101Driver  --  Milestone 2 bring-up only
//
// On `start`: wait for supply/crystal to settle, issue an SRES reset strobe,
// then read PARTNUM (0x30) and VERSION (0x31) and present them with a `valid`
// pulse. Shifts one byte at a time through SPIMaster; `spi_hold` keeps CSn low
// across the header+dummy pair of a register read (header out, value comes back
// on the second byte).
//
// TODO (later milestones, deliberately omitted here):
//   * manual power-on reset wiggle on CSn
//   * "wait for MISO low" (CHIP_RDYn) instead of the fixed post-reset delay
//   * ConfigRom walk, TX/RX engines
// For proving SPI + wiring + a live chip, fixed delays are enough.
//------------------------------------------------------------------------------
module CC1101Driver #(
    parameter int CLK_HZ     = 125_000_000,
    parameter int POWERUP_US = 50_000,   // settle after config (50 ms)
    parameter int POSTRES_US = 1_000     // settle after SRES (1 ms)
)(
    input  logic       clk,
    input  logic       rst,
    input  logic       start,      // pulse: run the bring-up sequence

    // SPIMaster handshake
    output logic       spi_start,
    output logic [7:0] spi_tx,
    output logic       spi_hold,
    input  logic       spi_done,
    input  logic [7:0] spi_rx,

    // results
    output logic [7:0] partnum,
    output logic [7:0] version,
    output logic       valid,      // 1-cycle pulse when partnum/version updated
    output logic       busy
);
    localparam int CYC_PER_US = CLK_HZ / 1_000_000;
    localparam int PWR_CYCLES  = CYC_PER_US * POWERUP_US; // bring up time
    localparam int POST_CYCLES = CYC_PER_US * POSTRES_US; // time after reset

    // Status registers overlap strobe addresses, so the burst bit (0x40) is
    // required to reach them. Read bit is 0x80.
    localparam logic [7:0] SRES_STROBE = 8'h30;                 // reset
    localparam logic [7:0] PARTNUM_RD  = 8'h80 | 8'h40 | 8'h30; // 0xF0 = 0x80 (READ) + 0x40 (BURST)
    localparam logic [7:0] VERSION_RD  = 8'h80 | 8'h40 | 8'h31; // 0xF1

    typedef enum logic [3:0] {
        S_IDLE,
        S_PWR,
        S_SRES,
        S_POST,
        S_PN_H, S_PN_D,
        S_VER_H, S_VER_D,
        S_XI, S_XW,      // generic byte transfer: issue, then wait-done
        S_FIN
    } st_t;

    typedef enum logic [1:0] { CAP_NONE, CAP_PN, CAP_VER } cap_t;

    st_t  state, ret_state;
    cap_t cap;
    logic [31:0] dly; // delay ycle counter

    always_ff @(posedge clk) begin
        if (rst) begin
            state     <= S_IDLE;
            ret_state <= S_IDLE;
            cap       <= CAP_NONE;
            dly       <= 0;
            spi_start <= 1'b0;
            spi_tx    <= 8'h00;
            spi_hold  <= 1'b0;
            partnum   <= 8'h00;
            version   <= 8'h00;
            valid     <= 1'b0;
            busy      <= 1'b0;
        end else begin
            // one-cycle-pulse defaults
            spi_start <= 1'b0;
            valid     <= 1'b0;

            case (state)
                S_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        dly   <= 0;
                        state <= S_PWR;
                    end
                end

                S_PWR:  if (dly == PWR_CYCLES-1) begin
                    state <= S_SRES; 
                    dly <= 0; 
                end else dly <= dly + 1;

                // SRES strobe: single byte, CSn released afterwards
                S_SRES: begin
                    spi_tx <= SRES_STROBE; 
                    spi_hold <= 1'b0; 
                    cap <= CAP_NONE;

                    ret_state <= S_POST;
                    state <= S_XI;
                end

                S_POST: if (dly == POST_CYCLES-1) begin
                    state <= S_PN_H; 
                    dly <= 0; 
                end else dly <= dly + 1;

                // PARTNUM: header (CSn held) then dummy (capture, CSn released)
                S_PN_H: begin
                    spi_tx <= PARTNUM_RD; spi_hold <= 1'b1; cap <= CAP_NONE;
                    ret_state <= S_PN_D; state <= S_XI;
                end
                S_PN_D: begin
                    spi_tx <= 8'h00; spi_hold <= 1'b0; cap <= CAP_PN;
                    ret_state <= S_VER_H; state <= S_XI;
                end

                // VERSION
                S_VER_H: begin
                    spi_tx <= VERSION_RD; spi_hold <= 1'b1; cap <= CAP_NONE;
                    ret_state <= S_VER_D; state <= S_XI;
                end
                S_VER_D: begin
                    spi_tx <= 8'h00; spi_hold <= 1'b0; cap <= CAP_VER;
                    ret_state <= S_FIN; state <= S_XI;
                end

                // ---- generic byte transfer ----
                S_XI: begin
                    spi_start <= 1'b1;      // one-cycle kick; SPIMaster samples start & !busy
                    state     <= S_XW;
                end
                S_XW: if (spi_done) begin
                    case (cap)
                        CAP_PN:  partnum <= spi_rx;
                        CAP_VER: version <= spi_rx;
                        default: ;
                    endcase
                    state <= ret_state;
                end

                S_FIN: begin
                    valid <= 1'b1;
                    busy  <= 1'b0;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
