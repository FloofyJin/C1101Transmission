`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// topRF  --  Milestone 4: command-driven driver, write-a-register-and-read-it-back
//
// A small mission sequencer issues commands to the (now command-driven) driver:
//   RESET -> READ_REG(PARTNUM) -> WRITE_REG(PKTLEN=0xAA) -> READ_REG(PKTLEN)
// then checks the read-back == 0xAA (`match`). All results are (* mark_debug *)
// nets for the ILA. Runs once after reset; BTN1 re-runs (chip already powered).
// led_done lights if the write/read-back matched.
//
// NOTE: the debug net set CHANGED from the milestone-2 build -- after synthesis,
// re-run "Set Up Debug" so the ILA picks up the new signals, and trigger on
// `match == 1` (or `step == 3` / `drv_done`).
//------------------------------------------------------------------------------
module topRF #(
    parameter int CLK_HZ     = 125_000_000,
    parameter int POWERUP_US = 50_000        // settle after config (50 ms)
)(
    input  logic sysclk,
    input  logic rst,          // active-high (btn0)
    input  logic start,        // re-run trigger (btn1)

    // CC1101 radio A
    output logic cc_sclk,
    output logic cc_mosi,
    input  logic cc_miso,
    output logic cc_csn,

    output logic led_done      // lights if milestone-4 write/read-back matched
);
    localparam int SPI_DIV    = CLK_HZ / (2 * 1_000_000);   // ~1 MHz SCLK
    localparam int PWR_CYCLES = (CLK_HZ / 1_000_000) * POWERUP_US;

    localparam logic [2:0]
        CMD_RESET = 3'd0, CMD_WRITE_REG = 3'd1, CMD_READ_REG = 3'd2;

    localparam logic [7:0] PARTNUM_ADDR = 8'h30;   // status reg (driver adds burst bit)
    localparam logic [7:0] TEST_ADDR    = 8'h06;   // PKTLEN -- plain R/W config reg
    localparam logic [7:0] TEST_DATA    = 8'hAA;

    // ---- CDC synchronizers ----
    (* mark_debug = "true" *) logic miso_s;
    logic miso_m;
    logic start_m, start_s, start_d;
    always_ff @(posedge sysclk) begin
        miso_m  <= cc_miso;  miso_s <= miso_m;
        start_m <= start;    start_s <= start_m;  start_d <= start_s;
    end
    wire start_edge = start_s & ~start_d;

    // ---- SPI master ----
    (* mark_debug = "true" *) logic       spi_start, spi_hold, spi_busy, spi_done;
    (* mark_debug = "true" *) logic [7:0] spi_tx, spi_rx;

    SPIMaster #(.CLK_DIV(SPI_DIV)) spi (
        .clk(sysclk), .rst(rst),
        .start(spi_start), .tx_data(spi_tx), .hold_cs(spi_hold),
        .rx_data(spi_rx), .busy(spi_busy), .done(spi_done),
        .sclk(cc_sclk), .mosi(cc_mosi), .miso(miso_s), .cs_n(cc_csn)
    );

    // ---- command-driven driver ----
    (* mark_debug = "true" *) logic [2:0] cmd;
    logic       cmd_valid;
    logic [7:0] cmd_addr, cmd_data;
    (* mark_debug = "true" *) logic [7:0] rd_data;
    (* mark_debug = "true" *) logic       drv_done, drv_busy;

    CC1101Driver #(.CLK_HZ(CLK_HZ)) drv (
        .clk(sysclk), .rst(rst),
        .cmd_valid(cmd_valid), .cmd(cmd), .cmd_addr(cmd_addr), .cmd_data(cmd_data),
        .rd_data(rd_data), .done(drv_done), .busy(drv_busy),
        .spi_start(spi_start), .spi_tx(spi_tx), .spi_hold(spi_hold),
        .spi_done(spi_done), .spi_rx(spi_rx)
    );

    // ---- mission sequencer ----
    typedef enum logic [2:0] { T_PWR, T_ISSUE, T_WAIT, T_CMP, T_IDLE } tst_t;
    tst_t tstate;

    (* mark_debug = "true" *) logic [2:0] step;
    (* mark_debug = "true" *) logic [7:0] partnum_rd;
    (* mark_debug = "true" *) logic [7:0] test_rd;
    (* mark_debug = "true" *) logic       match;
    logic [31:0] pdly;

    localparam logic [2:0] LAST_STEP = 3'd3;

    always_ff @(posedge sysclk) begin
        if (rst) begin
            tstate     <= T_PWR;
            step       <= 3'd0;
            pdly       <= 0;
            cmd_valid  <= 1'b0;
            cmd        <= 3'd0;
            cmd_addr   <= 8'h00;
            cmd_data   <= 8'h00;
            partnum_rd <= 8'h00;
            test_rd    <= 8'h00;
            match      <= 1'b0;
            led_done   <= 1'b0;
        end else begin
            cmd_valid <= 1'b0;   // one-cycle-pulse default

            case (tstate)
                // wait for the chip's supply/crystal to settle after config
                T_PWR: if (pdly == PWR_CYCLES-1) begin
                    pdly <= 0; step <= 3'd0; tstate <= T_ISSUE;
                end else pdly <= pdly + 1;

                // launch the command for the current step
                T_ISSUE: begin
                    cmd_valid <= 1'b1;
                    case (step)
                        3'd0:    begin cmd <= CMD_RESET;     cmd_addr <= 8'h00;        cmd_data <= 8'h00;     end
                        3'd1:    begin cmd <= CMD_READ_REG;  cmd_addr <= PARTNUM_ADDR; cmd_data <= 8'h00;     end
                        3'd2:    begin cmd <= CMD_WRITE_REG; cmd_addr <= TEST_ADDR;    cmd_data <= TEST_DATA; end
                        default: begin cmd <= CMD_READ_REG;  cmd_addr <= TEST_ADDR;    cmd_data <= 8'h00;     end // step 3
                    endcase
                    tstate <= T_WAIT;
                end

                // wait for the driver to finish; capture read results
                T_WAIT: if (drv_done) begin
                    if (step == 3'd1) partnum_rd <= rd_data;   // PARTNUM
                    if (step == 3'd3) test_rd    <= rd_data;   // PKTLEN read-back
                    if (step == LAST_STEP) tstate <= T_CMP;
                    else begin step <= step + 1'b1; tstate <= T_ISSUE; end
                end

                // did the write stick?
                T_CMP: begin
                    match    <= (test_rd == TEST_DATA);
                    led_done <= (test_rd == TEST_DATA);
                    tstate   <= T_IDLE;
                end

                T_IDLE: if (start_edge) begin
                    step <= 3'd0; tstate <= T_ISSUE;   // re-run (skip power-up wait)
                end

                default: tstate <= T_PWR;
            endcase
        end
    end
endmodule
