`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// topRF  --  Milestone 2 bring-up top level (ILA / JTAG readout)
//
// Reads PARTNUM and VERSION from CC1101 radio A. The results are exposed as
// (* mark_debug *) nets so a Vivado ILA can capture them over JTAG -- no UART,
// no USB-UART adapter, no PS. Runs once automatically after reset release; the
// `start` button re-runs it. led_done lights once a readback completes.
//
// To view the values:
//   1. Synthesize. Vivado shows "Set Up Debug" -- run it; the marked nets are
//      pre-selected. Sample clock = sysclk. Finish, then implement + program.
//   2. In Hardware Manager, set the ILA trigger to  drv_valid == 1.
//   3. Arm, press BTN1 (or reprogram). partnum should read 0x00; version is
//      typically 0x14 but varies by batch -- any non-zero, non-0xFF is a pass.
//
// Note: UartTx.sv / PacketGenerator / FrameBuilder / CRCModule stay in the
// project but are not part of this build (TX/RX + a permanent output path are
// later milestones).
//------------------------------------------------------------------------------
module topRF #(
    parameter int CLK_HZ = 125_000_000
)(
    input  logic sysclk,
    input  logic rst,          // active-high (btn0)
    input  logic start,        // manual re-trigger (btn1)

    // CC1101 radio A
    output logic cc_sclk,
    output logic cc_mosi,
    input  logic cc_miso,
    output logic cc_csn,

    output logic led_done      // lights after a readback completes
);
    localparam int SPI_DIV = CLK_HZ / (2 * 1_000_000);  // ~1 MHz SCLK

    // ---- synchronize async inputs (CDC) ----
    (* mark_debug = "true" *) logic miso_s;
    logic miso_m;
    logic start_m, start_s, start_d;
    always_ff @(posedge sysclk) begin
        miso_m  <= cc_miso;  miso_s  <= miso_m;
        start_m <= start;    start_s <= start_m;  start_d <= start_s;
    end
    wire start_edge = start_s & ~start_d;   // rising edge of the button

    // ---- one-shot auto-start after reset release ----
    logic ran, auto_start;
    always_ff @(posedge sysclk) begin
        if (rst) begin
            ran <= 1'b0; auto_start <= 1'b0;
        end else begin
            auto_start <= 1'b0;
            if (!ran) begin ran <= 1'b1; auto_start <= 1'b1; end
        end
    end
    wire drv_start = auto_start | start_edge;

    // ---- SPI master (marked nets let the ILA watch every transaction) ----
    (* mark_debug = "true" *) logic       spi_start;
    (* mark_debug = "true" *) logic       spi_hold;
    (* mark_debug = "true" *) logic       spi_busy;
    (* mark_debug = "true" *) logic       spi_done;
    (* mark_debug = "true" *) logic [7:0] spi_tx;
    (* mark_debug = "true" *) logic [7:0] spi_rx;

    SPIMaster #(.CLK_DIV(SPI_DIV)) spi (
        .clk(sysclk), .rst(rst),
        .start(spi_start), .tx_data(spi_tx), .hold_cs(spi_hold),
        .rx_data(spi_rx), .busy(spi_busy), .done(spi_done),
        .sclk(cc_sclk), .mosi(cc_mosi), .miso(miso_s), .cs_n(cc_csn)
    );

    // ---- CC1101 bring-up driver ----
    (* mark_debug = "true" *) logic [7:0] partnum;
    (* mark_debug = "true" *) logic [7:0] version;
    (* mark_debug = "true" *) logic       drv_valid;
    (* mark_debug = "true" *) logic       drv_busy;

    CC1101Driver #(.CLK_HZ(CLK_HZ)) drv (
        .clk(sysclk), .rst(rst), .start(drv_start),
        .spi_start(spi_start), .spi_tx(spi_tx), .spi_hold(spi_hold),
        .spi_done(spi_done), .spi_rx(spi_rx),
        .partnum(partnum), .version(version), .valid(drv_valid), .busy(drv_busy)
    );

    // ---- simple done indicator ----
    always_ff @(posedge sysclk) begin
        if (rst)            led_done <= 1'b0;
        else if (drv_valid) led_done <= 1'b1;
    end
endmodule
