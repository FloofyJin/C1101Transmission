`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// topRF  --  Milestone 5: configure both radios, verify config by read-back
//
// Per radio: SPIMaster + CC1101Driver + ConfigSeq. ConfigSeq resets the chip,
// writes the config ROM, then reads every register back and compares. Both
// radios get the identical config (same ConfigSeq / same ROM).
//
// led_done lights only if BOTH radios reported config_ok. ILA nets: a_config_ok
// / a_fail_addr / a_fail_got for radio A, b_* for radio B, plus each radio's SPI
// bytes. Re-run: btn1 (radio A), btn2 (radio B).
//
// NOTE: net set changed again -> clear debug_ila.xdc and re-run "Set Up Debug".
//------------------------------------------------------------------------------
module topRF #(
    parameter int CLK_HZ     = 125_000_000,
    parameter int POWERUP_US = 50_000
)(
    input  logic sysclk,
    input  logic rst,          // btn0
    input  logic start,        // radio A re-run (btn1)
    input  logic b_start,      // radio B re-run (btn2)

    // CC1101 radio A (Pmod JC)
    output logic cc_sclk,
    output logic cc_mosi,
    input  logic cc_miso,
    output logic cc_csn,

    // CC1101 radio B (Pmod JE)
    output logic cc_b_sclk,
    output logic cc_b_mosi,
    input  logic cc_b_miso,
    output logic cc_b_csn,

    output logic led_done      // lights if BOTH radios configured OK
);
    localparam int SPI_DIV = CLK_HZ / (2 * 1_000_000);   // ~1 MHz SCLK

    // ================= CDC synchronizers =================
    (* mark_debug = "true" *) logic miso_s;
    logic miso_m;
    logic start_m, start_s, start_d;

    (* mark_debug = "true" *) logic b_miso_s;
    logic b_miso_m;
    logic b_start_m, b_start_s, b_start_d;

    always_ff @(posedge sysclk) begin
        miso_m    <= cc_miso;    miso_s    <= miso_m;
        start_m   <= start;      start_s   <= start_m;   start_d   <= start_s;
        b_miso_m  <= cc_b_miso;  b_miso_s  <= b_miso_m;
        b_start_m <= b_start;    b_start_s <= b_start_m; b_start_d <= b_start_s;
    end
    wire start_edge   = start_s   & ~start_d;
    wire b_start_edge = b_start_s & ~b_start_d;

    // ================= RADIO A =================
    (* mark_debug = "true" *) logic       spi_start, spi_hold, spi_busy, spi_done;
    (* mark_debug = "true" *) logic [7:0] spi_tx, spi_rx;

    SPIMaster #(.CLK_DIV(SPI_DIV)) spi_a (
        .clk(sysclk), .rst(rst),
        .start(spi_start), .tx_data(spi_tx), .hold_cs(spi_hold),
        .rx_data(spi_rx), .busy(spi_busy), .done(spi_done),
        .sclk(cc_sclk), .mosi(cc_mosi), .miso(miso_s), .cs_n(cc_csn)
    );

    (* mark_debug = "true" *) logic [2:0] cmd;
    logic       cmd_valid;
    logic [7:0] cmd_addr, cmd_data;
    (* mark_debug = "true" *) logic [7:0] rd_data;
    logic       drv_done, drv_busy;

    CC1101Driver #(.CLK_HZ(CLK_HZ)) drv_a (
        .clk(sysclk), .rst(rst),
        .cmd_valid(cmd_valid), .cmd(cmd), .cmd_addr(cmd_addr), .cmd_data(cmd_data),
        .rd_data(rd_data), .done(drv_done), .busy(drv_busy),
        .spi_start(spi_start), .spi_tx(spi_tx), .spi_hold(spi_hold),
        .spi_done(spi_done), .spi_rx(spi_rx)
    );

    (* mark_debug = "true" *) logic       a_config_ok, a_cfg_done, a_cfg_busy;
    (* mark_debug = "true" *) logic [7:0] a_fail_addr, a_fail_got;

    ConfigSeq #(.CLK_HZ(CLK_HZ), .POWERUP_US(POWERUP_US)) cfg_a (
        .clk(sysclk), .rst(rst), .start(start_edge),
        .cmd_valid(cmd_valid), .cmd(cmd), .cmd_addr(cmd_addr), .cmd_data(cmd_data),
        .drv_done(drv_done), .rd_data(rd_data),
        .config_ok(a_config_ok), .done(a_cfg_done), .busy(a_cfg_busy),
        .fail_addr(a_fail_addr), .fail_got(a_fail_got)
    );

    // ================= RADIO B =================
    (* mark_debug = "true" *) logic       b_spi_start, b_spi_hold, b_spi_busy, b_spi_done;
    (* mark_debug = "true" *) logic [7:0] b_spi_tx, b_spi_rx;

    SPIMaster #(.CLK_DIV(SPI_DIV)) spi_b (
        .clk(sysclk), .rst(rst),
        .start(b_spi_start), .tx_data(b_spi_tx), .hold_cs(b_spi_hold),
        .rx_data(b_spi_rx), .busy(b_spi_busy), .done(b_spi_done),
        .sclk(cc_b_sclk), .mosi(cc_b_mosi), .miso(b_miso_s), .cs_n(cc_b_csn)
    );

    (* mark_debug = "true" *) logic [2:0] b_cmd;
    logic       b_cmd_valid;
    logic [7:0] b_cmd_addr, b_cmd_data;
    (* mark_debug = "true" *) logic [7:0] b_rd_data;
    logic       b_drv_done, b_drv_busy;

    CC1101Driver #(.CLK_HZ(CLK_HZ)) drv_b (
        .clk(sysclk), .rst(rst),
        .cmd_valid(b_cmd_valid), .cmd(b_cmd), .cmd_addr(b_cmd_addr), .cmd_data(b_cmd_data),
        .rd_data(b_rd_data), .done(b_drv_done), .busy(b_drv_busy),
        .spi_start(b_spi_start), .spi_tx(b_spi_tx), .spi_hold(b_spi_hold),
        .spi_done(b_spi_done), .spi_rx(b_spi_rx)
    );

    (* mark_debug = "true" *) logic       b_config_ok, b_cfg_done, b_cfg_busy;
    (* mark_debug = "true" *) logic [7:0] b_fail_addr, b_fail_got;

    ConfigSeq #(.CLK_HZ(CLK_HZ), .POWERUP_US(POWERUP_US)) cfg_b (
        .clk(sysclk), .rst(rst), .start(b_start_edge),
        .cmd_valid(b_cmd_valid), .cmd(b_cmd), .cmd_addr(b_cmd_addr), .cmd_data(b_cmd_data),
        .drv_done(b_drv_done), .rd_data(b_rd_data),
        .config_ok(b_config_ok), .done(b_cfg_done), .busy(b_cfg_busy),
        .fail_addr(b_fail_addr), .fail_got(b_fail_got)
    );

    // LED on only if BOTH radios configured and verified OK
    assign led_done = a_config_ok & b_config_ok;

endmodule
