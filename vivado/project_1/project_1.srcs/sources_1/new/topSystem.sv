`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// topSystem  --  top level for a bootable (non-JTAG) build
//
// Wraps the existing PL design (topRF) alongside the Zynq PS block design. The
// RF logic is UNCHANGED and unaware the PS exists -- the PS is here only so the
// board can boot from SD/QSPI. On a Zynq-7000 the PL is always configured by the
// PS (BootROM -> FSBL -> PL), so there is no PS-less path to a standalone boot.
//
// Why a hand-written top instead of putting topRF inside the block design:
// adding an RTL module to a BD renames its external ports (cc_sclk -> cc_sclk_0
// and so on), which would break every line of Constants.xdc. Keeping an RTL top
// that instantiates both preserves the pin names exactly.
//
// ADDING A PIN TAKES THREE EDITS: the topRF port, the topSystem port + its
// connection below, and Constants.xdc. Miss the topSystem half and the build
// still SUCCEEDS -- the XDC line becomes "[Vivado 12-4739] get_ports: No objects
// matched", only a critical warning, and the dangling topRF input is tied to 0.
// The signal is then silently stuck low on hardware. Grep the synthesis log for
// 12-4739 whenever a new switch or button appears to do nothing.
//
// The DDR_* and FIXED_IO_* ports need NO constraints in Constants.xdc -- they
// are hard-macro connections fixed by the PS7 configuration, and Vivado places
// them automatically. Do not add pin assignments for them.
//------------------------------------------------------------------------------
module topSystem #(
    parameter int CLK_HZ        = 125_000_000,
    parameter int POWERUP_US    = 50_000,
    parameter int TX_TIMEOUT_MS = 200
)(
    // ---- PL pins: names must match Constants.xdc exactly ----
    input  logic sysclk,
    input  logic rst,          // btn0
    input  logic tx_btn,       // btn1
    input  logic cfg_btn,      // btn2
    input  logic sw_auto,      // sw0
    input  logic sw_sender_enable,  // sw1 -- send enabled
    input  logic sw_receiver_enable,  // sw2 -- receive enabled

    output logic cc_sclk,
    output logic cc_mosi,
    input  logic cc_miso,
    output logic cc_csn,
    input  logic cc_gdo2,

    output logic cc_b_sclk,
    output logic cc_b_mosi,
    input  logic cc_b_miso,
    output logic cc_b_csn,
    input  logic cc_b_gdo2,

    output logic led_cfg,
    output logic led_tx,
    output logic led_err,
    output logic led_rx,

    // ---- PS hard-macro pins: no XDC entries required ----
    inout  [14:0] DDR_addr,
    inout  [2:0]  DDR_ba,
    inout         DDR_cas_n,
    inout         DDR_ck_n,
    inout         DDR_ck_p,
    inout         DDR_cke,
    inout         DDR_cs_n,
    inout  [3:0]  DDR_dm,
    inout  [31:0] DDR_dq,
    inout  [3:0]  DDR_dqs_n,
    inout  [3:0]  DDR_dqs_p,
    inout         DDR_odt,
    inout         DDR_ras_n,
    inout         DDR_reset_n,
    inout         DDR_we_n,
    inout         FIXED_IO_ddr_vrn,
    inout         FIXED_IO_ddr_vrp,
    inout  [53:0] FIXED_IO_mio,
    inout         FIXED_IO_ps_clk,
    inout         FIXED_IO_ps_porb,
    inout         FIXED_IO_ps_srstb
);

    // The RF design, exactly as before.
    topRF #(
        .CLK_HZ(CLK_HZ), .POWERUP_US(POWERUP_US), .TX_TIMEOUT_MS(TX_TIMEOUT_MS)
    ) rf_i (
        .sysclk(sysclk), .rst(rst), .tx_btn(tx_btn), .cfg_btn(cfg_btn),
        .sw_auto(sw_auto), .sw_receiver_enable(sw_receiver_enable), .sw_sender_enable(sw_sender_enable),
        .cc_sclk(cc_sclk), .cc_mosi(cc_mosi), .cc_miso(cc_miso),
        .cc_csn(cc_csn), .cc_gdo2(cc_gdo2),
        .cc_b_sclk(cc_b_sclk), .cc_b_mosi(cc_b_mosi), .cc_b_miso(cc_b_miso),
        .cc_b_csn(cc_b_csn), .cc_b_gdo2(cc_b_gdo2),
        .led_cfg(led_cfg), .led_tx(led_tx), .led_err(led_err), .led_rx(led_rx)
    );

    // The PS block design.
    //
    // The block design has no external PL-side ports -- the AXI GPIO is not
    // wired out -- so the only connections here are the PS hard-macro pins. If
    // you later expose a GPIO port on the BD, feed it from internal nets rather
    // than package pins: routing buttons/LEDs out again would put two top-level
    // ports on the same pins Constants.xdc already assigns, and fail placement.
    design_1_wrapper ps_i (
        .DDR_addr(DDR_addr),
        .DDR_ba(DDR_ba),
        .DDR_cas_n(DDR_cas_n),
        .DDR_ck_n(DDR_ck_n),
        .DDR_ck_p(DDR_ck_p),
        .DDR_cke(DDR_cke),
        .DDR_cs_n(DDR_cs_n),
        .DDR_dm(DDR_dm),
        .DDR_dq(DDR_dq),
        .DDR_dqs_n(DDR_dqs_n),
        .DDR_dqs_p(DDR_dqs_p),
        .DDR_odt(DDR_odt),
        .DDR_ras_n(DDR_ras_n),
        .DDR_reset_n(DDR_reset_n),
        .DDR_we_n(DDR_we_n),
        .FIXED_IO_ddr_vrn(FIXED_IO_ddr_vrn),
        .FIXED_IO_ddr_vrp(FIXED_IO_ddr_vrp),
        .FIXED_IO_mio(FIXED_IO_mio),
        .FIXED_IO_ps_clk(FIXED_IO_ps_clk),
        .FIXED_IO_ps_porb(FIXED_IO_ps_porb),
        .FIXED_IO_ps_srstb(FIXED_IO_ps_srstb)
    );

endmodule
