`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// ConfigSeq  --  Milestone 5: configure a CC1101 and verify by read-back
//
// Drives one CC1101Driver's command interface. On start (and once automatically
// after reset): RESET -> write every entry in the config ROM -> read every entry
// back and compare. `config_ok` goes high only if every read-back matched what
// was written; on the first mismatch it latches `fail_addr` / `fail_got` so the
// ILA tells you exactly which register didn't stick.
//
// IMPORTANT -- the register VALUES below are a widely-used 433.92 MHz GFSK
// baseline (SmartRF-style, assuming a 26 MHz crystal). They are NOT verified for
// your specific hardware. Regenerate with TI SmartRF Studio for your band, data
// rate, and crystal. In particular FREQ2/1/0 (0x0D/0x0E/0x0F) are band-specific.
// Read-back verify catches SPI/wiring errors, NOT wrong-but-valid RF values.
//------------------------------------------------------------------------------
module ConfigSeq #(
    parameter int CLK_HZ     = 125_000_000,
    parameter int POWERUP_US = 50_000,
    // PATABLE = output power. 0x12 is roughly -30 dBm at 433 MHz: the MINIMUM.
    // Two radios on the same bench are ~10 cm apart, which is far too loud at
    // full power -- the receiver's front end saturates and the link fails in a
    // way that looks like a config or logic bug. Turn this up only once the
    // radios are metres apart. (0x60 ~ 0 dBm, 0xC0 ~ +10 dBm.)
    parameter logic [7:0] PATABLE_VAL = 8'h12,
    // Longest legitimate wait on drv_done is CMD_RESET, which includes the
    // driver's 1 ms post-SRES settle. 50 ms is ~50x that -- long enough never to
    // fire on a healthy chip, short enough that a dead one reports promptly.
    parameter int DRV_TIMEOUT_MS = 50
)(
    input  logic       clk,
    input  logic       rst,
    input  logic       start,         // re-run (auto-runs once after reset)

    // to CC1101Driver command interface
    output logic       cmd_valid,
    output logic [2:0] cmd,
    output logic [7:0] cmd_addr,
    output logic [7:0] cmd_data,
    input  logic       drv_done,
    input  logic [7:0] rd_data,

    // status
    // config_ok is only published at C_DONE, so it is meaningful ONLY while
    // busy is low. It never reads high mid-sequence.
    output logic       config_ok,     // 1 = all read-backs matched
    output logic       done,          // 1-cycle pulse when sequence completes
    output logic       busy,
    output logic [7:0] fail_addr,     // register of first mismatch (0 if none)
    output logic [7:0] fail_got,      // value read at the mismatch
    output logic       timeout        // sticky: drv_done never arrived -> aborted
);
    import cc1101_pkg::*;   // CMD_* opcodes

    localparam int PWR_CYCLES = (CLK_HZ / 1_000_000) * POWERUP_US;
    localparam int WDT_CYCLES = (CLK_HZ / 1000) * DRV_TIMEOUT_MS;
    localparam logic [7:0] PATABLE_ADDR = 8'h3E;  // not a plain reg -> written, not verified

    // ---- config ROM: {addr[15:8], value[7:0]} ----
    // Stored as one flat constant rather than an unpacked array so that plain
    // simulators (iverilog) can read it too; entry 0 is the leftmost 16 bits.
    localparam int N_CFG = 37;
    localparam logic [N_CFG*16-1:0] CFG_FLAT = {
        16'h00_06,  // IOCFG2    GDO2 asserts on sync, deasserts at end of packet.
                    //           GDO2 (not GDO0/GDO1) because this module only
                    //           breaks out GDO1 and GDO2, and GDO1 is the SO pin
                    //           -- only valid while CSn is high, so it is unusable
                    //           as a status line during SPI polling.
        16'h02_2E,  // IOCFG0    3-state. GDO0 is not bonded out on this module and
                    //           its reset default is a XOSC/192 clock output; the
                    //           datasheet recommends disabling it for RF perf.
        16'h03_47,  // FIFOTHR
        16'h04_D3,  // SYNC1
        16'h05_91,  // SYNC0
        16'h06_FF,  // PKTLEN    max (variable-length mode)
        16'h07_04,  // PKTCTRL1  append status bytes (RSSI/LQI/CRC_OK)
        16'h08_05,  // PKTCTRL0  variable length + CRC enabled
        16'h0B_06,  // FSCTRL1
        16'h0C_00,  // FSCTRL0
        16'h0D_10,  // FREQ2     \
        16'h0E_B0,  // FREQ1      >  ~433.92 MHz @ 26 MHz xtal -- BAND SPECIFIC
        16'h0F_71,  // FREQ0     /
        16'h10_CA,  // MDMCFG4
        16'h11_83,  // MDMCFG3
        16'h12_13,  // MDMCFG2   GFSK, 30/32 sync-word detection
        16'h13_22,  // MDMCFG1
        16'h14_F8,  // MDMCFG0
        16'h15_35,  // DEVIATN
        16'h18_18,  // MCSM0     FS_AUTOCAL on IDLE->RX/TX  (do not omit)
        16'h19_16,  // FOCCFG
        16'h1A_6C,  // BSCFG
        16'h1B_43,  // AGCCTRL2
        16'h1C_40,  // AGCCTRL1
        16'h1D_91,  // AGCCTRL0
        16'h20_FB,  // WORCTRL
        16'h21_56,  // FREND1
        16'h22_10,  // FREND0
        16'h23_E9,  // FSCAL3
        16'h24_2A,  // FSCAL2
        16'h25_00,  // FSCAL1
        16'h26_1F,  // FSCAL0
        16'h29_59,  // FSTEST
        16'h2C_81,  // TEST2
        16'h2D_35,  // TEST1
        16'h2E_09,  // TEST0
        {PATABLE_ADDR, PATABLE_VAL}   // PATABLE  output power -- see parameter above
    };

    typedef enum logic [3:0] {
        C_PWR, C_RST_I, C_RST_W, C_WR_I, C_WR_W, C_RD_I, C_RD_W, C_DONE, C_IDLE
    } cst_t;
    cst_t cstate;

    logic [7:0]  idx;
    logic [31:0] pdly;
    logic [31:0] wdt;        // watchdog on every drv_done handshake
    logic        verify_ok;  // running verdict; copied to config_ok at C_DONE

    // idx never exceeds N_CFG-1 in practice, but synthesis still has to build the
    // mux for all 256 values of an 8-bit index, and (N_CFG-1-idx) goes negative
    // for idx > 35 -- that out-of-range select into CFG_FLAT crashes Vivado
    // 2024.2 outright (EXCEPTION_ACCESS_VIOLATION). Clamp so the select is
    // always in range.
    wire [5:0]  idx_c     = (idx < N_CFG) ? idx[5:0] : 6'd0;
    wire [15:0] cur_entry = CFG_FLAT[(N_CFG-1-idx_c)*16 +: 16];
    wire [7:0]  cur_addr  = cur_entry[15:8];
    wire [7:0]  cur_val   = cur_entry[7:0];

    always_ff @(posedge clk) begin
        if (rst) begin
            cstate    <= C_PWR;
            idx       <= 8'd0;
            pdly      <= 0;
            wdt       <= 0;
            cmd_valid <= 1'b0; cmd <= 3'd0; cmd_addr <= 8'h00; cmd_data <= 8'h00;
            config_ok <= 1'b0; done <= 1'b0; busy <= 1'b0;
            verify_ok <= 1'b0; timeout <= 1'b0;
            fail_addr <= 8'h00; fail_got <= 8'h00;
        end else begin
            cmd_valid <= 1'b0;   // one-cycle-pulse defaults
            done      <= 1'b0;

            case (cstate)
                // settle after config, then reset the chip
                C_PWR: begin
                    busy <= 1'b1;
                    if (pdly == PWR_CYCLES-1) begin pdly <= 0; cstate <= C_RST_I; end
                    else pdly <= pdly + 1;
                end

                C_RST_I: begin cmd_valid <= 1'b1; cmd <= CMD_RESET; wdt <= 0; cstate <= C_RST_W; end // send reset signal to CC1101
                C_RST_W: begin // once reset, go to write config
                    if (drv_done) begin idx <= 8'd0; cstate <= C_WR_I; end
                    else if (wdt == WDT_CYCLES-1) begin
                        timeout <= 1'b1; verify_ok <= 1'b0; cstate <= C_DONE;
                    end else wdt <= wdt + 32'd1;
                end

                // ---- write every config entry ----
                C_WR_I: begin
                    cmd_valid <= 1'b1; cmd <= CMD_WRITE_REG;
                    cmd_addr  <= cur_addr; cmd_data <= cur_val;
                    wdt <= 0; cstate <= C_WR_W;
                end
                C_WR_W: begin
                    if (drv_done) begin
                        if (idx == N_CFG-1) begin
                            // Writes are done, so the verdict starts optimistic --
                            // but this is verify_ok, NOT config_ok. Nothing is
                            // published to the outside world until C_DONE.
                            idx <= 8'd0; verify_ok <= 1'b1; cstate <= C_RD_I;
                        end else begin
                            idx <= idx + 8'd1; cstate <= C_WR_I;
                        end
                    end else if (wdt == WDT_CYCLES-1) begin
                        timeout <= 1'b1; verify_ok <= 1'b0;
                        fail_addr <= cur_addr; fail_got <= 8'h00;
                        cstate <= C_DONE;
                    end else wdt <= wdt + 32'd1;
                end

                // ---- read back + verify (skip PATABLE, not a plain register) ----
                C_RD_I: begin
                    if (cur_addr == PATABLE_ADDR) begin
                        if (idx == N_CFG-1) cstate <= C_DONE;
                        else begin idx <= idx + 8'd1; cstate <= C_RD_I; end
                    end else begin
                        cmd_valid <= 1'b1; cmd <= CMD_READ_REG; cmd_addr <= cur_addr;
                        wdt <= 0; cstate <= C_RD_W;
                    end
                end
                C_RD_W: begin
                    if (drv_done) begin
                        if ((rd_data != cur_val) && verify_ok) begin
                            verify_ok <= 1'b0;         // first mismatch only
                            fail_addr <= cur_addr;
                            fail_got  <= rd_data;
                        end
                        if (idx == N_CFG-1) cstate <= C_DONE;
                        else begin idx <= idx + 8'd1; cstate <= C_RD_I; end
                    end else if (wdt == WDT_CYCLES-1) begin
                        timeout <= 1'b1; verify_ok <= 1'b0;
                        fail_addr <= cur_addr; fail_got <= 8'h00;
                        cstate <= C_DONE;
                    end else wdt <= wdt + 32'd1;
                end

                // The single place config_ok is published. Reaching here via a
                // timeout carries verify_ok = 0, so a hung run reports failure
                // instead of freezing with a stale "OK" on the ILA.
                C_DONE: begin
                    config_ok <= verify_ok;
                    done <= 1'b1; busy <= 1'b0; cstate <= C_IDLE;
                end

                C_IDLE: if (start) begin
                    // Invalidate the previous run's verdict up front, so a re-run
                    // in progress can never be mistaken for a passing one.
                    idx       <= 8'd0;
                    busy      <= 1'b1;
                    config_ok <= 1'b0; verify_ok <= 1'b0; timeout <= 1'b0;
                    fail_addr <= 8'h00; fail_got <= 8'h00;
                    cstate    <= C_RST_I;
                end

                default: cstate <= C_PWR;
            endcase
        end
    end
endmodule
