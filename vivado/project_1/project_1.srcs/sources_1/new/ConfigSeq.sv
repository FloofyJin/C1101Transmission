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
// IMPORTANT -- 433.92 MHz GFSK at 250 kbps, 26 MHz crystal.
//
// ---- history, because it explains the register choices ----
// This began as a 38.4 kbps GFSK baseline. An attempt to jump straight to
// 500 kbps MSK FAILED on hardware: the link dropped most packets and the
// display teleported. The cause was not the rate -- 500 kBaud is a supported,
// TI-characterised operating point -- but that the rate moved while DEVIATN,
// the AGC block and the preamble did not. 250 kbps GFSK was chosen instead
// because TI publishes a COMPLETE characterisation for it at 433 MHz:
//
//   -95 dBm sensitivity, 127 kHz deviation, 540 kHz channel filter
//   (the datasheet revision history even records TI correcting the 250 kBaud
//    reference from MSK to GFSK -- this is their intended operating point)
//
// So four registers here are matched to published figures rather than guessed:
//
//   MDMCFG2 0x13  GFSK, unchanged -- the modulation that already worked
//   MDMCFG4 0x2D  DRATE_E=13, channel filter 541.7 kHz (TI's 540 kHz)
//   MDMCFG3 0x3B  DRATE_M=59  -> 249,939 bps
//   DEVIATN 0x62  +/-127.0 kHz  -> exactly TI's published deviation
//   MDMCFG1 0x62  16 preamble bytes -> 512 us of AGC settling
//
// ---- still NOT verified for this rate ----
// FSCTRL1 is interpolated, not published (see its comment below). FOCCFG,
// BSCFG, AGCCTRL2/1/0, FREND1 and TEST2/TEST1 are empirical AGC and
// loop-filter values still carrying their 38.4 kbps settings. Get them from
// TI SmartRF Studio; do not hand-derive them.
//
// The bench link tolerates that because two radios 10 cm apart sit tens of dB
// above the sensitivity limit, which hides a mistuned AGC. It will not survive
// real distance.
//
// FREQ2/1/0 (0x0D/0x0E/0x0F) remain band-specific.
// Read-back verify catches SPI/wiring errors, NOT wrong-but-valid RF values,
// and it CANNOT tell you that this table disagrees with the STM32's.
// Every change here must be mirrored in PacketSender/Core/Src/cc1101.c.
//------------------------------------------------------------------------------
module ConfigSeq #(
    parameter int CLK_HZ     = 125_000_000,
    parameter int POWERUP_US = 50_000,
    // PATABLE = output power. 0x12 is roughly -30 dBm at 433 MHz: the MINIMUM.
    // Two radios on the same bench are ~10 cm apart, which is far too loud at
    // full power -- the receiver's front end saturates and the link fails in a
    // way that looks like a config or logic bug. Turn this up only once the
    // radios are metres apart. (0x60 ~ 0 dBm, 0xC0 ~ +10 dBm.)
    // PATABLE = Power Amplifiable Table
    parameter logic [7:0] PATABLE_VAL = 8'h12,
    // Longest legitimate wait on drv_done is CMD_RESET, which includes the
    // driver's 1 ms post-SRES settle. 50 ms is ~50x that -- long enough never to
    // fire on a healthy chip, short enough that a dead one reports promptly.
    parameter int DRV_TIMEOUT_MS = 50
)(
    input  logic       clk,
    input  logic       rst,
    input  logic       start,         // re-run (auto-runs once after reset)
    input  logic       enable,

    output logic [2:0] cmd,
    output logic cmd_valid,
    output logic [7:0] cmd_addr,
    output logic [7:0] cmd_data,
    input logic[7:0]  rd_data,

    output logic busy,
    output logic [7:0] fail_addr,
    output logic [7:0] fail_value,
    output logic config_ok,
    output logic done,
    output logic timeout,
    input logic drv_done
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
        16'h0B_0C,  // FSCTRL1   IF 304.7 kHz. NOT a datasheet figure -- TI only
                    //           publishes 152 kHz @ 38.4k and 355 kHz @ 500k, so
                    //           this is interpolated for the 540 kHz filter.
                    //           FIRST thing to check against SmartRF Studio.
        16'h0C_00,  // FSCTRL0
        16'h0D_10,  // FREQ2     \
        16'h0E_B0,  // FREQ1      >  ~433.92 MHz @ 26 MHz xtal -- BAND SPECIFIC
        16'h0F_71,  // FREQ0     /
        16'h10_2D,  // MDMCFG4   249,939 bps, 541.7 kHz channel BW
                    //           (CHANBW_E=0, CHANBW_M=2, DRATE_E=13)
        16'h11_3B,  // MDMCFG3   DRATE_M=59; the exponent is in MDMCFG4[3:0]
        16'h12_13,  // MDMCFG2   GFSK, 30/32 sync-word detection. 8'b0 001 0 011
        16'h13_62,  // MDMCFG1   16 preamble bytes = 512 us for the receiver's
                    //           AGC to settle. Was 4 bytes: fine at 38.4k
                    //           (833 us) but only 128 us at 250k, which the
                    //           earlier notes already called marginal. Costs
                    //           ~28 spans of budget; worth it.
        16'h14_F8,  // MDMCFG0
        16'h15_62,  // DEVIATN   +/-127.0 kHz, which is exactly TI's published
                    //           250 kBaud GFSK figure (DEVIATION_E=6, _M=2).
                    //           Was 0x35 = 20.6 kHz, correct only for 38.4k.
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
        PWR_S, RST_S, RST_WS, WR_S, WR_WS, RD_S, RD_WS, DONE_S, IDLE_S
    } cstate_t;
    cstate_t cstate;

    logic [31:0] dly; // reset delay
    logic [31:0] wdt; //watchdog

    logic verify_ok;

    logic [31:0] idx;
    wire [5:0] idx_c = (idx < N_CFG) ? idx[5:0] : 6'b0;
    // MUST be wire, not logic. `logic x = expr;` at module scope is a variable
    // declaration initializer -- evaluated once at time zero, like an initial
    // block -- so these would freeze at entry 0 and never track idx. Only nets
    // get continuous-assignment semantics.
    wire [7:0] curr_addr  = CFG_FLAT[(N_CFG-idx_c)*16-1 -:8];
    wire [7:0] curr_value = CFG_FLAT[(N_CFG-1-idx_c)*16 +:8];

    always_ff @(posedge clk) begin
        if(rst)begin
            cstate     <= PWR_S;
            dly        <= '0;
            wdt        <= '0;
            idx        <= '0;
            verify_ok  <= 1'b0;
            config_ok  <= 1'b0;
            timeout    <= 1'b0;
            busy       <= 1'b0;
            done       <= 1'b0;
            cmd_valid  <= 1'b0;
            cmd        <= 3'd0;
            cmd_addr   <= 8'h00;
            cmd_data   <= 8'h00;
            fail_addr  <= 8'h00;
            fail_value <= 8'h00;
        end else begin
            cmd_valid <= 1'b0;
            done <= 1'b0;
            case (cstate)
                PWR_S: // POWER STATE
                begin
                    wdt <= '0;
                    busy <= 1'b1;
                    if(!enable) begin
                        // Park here while disabled. dly restarts from zero on
                        // enable, so the chip still gets its full settle time.
                        busy <= 1'b0;
                        dly  <= '0;
                    end else if(dly == PWR_CYCLES - 1) begin
                        dly <= '0;          // so a later reset still gets the full wait
                        cstate <= RST_S;
                    end else begin
                        dly <= dly + 1'b1;
                    end
                end

                RST_S: // RESET STATE
                begin
                    cmd <= CMD_RESET;
                    cmd_valid <= 1'b1;
                    wdt <= 1'b0;
                    idx <= '0;
                    verify_ok <= 1'b1;
                    cstate <= RST_WS;
                end

                RST_WS: // RESET WAIT STATE
                begin
                    if(drv_done)begin
                        cstate <= WR_S;
                    end else begin
                        if(wdt == WDT_CYCLES) begin
                            fail_addr <= curr_addr;
                            fail_value <= curr_value;
                            verify_ok <= 1'b0;
                            cstate <= DONE_S;
                        end else begin
                            wdt <= wdt + 1'b1;
                        end
                    end
                end

                WR_S: // WRITE STATE
                begin
                    cmd_valid <= 1'b1;
                    cmd_addr <= curr_addr;
                    cmd_data <= curr_value;
                    cmd <= CMD_WRITE_REG;
                    cstate <= WR_WS;
                    wdt <= 1'b0;
                end

                WR_WS: // WRITE WAIT STATE
                begin
                    if(drv_done)begin
                        if(idx == N_CFG-1)begin
                            idx <= '0;
                            cstate <= RD_S;
                        end else begin
                            idx <= idx + 1'b1;
                            cstate <= WR_S;
                        end
                    end else begin
                        if(wdt == WDT_CYCLES) begin
                            fail_addr <= curr_addr;
                            fail_value <= curr_value;
                            verify_ok <= 1'b0;
                            timeout <= 1'b1;
                            cstate <= DONE_S;
                        end else begin
                            wdt <= wdt + 1'b1;
                        end
                    end
                end

                RD_S: // READ STATE
                begin
                    // Test curr_addr (the current ROM entry), NOT cmd_addr -- that
                    // is the last address handed to the driver, which still holds
                    // PATABLE from the write phase and would end the verify before
                    // it ran a single read. The idx guard means termination does
                    // not depend on PATABLE happening to be the last ROM entry.
                    if (idx >= N_CFG || curr_addr == PATABLE_ADDR)begin
                        cstate <= DONE_S;
                    end else begin
                        cmd_valid <= 1'b1;
                        cmd_addr <= curr_addr;
                        cmd <= CMD_READ_REG;
                        wdt <= '0;
                        cstate <= RD_WS;
                    end
                end

                RD_WS: // READ WAIT STATE
                begin
                    if(drv_done)begin
                        if(rd_data != curr_value && verify_ok) begin
                            fail_addr <= curr_addr;
                            fail_value <= rd_data;
                            verify_ok <= 1'b0;
                        end
                        idx <= idx + 1'b1;
                        cstate <= RD_S;
                    end else begin
                        if(wdt == WDT_CYCLES) begin
                            fail_addr <= curr_addr;
                            fail_value <= curr_value;
                            verify_ok <= 1'b0;
                            cstate <= DONE_S;
                            timeout <= 1'b1;
                        end else begin
                            wdt <= wdt + 1'b1;
                        end
                    end
                end

                DONE_S: // DONE STATE -- the only place config_ok is published
                begin
                    config_ok <= verify_ok;
                    done <= 1'b1;
                    busy <= 1'b0;
                    cstate <= IDLE_S;
                end

                IDLE_S: // IDLE STATE -- wait for a re-run request
                begin
                    if(start && enable) begin
                        // Invalidate the previous run's verdict up front so a
                        // run in progress can never look like a passing one.
                        config_ok <= 1'b0;
                        timeout <= 1'b0;
                        fail_addr <= 8'h00;
                        fail_value <= 8'h00;
                        cstate <= RST_S;
                        busy <= 1'b1;
                    end
                end

                // 9 states in a 4-bit enum leaves 7 illegal encodings. Without
                // this, landing in one would hang the FSM permanently.
                default: cstate <= PWR_S;
            endcase
        end
    end

endmodule
