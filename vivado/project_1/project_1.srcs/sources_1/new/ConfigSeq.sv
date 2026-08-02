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
                    if(dly == PWR_CYCLES - 1) begin
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
                    if(start) begin
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
