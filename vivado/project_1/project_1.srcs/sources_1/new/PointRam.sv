`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// PointRam  --  the coordinate list the display scans out
//
// 255 entries of {x[7:0], y[7:0], dwell[1:0]} = 18 bits, which is exactly a
// BRAM-friendly width on Zynq-7000 (36Kb blocks configure as 2K x 18).
//
// Simple dual-port: the RF side writes, the scanout side reads. They are
// deliberately independent -- this module is the whole reason the radio and the
// display do not have to agree on a rate. The radio delivers ~36 full frames/s;
// the scanout walks this RAM at ~60 Hz regardless of whether anything arrived.
// Neither side ever waits for the other.
//
// dwell is stored per point but is NOT transmitted -- the RF write path fills
// it from a parameter. The field exists so that multi-contour frames and a
// Z-input scope do not require a width change later. See ARCHITECTURE.md
// "Dwell is FPGA-side".
//
// No write-conflict logic: the RF side is the only writer, and a read of an
// entry being written in the same cycle returns the OLD value (read-first).
// That is harmless here -- the worst case is one scanout pass showing a point
// one frame stale, which is exactly what a dropped packet does anyway.
//------------------------------------------------------------------------------
module PointRam #(
    parameter int N_POINTS   = 255,
    parameter int DWELL_BITS = 2
)(
    input  logic clk,

    // ---- write port: the RF side ----
    input  logic                          we,
    input  logic [7:0]                    waddr,
    input  logic [16+DWELL_BITS-1:0]      wdata,   // {x, y, dwell}

    // ---- read port: the scanout side ----
    input  logic [7:0]                    raddr,
    output logic [16+DWELL_BITS-1:0]      rdata
);
    localparam int W = 16 + DWELL_BITS;

    (* ram_style = "block" *) logic [W-1:0] mem [0:N_POINTS-1];

    // Zeroed so simulation does not start at X and so an un-uploaded display
    // parks the beam at the origin rather than somewhere random.
    initial begin
        for (int i = 0; i < N_POINTS; i++) mem[i] = '0;
    end

    always_ff @(posedge clk) begin
        if (we) mem[waddr] <= wdata;
        rdata <= mem[raddr];        // 1-cycle read latency
    end
endmodule
