`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// ScanoutEngine  --  Milestone 11/12: walk PointRam and draw it
//
// Reads corners out of PointRam, linearly interpolates between each consecutive
// pair, and feeds Mcp4922Driver one XY point at a time. Free-running: nothing
// gates it on the radio. This is the half of the design that makes the ~36 fps
// upload rate and the >=60 Hz refresh rate independent of each other.
//
//   forever:
//     for i in 0 .. n_active-1:
//         P0 = ram[i]
//         P1 = ram[(i+1) mod n_active]     <- the mod is what CLOSES the shape
//         for s in 0 .. STEPS-1:
//             emit lerp(P0, P1, s)
//
// ---- why there are n_active segments, not n_active-1 ----
// The last segment runs from the final corner back to corner 0. Drop it and the
// polygon is open -- a triangle draws as two sides and a gap.
//
// ---- one RAM read per SEGMENT, not per point ----
// Interpolation needs both endpoints, but they do not both have to be fetched:
// at each segment boundary the old P1 becomes the new P0, so only one new
// corner is read every STEPS points. The read port is idle 15/16 of the time at
// STEPS=16, which is why nothing here has to contend with the RF write port.
//
// ---- the interpolation is exact, with no divider ----
// Corners are 8-bit and the DAC is 12-bit, so corners shift up by 4. Give the
// accumulator FRAC = log2(STEPS) fractional bits and the per-step increment
// becomes the raw delta:
//
//     acc = P0 << FRAC                  at the start of each segment
//     acc += dx                         once per step
//     out  = acc >>> FRAC
//
// Over STEPS steps the accumulator advances STEPS*dx, and dx << FRAC IS
// dx * STEPS when FRAC = log2(STEPS). No truncation, no divider, and the beam
// lands exactly on each corner. Interpolating in 8-bit space and shifting up
// afterwards would throw away the sub-steps -- the low 4 bits are where the
// smoothness lives.
//
// ---- dwell ----
// rdata[1:0] is read but unused until M12, where it modulates how long the beam
// sits on each point (brightness ~ 1/writing speed). The field exists now so
// that adding it later is not a width change.
//------------------------------------------------------------------------------
module ScanoutEngine #(
    parameter int N_POINTS = 255,        // PointRam depth
    parameter int STEPS    = 16          // interpolation sub-steps per segment
)(
    input  logic        clk,
    input  logic        rst,

    // How many corners are live. 3 for the power-up triangle; eventually
    // whatever the RF side last loaded. Points past this are not drawn.
    input  logic [7:0]  n_active,

    // ---- PointRam read port (1-cycle latency) ----
    output logic [7:0]  raddr,
    input  logic [17:0] rdata,           // {x[7:0], y[7:0], dwell[1:0]}

    // ---- Mcp4922Driver ----
    output logic        start,           // 1-cycle pulse
    output logic [11:0] x,
    output logic [11:0] y,
    input  logic        done,            // one point finished (LDAC fired)

    // ---- status / ILA ----
    output logic [7:0]  corner_idx,      // which segment is being drawn
    output logic [15:0] frame_count      // completed passes over the list
);
    localparam int FRAC = $clog2(STEPS);
    localparam int ACCW = 12 + FRAC + 1;     // +1 for the sign bit

    typedef enum logic [2:0] {
        S_INIT,
        S_P0A, S_P0B,      // fetch the first corner
        S_P1A, S_P1B,      // fetch the next corner
        S_SEG,             // compute the segment
        S_EMIT, S_WAIT
    } st_t;
    st_t state;

    logic [17:0] p0, p1;
    logic signed [ACCW-1:0] acc_x, acc_y;
    logic signed [12:0]     dx, dy;          // +/- 4095
    logic [7:0]  s_cnt;

    // Guard the count: 0 would make the index arithmetic wrap oddly, and a
    // value past the RAM depth would read entries that were never written.
    wire [7:0] n_eff = (n_active == 8'd0)          ? 8'd1
                     : (n_active > N_POINTS[7:0])  ? N_POINTS[7:0]
                                                   : n_active;

    // Next / next-next corner, wrapping. corner_idx has not advanced yet when
    // these are used, so both are computed from its current value.
    wire [7:0] idx_n  = (corner_idx + 8'd1 >= n_eff) ? 8'd0 : corner_idx + 8'd1;
    wire [7:0] idx_nn = (idx_n      + 8'd1 >= n_eff) ? 8'd0 : idx_n      + 8'd1;

    // 8-bit corners shifted into the DAC's 12-bit space.
    wire [11:0] p0x12 = {p0[17:10], 4'b0};
    wire [11:0] p0y12 = {p0[9:2],   4'b0};
    wire [11:0] p1x12 = {p1[17:10], 4'b0};
    wire [11:0] p1y12 = {p1[9:2],   4'b0};

    // The DAC sees the integer part of the accumulator.
    assign x = acc_x[FRAC+11 -: 12];
    assign y = acc_y[FRAC+11 -: 12];

    always_ff @(posedge clk) begin
        if (rst) begin
            state       <= S_INIT;
            raddr       <= 8'd0;
            p0          <= 18'd0;  p1 <= 18'd0;
            acc_x       <= '0;     acc_y <= '0;
            dx          <= '0;     dy <= '0;
            s_cnt       <= 8'd0;
            corner_idx  <= 8'd0;
            frame_count <= 16'd0;
            start       <= 1'b0;
        end else begin
            start <= 1'b0;                       // one-cycle-pulse default

            case (state)
                S_INIT: begin
                    raddr      <= 8'd0;
                    corner_idx <= 8'd0;
                    state      <= S_P0A;
                end

                // One dead cycle per fetch: raddr is registered here, PointRam
                // registers its read, so rdata is valid two cycles after the
                // address is issued.
                S_P0A: state <= S_P0B;
                S_P0B: begin
                    p0    <= rdata;
                    raddr <= idx_n;
                    state <= S_P1A;
                end

                S_P1A: state <= S_P1B;
                S_P1B: begin
                    p1    <= rdata;
                    state <= S_SEG;
                end

                S_SEG: begin
                    // acc starts exactly on P0; the increment is the raw delta.
                    acc_x <= {{(ACCW-12-FRAC){1'b0}}, p0x12, {FRAC{1'b0}}};
                    acc_y <= {{(ACCW-12-FRAC){1'b0}}, p0y12, {FRAC{1'b0}}};
                    dx    <= $signed({1'b0, p1x12}) - $signed({1'b0, p0x12});
                    dy    <= $signed({1'b0, p1y12}) - $signed({1'b0, p0y12});
                    s_cnt <= 8'd0;
                    state <= S_EMIT;
                end

                S_EMIT: begin
                    start <= 1'b1;               // x/y are already valid
                    state <= S_WAIT;
                end

                S_WAIT: if (done) begin
                    if (s_cnt == STEPS-1) begin
                        // Segment finished. The old P1 becomes the new P0, so
                        // only one fresh corner has to be fetched.
                        p0         <= p1;
                        corner_idx <= idx_n;
                        raddr      <= idx_nn;
                        if (idx_n == 8'd0) frame_count <= frame_count + 16'd1;
                        state      <= S_P1A;
                    end else begin
                        acc_x <= acc_x + dx;
                        acc_y <= acc_y + dy;
                        s_cnt <= s_cnt + 8'd1;
                        state <= S_EMIT;
                    end
                end

                default: state <= S_INIT;
            endcase
        end
    end
endmodule
