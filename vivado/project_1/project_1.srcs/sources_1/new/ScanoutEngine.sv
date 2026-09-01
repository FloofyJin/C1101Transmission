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
// ---- the step count is COMPUTED, not fixed ----
// A fixed step count is unusable: every segment would take the same TIME
// regardless of its LENGTH, so a 10-unit span and a 176-unit span would be
// drawn at a 17x speed difference. That shows up three ways on the scope --
// straight ramps bending into S-curves, a brightness gradient across the
// image, and long spans breaking into visible dots while short ones are
// over-drawn. All one cause.
//
// Instead, put every DAC point the same DISTANCE from its neighbour:
//
//     n_steps = ceil(|delta| / SPACING), rounded UP to a power of two
//
// Constant distance per update means constant beam speed, so brightness is
// uniform with no compensation, and the DAC budget self-regulates: cost tracks
// total path length rather than segment count.
//
// ---- and the interpolation is still exact, with no divider ----
// The old scheme used FRAC = log2(STEPS) so the increment was the raw delta.
// That breaks once the step count varies. Keeping FRAC FIXED at 10 and folding
// the variation into the increment instead restores it:
//
//     n_steps  = 2^k
//     acc      = P0 << 10                 at the start of each segment
//     acc     += delta << (10 - k)        once per step
//     out      = acc >>> 10               fixed shift, no barrel shifter
//
// After 2^k steps the accumulator has advanced 2^k * delta * 2^(10-k), which
// is exactly delta << 10 -- so it lands precisely on P1 with no drift, for any
// k. Rounding n_steps UP to a power of two means the real spacing is always
// <= SPACING, never coarser.
//
// Interpolating in 8-bit space and shifting up afterwards would throw away the
// sub-steps; the low 4 bits are where the smoothness lives.
//
// ---- blanking (Z axis) ----
// Segments alternate because scanline fill sends points in PAIRS:
//
//   p0 -> p1   even   SPAN        draw
//   p1 -> p2   odd    connector   depends
//   p2 -> p3   even   SPAN        draw
//
// EVERY connector is blanked:
//
//   blank = odd segment
//
// An earlier version drew the serpentine row-steps, on the reasoning that they
// are ~1 unit long, sit on the silhouette's edge, and outline the shape for
// free. That holds only while consecutive rows have similar span layouts. A
// checkerboard breaks it: at a band boundary the columns shift 32 units, the
// row step becomes a long horizontal jump, and it renders as a bright streak
// straight through the image.
//
// Blanking all connectors costs nothing real. The span ENDPOINTS already trace
// the silhouette's edge -- consecutive rows differ by about a beam spot -- so
// the outline is still there; only the sub-pixel joining line goes away.
//
// The wrap (last point back to point 0) is odd whenever n_active is even, and
// n_active is always even because points are span pairs. So parity covers it.
//
// This is DERIVED rather than transmitted. Blanking is a property of a segment,
// so it needs both endpoints; RxSeq only ever has one at a time (the neighbour
// may be in a different packet), and sending a bit per point would cost ~6% of
// the link. ScanoutEngine has p0 and p1 in hand, so it is the only place the
// value is free.
//
// NOTE this assumes the scanline pairing. A closed polyline that is not made of
// point pairs -- the old triangle, for instance -- will blank the wrong
// segments. rdata[1:0] is reserved as a future explicit override.
//
// ---- the blank verdict lags by one point ----
// z_blank describes where the BEAM is, and the beam is one DAC transaction
// behind this state machine. Emitting a point at S_EMIT only starts a ~1.6 us
// SPI write; the output does not move until Mcp4922Driver pulses LDAC and
// raises `done`. So at S_SEG -- where the new segment's blank value is known --
// the DAC is still parked on the PREVIOUS segment's last point.
//
// Driving z_blank there un-blanks a whole point period too early, and the beam
// spends it standing in the middle of the gap it was supposed to cross dark:
//
//   gap 31 -> 64, blanked, n_steps = 4 after quartering
//     L->R row   31.0  39.25  47.5  55.75      unblanks HERE, then jumps to 64
//     R->L row   64.0  55.75  47.5  39.25      unblanks HERE, then jumps to 31
//
// One bright point per gap per row, at a fixed x, stacked over 128 rows -- so a
// checkerboard grew two vertical lines inside every gap, at 1/4 and 3/4 across,
// one from each serpentine direction. The mirror image was also true: z_blank
// went HIGH a point early, cutting every span one SPACING short of its endpoint
// and serrating the square edges.
//
// So the verdict is latched at S_SEG and applied at `done`, when LDAC has fired
// and the beam is standing on the point the verdict belongs to. The transit
// INTO a span's first point then still carries the connector's blank -- which is
// right, it is geometrically the tail of the gap -- and the transit into a
// connector's first point still carries the span's, so spans reach their
// endpoints. Both artifacts are the same bug and go away together.
//------------------------------------------------------------------------------
module ScanoutEngine #(
    parameter int N_POINTS = 1024,       // PointRam depth PER BANK
    // Distance between consecutive DAC points, in 8-bit coordinate units.
    // MUST be a power of two -- the divide becomes a shift. Set it so one
    // SPACING lands within a beam-spot width at your display scale; if the
    // dots do not merge into a line, this is the number to lower.
    parameter int SPACING  = 4
)(
    input  logic        clk,
    input  logic        rst,

    // How many points are live in the current frame. Latched by topRF from the
    // RF side at the buffer swap -- NOT a parameter, so the vectorizer picks
    // the fps/detail trade per frame without an RTL change.
    input  logic [IDX_W:0] n_active,

    // Which bank to read. Flipped by topRF at the wrap; `restart` comes with it
    // so the walk re-fetches both endpoints from the new bank rather than
    // straddling the swap.
    input  logic        rd_bank,
    input  logic        restart,

    // ---- PointRam read port (1-cycle latency) ----
    output logic [IDX_W:0] raddr,        // {bank, index}
    input  logic [17:0] rdata,           // {x[7:0], y[7:0], spare[1:0]}

    // ---- Mcp4922Driver ----
    output logic        start,           // 1-cycle pulse
    output logic [11:0] x,
    output logic [11:0] y,
    input  logic        done,            // one point finished (LDAC fired)

    // ---- oscilloscope Z axis (1 = beam off) ----
    output logic        z_blank,

    // ---- status / ILA ----
    output logic [IDX_W-1:0] corner_idx, // which segment is being drawn
    output logic        wrap,            // 1-cycle: just finished the last segment
    output logic [15:0] frame_count      // completed passes over the list
);
    localparam int IDX_W = $clog2(N_POINTS); // 10
    // Coordinates are 8-bit but the DAC is 12-bit, so one coordinate unit is
    // 16 DAC counts.
    localparam int SPACING_DAC = SPACING * 16;
    localparam int LOG2_SPD    = $clog2(SPACING_DAC);

    // FRAC is FIXED at 10 even though the step count varies. That is the whole
    // trick -- see the increment derivation below.
    localparam int FRAC = 10;
    localparam int ACCW = 12 + FRAC + 1;      // 23 bits, +1 for sign

    typedef enum logic [2:0] {
        S_INIT,
        S_P0A, S_P0B,      // fetch the first corner
        S_P1A, S_P1B,      // fetch the next corner
        S_SEG,             // compute the segment
        S_EMIT, S_WAIT
    } st_t;
    st_t state;

    logic [IDX_W-1:0] cidx;              // internal copy of corner_idx
    logic [17:0] p0, p1;
    logic signed [ACCW-1:0] acc_x, acc_y;
    logic signed [ACCW-1:0] dx, dy;          // delta, pre-scaled by 2^(FRAC-k)
    logic [7:0]  s_cnt;        // step within the current segment
    logic [3:0]  k;            // n_steps = 2^k, latched per segment
    logic        blank_pend;   // blank verdict for the point still in the DAC

    // Guard the count: 0 would make the index arithmetic wrap oddly. A value
    // past the RAM depth cannot occur -- n_active is IDX_W wide -- but 0 can,
    // if a frame arrives empty.
    // n_active is IDX_W+1 wide so a full-buffer frame (1024) is representable.
    // This is number of points on frame. when zero, it defaults to 1.
    wire [IDX_W:0] n_eff = (n_active == '0) ? {{IDX_W{1'b0}}, 1'b1} : n_active;

    // Next / next-next corner, wrapping. corner_idx has not advanced yet when
    // these are used, so both are computed from its current value. The compare
    // is done at IDX_W+1 bits, or n_eff == 1024 would never be reached.
    wire [IDX_W:0] cn = {1'b0, corner_idx} + 1'b1;
    wire [IDX_W-1:0] idx_n  = (cn >= n_eff) ? '0 : cn[IDX_W-1:0];
    wire [IDX_W:0] nn = {1'b0, idx_n} + 1'b1;
    wire [IDX_W-1:0] idx_nn = (nn >= n_eff) ? '0 : nn[IDX_W-1:0];

    // 8-bit corners shifted into the DAC's 12-bit space.
    wire [11:0] p0x12 = {p0[17:10], 4'b0};
    wire [11:0] p0y12 = {p0[9:2],   4'b0};
    wire [11:0] p1x12 = {p1[17:10], 4'b0};
    wire [11:0] p1y12 = {p1[9:2],   4'b0};

    // The DAC sees the integer part of the accumulator.
    assign x = acc_x[FRAC+11 -: 12];
    assign y = acc_y[FRAC+11 -: 12];

    // Points come in PAIRS, so even segments are spans and odd are connectors.
    // n_active is always even, which makes the wrap odd too.
    wire blank_w = corner_idx[0];

    // ---- how many steps this segment needs ----
    // Work from the LARGER axis so diagonal connectors are spaced too. For a
    // scanline span dy is zero, so this is just |dx|.
    wire signed [12:0] dx_w = $signed({1'b0, p1x12}) - $signed({1'b0, p0x12});
    wire signed [12:0] dy_w = $signed({1'b0, p1y12}) - $signed({1'b0, p0y12});
    wire [12:0] adx  = dx_w[12] ? (~dx_w + 13'd1) : dx_w;
    wire [12:0] ady  = dy_w[12] ? (~dy_w + 13'd1) : dy_w;
    wire [12:0] amax = (adx > ady) ? adx : ady;

    // q = ceil(|delta| / SPACING_DAC). Ceiling, so spacing errs FINER.
    wire [12:0] q = (amax + SPACING_DAC[12:0] - 13'd1) >> LOG2_SPD;

    // k = ceil(log2(q)), i.e. the smallest power of two >= q. A zero-length
    // segment still gets one step so its start point is emitted.
    logic [3:0] k_w;
    always_comb begin
        if      (q <= 13'd1)   k_w = 4'd0;
        else if (q <= 13'd2)   k_w = 4'd1;
        else if (q <= 13'd4)   k_w = 4'd2;
        else if (q <= 13'd8)   k_w = 4'd3;
        else if (q <= 13'd16)  k_w = 4'd4;
        else if (q <= 13'd32)  k_w = 4'd5;
        else if (q <= 13'd64)  k_w = 4'd6;
        else if (q <= 13'd128) k_w = 4'd7;
        else                   k_w = 4'd8;      // clamp: 256 steps
    end
    // A BLANKED segment is invisible, so it needs no interpolation density --
    // only enough steps that the DAC settles before the next visible point.
    // Quartering them is a large saving on gap-heavy images: a checkerboard
    // spends 61% of its points crossing gaps, which is the difference between
    // 629 k/s (over the ceiling) and ~390 k/s.
    //
    // Quarter rather than collapse to one step: a single jump across a wide
    // gap is a full-scale DAC step that may not settle inside one point
    // period, which would smear the START of the next visible span.
    wire [3:0] k_eff = blank_w ? ((k_w > 4'd2) ? k_w - 4'd2 : 4'd0) : k_w;

    wire [8:0] n_steps = 9'd1 << k_eff;    // for the segment being SET UP
    wire [8:0] n_steps_cur = 9'd1 << k;    // for the segment being DRAWN

    // Blanking verdict for the segment about to be drawn. Latched at S_SEG into
    // blank_pend, where both endpoints are valid, and transferred to z_blank one
    // point later -- see "the blank verdict lags by one point" above.
    always_ff @(posedge clk) begin
        if (rst) begin
            state       <= S_INIT;
            raddr       <= 8'd0;
            p0          <= 18'd0;  p1 <= 18'd0;
            acc_x       <= '0;     acc_y <= '0;
            dx          <= '0;     dy <= '0;
            s_cnt       <= 8'd0;
            k           <= 4'd0;
            corner_idx  <= '0;
            wrap        <= 1'b0;
            frame_count <= 16'd0;
            start       <= 1'b0;
            z_blank     <= 1'b1;             // beam off until we know better
            blank_pend  <= 1'b1;
        end else if (restart) begin
            // Bank just swapped. Re-fetch both endpoints from the new bank
            // rather than continuing with a p0 read from the old one.
            state      <= S_INIT;
            start      <= 1'b0;
            wrap       <= 1'b0;
            z_blank    <= 1'b1;
            blank_pend <= 1'b1;
        end else begin
            start <= 1'b0;                       // one-cycle-pulse defaults
            wrap  <= 1'b0;

            case (state)
                S_INIT: begin
                    raddr      <= {rd_bank, {IDX_W{1'b0}}};
                    corner_idx <= '0;
                    z_blank    <= 1'b1;
                    blank_pend <= 1'b1;
                    state      <= S_P0A;
                end

                // One dead cycle per fetch: raddr is registered here, PointRam
                // registers its read, so rdata is valid two cycles after the
                // address is issued.
                S_P0A: state <= S_P0B;
                S_P0B: begin
                    p0    <= rdata;
                    raddr <= {rd_bank, idx_n};
                    state <= S_P1A;
                end

                S_P1A: state <= S_P1B;
                S_P1B: begin
                    p1    <= rdata;
                    state <= S_SEG;
                end

                S_SEG: begin
                    // NOT z_blank -- see "the blank verdict lags by one point"
                    // above. The DAC is still holding the PREVIOUS segment's
                    // last point at this moment.
                    blank_pend <= blank_w;
                    k          <= k_eff;
                    // acc starts exactly on P0. The increment is the delta
                    // pre-scaled by 2^(FRAC-k), so 2^k of them land on P1.
                    acc_x <= {{(ACCW-12-FRAC){1'b0}}, p0x12, {FRAC{1'b0}}}; // {0, p0x12, 10'b0}
                    acc_y <= {{(ACCW-12-FRAC){1'b0}}, p0y12, {FRAC{1'b0}}};
                    dx    <= dx_w <<< (FRAC[3:0] - k_eff);
                    dy    <= dy_w <<< (FRAC[3:0] - k_eff);
                    s_cnt <= 8'd0;
                    state <= S_EMIT;
                end

                S_EMIT: begin
                    start <= 1'b1;               // x/y are already valid
                    state <= S_WAIT;
                end

                S_WAIT: if (done) begin
                    // `done` means LDAC has fired, so the beam is NOW standing
                    // on the point that was emitted. Only here does this
                    // segment's blank verdict describe where the beam actually
                    // is; applying it any earlier blanks the wrong transit.
                    z_blank <= blank_pend;
                    if (s_cnt == n_steps_cur - 9'd1) begin
                        // Segment finished. The old P1 becomes the new P0, so
                        // only one fresh corner has to be fetched.
                        p0         <= p1;
                        corner_idx <= idx_n;
                        raddr      <= {rd_bank, idx_nn};
                        if (idx_n == '0) begin
                            frame_count <= frame_count + 16'd1;
                            wrap        <= 1'b1;   // topRF may swap banks now
                        end
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
