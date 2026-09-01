`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// PointRam  --  the coordinate list the display scans out
//
// TWO BANKS of N_POINTS entries, each {x[7:0], y[7:0], spare[1:0]} = 18 bits
// -- a natural BRAM width on Zynq (36Kb blocks configure as 2K x 18), so the
// two spare bits are free.
//
// The spare bits are written as zero and read by nobody. Blanking is DERIVED
// in ScanoutEngine from segment parity, not stored here; these two are held
// in reserve as a future EXPLICIT blank override, for artwork that is not
// made of scanline point pairs. (They were called dwell bits until 2026-08-11,
// when per-point dwell was cancelled -- Z blanking replaced it.)
//
// Address is {bank, index}. The RF side writes one bank while the scanout side
// reads the other; on end-of-frame they swap. Single-buffered, roughly a third
// of a frame's packets land during each redraw, so every drawn frame would be
// part old and part new with a tear line sweeping down the image.
//
//   1024 x 2 banks x 18 bits = 37 kbit  of the Zynq-7010's ~2.1 Mbit -> 1.7%
//
// Size the RAM generously and let n_active pick the frame size at RUNTIME.
// Baking a smaller depth in would lock the fps/detail trade into the RTL; at
// 1024 the vectorizer chooses per frame -- 19 packets for a simple frame at
// 28 fps, 27 for a complex one at 20 fps.
//
// The two banks are independent regions of one array, so a swap moves nothing.
// Copying 800 entries would take time, need arbitration, and could tear
// mid-copy; flipping one address bit cannot.
//
// No write-conflict logic: the RF side is the only writer, and a read of an
// entry being written in the same cycle returns the OLD value. Harmless -- the
// banks are disjoint, so that case does not arise in normal operation.
//------------------------------------------------------------------------------
module PointRam #(
    parameter int N_POINTS   = 1024,     // per ban
    parameter int SPARE_BITS = 2
)(
    input  logic clk,

    // ---- write port: the RF side ----
    input  logic                          we,
    input  logic [ADDR_W-1:0]             waddr,   // {bank, index}
    input  logic [17:0]                   wdata,   // {x, y, spare[1:0]}

    // ---- read port: the scanout side ----
    input  logic [ADDR_W-1:0]             raddr,   // {bank, index}
    output logic [17:0]                   rdata    // {x, y, spare[1:0]}
);
    localparam int W      = 16 + SPARE_BITS;
    localparam int IDX_W  = $clog2(N_POINTS);
    localparam int ADDR_W = IDX_W + 1;            // +1 for the bank select

    (* ram_style = "block" *) logic [W-1:0] mem [0:(2*N_POINTS)-1];

    // Zeroed so simulation does not start at X and an un-uploaded display parks
    // the beam at the origin rather than somewhere random.
    //
    // Then seeded, in BOTH banks, with a CHECKERBOARD.
    //
    //   CK_ROWS scanlines x CK_COLS columns.  Half the columns are filled, so
    //   there are CK_COLS/2 SPANS per row and CK_COLS points per row:
    //
    //     col:  0    1    2    3    4    5    6    7
    //         ####      ####      ####      ####        4 filled = 4 spans
    //             ####      ####      ####      ####    next band, still 4
    //
    //   total points = CK_ROWS * CK_COLS   (must be <= N_POINTS)
    //
    // Chosen over a triangle because it is the first pattern with MULTIPLE
    // SPANS PER ROW -- the case a single-span shape never exercises. The gaps
    // between squares must be blanked, or the whole thing renders as solid
    // horizontal bars. Z wrong -> stripes; Z right -> squares.
    //
    // It also stresses what a simple shape does not:
    //   - in-row gap blanking          (CK_COLS/2 - 1 gaps per row)
    //   - serpentine across many rows
    //   - a realistic n_active         (not a handful)
    //   - variable segment lengths     (spans vs gaps vs row steps)
    //
    // Rows land ~2 coordinate units apart at 128 rows, which is finer than a
    // beam spot at a ~3.3 cm image, so squares should read solid. If they look
    // striped the spot is smaller than that: shrink the image or lower SPACING.
    //
    // 128 x 8 = 1024 points, which exactly fills a bank. Drop CK_ROWS if you
    // want headroom, or raise CK_COLS for more squares at fewer rows.
    //
    // Both banks are seeded so the display is sane whichever bank comes up
    // first. Putting the pattern HERE rather than in a separate ROM means
    // scanout is exercised over the real data path; an RF upload overwrites it.
    localparam int CK_ROWS = 128;                 // scanlines
    localparam int CK_COLS = 8;                   // columns -> CK_COLS/2 spans
    localparam int CK_BAND = CK_ROWS / CK_COLS;   // scanlines per square
    localparam int CK_W    = 256 / CK_COLS;       // column width in coord units
    localparam int CK_DY   = 256 / CK_ROWS;       // row pitch in coord units

    integer i, row, band, col, c, base, bk;
    reg [7:0] yv, xl, xr;
    initial begin
        for (i = 0; i < 2*N_POINTS; i = i + 1) mem[i] = '0;
        for (bk = 0; bk < 2; bk = bk + 1)
            for (row = 0; row < CK_ROWS; row = row + 1) begin
                yv   = CK_DY[7:0] / 8'd2 + row[7:0] * CK_DY[7:0];
                band = row / CK_BAND;
                base = bk*N_POINTS + row*CK_COLS;
                for (c = 0; c < CK_COLS/2; c = c + 1) begin
                    // filled squares are those where (col + band) is even
                    col = 2*c + (band % 2);
                    xl  = col[7:0] * CK_W[7:0];
                    xr  = col[7:0] * CK_W[7:0] + CK_W[7:0] - 8'd1;
                    if (row % 2 == 0) begin
                        mem[base + c*2 + 0] = {xl, yv, 2'b00};   // left to right
                        mem[base + c*2 + 1] = {xr, yv, 2'b00};
                    end else begin
                        // Right to left: the WHOLE point list reverses, so span
                        // c lands at position (CK_COLS/2-1-c) and its endpoints
                        // swap. That keeps the pairing intact -- even segments
                        // stay spans, odd stay connectors.
                        mem[base + (CK_COLS/2-1-c)*2 + 0] = {xr, yv, 2'b00};
                        mem[base + (CK_COLS/2-1-c)*2 + 1] = {xl, yv, 2'b00};
                    end
                end
            end
    end

    always_ff @(posedge clk) begin
        if (we) mem[waddr] <= wdata;
        rdata <= mem[raddr];        // 1-cycle read latency
    end
endmodule
