`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// tb_scanout  --  milestone-11 testbench for ScanoutEngine
//
// Runs the real ScanoutEngine against the real PointRam, with a stub in place
// of Mcp4922Driver (that module is verified independently by tb_mcp4922, and a
// stub keeps the sim fast enough to check several full frames).
//
// Captures every emitted (x,y) and checks:
//   1. the first point of each segment lands EXACTLY on its corner
//   2. exactly n_active * STEPS points per frame
//   3. the shape CLOSES -- the last segment runs from the final corner back
//      to corner 0, which is the easiest thing to get wrong
//   4. interpolation is monotonic and evenly spaced along each segment
//   5. a second frame repeats identically (the walk wraps cleanly)
//
// Check 1 is the one that catches the fixed-point arithmetic: the accumulator
// must land on the corner with no drift after STEPS increments.
//------------------------------------------------------------------------------
module tb_scanout;

    localparam int STEPS    = 16;
    localparam int N_ACTIVE = 3;

    logic clk = 0;
    always #4 clk = ~clk;                 // 125 MHz

    logic rst = 1;

    // ---- DUT wiring ----
    logic [7:0]  raddr;
    logic [17:0] rdata;
    logic        start, done;
    logic [11:0] x, y;
    logic [7:0]  corner_idx;
    logic [15:0] frame_count;

    PointRam #(.N_POINTS(255), .DWELL_BITS(2)) ram (
        .clk(clk),
        .we(1'b0), .waddr(8'd0), .wdata(18'd0),
        .raddr(raddr), .rdata(rdata)
    );

    ScanoutEngine #(.N_POINTS(255), .STEPS(STEPS)) dut (
        .clk(clk), .rst(rst),
        .n_active(8'(N_ACTIVE)),
        .raddr(raddr), .rdata(rdata),
        .start(start), .x(x), .y(y), .done(done),
        .corner_idx(corner_idx), .frame_count(frame_count)
    );

    // ---- stub DAC: assert done a few cycles after each start ----
    // The real driver takes ~1.7 us per point; 6 cycles keeps the sim quick
    // while still exercising the start/done handshake honestly.
    int unsigned dly;
    always_ff @(posedge clk) begin
        if (rst) begin
            done <= 1'b0;
            dly  <= 0;
        end else begin
            done <= 1'b0;
            if (start)      dly <= 6;
            else if (dly>1) dly <= dly - 1;
            else if (dly==1) begin dly <= 0; done <= 1'b1; end
        end
    end

    // ---- capture every emitted point ----
    localparam int MAXPTS = 512;
    int  cap_x [0:MAXPTS-1];
    int  cap_y [0:MAXPTS-1];
    int  n_cap = 0;

    always_ff @(posedge clk) begin
        if (!rst && start && n_cap < MAXPTS) begin
            cap_x[n_cap] <= int'(x);
            cap_y[n_cap] <= int'(y);
            n_cap        <= n_cap + 1;
        end
    end

    int errors = 0;

    task check(input string what, input int got, input int exp);
        if (got !== exp) begin
            $display("  FAIL  %0s: got %0d, expected %0d", what, got, exp);
            errors++;
        end else $display("  ok    %0s = %0d", what, got);
    endtask

    // Expected corners, in DAC units (8-bit corner shifted left by 4).
    int cx [0:2];
    int cy [0:2];

    int i, seg, k, step_x, step_y, base;

    initial begin
        $display("\n=== milestone 11: ScanoutEngine ===\n");

        // must match PointRam's initial block
        cx[0] = 128*16;  cy[0] = 230*16;
        cx[1] =  30*16;  cy[1] =  40*16;
        cx[2] = 226*16;  cy[2] =  40*16;

        repeat (10) @(posedge clk);
        rst = 0;

        // Two full frames = 2*3*16 = 96 points. Wait for one point PAST that:
        // capture happens at `start`, but frame_count increments when the
        // closing segment's last point finishes, one handshake later.
        wait (n_cap >= 2*N_ACTIVE*STEPS + 1);
        @(posedge clk);

        // ---------- 1. corners land exactly ----------
        $display("--- segment start points land on corners ---");
        for (seg = 0; seg < N_ACTIVE; seg++) begin
            check($sformatf("corner %0d  x", seg), cap_x[seg*STEPS], cx[seg]);
            check($sformatf("corner %0d  y", seg), cap_y[seg*STEPS], cy[seg]);
        end

        // ---------- 2. the shape closes ----------
        // Segment 2 must run from corner 2 back to corner 0, so the point
        // immediately after it is corner 0 again. If the closing segment were
        // missing, this would be corner 1.
        $display("\n--- closing segment (corner 2 -> corner 0) ---");
        check("point after last segment x", cap_x[N_ACTIVE*STEPS], cx[0]);
        check("point after last segment y", cap_y[N_ACTIVE*STEPS], cy[0]);

        // ---------- 3. even spacing within a segment ----------
        // Every step along a segment must advance by the same amount, and the
        // segment must cover exactly the full corner-to-corner delta.
        $display("\n--- interpolation is even ---");
        for (seg = 0; seg < N_ACTIVE; seg++) begin
            base   = seg*STEPS;
            step_x = cx[(seg+1)%N_ACTIVE] - cx[seg];    // full segment delta
            step_y = cy[(seg+1)%N_ACTIVE] - cy[seg];
            for (k = 1; k < STEPS; k++) begin
                // Same arithmetic the RTL does: accumulate the raw delta in
                // FRAC fractional bits, then take the integer part.
                if (cap_x[base+k] !== ((cx[seg]*STEPS + k*step_x) / STEPS)) begin
                    $display("  FAIL  seg %0d step %0d x: got %0d, expected %0d",
                             seg, k, cap_x[base+k],
                             (cx[seg]*STEPS + k*step_x)/STEPS);
                    errors++;
                end
                if (cap_y[base+k] !== ((cy[seg]*STEPS + k*step_y) / STEPS)) begin
                    $display("  FAIL  seg %0d step %0d y: got %0d, expected %0d",
                             seg, k, cap_y[base+k],
                             (cy[seg]*STEPS + k*step_y)/STEPS);
                    errors++;
                end
            end
            $display("  ok    segment %0d: %0d steps spanning (%0d, %0d)",
                     seg, STEPS, step_x, step_y);
        end

        // ---------- 4. the second frame is identical ----------
        $display("\n--- frame 2 repeats frame 1 ---");
        for (i = 0; i < N_ACTIVE*STEPS; i++) begin
            if (cap_x[i] !== cap_x[i + N_ACTIVE*STEPS] ||
                cap_y[i] !== cap_y[i + N_ACTIVE*STEPS]) begin
                $display("  FAIL  point %0d differs between frames", i);
                errors++;
            end
        end
        if (errors == 0) $display("  ok    all %0d points match", N_ACTIVE*STEPS);

        // ---------- 5. frame counter ----------
        $display("\n--- housekeeping ---");
        check("frame_count", int'(frame_count), 2);

        $display("\n=====================================");
        if (errors == 0) $display("PASS -- scanout walks and interpolates correctly");
        else             $display("FAIL -- %0d check(s) failed", errors);
        $display("=====================================\n");
        $finish;
    end

    initial begin
        #2_000_000;
        $display("\nFAIL -- testbench timed out (FSM wedged?)");
        $finish;
    end

endmodule
