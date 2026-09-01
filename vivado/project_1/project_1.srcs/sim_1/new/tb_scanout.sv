`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// tb_scanout  --  ScanoutEngine: walk, spatial interpolation, Z blanking
//
// Real ScanoutEngine against real PointRam, stub for Mcp4922Driver (verified
// separately by tb_mcp4922; a stub keeps the sim fast).
//
// The TB loads its own patterns through the write port rather than relying on
// PointRam's initial block, so changing the seeded pattern cannot silently
// rewrite these expectations.
//
// Segment boundaries are found by watching corner_idx rather than assuming a
// fixed points-per-segment -- with spatial spacing the step count VARIES with
// segment length, and that is the property under test.
//------------------------------------------------------------------------------
module tb_scanout;

    localparam int SPACING     = 4;
    localparam int SPACING_DAC = SPACING * 16;

    logic clk = 0;
    always #4 clk = ~clk;
    logic rst = 1;

    logic [7:0]  raddr;
    logic [17:0] rdata;
    logic        start, done, z_blank;
    logic [11:0] x, y;
    logic [7:0]  corner_idx;
    logic [15:0] frame_count;
    logic [7:0]  n_act = 8'd3;

    logic        we = 0;
    logic [7:0]  waddr;
    logic [17:0] wdata;

    PointRam #(.N_POINTS(255), .SPARE_BITS(2)) ram (
        .clk(clk), .we(we), .waddr(waddr), .wdata(wdata),
        .raddr(raddr), .rdata(rdata));

    ScanoutEngine #(.N_POINTS(255), .SPACING(SPACING)) dut (
        .clk(clk), .rst(rst), .n_active(n_act),
        .raddr(raddr), .rdata(rdata),
        .start(start), .x(x), .y(y), .done(done), .z_blank(z_blank),
        .corner_idx(corner_idx), .frame_count(frame_count));

    task load(input [7:0] a, input [7:0] px, input [7:0] py);
        begin
            @(posedge clk); we <= 1'b1; waddr <= a; wdata <= {px, py, 2'b00};
            @(posedge clk); we <= 1'b0;
        end
    endtask

    int unsigned dly;
    always_ff @(posedge clk) begin
        if (rst) begin done <= 0; dly <= 0; end
        else begin
            done <= 0;
            if (start)       dly <= 4;
            else if (dly>1)  dly <= dly - 1;
            else if (dly==1) begin dly <= 0; done <= 1; end
        end
    end

    localparam int MAXPTS = 2048;
    int cap_x[0:MAXPTS-1], cap_y[0:MAXPTS-1], cap_s[0:MAXPTS-1];
    bit cap_z[0:MAXPTS-1];
    int n_cap = 0;

    always_ff @(posedge clk)
        if (!rst && start && n_cap < MAXPTS) begin
            cap_x[n_cap] <= int'(x);
            cap_y[n_cap] <= int'(y);
            cap_z[n_cap] <= z_blank;
            cap_s[n_cap] <= int'(corner_idx);
            n_cap        <= n_cap + 1;
        end

    int errors = 0;
    task check(input string what, input int got, input int exp);
        if (got !== exp) begin
            $display("  FAIL  %0s: got %0d, expected %0d", what, got, exp);
            errors++;
        end else $display("  ok    %0s = %0d", what, got);
    endtask

    function int seg_start(input int sg);
        int i;
        begin
            seg_start = -1;
            for (i = 0; i < n_cap; i++)
                if (cap_s[i] == sg && seg_start < 0) seg_start = i;
        end
    endfunction

    int i, sg, worst, d, a, b;
    bit exp_blank, prev_blank, ok;

    initial begin
        $display("");
        $display("=== ScanoutEngine: walk + spatial spacing + blanking ===");
        $display("");

        repeat (4) @(posedge clk);
        load(8'd0, 8'd128, 8'd230);
        load(8'd1, 8'd30,  8'd40);
        load(8'd2, 8'd226, 8'd40);
        repeat (4) @(posedge clk);
        rst = 0;

        wait (frame_count == 2);
        @(posedge clk);

        $display("--- segment starts land exactly on corners ---");
        check("seg 0 x", cap_x[seg_start(0)], 128*16);
        check("seg 0 y", cap_y[seg_start(0)], 230*16);
        check("seg 1 x", cap_x[seg_start(1)],  30*16);
        check("seg 1 y", cap_y[seg_start(1)],  40*16);
        check("seg 2 x", cap_x[seg_start(2)], 226*16);
        check("seg 2 y", cap_y[seg_start(2)],  40*16);

        $display("");
        $display("--- spacing between consecutive points ---");
        // Only DRAWN segments. Blanked ones deliberately use a quarter of the
        // steps -- nobody sees them, and on gap-heavy images that is the
        // difference between fitting the DAC budget and not.
        worst = 0;
        for (i = 1; i < n_cap; i++)
            if (cap_s[i] == cap_s[i-1] && !cap_z[i]) begin
                d = (cap_x[i] > cap_x[i-1]) ? cap_x[i]-cap_x[i-1] : cap_x[i-1]-cap_x[i];
                if (d > worst) worst = d;
                d = (cap_y[i] > cap_y[i-1]) ? cap_y[i]-cap_y[i-1] : cap_y[i-1]-cap_y[i];
                if (d > worst) worst = d;
            end
        if (worst > SPACING_DAC) begin
            $display("  FAIL  worst gap %0d DAC units exceeds SPACING (%0d)",
                     worst, SPACING_DAC);
            errors++;
        end else
            $display("  ok    worst gap = %0d DAC units of %0d allowed (drawn segments)",
                     worst, SPACING_DAC);

        check("frame_count", int'(frame_count), 2);

        $display("");
        $display("=== scanline pattern: 2 rows x 2 spans ===");
        rst = 1; repeat (4) @(posedge clk);
        load(8'd0, 8'd60,  8'd100);
        load(8'd1, 8'd100, 8'd100);
        load(8'd2, 8'd160, 8'd100);
        load(8'd3, 8'd200, 8'd100);
        load(8'd4, 8'd200, 8'd120);
        load(8'd5, 8'd160, 8'd120);
        load(8'd6, 8'd100, 8'd120);
        load(8'd7, 8'd60,  8'd120);
        n_act = 8'd8;
        n_cap = 0;
        repeat (4) @(posedge clk);
        rst = 0;

        wait (frame_count >= 1);
        @(posedge clk);

        // Points come in pairs, so EVERY odd segment is a connector -- an
        // in-row gap, a serpentine row step, or the wrap. All are blanked.
        //
        // An earlier rule drew short row steps to outline the shape for free.
        // A checkerboard broke it: at a band boundary the columns shift 32
        // units and that "short" step becomes a bright streak across the
        // image. The span endpoints already trace the edge, so nothing is lost.
        //
        // cap_z[i] is sampled at the `start` pulse for point i, so it is the
        // blanking in force while the beam TRAVELS from point i-1 to point i.
        // That makes two DIFFERENT properties, and conflating them is what let
        // the bug through:
        //
        //   interior transits of segment sg   ->  blank(sg)
        //   the ENTRY transit into segment sg ->  blank(sg-1)
        //
        // The second one is the fix. Moving onto a span's first point is
        // geometrically the last chunk of the GAP, so it has to stay dark. A
        // z_blank that flipped with the segment index un-blanked it a full DAC
        // point early and left the beam standing in the gap -- which stacked
        // over 128 rows into two vertical lines down every gap of the
        // checkerboard, at 1/4 and 3/4 across, one per serpentine direction.
        for (sg = 0; sg < 8; sg++) begin
            a = seg_start(sg);
            b = a + 1;
            while (b < n_cap && cap_s[b] == sg) b++;   // one past this segment

            exp_blank  = (sg % 2 == 1);
            prev_blank = (((sg + 7) % 8) % 2 == 1);

            ok = 1;
            for (i = a + 1; i < b; i++)
                if (cap_z[i] !== exp_blank) ok = 0;
            if (!ok) begin
                $display("  FAIL  segment %0d interior z_blank: expected %0b",
                         sg, exp_blank);
                errors++;
            end else
                $display("  ok    segment %0d %s", sg, exp_blank ? "BLANKED" : "drawn");

            if (cap_z[a] !== prev_blank) begin
                $display("  FAIL  segment %0d entry transit: got %0b, expected %0b (segment %0d's)",
                         sg, cap_z[a], prev_blank, (sg+7) % 8);
                errors++;
            end
        end

        // The same three assertions in plain language, on interior transits.
        if (cap_z[seg_start(1)+1] !== 1'b1) begin
            $display("  FAIL  the gap between the two spans is NOT blanked"); errors++;
        end
        if (cap_z[seg_start(3)+1] !== 1'b1) begin
            $display("  FAIL  the serpentine row step is NOT blanked"); errors++;
        end
        if (cap_z[seg_start(0)+1] !== 1'b0) begin
            $display("  FAIL  a SPAN was blanked -- nothing would be drawn"); errors++;
        end
        // And the regression itself: the beam must still be dark when it
        // arrives at a span's first point.
        if (cap_z[seg_start(2)] !== 1'b1) begin
            $display("  FAIL  the transit ONTO span 2 was drawn -- this is the gap streak"); errors++;
        end

        $display("");
        $display("=====================================");
        if (errors == 0) $display("PASS -- walk, spacing and blanking all correct");
        else             $display("FAIL -- %0d check(s) failed", errors);
        $display("=====================================");
        $display("");
        $finish;
    end

    initial begin
        #5_000_000;
        $display("");
        $display("FAIL -- testbench timed out (FSM wedged?)");
        $finish;
    end
endmodule
