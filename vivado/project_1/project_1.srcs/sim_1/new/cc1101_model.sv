`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// cc1101_model  --  simulation-only stub of the CC1101 chip
//
// NOT a faithful model. It implements just enough to prove our side is right:
//   * SPI mode 0 slave (sample MOSI on rising SCLK, present MISO on falling)
//   * header-byte decode (R/W bit, burst bit, 6-bit address)
//   * register write / read-back, so ConfigSeq's verify pass succeeds
//   * status registers PARTNUM / VERSION / MARCSTATE / TXBYTES / RXBYTES
//   * strobes SRES / SIDLE / STX / SFTX / SRX / SFRX, and a GDO0 pulse after STX
//   * an RX FIFO plus rx_packet(), which stages a received frame and pulses
//     GDO0 the way a real reception would
//
// What it deliberately does NOT model: RF, timing, calibration, FIFO limits.
// Passing this testbench proves the FPGA emits the right bytes in the right
// order with the right CSn framing. It says nothing about whether the register
// VALUES are correct for the air interface -- only hardware can tell you that.
//------------------------------------------------------------------------------
module cc1101_model #(
    parameter [7:0] VERSION_VAL = 8'h14,
    parameter       GDO0_DELAY  = 20000,   // ns from STX to sync word on air
    parameter       GDO0_WIDTH  = 40000    // ns the packet takes to go out
)(
    input  wire cs_n,
    input  wire sclk,
    input  wire mosi,
    output reg  miso,
    output reg  gdo0
);
    reg [7:0] regs [0:63];
    reg [7:0] fifo [0:63];

    // RX FIFO -- exactly 64 bytes, like the real part.
    //
    // This was 128 bytes once, "so an over-long injected frame overruns the
    // test rather than the array". That hid the exact constraint it should
    // have enforced: a 30-point packet needs 1 len + 62 payload + 2 status =
    // 65 bytes and DOES overflow real silicon, but passed in simulation.
    // rx_deliver() now asserts instead.
    reg [7:0] rx_fifo [0:63];
    reg [7:0] rxbytes;
    reg [7:0] rx_rd_ptr;

    reg [7:0] shift_in, shift_out;
    reg [3:0] bit_cnt;
    reg [7:0] byte_cnt;
    reg [7:0] header;
    reg [7:0] marcstate, txbytes;
    reg       stx_evt;

    // decoded header fields
    wire       is_read = header[7];
    wire       is_bst  = header[6];
    wire [5:0] addr    = header[5:0];

    // observation hooks for the testbench
    reg [7:0] seen [0:255];
    integer   n_seen;

    integer i;
    initial begin
        for (i = 0; i < 64; i = i + 1) regs[i] = 8'h00;
        miso      = 1'b0;
        gdo0      = 1'b0;
        marcstate = 8'h01;      // IDLE
        txbytes   = 8'h00;
        shift_in  = 8'h00;
        shift_out = 8'h00;
        bit_cnt   = 0;
        byte_cnt  = 0;
        header    = 8'h00;
        stx_evt   = 1'b0;
        n_seen    = 0;
        rxbytes   = 8'h00;
        rx_rd_ptr = 8'h00;
        for (i = 0; i < 64; i = i + 1) rx_fifo[i] = 8'h00;
    end

    function [7:0] status_read(input [5:0] a);
        case (a)
            6'h30:   status_read = 8'h00;         // PARTNUM
            6'h31:   status_read = VERSION_VAL;   // VERSION
            6'h35:   status_read = marcstate;     // MARCSTATE
            6'h3A:   status_read = txbytes;       // TXBYTES
            6'h3B:   status_read = rxbytes;       // RXBYTES
            default: status_read = 8'h00;
        endcase
    endfunction

    // ---- CSn falling: start of a transaction. MISO low = "chip ready". ----
    always @(negedge cs_n) begin
        bit_cnt  <= 0;
        byte_cnt <= 0;
        miso     <= 1'b0;
    end

    // ---- sample MOSI ----
    always @(posedge sclk) if (!cs_n) begin
        shift_in <= {shift_in[6:0], mosi};
        if (bit_cnt == 7) begin
            bit_cnt <= 0;
            process_byte({shift_in[6:0], mosi});
        end else begin
            bit_cnt <= bit_cnt + 1;
        end
    end

    // ---- present MISO ----
    always @(negedge sclk) if (!cs_n) begin
        miso      <= shift_out[7];
        shift_out <= {shift_out[6:0], 1'b0};
    end

    task process_byte(input [7:0] b);
        begin
            if (n_seen < 256) begin seen[n_seen] = b; n_seen = n_seen + 1; end

            if (byte_cnt == 0) begin
                header <= b;
                // strobe = write, no burst, address in the command range
                if (!b[7] && !b[6] && b[5:0] >= 6'h30 && b[5:0] <= 6'h3D) begin
                    do_strobe(b[5:0]);
                    shift_out <= 8'h00;
                end else if (b[7]) begin
                    // read: load the first data byte now, it goes out next byte
                    if (b[6] && b[5:0] == 6'h3F) begin
                        // RX FIFO burst read (0xFF). Address 0x3F is past the
                        // 0x30-0x3D status window, so it must be tested FIRST
                        // or it falls through to a plain regs[] read.
                        shift_out <= rx_fifo[0];
                        rx_rd_ptr  = 8'd1;
                    end else if (b[6] && b[5:0] >= 6'h30 && b[5:0] <= 6'h3D)
                        shift_out <= status_read(b[5:0]);
                    else
                        shift_out <= regs[b[5:0]];
                end else begin
                    shift_out <= 8'h00;
                end
            end else begin
                if (!header[7]) begin
                    if (header[5:0] == 6'h3F) begin
                        // TX FIFO: first byte is the length, then the payload
                        fifo[txbytes] = b;
                        txbytes       = txbytes + 1;
                    end else begin
                        regs[header[5:0]] = b;
                    end
                end else if (header[6] && header[5:0] == 6'h3F) begin
                    // Continuing an RX FIFO burst: hand up the next byte. Without
                    // this only the first byte of a burst would be real and the
                    // rest would repeat whatever was left in shift_out.
                    shift_out <= rx_fifo[rx_rd_ptr];
                    rx_rd_ptr  = rx_rd_ptr + 8'd1;
                end
            end
            byte_cnt <= byte_cnt + 1;
        end
    endtask

    task do_strobe(input [5:0] s);
        begin
            case (s)
                6'h30: begin marcstate = 8'h01; txbytes = 0; end          // SRES
                6'h36: marcstate = 8'h01;                                  // SIDLE
                6'h3B: txbytes   = 0;                                      // SFTX
                // 0x3A is SFRX as a strobe and TXBYTES as a status address.
                // do_strobe is only reached on write-without-burst, so there is
                // no ambiguity here -- that is exactly what the burst bit is for.
                6'h3A: begin rxbytes = 0; rx_rd_ptr = 0; end               // SFRX
                6'h34: marcstate = 8'h0D;                                  // SRX
                6'h35: begin marcstate = 8'h13; stx_evt = ~stx_evt; end    // STX
                default: ;
            endcase
        end
    endtask

    // GDO0 with IOCFG0=0x06: rises at sync word, falls at end of packet
    always @(stx_evt) begin
        if ($time > 0) begin
            #(GDO0_DELAY) gdo0 = 1'b1;
            #(GDO0_WIDTH) gdo0 = 1'b0;
            marcstate = 8'h01;   // MCSM1 default returns the chip to IDLE
            txbytes   = 0;
        end
    end

    // ---- receive injection, for testing the RX side ----
    //
    // The testbench fills rx_fifo[0..n-1] itself and then calls rx_deliver(n).
    // Passing the frame in as an argument would mean a fixed-width port; letting
    // the TB write the array directly keeps packet length free, which matters
    // because the 65-byte maximum is the case most worth testing.
    //
    // The frame must already be in chip layout:
    //     [len] [payload...] [RSSI] [LQI | CRC_OK<<7]
    //
    // GDO0 is driven here as well as by the STX block above. That is safe only
    // because a given radio instance is used for one direction at a time --
    // never strobe STX on an instance you are also injecting into.
    task rx_deliver(input [7:0] n);
        begin
            if (n > 8'd64) begin
                $display("  MODEL ERROR: rx_deliver(%0d) overflows the 64-byte RX FIFO.",
                         n);
                $display("               1 len + payload + 2 status must be <= 64,");
                $display("               so payload <= 61 bytes = 28 points + 3 header.");
                $fatal(1);
            end
            rxbytes   = n;
            rx_rd_ptr = 8'd0;
            marcstate = 8'h0D;          // RX
            gdo0      = 1'b1;           // sync word detected
            #(GDO0_WIDTH);
            gdo0      = 1'b0;           // end of packet, frame is in the FIFO
        end
    endtask
endmodule
