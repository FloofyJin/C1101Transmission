`timescale 1ns / 1ps

module SPIMaster #(
    parameter CLK_DIV = 6      // Number of FPGA clocks per half SPI clock
)(
    input  wire       clk,
    input  wire       rst,

    input  wire       start,
    input  wire [7:0] tx_data,
    input  wire       hold_cs,   // keep CSn asserted after this byte (for header+data / bursts)

    output reg  [7:0] rx_data,
    output reg        busy,
    output reg        done,

    output reg        sclk,
    output reg        mosi,
    input  wire       miso,
    output reg        cs_n
);
    reg [7:0] shift_tx;
    reg [7:0] shift_rx;

    reg [2:0] bit_count;
    reg [15:0] clk_count;

    always @(posedge clk) begin
        if (rst) begin
            busy      <= 0;
            done      <= 0;
            sclk      <= 0;
            cs_n      <= 1;
            mosi      <= 0;

            clk_count <= 0;
            bit_count <= 0;

            shift_tx  <= 0;
            shift_rx  <= 0;
            rx_data   <= 0;
        end
        else begin
            done <= 0;
            if (start && !busy) begin
                busy      <= 1;
                cs_n      <= 0;
                sclk      <= 0;
                clk_count <= 0;
                bit_count <= 7;
                shift_tx  <= tx_data;
                shift_rx  <= 0;

                // Present first bit before first clock edge
                mosi <= tx_data[7];
            end
            else if (busy) begin
                clk_count <= clk_count + 1;
                if (clk_count == CLK_DIV-1) begin
                    clk_count <= 0;
                    sclk <= ~sclk;

                    // rising edge
                    if (sclk == 0) begin
                        shift_rx <= {shift_rx[6:0], miso}; // shifts miso left
                    end
                    // falling edge. set the mosi for next rise
                    else begin
                        if (bit_count != 0) begin
                            shift_tx <= {shift_tx[6:0],1'b0}; //shift tx data left
                            bit_count <= bit_count - 1;
                            mosi <= shift_tx[6]; // set mosi out bit
                        end
                        else begin
                            busy    <= 0;
                            cs_n    <= hold_cs ? 1'b0 : 1'b1; // keep CSn low across a multi-byte transaction
                            sclk    <= 0;
                            rx_data <= shift_rx; // all 8 bits already shifted in on the rising edges
                            done    <= 1;
                        end
                    end
                end
            end
        end
    end

endmodule