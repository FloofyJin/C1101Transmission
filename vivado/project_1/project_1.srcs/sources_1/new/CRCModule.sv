`timescale 1ns / 1ps

module CRCModule (
    input  wire       clk,
    input  wire       rst,

    // Start a new CRC calculation
    input  wire       start,

    // Feed one byte into the CRC
    input  wire       data_valid,
    input  wire [7:0] data_in,

    // Current CRC value
    output wire [15:0] crc_out
);

    reg [15:0] crc_reg;

    assign crc_out = crc_reg;

    function [15:0] crc16_next;

        input [15:0] crc;
        input [7:0]  data;

        reg [15:0] c;
        integer i;

        begin
            c = crc ^ (data << 8);

            for (i = 0; i < 8; i = i + 1) begin
                if (c[15])
                    c = (c << 1) ^ 16'h1021;
                else
                    c = c << 1;
            end

            crc16_next = c;
        end

    endfunction

    always @(posedge clk) begin

        if (rst) begin
            crc_reg <= 16'hFFFF;
        end
        else begin

            // Beginning of a new frame
            if (start)
                crc_reg <= 16'hFFFF;

            // Consume one byte
            else if (data_valid)
                crc_reg <= crc16_next(crc_reg, data_in);

        end

    end

endmodule