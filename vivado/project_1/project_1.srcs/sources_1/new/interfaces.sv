`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07/22/2026 12:25:32 AM
// Design Name: 
// Module Name: interfaces
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

interface ByteStream;

    logic [7:0] data;
    logic valid;
    logic ready;
    logic last;

    modport source (
        output data,
        output valid,
        output last,
        input  ready
    );

    modport sink (
        input  data,
        input  valid,
        input  last,
        output ready
    );

endinterface