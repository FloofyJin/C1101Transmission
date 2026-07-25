`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07/21/2026 10:26:01 PM
// Design Name: 
// Module Name: PacketGenerator
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


module PacketGenerator #(
    parameter PAYLOAD_LENGTH = 16
)(
    input logic clk,
    input logic rst,
    input logic start,
    ByteStream.source tx
);

reg [$clog2(PAYLOAD_LENGTH):0] byte_counter;

typedef enum logic [1:0] {
    IDLE,
    SEND
} pg_state_t;

pg_state_t state;

always_ff @(posedge clk) begin

    if (rst) begin
        state <= IDLE;
        tx.valid <= 1'b0;
        byte_counter <= '0;
        tx.data <= 16'd0;
    end
    else begin
        tx.valid <= 1'b0;
        tx.last <= 1'b0;

        case (state) 
            IDLE: 
            begin
                byte_counter <= '0;
                if (start)
                    state <= SEND;
            end

            SEND: 
            begin
                if (tx.ready) begin
                    tx.data <= byte_counter;
                    tx.valid <= 1'b1;

                    if(byte_counter == PAYLOAD_LENGTH-1)begin
                        tx.last <= 1'b1;
                        state <= IDLE;
                    end else begin
                        byte_counter <=  byte_counter + 1'b1;
                    end
                end
            end
        endcase
    end
end

endmodule