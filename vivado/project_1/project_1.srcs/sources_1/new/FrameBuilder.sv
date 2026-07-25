`timescale  1ns / 1ps

module FrameBuilder #(
    parameter MAX_PAYLOAD = 16
)(
    input clk,
    input rst,

    input start,
    input [7:0] payload_data,
    input payload_valid,
    input payload_last,
    input [7:0] payload_length,

    output reg [7:0] frame_byte,
    output reg frame_valid,
    output reg busy
);

localparam PREAMBLE_BYTE = 8'hAA;
localparam PREAMBLE_COUNT = 4;
localparam SYNC0 = 8'hD3;
localparam SYNC1 = 8'h91;

typedef enum logic [2:0] {
    IDLE,
    PREAMBLE,
    SYNC,
    LENGTH,
    PAYLOAD,
    DONE
} frame_state_t;

frame_state_t state;

logic [3:0] preamble_counter;
logic sync_counter;
logic [$clog2(MAX_PAYLOAD)-1:0] payload_counter;

always @(posedge clk) begin
    if (rst)begin
        state <= IDLE;
        preamble_counter <= 0;
        sync_counter <= '0;
        payload_counter <= '0;
        frame_byte <= '0;
        frame_valid <= 1'b0;
        busy <= 1'b0;
    end else begin
        case (state)
            IDLE:
            begin
                frame_byte <= '0;
                busy <= 0;
                frame_valid <= 0;
                if(start) begin
                    state <= PREAMBLE;
                    frame_valid <= 1'b1;
                    busy <= 1'b1;

                    preamble_counter <= 1'b0;
                    payload_counter <= 1'b0;
                    sync_counter <= 1'b0;
                end
            end

            PREAMBLE:
            begin
                if(preamble_counter == PREAMBLE_COUNT-1) begin
                    frame_byte <= PREAMBLE_BYTE;
                    state <= SYNC;
                end else begin
                    frame_byte <= PREAMBLE_BYTE;
                    preamble_counter <= preamble_counter + 1'b1;
                end
            end

            SYNC:
            begin
                if (sync_counter == 1'b0) begin
                    frame_byte <= SYNC0;
                    sync_counter <= sync_counter + 1'b1;
                end else begin
                    frame_byte <= SYNC1;
                    state <= LENGTH;
                end
            end

            LENGTH:
            begin
                frame_byte <= payload_length;
                state <= PAYLOAD;
            end

            PAYLOAD:
            begin
                if(payload_valid) begin
                    if(payload_counter < payload_length)begin
                        frame_byte <= payload_data;
                        payload_counter <= payload_counter + 1'b1;
                    end else begin
                        state <= DONE;
                    end
                end
            end

            DONE:
            begin
                busy <= 0;
                state <= IDLE;
                frame_valid <= 0;
            end
        endcase
    end
end

endmodule