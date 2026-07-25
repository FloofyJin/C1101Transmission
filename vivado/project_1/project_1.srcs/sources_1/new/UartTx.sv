`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// UartTx  --  8N1 serial transmitter, LSB first, idle high.
// Assert `send` for one cycle with `data` valid; ignored while `busy`.
// `done` pulses for one cycle at the end of the stop bit.
//------------------------------------------------------------------------------
module UartTx #(
    parameter int CLK_HZ = 125_000_000,
    parameter int BAUD   = 115200
)(
    input  logic       clk,
    input  logic       rst,
    input  logic       send,
    input  logic [7:0] data,
    output logic       tx,
    output logic       busy,
    output logic       done
);
    localparam int DIV = CLK_HZ / BAUD;   // 1085 @ 125 MHz / 115200

    typedef enum logic [1:0] { U_IDLE, U_START, U_DATA, U_STOP } ust_t;
    ust_t state;

    logic [15:0] baud_cnt;
    logic [2:0]  bit_idx;
    logic [7:0]  sh;

    always_ff @(posedge clk) begin
        if (rst) begin
            state    <= U_IDLE;
            tx       <= 1'b1;
            busy     <= 1'b0;
            done     <= 1'b0;
            baud_cnt <= 0;
            bit_idx  <= 0;
            sh       <= 8'h00;
        end else begin
            done <= 1'b0;
            case (state)
                U_IDLE: begin
                    tx   <= 1'b1;
                    busy <= 1'b0;
                    if (send) begin
                        sh       <= data;
                        busy     <= 1'b1;
                        baud_cnt <= 0;
                        tx       <= 1'b0;      // start bit
                        state    <= U_START;
                    end
                end
                U_START: begin
                    if (baud_cnt == DIV-1) begin
                        baud_cnt <= 0;
                        tx       <= sh[0];     // first data bit (LSB)
                        bit_idx  <= 0;
                        state    <= U_DATA;
                    end else baud_cnt <= baud_cnt + 1;
                end
                U_DATA: begin
                    if (baud_cnt == DIV-1) begin
                        baud_cnt <= 0;
                        if (bit_idx == 3'd7) begin
                            tx    <= 1'b1;     // stop bit
                            state <= U_STOP;
                        end else begin
                            sh      <= {1'b0, sh[7:1]};
                            tx      <= sh[1];  // next data bit
                            bit_idx <= bit_idx + 1;
                        end
                    end else baud_cnt <= baud_cnt + 1;
                end
                U_STOP: begin
                    if (baud_cnt == DIV-1) begin
                        baud_cnt <= 0;
                        busy     <= 1'b0;
                        done     <= 1'b1;
                        state    <= U_IDLE;
                    end else baud_cnt <= baud_cnt + 1;
                end
            endcase
        end
    end
endmodule
