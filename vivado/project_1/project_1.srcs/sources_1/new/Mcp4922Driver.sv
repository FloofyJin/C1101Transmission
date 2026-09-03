`timescale 1ns / 1ps

/*
 * The goal of this module is to transmit data to MCP4922
 * It should control timing and framing
*/

module Mcp4922Driver #(
    parameter DESIRED_SPEED = 20_000_000, // Clock speed in Hz
    parameter SYSTEM_CLK_SPEED = 125_000_000, // Zybo clock speed in Hz
    parameter T_CSUP_CLK = 5,
    parameter T_LS_CLK = 5,
    parameter T_LDAC_CLK = 13
) (
    input logic clk,
    input logic rst,
    input logic start,
    input logic [11:0] x, y,

    output logic done,
    output logic busy,
    output logic dac_cs,
    output logic dac_ldac,
    output logic dac_sclk,
    output logic dac_sdi
);

localparam logic [3:0] CFG_A = 4'b0011; // 0x3: MSB=0 is VoutA
localparam logic [3:0] CFG_B = 4'b1011; // 0xB: MSB=1 is VoutB

// You want to double your desired speed bc we toggle between rise and fall of sclk
localparam int CLK_DIV = SYSTEM_CLK_SPEED / (2 * DESIRED_SPEED); 

typedef enum logic [2:0] {
    IDLE, WRITE_A, CS_UP,WRITE_B, LDAC_WAIT, LATCH, DONE
} state_t;

state_t state;

logic [15:0] curr_x, curr_y;
logic [3:0] x_index, y_index; // Must count up to 16 bc theres 16 bits in each packet

logic [3:0] cs_up_counter;
logic  [4:0] ldac_counter;

localparam int CNT_W = $clog2(CLK_DIV);
logic [CNT_W-1:0] div_cnt;
logic tick;

always_ff @(posedge clk) begin
    if(rst)begin
        curr_x <= '0;
        curr_y <= '0;
        busy <= '0;
        done <= '0;
        div_cnt <= '0;
        dac_cs <= 1'b1; dac_ldac <= 1'b1; dac_sclk <= '0; dac_sdi <= '0;
        state <= IDLE;
        tick <= 1'b0;
    end else begin
        done <= 1'b0;

        if(!dac_cs) begin
            if(div_cnt == CLK_DIV-1) begin
                div_cnt <= '0;
                dac_sclk <= ~dac_sclk;
                tick <= 1'b1;
            end else begin
                tick <= 1'b0;
                div_cnt <= div_cnt + 1;
            end
        end else begin
            div_cnt <= '0;
            dac_sclk <= 1'b0;
            tick <= 1'b0;
        end
        case(state)
            IDLE:
            begin
                // Entire write cycle for xy coord is 16bit for X + 16 bit Y = 32bit
                // But since we use both rising and falling edge, we need 64 cycles
                if(start)begin
                    busy <= 1'b1;
                    state <= WRITE_A;
                    cs_up_counter <= '0;
                    ldac_counter <= '0;
                    curr_x <= {CFG_A, x};
                    curr_y <= {CFG_B, y};
                    x_index <= 4'd15; y_index <= 4'd15;
                    dac_ldac <= 1'b1;
                    dac_cs <= 1'b0;
                    dac_sdi <= CFG_A[3]; //cant use curr_x bc it isnt set yet
                end
            end

            WRITE_A:
            begin
                if(tick)begin
                    if(dac_sclk == 1'b0) begin // falling edge
                        dac_sdi <= curr_x[x_index];
                    end else begin // rising edge: dac_sdi is read on rise
                        x_index <= x_index - 1;
                        if(x_index == 0)begin
                            state <= CS_UP;
                            dac_cs <= 1'b1;
                        end
                    end
                end
            end

            CS_UP:
            begin
                if(cs_up_counter == T_CSUP_CLK-1)begin
                    if(dac_sclk == 1'b0) begin 
                        state <= WRITE_B; // rising edge
                        dac_cs <= 1'b0;
                        dac_sdi <= curr_y[15];
                    end
                end else cs_up_counter <= cs_up_counter + 1;
            end

            WRITE_B:
            begin
                if(tick)begin
                    if(dac_sclk == 1'b0)begin
                        dac_sdi <= curr_y[y_index];
                    end else begin
                        y_index <= y_index - 1;
                        if(y_index == 0)begin
                            state <= LDAC_WAIT;
                            dac_cs <= 1'b1;
                        end
                    end
                end
            end

            LDAC_WAIT: 
            begin                        // tLS -- LDAC stays high
                if (ldac_counter == T_LS_CLK-1) begin
                    ldac_counter <= '0;
                    state        <= LATCH;
                end else ldac_counter <= ldac_counter + 1;
            end

            LATCH: 
            begin                            // tLD -- LDAC low
                dac_ldac <= 1'b0;
                if (ldac_counter == T_LDAC_CLK) begin
                    dac_ldac <= 1'b1;
                    state    <= DONE;
                end else ldac_counter <= ldac_counter + 1;
            end

            DONE:
            begin
                done <= 1'b1;
                busy <= 1'b0;
                state <= IDLE;
            end
        endcase
    end
end

endmodule