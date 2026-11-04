`timescale 1ns / 1ps

module mig_temp_monitor (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        init_calib_complete,
    input  wire [11:0] mig_device_temp,
    output reg  [15:0] temp_raw,
    output reg         temp_valid,
    output reg         temp_update_pulse,
    output reg         temp_stale,
    output wire        temp_alarm
);

(* ASYNC_REG = "TRUE" *) reg init_calib_complete_meta;
(* ASYNC_REG = "TRUE" *) reg init_calib_complete_sync;

wire        sample_valid = init_calib_complete_sync && (mig_device_temp != 12'd0);
wire [15:0] sample_raw   = {mig_device_temp, 4'd0};

assign temp_alarm = 1'b0;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        init_calib_complete_meta <= 1'b0;
        init_calib_complete_sync <= 1'b0;
        temp_raw          <= 16'd0;
        temp_valid        <= 1'b0;
        temp_update_pulse <= 1'b0;
        temp_stale        <= 1'b1;
    end else begin
        init_calib_complete_meta <= init_calib_complete;
        init_calib_complete_sync <= init_calib_complete_meta;
        temp_update_pulse <= 1'b0;

        if (sample_valid) begin
            temp_update_pulse <= (!temp_valid || (temp_raw != sample_raw));
            temp_raw          <= sample_raw;
            temp_valid        <= 1'b1;
            temp_stale        <= 1'b0;
        end else begin
            temp_raw          <= 16'd0;
            temp_valid        <= 1'b0;
            temp_stale        <= 1'b1;
        end
    end
end

endmodule
