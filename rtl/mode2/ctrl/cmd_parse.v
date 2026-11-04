`timescale 1ns / 1ps
//******************************************************************************
//  File Name:   cmd_parse.v
//  Module:      cmd_parse
//  Description: Parse unified 24-bit command bus and update work mode/threshold.
//
//  Internal command format:
//      cmd_data[23:16] = cmd_id
//      cmd_data[15:0]  = cmd_arg
//
//  Supported commands:
//      24'h01_0000  -> test mode
//      24'h02_0000  -> trigger mode
//      24'h80_xxxx  -> threshold = xxxx * 10, when xxxx <= 6553
//******************************************************************************

module cmd_parse(
    // system signals
    input   wire                clk,
    input   wire                rst_n,

    // command input
    input   wire                cmd_valid,
    input   wire    [23:0]      cmd_data,

    // decoded output
    output  reg     [1:0]       work_mode,
    output  reg     [15:0]      threshold
);

localparam MODE_IDLE    = 2'b00;
localparam MODE_TEST    = 2'b01;
localparam MODE_TRIG    = 2'b10;

localparam CMD_TEST      = 8'h01;
localparam CMD_TRIG      = 8'h02;
localparam CMD_THRESHOLD = 8'h80;

localparam DEFAULT_THRESHOLD  = 16'd10000;
localparam MAX_THRESHOLD_STEP = 16'd6553;

wire    [7:0]       cmd_id;
wire    [15:0]      threshold_step;
wire    [19:0]      threshold_step_ext;
wire    [19:0]      threshold_calc;

assign cmd_id             = cmd_data[23:16];
assign threshold_step     = cmd_data[15:0];
assign threshold_step_ext = {4'd0, threshold_step};
assign threshold_calc     = (threshold_step_ext << 3) + (threshold_step_ext << 1);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        work_mode <= MODE_IDLE;
        threshold <= DEFAULT_THRESHOLD;
    end
    else if (cmd_valid) begin
        case (cmd_id)
            CMD_TEST: begin
                work_mode <= MODE_TEST;
            end

            CMD_TRIG: begin
                work_mode <= MODE_TRIG;
            end

            CMD_THRESHOLD: begin
                if (threshold_step <= MAX_THRESHOLD_STEP) begin
                    threshold <= threshold_calc[15:0];
                end
            end

            default: ;
        endcase
    end
end

endmodule
