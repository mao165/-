`timescale 1ns / 1ps

// 40MHz 数据 epoch 恢复屏障。输入 ACK 必须已由集成层逐项同步到 clk。
// tx_quiet 由 TX 层保证：已见 hold、无在途 grant 帧且连续两拍静默。
module data_chain_recovery #(
    parameter integer RESET_CYCLES = 64,
    parameter integer FIFO_CYCLES = 64
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        fault_overwrite,
    input  wire        fault_calib_lost,
    input  wire        calib_ready,
    input  wire        tx_quiet,
    input  wire [2:0]  clear_seen,
    input  wire [2:0]  fifo_seen,
    input  wire [2:0]  ready_seen,
    input  wire [2:0]  run_seen,
    input  wire        local_ready_40,
    output reg         hold_request,
    output reg         clear_request,
    output reg         fifo_request,
    output reg         mig_sys_rst_n,
    output reg         business_enable,
    output reg  [2:0]  state,
    output reg  [1:0]  fault_reason,
    output reg  [31:0] recovery_count,
    output reg  [31:0] wait_age
);

localparam [2:0] QUIESCE = 3'd0;
localparam [2:0] MIG_RESET = 3'd1;
localparam [2:0] WAIT_CALIB = 3'd2;
localparam [2:0] FIFO_RESET = 3'd3;
localparam [2:0] WAIT_READY = 3'd4;
localparam [2:0] RELEASE = 3'd5;
localparam [2:0] RUN = 3'd6;
// 不允许较小参数意外缩短专用复位脉冲；生产默认64，单元测试可用4。
localparam [31:0] RESET_LAST = (RESET_CYCLES < 4) ? 32'd3 : RESET_CYCLES - 1;
localparam [31:0] FIFO_LAST = (FIFO_CYCLES < 4) ? 32'd3 : FIFO_CYCLES - 1;

reg [2:0] next_state;
reg calib_low_seen;

always @(*) begin
    next_state = state;
    case (state)
        QUIESCE:
            if (tx_quiet) next_state = MIG_RESET;
        MIG_RESET:
            if (wait_age >= RESET_LAST) next_state = WAIT_CALIB;
        WAIT_CALIB:
            if (calib_low_seen && calib_ready && clear_seen == 3'b111 &&
                fifo_seen == 3'b000 && run_seen == 3'b000)
                next_state = FIFO_RESET;
        FIFO_RESET:
            if (!calib_ready) next_state = MIG_RESET;
            else if (wait_age >= FIFO_LAST && fifo_seen == 3'b111)
                next_state = WAIT_READY;
        WAIT_READY:
            if (!calib_ready) next_state = MIG_RESET;
            else if (clear_seen == 3'b000 && fifo_seen == 3'b000 &&
                     ready_seen == 3'b111 && local_ready_40)
                next_state = RELEASE;
        RELEASE:
            if (!calib_ready) next_state = MIG_RESET;
            else if (run_seen == 3'b111 && local_ready_40)
                next_state = RUN;
        RUN:
            if (fault_overwrite || fault_calib_lost) next_state = QUIESCE;
        default: next_state = MIG_RESET;
    endcase
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= MIG_RESET;
        hold_request <= 1'b1;
        clear_request <= 1'b1;
        fifo_request <= 1'b0;
        mig_sys_rst_n <= 1'b0;
        business_enable <= 1'b0;
        calib_low_seen <= 1'b0;
        fault_reason <= 2'b00;
        recovery_count <= 32'd0;
        wait_age <= 32'd0;
    end else begin
        state <= next_state;
        if (next_state != state)
            wait_age <= 32'd0;
        else if (wait_age != 32'hffffffff)
            wait_age <= wait_age + 32'd1;

        if (next_state == MIG_RESET && state != MIG_RESET)
            calib_low_seen <= 1'b0;
        else if ((state == MIG_RESET || state == WAIT_CALIB) && !calib_ready)
            calib_low_seen <= 1'b1;

        if (state == RUN && (fault_overwrite || fault_calib_lost)) begin
            fault_reason <= {fault_calib_lost, fault_overwrite};
            if (recovery_count != 32'hffffffff)
                recovery_count <= recovery_count + 32'd1;
        end

        // 请求和 MIG 复位均为真实寄存器输出，避免状态译码毛刺进入外部复位。
        case (next_state)
            QUIESCE: begin
                hold_request <= 1'b1; clear_request <= 1'b0; fifo_request <= 1'b0;
                mig_sys_rst_n <= 1'b1; business_enable <= 1'b0;
            end
            MIG_RESET: begin
                hold_request <= 1'b1; clear_request <= 1'b1; fifo_request <= 1'b0;
                mig_sys_rst_n <= 1'b0; business_enable <= 1'b0;
            end
            WAIT_CALIB: begin
                hold_request <= 1'b1; clear_request <= 1'b1; fifo_request <= 1'b0;
                mig_sys_rst_n <= 1'b1; business_enable <= 1'b0;
            end
            FIFO_RESET: begin
                hold_request <= 1'b1; clear_request <= 1'b1; fifo_request <= 1'b1;
                mig_sys_rst_n <= 1'b1; business_enable <= 1'b0;
            end
            WAIT_READY: begin
                hold_request <= 1'b1; clear_request <= 1'b0; fifo_request <= 1'b0;
                mig_sys_rst_n <= 1'b1; business_enable <= 1'b0;
            end
            RELEASE: begin
                hold_request <= 1'b0; clear_request <= 1'b0; fifo_request <= 1'b0;
                mig_sys_rst_n <= 1'b1; business_enable <= 1'b0;
            end
            RUN: begin
                hold_request <= 1'b0; clear_request <= 1'b0; fifo_request <= 1'b0;
                mig_sys_rst_n <= 1'b1; business_enable <= 1'b1;
            end
            default: begin
                hold_request <= 1'b1; clear_request <= 1'b1; fifo_request <= 1'b0;
                mig_sys_rst_n <= 1'b0; business_enable <= 1'b0;
            end
        endcase
    end
end
endmodule
