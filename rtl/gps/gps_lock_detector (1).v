`timescale 1ns / 1ps
//******************************************************************************
//  文件名:		gps_lock_detector.v
//  模块名:		gps_lock_detector
//  功能描述:	GPS锁定状态检测模块
//				- 通过检测PPS信号脉宽判断GPS锁定状态
//				- 未锁定: 脉宽≈10ms, 已锁定: 脉宽≈500ms
//				- 支持超时检测和脉宽超限检测
//------------------------------------------------------------------------------
//  判定标准 (20MHz时钟):
//  ┌──────────────┬────────────────┬──────────────────┐
//  │   状态       │   脉宽范围     │   时钟周期数     │
//  ├──────────────┼────────────────┼──────────────────┤
//  │ 未锁定       │  8ms ~ 12ms    │  160k ~ 240k     │
//  │ 已锁定       │ 480ms ~ 520ms  │ 9.6M ~ 10.4M     │
//  │ 异常         │ 其他/超时      │  -               │
//  └──────────────┴────────────────┴──────────────────┘
//------------------------------------------------------------------------------
//  鲁棒性设计:
//  1. PPS信号三级同步，防止亚稳态
//  2. 连续3次测量一致才切换状态，防止干扰
//  3. 1.5秒超时检测，识别无信号状态
//  4. 540ms脉宽超限检测，识别PPS拉高异常
//------------------------------------------------------------------------------
//  状态编码 (适配LED模块):
//  - 2'b00 = 未锁定 (LED常灭)
//  - 2'b01 = 已锁定 (LED常亮)
//  - 2'b10 = 异常   (LED闪烁)
//

module gps_lock_detector(
	//==========================================================================
	// 系统信号
	//==========================================================================
	input	wire				clk,				// 20MHz系统时钟
	input	wire				rst_n,				// 系统复位（低有效）
	
	//==========================================================================
	// PPS输入
	//==========================================================================
	input	wire				pps,				// PPS脉冲信号
	
	//==========================================================================
	// 状态输出
	//==========================================================================
	output	reg		[1:0]		gps_lock_status,	// GPS锁定状态
	output	wire				pps_pos_edge_sync,	// PPS同步上升沿
	output	wire				pps_pos_edge_extended,	// PPS展宽信号（调试用，约50ms）
	output	reg		[31:0]		pps_pulse_width_dbg	// 脉宽调试信号
);

//==============================================================================
// 参数定义
//==============================================================================

// 脉宽阈值 (20MHz)
localparam	UNLOCKED_MIN	= 32'd160_000;		// 8ms   @ 20MHz
localparam	UNLOCKED_MAX	= 32'd240_000;		// 12ms  @ 20MHz
localparam	LOCKED_MIN		= 32'd9_600_000;	// 480ms @ 20MHz
localparam	LOCKED_MAX		= 32'd10_400_000;	// 520ms @ 20MHz
localparam	PULSE_WIDTH_MAX	= 32'd10_800_000;	// 540ms超限 @ 20MHz
localparam	TIMEOUT_MAX		= 32'd30_000_000;	// 1.5s超时 @ 20MHz
localparam	CONSECUTIVE_CNT	= 3'd3;				// 连续判定次数

// 状态编码
localparam	STATE_UNLOCKED	= 2'b00;
localparam	STATE_LOCKED	= 2'b01;
localparam	STATE_ABNORMAL	= 2'b10;

//==============================================================================
// 内部信号定义
//==============================================================================

// PPS三级同步
reg							pps_reg1;
reg							pps_reg2;
reg							pps_reg3;

// 边沿检测
wire						pps_pos_edge;
wire						pps_neg_edge;
wire	[1:0]				measurement_from_result;

// PPS边沿展宽（用于调试）
reg		[21:0]				pps_edge_extend_cnt;	// 约50ms展宽计数器
reg							pps_pos_edge_extended_reg;	// 展宽后的边沿信号

assign pps_pos_edge_extended = pps_pos_edge_extended_reg;

// 脉宽测量
reg		[31:0]				pps_width_cnt;
reg		[31:0]				pps_width_result;
reg							pps_width_valid;
reg							pps_width_overflow;

// 超时检测
reg		[31:0]				timeout_cnt;
reg							timeout_flag;

// 状态判定
reg		[1:0]				last_measurement;
reg		[2:0]				consecutive_cnt;

assign measurement_from_result =
	(pps_width_result >= UNLOCKED_MIN && pps_width_result <= UNLOCKED_MAX) ? STATE_UNLOCKED :
	(pps_width_result >= LOCKED_MIN   && pps_width_result <= LOCKED_MAX)   ? STATE_LOCKED   :
	STATE_ABNORMAL;

//==============================================================================
// PPS信号三级同步
//==============================================================================
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		pps_reg1 <= 1'b0;
		pps_reg2 <= 1'b0;
		pps_reg3 <= 1'b0;
	end
	else begin
		pps_reg1 <= pps;
		pps_reg2 <= pps_reg1;
		pps_reg3 <= pps_reg2;
	end
end

// 边沿检测
assign pps_pos_edge = pps_reg2 & ~pps_reg3;
assign pps_neg_edge = ~pps_reg2 & pps_reg3;
assign pps_pos_edge_sync = pps_pos_edge;

//==============================================================================
// PPS边沿展宽（调试用，展宽约50ms）
//==============================================================================
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		pps_edge_extend_cnt <= 22'd0;
		pps_pos_edge_extended_reg <= 1'b0;
	end
	else begin
		if (pps_pos_edge) begin
			pps_edge_extend_cnt <= 22'd1_000_000;	// 50ms @ 20MHz
			pps_pos_edge_extended_reg <= 1'b1;
		end
		else if (pps_edge_extend_cnt > 0) begin
			pps_edge_extend_cnt <= pps_edge_extend_cnt - 1'b1;
		end
		else begin
			pps_pos_edge_extended_reg <= 1'b0;
		end
	end
end

//==============================================================================
// PPS脉宽测量
//==============================================================================
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		pps_width_cnt		<= 32'd0;
		pps_width_result	<= 32'd0;
		pps_width_valid		<= 1'b0;
		pps_width_overflow	<= 1'b0;
	end
	else begin
		pps_width_valid		<= 1'b0;
		pps_width_overflow	<= 1'b0;
		
		if (pps_pos_edge) begin
			pps_width_cnt <= 32'd1;
		end
		else if (pps_reg2) begin
			if (pps_width_cnt < PULSE_WIDTH_MAX)
				pps_width_cnt <= pps_width_cnt + 1'b1;
			else
				pps_width_overflow <= 1'b1;
		end
		else if (pps_neg_edge) begin
			pps_width_result	<= pps_width_cnt;
			pps_width_valid		<= 1'b1;
			pps_width_cnt		<= 32'd0;
		end
	end
end

//==============================================================================
// 超时检测
//==============================================================================
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		timeout_cnt		<= 32'd0;
		timeout_flag	<= 1'b0;
	end
	else begin
		timeout_flag <= 1'b0;
		
		if (pps_pos_edge) begin
			timeout_cnt <= 32'd0;
		end
		else if (timeout_cnt < TIMEOUT_MAX) begin
			timeout_cnt <= timeout_cnt + 1'b1;
			if (timeout_cnt == TIMEOUT_MAX - 1)
				timeout_flag <= 1'b1;
		end
	end
end

//==============================================================================
// 连续判定与状态切换（3次消抖）
//==============================================================================
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		gps_lock_status		<= STATE_ABNORMAL;
		last_measurement	<= STATE_ABNORMAL;
		consecutive_cnt		<= 3'd0;
	end
	else begin
		// 超时或脉宽超限：立即切换到异常
		if (timeout_flag || pps_width_overflow) begin
			gps_lock_status		<= STATE_ABNORMAL;
			last_measurement	<= STATE_ABNORMAL;
			consecutive_cnt		<= 3'd0;
		end
		// 正常脉宽测量：3次消抖
		else if (pps_width_valid) begin
			if (measurement_from_result == last_measurement) begin
				if (consecutive_cnt < CONSECUTIVE_CNT)
					consecutive_cnt <= consecutive_cnt + 1'b1;
				if (consecutive_cnt == CONSECUTIVE_CNT - 1)
					gps_lock_status <= measurement_from_result;
			end
			else begin
				last_measurement	<= measurement_from_result;
				consecutive_cnt		<= 3'd1;
			end
		end
	end
end

//==============================================================================
// 调试信号输出
//==============================================================================
always @(posedge clk or negedge rst_n) begin
	if (!rst_n)
		pps_pulse_width_dbg <= 32'd0;
	else if (pps_width_valid)
		pps_pulse_width_dbg <= pps_width_result;
end

endmodule
