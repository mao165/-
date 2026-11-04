`timescale 1ns / 1ps
//******************************************************************************
//  文件名:		key_ctrl.v
//  模块名:		key_ctrl
//  功能描述:	按键控制模块
//				- 按键消抖处理（20ms消抖时间）
//				- 检测按键下降沿（按下事件）
//				- 生成命令信号给cmd_parse模块
//------------------------------------------------------------------------------
//  按键功能定义:
//		key1 - 按一下发送测试模式命令 (0x01)
//		key2 - 按一下发送触发模式命令 (0x02)
//------------------------------------------------------------------------------
//  输出命令格式（与cmd_parse兼容）:
//		0x01 - 测试模式命令
//		0x02 - 触发模式命令
//------------------------------------------------------------------------------


module key_ctrl(
	//==========================================================================
	// 系统信号
	//==========================================================================
	input	wire					clk,				// 用户时钟
	input	wire					rst_n,				// 系统复位（低有效）
	
	//==========================================================================
	// 按键输入（active low，按下为低电平）
	//==========================================================================
	input	wire					key1_n,				// 按键1（测试模式）
	input	wire					key2_n,				// 按键2（触发模式）
	
	//==========================================================================
	// 命令输出（与cmd_parse模块接口兼容）
	//==========================================================================
	output	reg						cmd_valid,			// 命令有效（单周期脉冲）
	output	reg		[7:0]			cmd_data			// 命令数据
);

//==============================================================================
// 参数定义
//==============================================================================

// 消抖时间参数
// 20ms @ 20MHz = 400,000个时钟周期
localparam	DEBOUNCE_CNT	= 20'd400_000;

// 命令定义（与cmd_parse兼容）
localparam	CMD_TEST		= 8'h01;			// 测试模式命令
localparam	CMD_TRIG		= 8'h02;			// 触发模式命令

//==============================================================================
// 内部信号定义
//==============================================================================

// 按键1消抖信号
reg		[2:0]			key1_sync;				// 同步寄存器（防亚稳态）
reg		[19:0]			key1_cnt;				// 消抖计数器
reg						key1_stable;			// 消抖后稳定值
reg						key1_stable_d;			// 稳定值延迟（用于边沿检测）
wire					key1_pressed;			// 按键1按下事件

// 按键2消抖信号
reg		[2:0]			key2_sync;				// 同步寄存器
reg		[19:0]			key2_cnt;				// 消抖计数器
reg						key2_stable;			// 消抖后稳定值
reg						key2_stable_d;			// 稳定值延迟
wire					key2_pressed;			// 按键2按下事件

//==============================================================================
// 按键1同步与消抖
//==============================================================================

// 三级同步，防止亚稳态
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		key1_sync <= 3'b111;					// 默认高电平（未按下）
	end
	else begin
		key1_sync <= {key1_sync[1:0], key1_n};
	end
end

// 消抖计数器
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		key1_cnt <= 20'd0;
		key1_stable <= 1'b1;
	end
	else begin
		if (key1_sync[2] != key1_stable) begin
			// 检测到电平变化，开始计数
			if (key1_cnt < DEBOUNCE_CNT) begin
				key1_cnt <= key1_cnt + 1'b1;
			end
			else begin
				// 计数完成，更新稳定值
				key1_stable <= key1_sync[2];
				key1_cnt <= 20'd0;
			end
		end
		else begin
			// 电平未变化，清零计数器
			key1_cnt <= 20'd0;
		end
	end
end

// 边沿检测（检测下降沿 = 按下事件）
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		key1_stable_d <= 1'b1;
	end
	else begin
		key1_stable_d <= key1_stable;
	end
end

assign key1_pressed = key1_stable_d & (~key1_stable);	// 下降沿检测

//==============================================================================
// 按键2同步与消抖
//==============================================================================

// 三级同步
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		key2_sync <= 3'b111;
	end
	else begin
		key2_sync <= {key2_sync[1:0], key2_n};
	end
end

// 消抖计数器
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		key2_cnt <= 20'd0;
		key2_stable <= 1'b1;
	end
	else begin
		if (key2_sync[2] != key2_stable) begin
			if (key2_cnt < DEBOUNCE_CNT) begin
				key2_cnt <= key2_cnt + 1'b1;
			end
			else begin
				key2_stable <= key2_sync[2];
				key2_cnt <= 20'd0;
			end
		end
		else begin
			key2_cnt <= 20'd0;
		end
	end
end

// 边沿检测
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		key2_stable_d <= 1'b1;
	end
	else begin
		key2_stable_d <= key2_stable;
	end
end

assign key2_pressed = key2_stable_d & (~key2_stable);

//==============================================================================
// 命令输出生成
//==============================================================================
// key1按下 → 发送测试模式命令 (0x01)
// key2按下 → 发送触发模式命令 (0x02)

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		cmd_valid <= 1'b0;
		cmd_data <= 8'h00;
	end
	else begin
		if (key1_pressed) begin
			cmd_valid <= 1'b1;
			cmd_data <= CMD_TEST;				// 0x01 测试模式
		end
		else if (key2_pressed) begin
			cmd_valid <= 1'b1;
			cmd_data <= CMD_TRIG;				// 0x02 触发模式
		end
		else begin
			cmd_valid <= 1'b0;
		end
	end
end

endmodule
