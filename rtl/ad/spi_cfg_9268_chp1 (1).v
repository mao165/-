`timescale 1ns / 1ps


module spi_cfg_9268_chp1(
	//==========================================================================
	// 系统接口
	//==========================================================================
	input						clk_10m,			// 10MHz配置时钟
	input						rst,				// 复位信号（高有效）
	
	//==========================================================================
	// 状态输出
	//==========================================================================
	output	reg					cfg_done,			// 配置完成标志（高有效）
	
	//==========================================================================
	// SPI物理接口
	//==========================================================================
	inout						spi_sdio,			// SPI双向数据线
	output						spi_sclk,			// SPI时钟
	output						spi_csn				// SPI片选（低有效）
);

//==============================================================================
// 参数定义
//==============================================================================

// 配置寄存器数量
localparam	REG_NUMS		= 12;				// AD9268需配置12个寄存器

// FSM状态编码
localparam	S_IDLE			= 4'd0;				// 空闲，等待SPI空闲
localparam	S_WRITE			= 4'd1;				// 发起写操作
localparam	S_WAIT			= 4'd2;				// 等待SPI完成
localparam	S_DONE			= 4'd3;				// 配置完成

// SPI时钟分频参数
// 10MHz / 8 = 1.25MHz SPI时钟
localparam	SPI_CLK_DIV_BIT	= 2;				// 使用分频计数器的bit[2]

//==============================================================================
// 内部信号定义
//==============================================================================

// SPI时钟分频
reg			[2:0]			spi_clk_div;		// 分频计数器
wire						spi_clk;			// 分频后的SPI工作时钟(1.25MHz)

// SPI接口信号
wire						spi_busy;			// SPI忙标志
reg							spi_en;				// SPI传输使能
reg			[23:0]			spi_data;			// SPI发送数据 {R/W, ADDR[14:0], DATA[7:0]}

// 配置控制
reg			[7:0]			reg_index;			// 当前配置的寄存器索引
reg			[3:0]			state;				// FSM当前状态

//==============================================================================
// AD9268配置数据表 (20MSPS采样率配置)
//==============================================================================
// 数据格式: {1'b0(写), ADDR[14:0], DATA[7:0]} = 24bit

wire		[23:0]			cfg_data [0:REG_NUMS-1];

assign cfg_data[0]  = 24'h0005_03;				// Device Index
assign cfg_data[1]  = 24'h0008_80;				// Device Configuration
assign cfg_data[2]  = 24'h0009_01;				// Input Configuration
assign cfg_data[3]  = 24'h000b_00;				// Clock Configuration
assign cfg_data[4]  = 24'h000d_00;				// Test Mode 00 >> normal module ;04 >> check board module 0101010101
assign cfg_data[5]  = 24'h0010_00;				// Output Mode
assign cfg_data[6]  = 24'h0014_21;				// Output Phase/Delay
assign cfg_data[7]  = 24'h0016_00;				// VREF Configuration
assign cfg_data[8]  = 24'h0017_10;				// Analog Input
assign cfg_data[9]  = 24'h0018_c0;				// Buffer Control
assign cfg_data[10] = 24'h0030_10;				// Digital Output
assign cfg_data[11] = 24'h00ff_01;				// Software Reset/Update

//==============================================================================
// SPI时钟分频 (10MHz → 1.25MHz)
//==============================================================================

always @(posedge clk_10m or posedge rst) begin
	if (rst)
		spi_clk_div <= 3'd0;
	else
		spi_clk_div <= spi_clk_div + 1'b1;
end

assign spi_clk = spi_clk_div[SPI_CLK_DIV_BIT];

//==============================================================================
// 配置状态机
//==============================================================================

always @(posedge spi_clk or posedge rst) begin
	if (rst) begin
		spi_en		<= 1'b0;
		spi_data	<= 24'd0;
		reg_index	<= 8'd0;
		cfg_done	<= 1'b0;
		state		<= S_IDLE;
	end
	else begin
		case (state)
			//------------------------------------------------------------------
			// S_IDLE: 等待SPI模块空闲后开始配置
			//------------------------------------------------------------------
			S_IDLE: begin
				reg_index	<= 8'd1;
				cfg_done	<= 1'b0;
				
				if (!spi_busy) begin
					spi_en		<= 1'b1;
					spi_data	<= cfg_data[0];
					state		<= S_WRITE;
				end
				else begin
					spi_en		<= 1'b0;
					spi_data	<= 24'd0;
				end
			end
			
			//------------------------------------------------------------------
			// S_WRITE: 等待SPI空闲后写入当前寄存器
			//------------------------------------------------------------------
			S_WRITE: begin
				if (!spi_busy) begin
					spi_en		<= 1'b1;
					spi_data	<= cfg_data[reg_index];
					reg_index	<= reg_index + 1'b1;
					state		<= S_WAIT;
				end
			end
			
			//------------------------------------------------------------------
			// S_WAIT: 等待当前寄存器写入完成
			//------------------------------------------------------------------
			S_WAIT: begin
				spi_en		<= 1'b0;
				spi_data	<= 24'd0;
				
				if (reg_index >= REG_NUMS) begin
					state	<= S_DONE;
				end
				else begin
					state	<= S_WRITE;
				end
			end
			
			//------------------------------------------------------------------
			// S_DONE: 配置完成，保持状态
			//------------------------------------------------------------------
			S_DONE: begin
				cfg_done	<= 1'b1;
			end
			
			default: begin
				state		<= S_IDLE;
			end
		endcase
	end
end

//==============================================================================
// SPI主控模块实例化
//==============================================================================

spi u_spi (
	.clk				(spi_clk),
	.rst				(rst),
	.en					(spi_en),
	.rw					(spi_data[23]),
	.addr				(spi_data[22:8]),
	.wr_data			(spi_data[7:0]),
	.rd_data			(),
	.rd_valid			(),
	.busy				(spi_busy),
	.spi_sdio			(spi_sdio),
	.spi_sclk			(spi_sclk),
	.spi_csn			(spi_csn)
);

endmodule

