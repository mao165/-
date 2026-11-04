`timescale 1ns / 1ps


module ad_top(
	//==========================================================================
	// 系统时钟和控制信号
	//==========================================================================
	input						clk_50m,			// 50MHz系统时钟（用于启动延时计数）
	input						clk_10m,			// 10MHz配置时钟，用于SPI配置
	input						rst_n,				// 系统复位信号（低有效）
	input						locked,				// PLL锁定信号（高有效表示时钟稳定）
	output						power_good,			// FMC电源使能信号（必须拉高）
	
	//==========================================================================
	// ADC1接口 (AD9268芯片1)
	//==========================================================================
	input						adc1_dco,			// ADC1数据时钟输出(DCO)，20MHz，与数据同步
	input			[15:0]		adc1_data,			// ADC1并行数据[15:0]，DDR模式
	output						adc1_spi_ce,		// ADC1 SPI片选（低有效）
	output						adc1_spi_sclk,		// ADC1 SPI时钟
	inout						adc1_spi_io,		// ADC1 SPI数据线（双向SDIO）
	
	//==========================================================================
	// ADC2接口 (AD9268芯片2)
	//==========================================================================
	input						adc2_dco,			// ADC2数据时钟输出(DCO)，20MHz
	input			[15:0]		adc2_data,			// ADC2并行数据[15:0]
	output						adc2_spi_ce,		// ADC2 SPI片选（低有效）
	output						adc2_spi_sclk,		// ADC2 SPI时钟
	inout						adc2_spi_io,		// ADC2 SPI数据线（双向SDIO）
	
	//==========================================================================
	// AD9516时钟芯片接口
	//==========================================================================
	output						ad9516_spi_cs,		// AD9516 SPI片选（低有效）
	output						ad9516_spi_sclk,	// AD9516 SPI时钟
	inout						ad9516_spi_sdio,	// AD9516 SPI数据线（双向SDIO）
	
	//==========================================================================
	// 时钟输出接口
	//==========================================================================
	output						clk_adc1_acq,		// ADC1随路采集时钟，20MHz，供后级模块使用
	output						clk_200m,			// 200MHz时钟，供DDR3等模块使用
	output						clk_dco_locked,		// clk_dco MMCM锁定状态（高有效）
	
	//==========================================================================
	// 数据输出接口
	//==========================================================================
	output	reg		[15:0]		adc1_data_posedge,	// ADC1通道数据（上升沿采样）
	output	wire	[15:0]		adc1_data_raw_debug,	// 临时ILA：DCO域一级采样值
	output wire debug_startup_done,
	output wire debug_clock_cfg_done,
	output wire debug_adc1_cfg_done,
	output wire debug_adc2_cfg_done
	
	// [4通道扩展预留] 其他通道数据输出
	// output reg	[15:0]		adc1_data_negedge,	// ADC1通道B数据（下降沿采样）
	// output reg	[15:0]		adc2_data_posedge,	// ADC2通道A数据（上升沿采样）
	// output reg	[15:0]		adc2_data_negedge	// ADC2通道B数据（下降沿采样）
);

//==============================================================================
// 参数定义
//==============================================================================

// 启动延时参数
// 说明: FPGA上电后需等待外部芯片(AD9516/AD9268)电源稳定后再进行SPI配置
// 计算: 500ms = 25,000,000个时钟周期 @ 50MHz
localparam	STARTUP_DELAY_CNT	= 32'd25_000_000;	// 启动延时计数值(500ms@50MHz)

// Clock chip selection:
//   1'b1 = AD9528 new FMC board
//   1'b0 = AD9516 old FMC board
localparam	USE_AD9528			= 1'b1;

//==============================================================================
// 内部信号定义
//==============================================================================

// 时钟信号
wire						adc1_clk_bufg;			// ADC1 DCO经BUFG后的全局时钟
wire						clk_adc1_ila;			// ADC1 ILA调试时钟
wire						clk_adc1_mmcm;			// MMCM输出的50MHz时钟（用于后级模块）

// [4通道扩展预留] ADC2时钟信号
// wire					adc2_clk_bufg;			// ADC2 DCO经BUFG后的全局时钟
// wire					clk_adc2_acq;			// ADC2采样时钟
// wire					clk_adc2_ila;			// ADC2 ILA调试时钟

// [4通道扩展预留] ADC数据寄存器（双沿采样实现4通道）
// reg		[15:0]		adc1_data_negedge;		// ADC1下降沿采样数据（通道B）
// reg		[15:0]		adc2_data_posedge;		// ADC2上升沿采样数据（通道A）
// reg		[15:0]		adc2_data_negedge;		// ADC2下降沿采样数据（通道B）

// 启动控制信号
reg						startup_done;			// 启动延时完成标志
reg		[31:0]			cnt_startup = 32'd0;	// 启动延时计数器

//==============================================================================
// FMC电源控制
//==============================================================================
// FMC插座的PowerGood信号必须拉高，否则FMC子卡(ADC采集卡)无法正常工作
assign power_good = 1'b1;
assign debug_startup_done = startup_done;

//==============================================================================
// 启动延时逻辑
//==============================================================================
// 功能: FPGA固化程序上电后，等待500ms再启动SPI配置
// 原因: 确保外部芯片电源稳定、晶振起振完成后再进行配置，提高配置成功率
// 时序: 上电 → 等待500ms → startup_done拉高 → SPI配置开始

always @(posedge clk_50m) begin
	if (!rst_n || !locked) begin
		cnt_startup	<= 32'd0;
		startup_done	<= 1'b0;
	end
	else if (cnt_startup < STARTUP_DELAY_CNT) begin
		cnt_startup		<= cnt_startup + 1'b1;
		startup_done	<= 1'b0;
	end
	else begin
		startup_done	<= 1'b1;
	end
end

//==============================================================================
// ADC数据采样
//==============================================================================
// AD9268工作在DDR模式: 上升沿输出ChA数据，下降沿输出ChB数据
// 当前仅使用上升沿采样(ChA)，下降沿采样预留用于4通道扩展
// 注意: 必须用BUFG后的DCO时钟直接采样，不能用MMCM输出时钟，否则相位不对

// 第一级采样：用DCO时钟采样数据（保持源同步关系）
reg		[15:0]		adc1_data_raw;
assign adc1_data_raw_debug = adc1_data_raw;

always @(posedge adc1_clk_bufg) begin
	adc1_data_raw <= adc1_data;
end

// 第二级采样：同步到MMCM时钟域（用于后级模块）
always @(posedge clk_adc1_acq) begin
	adc1_data_posedge <= adc1_data_raw;
end

// [4通道扩展预留] ADC1下降沿采样 - 捕获通道B数据
// always @(negedge clk_adc1_acq) begin
//     adc1_data_negedge <= adc1_data;
// end

// [4通道扩展预留] ADC2双沿采样
// always @(posedge clk_adc2_acq) begin
//     adc2_data_posedge <= adc2_data;
// end
// always @(negedge clk_adc2_acq) begin
//     adc2_data_negedge <= adc2_data;
// end

//==============================================================================
// SPI配置模块实例化
//==============================================================================
// 配置时序: PLL锁定 && 启动延时完成 → 释放复位 → 开始SPI配置
// 复位条件: locked=0(PLL未锁定) 或 startup_done=0(延时未完成) 时保持复位

// Clock chip SPI configuration.
// Keep both AD9528 and AD9516 paths so new/old FMC boards can be selected by
// changing USE_AD9528 above. The external pin names stay ad9516_spi_* because
// the board-level SPI pins are shared by both clock-chip versions.
generate
	if (USE_AD9528) begin : gen_ad9528_cfg
		spi_cfg_9528 u_ad9528_cfg (
			.clk_spi			(clk_50m),
			.rst				((!locked) || (!startup_done)),		// 高有效复位
			.cfg_done			(debug_clock_cfg_done),									// 配置完成标志（生产诊断接出）
			.spi_sdio			(ad9516_spi_sdio),
			.spi_sclk			(ad9516_spi_sclk),
			.spi_csn			(ad9516_spi_cs)
		);
	end
	else begin : gen_ad9516_cfg
		spi_cfg_9516 u_ad9516_cfg (
			.clk_10m			(clk_10m),
			.rst				((!locked) || (!startup_done)),		// 高有效复位
			.cfg_done			(debug_clock_cfg_done),									// 配置完成标志（生产诊断接出）
			.spi_sdio			(ad9516_spi_sdio),
			.spi_sclk			(ad9516_spi_sclk),
			.spi_csn			(ad9516_spi_cs)
		);
	end
endgenerate

// ADC1芯片配置 - 配置AD9268工作模式、输出格式等
spi_cfg_9268_chp1 u_adc1_cfg (
	.clk_10m			(clk_10m),
	.rst				((!locked) || (!startup_done)),
	.cfg_done			(debug_adc1_cfg_done),
	.spi_sdio			(adc1_spi_io),
	.spi_sclk			(adc1_spi_sclk),
	.spi_csn			(adc1_spi_ce)
);

// ADC2芯片配置
spi_cfg_9268_chp2 u_adc2_cfg (
	.clk_10m			(clk_10m),
	.rst				((!locked) || (!startup_done)),
	.cfg_done			(debug_adc2_cfg_done),
	.spi_sdio			(adc2_spi_io),
	.spi_sclk			(adc2_spi_sclk),
	.spi_csn			(adc2_spi_ce)
);

//==============================================================================
// 时钟网络
//==============================================================================
// ADC DCO时钟处理流程: DCO引脚 → BUFG(全局时钟缓冲) → MMCM → 采样时钟/200MHz时钟

// ADC1 DCO时钟BUFG - 将外部DCO时钟引入FPGA全局时钟网络
BUFG BUFG_adc1_dco (
	.O					(adc1_clk_bufg),			// 输出: 全局时钟
	.I					(adc1_dco)					// 输入: ADC1 DCO引脚
);

// [4通道扩展预留] ADC2 DCO时钟BUFG
// BUFG BUFG_adc2_dco (
//     .O				(adc2_clk_bufg),
//     .I				(adc2_dco)
// );

// ADC1时钟管理模块(MMCM) - 生成采样时钟、ILA调试时钟和200MHz时钟
// IP核配置 (Vivado Clocking Wizard):
//   - IP名称: clk_dco
//   - 输入时钟: 50MHz，单端，来自BUFG
//   - 输出时钟1 (clk_out1): 50MHz  - 采样时钟（用于后级模块和跨时钟域同步）
//   - 输出时钟2 (clk_out2): 200MHz - DDR3参考时钟
//   - 复位类型: 无复位
//   - 输出locked信号
clk_dco u_clk_dco (
	.clk_in1			(adc1_clk_bufg),			// 输入: DCO全局时钟 50MHz
	.clk_out1			(clk_adc1_acq),				// 输出: 50MHz采样时钟
	.clk_out2			(clk_adc1_ila),
	.clk_out3			(clk_200m),					// 输出: 200MHz DDR3参考时钟
	.locked				(clk_dco_locked)			// 输出: MMCM锁定状态
);

// [4通道扩展预留] ADC2时钟管理模块
// adc2_clock u_adc2_clk_gen (
//     .clk_out1		(clk_adc2_acq),
//     .clk_out2		(clk_adc2_ila),
//     .clk_in1		(adc2_clk_bufg)
// );

// 生产采样探针在mode2的20MHz原生域统一观测。
endmodule
