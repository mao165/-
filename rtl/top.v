`timescale 1ns / 1ps
//******************************************************************************
//  文件名:		top.v
//  模块名:		top
//  功能描述:	系统顶层模块
//				- 整合时钟管理、AD采集、GPS定位、按键控制、双模式核心、LED指示
//				- 仅包含模块间信号互联，无逻辑处理
//------------------------------------------------------------------------------
//  系统架构:
//  ┌─────────────────────────────────────────────────────────────────────────┐
//  │                               top.v                                     │
//  │                                                                         │
//  │   sys_clk ────────> ┌───────────┐                                       │
//  │                     │ clock_top │──> clk_50m ──────────────────┐        │
//  │                     │           │──> clk_10m ──────────────────┼──┐     │
//  │                     │           │──> clk_200m ────────────────┐ │  │     │
//  │                     └───────────┘                              │ │  │     │
//  │                                                                │ │  │     │
//  │   ADC接口 ────────> ┌───────────┐                              │ │  │     │
//  │                     │  ad_top   │──> clk_adc1_acq ─────────────┼─┼──┼──┐  │
//  │                     │           │──> clk_200m ─────────────────┼─┼──┼──┼──┐│
//  │                     │           │──> adc_data ─────────────────┼─┼──┼──┼──┼┤
//  │                     └───────────┘                              │ │  │  │  ││
//  │                                                                │ │  │  │  ││
//  │   GPS接口 ────────> ┌───────────┐                              │ │  │  │  ││
//  │                     │  GPS_top  │<── clk_adc1_acq ─────────────┼─┼──┘  │  ││
//  │                     │           │──> pps_sync ─────────────────┼─┼──┐  │  ││
//  │                     │           │──> gps_lock_status ──────────┼─┼──┼──┼──┼┤
//  │                     └───────────┘                              │ │  │  │  ││
//  │                                                                │ │  │  │  ││
//  │   key1/key2 ──────> ┌───────────┐                              │ │  │  │  ││
//  │                     │ key_ctrl  │──> cmd_valid/cmd_data ───────┼─┼──┼──┼──┼┤
//  │                     └───────────┘                              │ │  │  │  ││
//  │                                                                │ │  │  │  ││
//  │                     ┌───────────────────────────────────────┐  │ │  │  │  ││
//  │                     │           mode2_top                   │<─┴─┴──┼──┘  ││
//  │   DDR3接口 <══════> │  (DDR3缓存 + 双模式数据处理)          │       │     │
//  │                     │                                       │──> ddr_init_done
//  │                     └───────────────────────────────────────┘       │     │
//  │                                                                     │     │
//  │   LED输出 <──────── ┌───────────┐                                   │     │
//  │                     │  led_top  │<──────────────────────────────────┘     │
//  │                     └───────────┘                                         │
//  └─────────────────────────────────────────────────────────────────────────┘


module top(
	//==========================================================================
	// 系统时钟和复位
	//==========================================================================
	input					sys_clk_p,				// 差分时钟正端（来自板载晶振100MHz）
	input					sys_clk_n,				// 差分时钟负端
	input					sys_rst_n,				// 系统复位（低有效，来自复位按键）
	
	//==========================================================================
	// ADC1 (AD9268芯片1) 接口
	//==========================================================================
	input					adc1_dco,				// ADC1数据时钟（随路时钟20MHz）
	input		[15:0]		adc1_data,				// ADC1数据总线（DDR模式）
	output					adc1_spi_ce,			// ADC1 SPI片选（低有效）
	output					adc1_spi_sclk,			// ADC1 SPI时钟
	inout					adc1_spi_io,			// ADC1 SPI数据（双向）
	
	//==========================================================================
	// ADC2 (AD9268芯片2) 接口
	//==========================================================================
	input					adc2_dco,				// ADC2数据时钟
	input		[15:0]		adc2_data,				// ADC2数据总线
	output					adc2_spi_ce,			// ADC2 SPI片选
	output					adc2_spi_sclk,			// ADC2 SPI时钟
	inout					adc2_spi_io,			// ADC2 SPI数据
	
	//==========================================================================
	// AD9516时钟芯片接口
	//==========================================================================
	output					ad9516_spi_cs,			// AD9516 SPI片选
	output					ad9516_spi_sclk,		// AD9516 SPI时钟
	inout					ad9516_spi_sdio,		// AD9516 SPI数据
	output					power_good,				// FMC电源使能
	
	//==========================================================================
	// GPS模块接口
	//==========================================================================
	input					rx,						// GPS UART接收
	output					tx,						// GPS UART发送
	input					pps,					// GPS PPS脉冲
	
	//==========================================================================
	// 按键输入
	//==========================================================================
	input					key1_n,					// 按键1（测试模式，低有效）
	input					key2_n,					// 按键2（触发模式，低有效）
	
	//==========================================================================
	// 以太网RGMII接口 【新增】
	//==========================================================================
	input					eth_rxc,				// RGMII接收时钟
	input					eth_rx_ctl,				// RGMII接收数据有效
	input		[3:0]		eth_rxd,				// RGMII接收数据
	output					eth_txc,				// RGMII发送时钟
	output					eth_tx_ctl,				// RGMII发送数据有效
	output		[3:0]		eth_txd,				// RGMII发送数据
	output					eth_rst_n,				// PHY芯片复位
	
	//==========================================================================
	// DDR3物理接口
	//==========================================================================
	inout		[63:0]		ddr3_dq,				// 数据总线
	inout		[7:0]		ddr3_dqs_n,				// 数据选通负
	inout		[7:0]		ddr3_dqs_p,				// 数据选通正
	output		[14:0]		ddr3_addr,				// 地址总线
	output		[2:0]		ddr3_ba,				// Bank地址
	output					ddr3_ras_n,				// 行地址选通
	output					ddr3_cas_n,				// 列地址选通
	output					ddr3_we_n,				// 写使能
	output					ddr3_reset_n,			// DDR3复位
	output		[0:0]		ddr3_ck_p,				// 时钟正
	output		[0:0]		ddr3_ck_n,				// 时钟负
	output		[0:0]		ddr3_cke,				// 时钟使能
	output		[0:0]		ddr3_cs_n,				// 片选
	output		[7:0]		ddr3_dm,				// 数据掩码
	output		[0:0]		ddr3_odt,				// 片上终端
	
	//==========================================================================
	// LED状态指示
	//==========================================================================
	output					led_gps_lock,			// GPS锁定状态LED
	output					led_ddr_init,			// DDR3初始化状态LED
	output					led_sys_run,			// 系统运行状态LED
	output					led_transfer			// 数据传输状态LED
);

//==============================================================================
// 内部信号定义
//==============================================================================

//------------------------------------------------------------------------------
// 时钟信号（由clock_top生成）
//------------------------------------------------------------------------------
wire					clk_50m;					// 50MHz通用时钟
wire					clk_10m;					// 10MHz配置时钟
wire					clk_200m;					// 200MHz DDR3参考时钟（新增，来自板载晶振）
wire					clk_40m;					// 40MHz readout clock
wire					locked;						// PLL锁定状态
wire					rst_n;						// 全局复位信号

//------------------------------------------------------------------------------
// 时钟信号（由ad_top生成）
//------------------------------------------------------------------------------

wire					clk_dco_20m;				// 20MHz DCO时钟（ad_top输出）
wire					clk_dco_200m;				// 200MHz DCO时钟（ad_top输出）
wire					clk_dco_locked;				// clk_dco MMCM锁定状态

//------------------------------------------------------------------------------
// 复位信号
//------------------------------------------------------------------------------
wire					rst_n_dco_req;				// reset request before per-clock release sync
wire					rst_n_dco;					// DCO/20MHz synchronized reset
wire					rst_n_40m;					// 40MHz synchronized reset
wire					rst_n_gmii_rx;				// GMII RX synchronized reset
wire					rst_n_gmii_tx;				// GMII TX synchronized reset

//------------------------------------------------------------------------------
// ADC数据信号
//------------------------------------------------------------------------------
wire		[15:0]		adc_data;					// ADC1采样数据
wire		[15:0]		adc_data_raw_debug;			// 临时ILA：ADC一级采样数据

//------------------------------------------------------------------------------
// GPS信号
//------------------------------------------------------------------------------
wire		[1:0]		gps_lock_status;			// GPS锁定状态
wire					pps_pos_edge_sync;
wire					pps_pos_edge_extended;			// PPS同步上升沿

(* KEEP = "TRUE" *) 
wire		[23:0]		rmc_time;					// RMC北京时间

(* KEEP = "TRUE" *) 
wire		[39:0]		rmc_latitude;				// RMC纬度

(* KEEP = "TRUE" *) 
wire		[39:0]		rmc_longitude;				// RMC经度

(* KEEP = "TRUE" *) 
wire		[23:0]		rmc_date;					// RMC日期

//------------------------------------------------------------------------------
// 按键命令信号
//------------------------------------------------------------------------------
wire					key_cmd_valid;				// 按键命令有效
wire		[7:0]		key_cmd_data;				// 按键命令数据

//------------------------------------------------------------------------------
// mode2状态信号
//------------------------------------------------------------------------------
wire					ddr_init_done;				// DDR3初始化完成
wire					sys_running;				// 系统运行标志
wire					transfer_active;			// 传输进行中标志

// 数据输出（预留，用于后续扩展）
wire		[15:0]		data_out;					// 输出数据
wire					data_out_valid;				// 输出数据有效
wire					transfer_flag;				// 传输标志
wire					data_stream_flag;			// 数据流标志
wire                    ts_req;                     // timestamp packet request
wire        [2:0]       ts_sec_idx;                 // timestamp partition index
wire        [1:0]       ts_work_mode;               // timestamp source work mode
wire        [1:0]       ts_gps_lock_status;         // timestamp GPS lock status
wire        [23:0]      ts_date;                    // timestamp date
wire        [23:0]      ts_time;                    // timestamp time
wire        [15:0]      ts_temp_raw;                // timestamp-bound FPGA temperature raw code
wire        [7:0]       ts_temp_flags;              // timestamp-bound FPGA temperature flags
//------------------------------------------------------------------------------
// LED状态编码
//------------------------------------------------------------------------------
wire		[1:0]		ddr_init_status;			// DDR初始化状态
wire		[1:0]		sys_run_status;				// 系统运行状态
wire		[1:0]		transfer_status;			// 传输状态

//------------------------------------------------------------------------------
// 以太网信号 【新增】
//------------------------------------------------------------------------------
// 以太网时钟
wire					gmii_rx_clk;				// GMII接收时钟（125MHz，来自PHY）
wire					gmii_tx_clk;				// GMII发送时钟（125MHz）

// UDP接收信号
wire					udp_rec_pkt_done;			// UDP包接收完成
wire					udp_rec_en;					// UDP接收使能
wire		[7:0]		udp_rec_data;				// UDP接收数据
wire		[15:0]		udp_rec_byte_num;			// UDP接收字节数

// UDP发送信号
wire					udp_tx_start_en;			// UDP开始发送
wire		[7:0]		udp_tx_data;				// UDP发送数据
wire		[15:0]		udp_tx_byte_num;			// UDP发送字节数
wire					udp_tx_done;				// UDP发送完成
wire					udp_tx_req;					// UDP发送请求
wire                    ts_req_125m;
wire        [126:0]      ts_sync_data_125m;
wire        [15:0]       ts_threshold;
wire        [15:0]       ts_threshold_125m;
wire        [1:0]       ts_work_mode_125m;
wire        [1:0]       ts_gps_lock_status_125m;
wire        [2:0]       ts_sec_idx_125m;
wire        [23:0]      ts_date_125m;
wire        [23:0]      ts_time_125m;
wire        [15:0]      ts_temp_raw_125m;
wire        [7:0]       ts_temp_flags_125m;
wire                    ts_send_done_40m;
wire        [0:0]       ts_send_done_data_40m;
wire                    cmd_mailbox_overflow_125m;
wire                    ts_req_mailbox_overflow_40m;
wire                    ts_done_mailbox_overflow_125m;
reg                     data_tx_enable_125m;
reg         [2:0]       data_sec_idx_125m;

wire                    ts_udp_tx_start_en;
wire        [7:0]       ts_udp_tx_data;
wire        [15:0]      ts_udp_tx_byte_num;
wire                    ts_udp_tx_req;
wire                    ts_udp_tx_done;
wire                    ts_pkt_busy;
wire                    ts_pkt_done;

wire                    data_udp_tx_start_en;
wire        [7:0]       data_udp_tx_data;
wire        [15:0]      data_udp_tx_byte_num;
wire                    data_udp_tx_req;
wire                    data_udp_tx_done;
wire                    data_tx_busy;
wire        [15:0]      data_pkt_idx;
wire        [11:0]      data_fifo_rdusedw;
wire                    data_segment_done;
wire                    data_segment_ready;
wire [4:0]              eth_debug_segment_barrier;
wire                    data_fifo_full;
wire                    data_fifo_drop_sticky;
wire        [1:0]       debug_udp_tx_sel;
// 以太网命令解析信号（125MHz域）
wire					eth_cmd_valid_125m;			// 命令有效（125MHz域）
wire		[23:0]		eth_cmd_data_125m;			// 命令数据（125MHz域）

// 以太网命令同步信号（20MHz域）
wire					eth_cmd_valid_sync;			// 命令有效（同步到20MHz）
wire		[23:0]		eth_cmd_data_sync;			// 命令数据（同步到20MHz）

// 合并后的命令信号
wire					cmd_valid;					// 最终命令有效（按键或以太网）
wire		[23:0]		cmd_data;					// 最终命令数据
//------------------------------------------------------------------------------
// 生产系统总览只同步独立单bit状态，不同步样点/时间/模式多位总线。
(* ASYNC_REG = "TRUE" *) reg clk_dco_locked_s1, clk_dco_locked_s2, clk_dco_locked_s3;
(* ASYNC_REG = "TRUE" *) reg rst_n_dco_s1, rst_n_dco_s2, rst_n_dco_s3;
(* ASYNC_REG = "TRUE" *) reg locked_s1, locked_s2, locked_s3;
(* ASYNC_REG = "TRUE" *) reg ddr_init_done_s1, ddr_init_done_s2, ddr_init_done_s3;
reg sys_running_observe_dco; // Register diagnostic combinational status before CDC.
reg transfer_flag_observe_40m;
(* ASYNC_REG = "TRUE" *) reg sys_running_s1, sys_running_s2, sys_running_s3;
(* ASYNC_REG = "TRUE" *) reg transfer_flag_s1, transfer_flag_s2, transfer_flag_s3;
(* ASYNC_REG = "TRUE" *) reg transfer_flag_20m_s1, transfer_flag_20m_s2;

wire ad_startup_done, ad_clock_cfg_done, ad_adc1_cfg_done, ad_adc2_cfg_done;
wire [3:0] gps_debug_events;
wire [23:0] gps_debug_candidate_date, gps_debug_candidate_time;
wire [11:0] gps_debug_state;
wire [31:0] gps_debug_counts, gps_debug_pps_width;
wire [15:0] mode2_debug_read_control, mode2_debug_unpack_level;
wire [34:0] mode2_debug_read_identity;
wire [63:0] mode2_debug_read_word;
wire [36:0] mode2_debug_read_count;
wire [47:0] mode2_debug_read_sample;
wire [4:0] mode2_debug_gps_events;
wire [31:0] mode2_debug_time_transaction_seq;
wire [25:0] mode2_debug_command_state;
wire mode2_debug_ui_heartbeat;
wire [31:0] ts_event_seq;
wire cmd_mailbox_busy_125m, ts_req_mailbox_busy_40m, ts_done_mailbox_busy_125m;
wire debug_arbiter_ts_pending;
wire [3:0] debug_ts_byte_idx;
wire eth_debug_fifo_wr_accept, eth_debug_fifo_drop_reset;
wire eth_debug_fifo_wr_rst_busy, eth_debug_fifo_rd_rst_busy;
wire eth_debug_payload_underflow, eth_debug_fifo_rd_en, eth_debug_fifo_empty;
wire [10:0] eth_debug_tx_byte_idx;
wire [7:0] eth_debug_fifo_dout;
wire [5:0] eth_debug_fifo_rst_cnt;
wire [31:0] debug_ts_seq_125m;
reg [31:0] debug_data_seq_125m;
wire [31:0] data_segment_done_seq_125m;
wire segment_send_done_40m;
wire [31:0] segment_send_done_seq_40m;
wire segment_done_mailbox_busy_125m, segment_done_mailbox_overflow_125m;

// Whole-chain recovery is coordinated in the continuously running 40 MHz domain.
wire recovery_hold_request, recovery_clear_request, recovery_fifo_request;
wire recovery_mig_rst_n, recovery_business_enable;
wire [2:0] recovery_state, recovery_clear_seen, recovery_fifo_seen;
wire [2:0] recovery_ready_seen, recovery_run_seen;
wire [1:0] recovery_fault_reason;
wire [31:0] recovery_count, recovery_wait_age;
wire recovery_calib_ready, recovery_calib_lost, recovery_overwrite;
wire recovery_mode2_ready, recovery_local_ready_40m;
wire data_rst_n_40m, data_rst_n_tx;
wire recovery_hold_tx, recovery_clear_tx, recovery_fifo_tx;
wire recovery_tx_clear_seen, recovery_tx_fifo_seen, recovery_tx_ready, recovery_tx_run;
wire recovery_ts_quiet, recovery_data_quiet, recovery_arbiter_quiet;
wire ts_start_accept, data_start_accept;
wire [2:0] ts_done_sec_idx_125m;
wire [31:0] ts_done_seq_125m;
reg [1:0] recovery_tx_quiet_pipe;
(* ASYNC_REG = "TRUE" *) reg [4:0] recovery_tx_s1, recovery_tx_s2;
wire recovery_tx_quiet_40m = recovery_tx_s2[4];
wire recovery_tx_local_ready = !eth_debug_fifo_rd_rst_busy &&
    !ts_done_mailbox_busy_125m && !segment_done_mailbox_busy_125m;
assign recovery_local_ready_40m = data_rst_n_40m && recovery_mode2_ready &&
    !ts_req_mailbox_busy_40m && !eth_debug_fifo_wr_rst_busy;
assign {recovery_run_seen[2],recovery_ready_seen[2],
        recovery_fifo_seen[2],recovery_clear_seen[2]} = recovery_tx_s2[3:0];

reset_sync u_recovery_reset_40m (
    .clk(clk_40m), .arst_n(rst_n_40m && !recovery_clear_request),
    .srst_n(data_rst_n_40m)
);
recovery_domain_endpoint u_recovery_tx (
    .clk(gmii_tx_clk), .rst_n(rst_n_gmii_tx),
    .hold_request(recovery_hold_request), .clear_request(recovery_clear_request),
    .fifo_request(recovery_fifo_request), .local_ready(recovery_tx_local_ready),
    .hold_local(recovery_hold_tx), .clear_local(recovery_clear_tx),
    .fifo_local(recovery_fifo_tx), .clear_seen(recovery_tx_clear_seen),
    .fifo_seen(recovery_tx_fifo_seen), .ready_seen(recovery_tx_ready),
    .run_seen(recovery_tx_run), .data_rst_n(data_rst_n_tx)
);
always @(posedge gmii_tx_clk or negedge rst_n_gmii_tx) begin
    if (!rst_n_gmii_tx) recovery_tx_quiet_pipe <= 2'b00;
    else if (recovery_hold_tx && recovery_ts_quiet && recovery_data_quiet &&
             recovery_arbiter_quiet && !udp_tx_start_en && !udp_tx_done)
        recovery_tx_quiet_pipe <= {recovery_tx_quiet_pipe[0],1'b1};
    else recovery_tx_quiet_pipe <= 2'b00;
end
always @(posedge clk_40m or negedge rst_n_40m) begin
    if (!rst_n_40m) begin
        recovery_tx_s1 <= 5'd0;
        recovery_tx_s2 <= 5'd0;
    end else begin
        // Five unrelated acknowledgement levels; no multi-bit data crosses here.
        recovery_tx_s1 <= {recovery_tx_quiet_pipe[1],recovery_tx_run,
            recovery_tx_ready,recovery_tx_fifo_seen,recovery_tx_clear_seen};
        recovery_tx_s2 <= recovery_tx_s1;
    end
end
data_chain_recovery u_data_chain_recovery (
    .clk(clk_40m), .rst_n(rst_n_40m),
    .fault_overwrite(recovery_overwrite),
    .fault_calib_lost(recovery_calib_lost || !recovery_calib_ready),
    .calib_ready(recovery_calib_ready), .tx_quiet(recovery_tx_quiet_40m),
    .clear_seen(recovery_clear_seen), .fifo_seen(recovery_fifo_seen),
    .ready_seen(recovery_ready_seen), .run_seen(recovery_run_seen),
    .local_ready_40(recovery_local_ready_40m),
    .hold_request(recovery_hold_request), .clear_request(recovery_clear_request),
    .fifo_request(recovery_fifo_request), .mig_sys_rst_n(recovery_mig_rst_n),
    .business_enable(recovery_business_enable), .state(recovery_state),
    .fault_reason(recovery_fault_reason), .recovery_count(recovery_count),
    .wait_age(recovery_wait_age)
);

//==============================================================================
// Reset synchronization
//==============================================================================
assign rst_n_dco_req = sys_rst_n & locked & clk_dco_locked;

reset_sync u_rst_sync_20m (
    .clk        (clk_dco_20m),
    .arst_n     (rst_n_dco_req),
    .srst_n     (rst_n_dco)
);

reset_sync u_rst_sync_40m (
    .clk        (clk_40m),
    .arst_n     (rst_n_dco_req),
    .srst_n     (rst_n_40m)
);

reset_sync u_rst_sync_gmii_rx (
    .clk        (gmii_rx_clk),
    .arst_n     (rst_n_dco_req),
    .srst_n     (rst_n_gmii_rx)
);

reset_sync u_rst_sync_gmii_tx (
    .clk        (gmii_tx_clk),
    .arst_n     (rst_n_dco_req),
    .srst_n     (rst_n_gmii_tx)
);

//==============================================================================
// ILA observation synchronizers
//==============================================================================
// Only LED/ILA status takes this extra native-domain register.
always @(posedge clk_40m or negedge rst_n_40m) begin
    if (!rst_n_40m) transfer_flag_observe_40m <= 1'b0;
    else transfer_flag_observe_40m <= transfer_flag;
end

always @(posedge clk_dco_20m or negedge rst_n_dco) begin
    if (!rst_n_dco) begin
        sys_running_observe_dco <= 1'b0;
        transfer_flag_20m_s1 <= 1'b0;
        transfer_flag_20m_s2 <= 1'b0;
    end else begin
        sys_running_observe_dco <= sys_running;
        transfer_flag_20m_s1 <= transfer_flag_observe_40m;
        transfer_flag_20m_s2 <= transfer_flag_20m_s1;
    end
end

always @(posedge clk_50m or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        {clk_dco_locked_s3, clk_dco_locked_s2, clk_dco_locked_s1} <= 3'b000;
        {rst_n_dco_s3, rst_n_dco_s2, rst_n_dco_s1} <= 3'b000;
        {locked_s3, locked_s2, locked_s1} <= 3'b000;
        {ddr_init_done_s3, ddr_init_done_s2, ddr_init_done_s1} <= 3'b000;
        {sys_running_s3, sys_running_s2, sys_running_s1} <= 3'b000;
        {transfer_flag_s3, transfer_flag_s2, transfer_flag_s1} <= 3'b000;
    end else begin
        clk_dco_locked_s1 <= clk_dco_locked; clk_dco_locked_s2 <= clk_dco_locked_s1; clk_dco_locked_s3 <= clk_dco_locked_s2;
        rst_n_dco_s1 <= rst_n_dco; rst_n_dco_s2 <= rst_n_dco_s1; rst_n_dco_s3 <= rst_n_dco_s2;
        locked_s1 <= locked; locked_s2 <= locked_s1; locked_s3 <= locked_s2;
        ddr_init_done_s1 <= ddr_init_done; ddr_init_done_s2 <= ddr_init_done_s1; ddr_init_done_s3 <= ddr_init_done_s2;
        sys_running_s1 <= sys_running_observe_dco; sys_running_s2 <= sys_running_s1; sys_running_s3 <= sys_running_s2;
        transfer_flag_s1 <= transfer_flag_observe_40m; transfer_flag_s2 <= transfer_flag_s1; transfer_flag_s3 <= transfer_flag_s2;
    end
end
//==============================================================================
// 状态编码逻辑
//==============================================================================

// DDR初始化状态: 00=未完成, 01=已完成
assign ddr_init_status = ddr_init_done ? 2'b01 : 2'b00;

// 系统运行状态: 00=待机, 01=运行中
assign sys_run_status = sys_running ? 2'b01 : 2'b00;

// 传输状态: 00=无传输, 10=传输中(慢闪)
assign transfer_status = transfer_flag_20m_s2 ? 2'b10 : 2'b00;

//==============================================================================
// 命令源合并逻辑 【新增】
//==============================================================================
// 按键命令和以太网命令任一有效即可触发
// 优先级：按键优先（因为以太网命令需要跨时钟域同步，有延迟）
assign cmd_valid = key_cmd_valid | eth_cmd_valid_sync;
assign cmd_data  = key_cmd_valid ? {key_cmd_data, 16'h0000} : eth_cmd_data_sync;

//==============================================================================
// 时钟管理模块
// 功能：差分转单端、PLL生成多路时钟、复位生成
//==============================================================================
clock_top u_clock_top(
	// 系统时钟输入
	.sys_clk_p				(sys_clk_p),
	.sys_clk_n				(sys_clk_n),
	.sys_rst_n				(sys_rst_n),
	// 时钟输出
	.clk_50m				(clk_50m),
	.clk_10m				(clk_10m),
	.clk_200m				(clk_200m),				// 新增：200MHz DDR3参考时钟
	.clk_40m				(clk_40m),				// 40MHz readout clock
	// 状态输出
	.locked					(locked),
	.rst_n					(rst_n)
);

//==============================================================================
// AD采集模块
// 功能：AD9516/AD9268 SPI配置、ADC数据采集、DCO时钟处理
//==============================================================================
ad_top u_ad_top(
	// 时钟和复位
	.clk_50m				(clk_50m),
	.clk_10m				(clk_10m),
	.rst_n					(rst_n),
	.locked					(locked),
	.power_good				(power_good),
	// ADC1接口
	.adc1_dco				(adc1_dco),
	.adc1_data				(adc1_data),
	.adc1_spi_ce			(adc1_spi_ce),
	.adc1_spi_sclk			(adc1_spi_sclk),
	.adc1_spi_io			(adc1_spi_io),
	// ADC2接口
	.adc2_dco				(adc2_dco),
	.adc2_data				(adc2_data),
	.adc2_spi_ce			(adc2_spi_ce),
	.adc2_spi_sclk			(adc2_spi_sclk),
	.adc2_spi_io			(adc2_spi_io),
	// AD9516接口
	.ad9516_spi_cs			(ad9516_spi_cs),
	.ad9516_spi_sclk		(ad9516_spi_sclk),
	.ad9516_spi_sdio		(ad9516_spi_sdio),
	// 时钟输出
	.clk_adc1_acq			(clk_dco_20m),
	.clk_200m				(clk_dco_200m),
	.clk_dco_locked			(clk_dco_locked),
	// 数据输出
	.adc1_data_posedge		(adc_data),
	.adc1_data_raw_debug	(adc_data_raw_debug),
    .debug_startup_done       (ad_startup_done),
    .debug_clock_cfg_done     (ad_clock_cfg_done),
    .debug_adc1_cfg_done      (ad_adc1_cfg_done),
    .debug_adc2_cfg_done      (ad_adc2_cfg_done)
);

//==============================================================================
// GPS解析模块
// 功能：解析GPS NMEA报文，提取时间/位置信息，检测锁定状态
// 注意：使用clk_dco_20m作为工作时钟（与ADC同源，20MHz）
//==============================================================================
GPS_top u_GPS_top(
	// 时钟和复位
	.clk					(clk_dco_20m),
	.clk_50m				(clk_50m),
	.sys_rst_n				(rst_n_dco),
	// UART接口
	.rx						(rx),
	.tx						(tx),
	// 按键（未使用，固定为未按下状态）
	.key					(3'b111),
	// PPS信号
	.pps					(pps),
	// 状态输出
	.gps_lock_status		(gps_lock_status),
	.pps_pos_edge_sync		(pps_pos_edge_sync),
	.pps_pos_edge_extended	(pps_pos_edge_extended),
	// RMC数据输出
	.rmc_time				(rmc_time),
	.rmc_latitude			(rmc_latitude),
	.rmc_longitude			(rmc_longitude),
	.rmc_date				(rmc_date),
    .debug_ila_events         (gps_debug_events),
    .debug_ila_candidate_date (gps_debug_candidate_date),
    .debug_ila_candidate_time (gps_debug_candidate_time),
    .debug_ila_state          (gps_debug_state),
    .debug_ila_counts         (gps_debug_counts),
    .debug_ila_pps_width      (gps_debug_pps_width)
);

//==============================================================================
// 按键控制模块
// 功能：按键消抖、生成模式切换命令
// key1 → 测试模式命令(0x01)
// key2 → 触发模式命令(0x02)
//==============================================================================
key_ctrl u_key_ctrl(
	// 时钟和复位
	.clk					(clk_dco_20m),
	.rst_n					(rst_n_dco),
	// 按键输入
	.key1_n					(key1_n),
	.key2_n					(key2_n),
	// 命令输出
	.cmd_valid				(key_cmd_valid),
	.cmd_data				(key_cmd_data)
);

//==============================================================================
// 双模式核心模块
// 功能：DDR3缓存管理、测试模式/触发模式数据处理
// 注意：clk_200m使用板载晶振生成，确保DDR3初始化稳定
//==============================================================================
mode2_top u_mode2_top(
    .recover_hold_request(recovery_hold_request),
    .recover_clear_request(recovery_clear_request),
    .recover_fifo_request(recovery_fifo_request), .recover_mig_rst_n(recovery_mig_rst_n),
    .recover_hold_40m(!recovery_business_enable), .data_rst_n_40m(data_rst_n_40m),
    .recover_dco_clear_seen_40m(recovery_clear_seen[0]),
    .recover_dco_fifo_seen_40m(recovery_fifo_seen[0]),
    .recover_dco_ready_40m(recovery_ready_seen[0]), .recover_dco_run_40m(recovery_run_seen[0]),
    .recover_ui_clear_seen_40m(recovery_clear_seen[1]),
    .recover_ui_fifo_seen_40m(recovery_fifo_seen[1]),
    .recover_ui_ready_40m(recovery_ready_seen[1]), .recover_ui_run_40m(recovery_run_seen[1]),
    .recover_calib_40m(recovery_calib_ready), .recover_calib_lost_40m(recovery_calib_lost),
    .recover_overwrite_40m(recovery_overwrite), .recover_local_ready_40m(recovery_mode2_ready),
	// 系统时钟与复位
	.clk					(clk_dco_20m),
	.clk_50m				(clk_50m),
	.clk_200m				(clk_200m),				// 改用板载晶振的200MHz
	.clk_40m				(clk_40m),				// 40MHz读出时钟
	.rst_n					(rst_n_dco),
	.rst_n_40m				(rst_n_40m),
	// ADC数据接口
	.adc_data				(adc_data),
	.adc_data_raw_debug		(adc_data_raw_debug),
	// PPS接口
	.pps_pos_edge_sync		(pps_pos_edge_sync),
	// GPS timestamp interface
	.gps_lock_status		(gps_lock_status),
	.rmc_time				(rmc_time),
	.rmc_date				(rmc_date),
	// 命令接口（合并按键和以太网命令）
	.cmd_valid				(cmd_valid),
	.cmd_data				(cmd_data),
	// 数据输出接口
	.data_out				(data_out),
	.data_out_valid			(data_out_valid),
	.transfer_flag			(transfer_flag),
	.data_stream_flag		(data_stream_flag),
	// Timestamp packet request output
	.ts_req				(ts_req),
	.ts_sec_idx			(ts_sec_idx),
	.ts_work_mode		(ts_work_mode),
	.ts_gps_lock_status	(ts_gps_lock_status),
	.ts_date				(ts_date),
	.ts_time				(ts_time),
	.ts_temp_raw			(ts_temp_raw),
	.ts_temp_flags			(ts_temp_flags),
    .ts_threshold          (ts_threshold),
	.ts_send_done		(ts_send_done_40m),
    .segment_send_done            (segment_send_done_40m),
    .segment_send_done_seq        (segment_send_done_seq_40m),
    .ts_event_seq                 (ts_event_seq),
    .ts_mailbox_busy_40m           (ts_req_mailbox_busy_40m),
    .ts_mailbox_overflow_40m       (ts_req_mailbox_overflow_40m),
    .debug_ila_read_control        (mode2_debug_read_control),
    .debug_ila_read_identity       (mode2_debug_read_identity),
    .debug_ila_read_word           (mode2_debug_read_word),
    .debug_ila_read_count          (mode2_debug_read_count),
    .debug_ila_unpack_level        (mode2_debug_unpack_level),
    .debug_ila_read_sample         (mode2_debug_read_sample),
    .debug_ila_gps_events          (mode2_debug_gps_events),
    .debug_ila_time_transaction_seq(mode2_debug_time_transaction_seq),
    .debug_ila_command_state       (mode2_debug_command_state),
    .debug_ila_ui_heartbeat        (mode2_debug_ui_heartbeat),
	// DDR3物理接口
	.ddr3_dq				(ddr3_dq),
	.ddr3_dqs_n				(ddr3_dqs_n),
	.ddr3_dqs_p				(ddr3_dqs_p),
	.ddr3_addr				(ddr3_addr),
	.ddr3_ba				(ddr3_ba),
	.ddr3_ras_n				(ddr3_ras_n),
	.ddr3_cas_n				(ddr3_cas_n),
	.ddr3_we_n				(ddr3_we_n),
	.ddr3_reset_n			(ddr3_reset_n),
	.ddr3_ck_p				(ddr3_ck_p),
	.ddr3_ck_n				(ddr3_ck_n),
	.ddr3_cke				(ddr3_cke),
	.ddr3_cs_n				(ddr3_cs_n),
	.ddr3_dm				(ddr3_dm),
	.ddr3_odt				(ddr3_odt),
	// 状态输出
	.ddr_init_done			(ddr_init_done),
	.sys_running			(sys_running)
);

//==============================================================================
// 以太网顶层模块 【新增】
// 功能：RGMII接口、ARP协议、UDP协议
//==============================================================================
eth_top #(
	.BOARD_MAC				(48'h00_11_22_33_44_55),		// 开发板MAC地址
	.BOARD_IP				({8'd192,8'd168,8'd1,8'd10}),	// 开发板IP: 192.168.1.10
	.DES_MAC				(48'hff_ff_ff_ff_ff_ff),		// 目的MAC（广播）
	.DES_IP					({8'd192,8'd168,8'd1,8'd102})	// 目的IP: 192.168.1.102
) u_eth_top (
	.sys_rst_n				(rst_n_dco_req),
	// RGMII接口
	.idelay_clk				(clk_200m),					// IDELAY参考时钟（200MHz）
	.eth_rxc				(eth_rxc),
	.eth_rx_ctl				(eth_rx_ctl),
	.eth_rxd				(eth_rxd),
	.eth_txc				(eth_txc),
	.eth_tx_ctl				(eth_tx_ctl),
	.eth_txd				(eth_txd),
	.eth_rst_n				(eth_rst_n),
	// GMII时钟输出
	.gmii_rx_clk			(gmii_rx_clk),
	.gmii_tx_clk			(gmii_tx_clk),
	// UDP发送接口
	.udp_tx_start_en		(udp_tx_start_en),
	.tx_data				(udp_tx_data),
	.tx_byte_num			(udp_tx_byte_num),
	.udp_tx_done			(udp_tx_done),
	.tx_req					(udp_tx_req),
	// UDP接收接口
	.rec_pkt_done			(udp_rec_pkt_done),
	.rec_en					(udp_rec_en),
	.rec_data				(udp_rec_data),
	.rec_byte_num			(udp_rec_byte_num)
);

//==============================================================================
// 以太网命令解析模块 【新增】
// 功能：解析UDP接收的命令，输出cmd_valid和cmd_data
// 时钟域：gmii_rx_clk (125MHz)
//==============================================================================
eth_cmd_parse u_eth_cmd_parse(
	.clk					(gmii_rx_clk),
	.rst_n					(rst_n_gmii_rx),
	// UDP接收接口
	.udp_rec_pkt_done		(udp_rec_pkt_done),
	.udp_rec_en				(udp_rec_en),
	.udp_rec_data			(udp_rec_data),
	.udp_rec_byte_num		(udp_rec_byte_num),
	// 命令输出（125MHz域）
	.cmd_valid				(eth_cmd_valid_125m),
	.cmd_data				(eth_cmd_data_125m)
);

//==============================================================================
// 命令跨时钟域同步 【新增】
// 功能：将125MHz域的命令同步到20MHz域
//==============================================================================
cdc_event_mailbox #(
    .PAYLOAD_WIDTH          (24)
) u_cmd_sync (
    .src_clk                (gmii_rx_clk),
    .src_rst_n              (rst_n_gmii_rx),
    .src_send               (eth_cmd_valid_125m),
    .src_payload            (eth_cmd_data_125m),
    .src_busy               (cmd_mailbox_busy_125m),
    .src_overflow           (cmd_mailbox_overflow_125m),
    .dst_clk                (clk_dco_20m),
    .dst_rst_n              (rst_n_dco),
    .dst_valid              (eth_cmd_valid_sync),
    .dst_payload            (eth_cmd_data_sync)
);

//==============================================================================
// Timestamp request CDC and UDP TX payload arbitration
//==============================================================================
// 高32位用于内部任务身份和ILA，原79位载荷及线上协议保持原位。
assign debug_ts_seq_125m = ts_sync_data_125m[110:79];
always @(posedge gmii_tx_clk or negedge rst_n_gmii_tx) begin
    if (!rst_n_gmii_tx) debug_data_seq_125m <= 32'd0;
    else if (recovery_clear_tx) debug_data_seq_125m <= 32'd0;
    else if (ts_pkt_done) debug_data_seq_125m <= ts_done_seq_125m;
end

assign ts_threshold_125m       = ts_sync_data_125m[126:111];
assign ts_work_mode_125m       = ts_sync_data_125m[78:77];
assign ts_gps_lock_status_125m = ts_sync_data_125m[76:75];
assign ts_sec_idx_125m         = ts_sync_data_125m[74:72];
assign ts_date_125m            = ts_sync_data_125m[71:48];
assign ts_time_125m            = ts_sync_data_125m[47:24];
assign ts_temp_raw_125m        = ts_sync_data_125m[23:8];
assign ts_temp_flags_125m      = ts_sync_data_125m[7:0];

cdc_event_mailbox #(
    .PAYLOAD_WIDTH          (127)
) u_ts_req_sync (
    .src_clk                (clk_40m),
    .src_rst_n              (data_rst_n_40m),
    .src_send               (ts_req),
    .src_payload            ({ts_threshold, ts_event_seq, ts_work_mode, ts_gps_lock_status, ts_sec_idx, ts_date, ts_time, ts_temp_raw, ts_temp_flags}),
    .src_busy               (ts_req_mailbox_busy_40m),
    .src_overflow           (ts_req_mailbox_overflow_40m),
    .dst_clk                (gmii_tx_clk),
    .dst_rst_n              (data_rst_n_tx),
    .dst_valid              (ts_req_125m),
    .dst_payload            (ts_sync_data_125m)
);

cdc_event_mailbox #(
    .PAYLOAD_WIDTH          (1)
) u_ts_done_sync (
    .src_clk                (gmii_tx_clk),
    .src_rst_n              (data_rst_n_tx),
    // Release DDR reads only after both FIFO domains confirm this segment reset.
    .src_send               (data_segment_ready),
    .src_payload            (1'b1),
    .src_busy               (ts_done_mailbox_busy_125m),
    .src_overflow           (ts_done_mailbox_overflow_125m),
    .dst_clk                (clk_40m),
    .dst_rst_n              (data_rst_n_40m),
    .dst_valid              (ts_send_done_40m),
    .dst_payload            (ts_send_done_data_40m)
);

always @(posedge gmii_tx_clk or negedge rst_n_gmii_tx) begin
	if (!rst_n_gmii_tx) begin
		data_tx_enable_125m <= 1'b0;
		data_sec_idx_125m   <= 3'd0;
	end
    else if (recovery_hold_tx || recovery_clear_tx) begin
        data_tx_enable_125m <= 1'b0;
        if (recovery_clear_tx) data_sec_idx_125m <= 3'd0;
    end
	else if (ts_req_125m) begin
		data_tx_enable_125m <= 1'b0;
		data_sec_idx_125m   <= ts_sec_idx_125m;
	end
	else if (ts_pkt_done) begin
		data_tx_enable_125m <= 1'b1;
		data_sec_idx_125m   <= ts_done_sec_idx_125m;
	end
	else if (data_segment_done) begin
		data_tx_enable_125m <= 1'b0;
	end
end

eth_ts_pkt u_eth_ts_pkt(
    .tx_hold(recovery_hold_tx), .tx_clear(recovery_clear_tx),
    .udp_tx_start_accept(ts_start_accept), .tx_quiet(recovery_ts_quiet),
    .ts_sec_idx(ts_sec_idx_125m), .ts_seq(debug_ts_seq_125m),
    .ts_done_sec_idx(ts_done_sec_idx_125m), .ts_done_seq(ts_done_seq_125m),
	.clk					(gmii_tx_clk),
	.rst_n					(rst_n_gmii_tx),
	.ts_req					(ts_req_125m),
	.ts_work_mode			(ts_work_mode_125m),
	.ts_gps_lock_status	(ts_gps_lock_status_125m),
	.ts_date				(ts_date_125m),
	.ts_time				(ts_time_125m),
	.ts_temp_raw			(ts_temp_raw_125m),
	.ts_temp_flags		(ts_temp_flags_125m),
    .ts_threshold          (ts_threshold_125m),
	.udp_tx_req			(ts_udp_tx_req),
	.udp_tx_done			(ts_udp_tx_done),
	.udp_tx_start_en		(ts_udp_tx_start_en),
	.udp_tx_data			(ts_udp_tx_data),
	.udp_tx_byte_num		(ts_udp_tx_byte_num),
	.ts_busy				(ts_pkt_busy),
	.ts_done				(ts_pkt_done),
    .debug_byte_idx (debug_ts_byte_idx)
);

//==============================================================================
// Ethernet ADC data packet module
//==============================================================================
eth_data_pkt u_eth_data_pkt(
    .tx_hold(recovery_hold_tx), .tx_clear(recovery_clear_tx),
    .udp_tx_start_accept(data_start_accept), .tx_quiet(recovery_data_quiet),
    .wr_hold(!recovery_business_enable), .wr_clear(!data_rst_n_40m),
    .wr_fifo_reset(recovery_fifo_request),
	.rst_n					(rst_n_gmii_tx),
	// data input interface (40MHz)
	.wr_clk					(clk_40m),
	.data_out				(data_out),
	.data_out_valid			(data_out_valid),
	// Ethernet transmit interface (125MHz)
	.eth_tx_clk				(gmii_tx_clk),
	.tx_enable				(data_tx_enable_125m),
	.segment_start			(ts_pkt_done),
    .segment_ready            (data_segment_ready),
    .debug_segment_barrier    (eth_debug_segment_barrier),
    .segment_seq              (ts_done_seq_125m),
    .segment_done_seq         (data_segment_done_seq_125m),
	.sec_idx				(data_sec_idx_125m),
	.udp_tx_req				(data_udp_tx_req),
	.udp_tx_done			(data_udp_tx_done),
	.udp_tx_start_en		(data_udp_tx_start_en),
	.udp_tx_data			(data_udp_tx_data),
	.udp_tx_byte_num		(data_udp_tx_byte_num),
	.tx_busy				(data_tx_busy),
	.segment_done			(data_segment_done),
	.debug_pkt_idx			(data_pkt_idx),
	.debug_fifo_rdusedw	(data_fifo_rdusedw),
	.debug_fifo_full		(data_fifo_full),
	.debug_fifo_drop_sticky(data_fifo_drop_sticky),
    .debug_fifo_wr_accept   (eth_debug_fifo_wr_accept),
    .debug_fifo_drop_reset  (eth_debug_fifo_drop_reset),
    .debug_fifo_wr_rst_busy (eth_debug_fifo_wr_rst_busy),
    .debug_fifo_rd_rst_busy (eth_debug_fifo_rd_rst_busy),
    .debug_payload_underflow(eth_debug_payload_underflow),
    .debug_tx_byte_idx      (eth_debug_tx_byte_idx),
    .debug_fifo_dout        (eth_debug_fifo_dout),
    .debug_fifo_rd_en       (eth_debug_fifo_rd_en),
    .debug_fifo_empty       (eth_debug_fifo_empty),
    .debug_fifo_rst_cnt     (eth_debug_fifo_rst_cnt)
);

// Completion is accepted in the queue's native40MHz domain with full identity.
// Source busy/overflow remain visible in the TX-domain diagnostic capture state.
cdc_event_mailbox #(.PAYLOAD_WIDTH(32)) u_segment_done_sync (
    .src_clk(gmii_tx_clk), .src_rst_n(data_rst_n_tx),
    .src_send(data_segment_done), .src_payload(data_segment_done_seq_125m),
    .src_busy(segment_done_mailbox_busy_125m),
    .src_overflow(segment_done_mailbox_overflow_125m),
    .dst_clk(clk_40m), .dst_rst_n(data_rst_n_40m),
    .dst_valid(segment_send_done_40m), .dst_payload(segment_send_done_seq_40m)
);

eth_tx_arbiter u_eth_tx_arbiter(
    .tx_hold(recovery_hold_tx), .tx_clear(recovery_clear_tx),
    .ts_start_accept(ts_start_accept), .data_start_accept(data_start_accept),
    .tx_quiet(recovery_arbiter_quiet),
	.clk					(gmii_tx_clk),
	.rst_n					(rst_n_gmii_tx),
	.ts_udp_tx_start_en	(ts_udp_tx_start_en),
	.ts_udp_tx_data		(ts_udp_tx_data),
	.ts_udp_tx_byte_num	(ts_udp_tx_byte_num),
	.ts_udp_tx_req		(ts_udp_tx_req),
	.ts_udp_tx_done		(ts_udp_tx_done),
	.data_udp_tx_start_en	(data_udp_tx_start_en),
	.data_udp_tx_data		(data_udp_tx_data),
	.data_udp_tx_byte_num	(data_udp_tx_byte_num),
	.data_udp_tx_req		(data_udp_tx_req),
	.data_udp_tx_done	(data_udp_tx_done),
	.udp_tx_start_en		(udp_tx_start_en),
	.udp_tx_data			(udp_tx_data),
	.udp_tx_byte_num		(udp_tx_byte_num),
	.udp_tx_req			(udp_tx_req),
	.udp_tx_done			(udp_tx_done),
	.debug_tx_sel			(debug_udp_tx_sel),
    .debug_ts_pending (debug_arbiter_ts_pending)
);

//==============================================================================
// LED状态指示模块
// 功能：统一管理所有LED状态指示
//==============================================================================
led_top u_led_top(
	// 时钟和复位
	.clk					(clk_dco_20m),
	.rst_n					(rst_n_dco),
	// 状态输入
	.gps_lock_status		(gps_lock_status),
	.ddr_init_status		(ddr_init_status),
	.sys_run_status			(sys_run_status),
	.transfer_status		(transfer_status),
	// LED输出
	.led_gps_lock			(led_gps_lock),
	.led_ddr_init			(led_ddr_init),
	.led_sys_run			(led_sys_run),
	.led_transfer			(led_transfer)
);

//==============================================================================
// 生产固定ILA：全部探针在原生域采样；仅核实例依赖综合环境。
//==============================================================================
wire debug_ila_sys_event, debug_ila_gps_event, debug_ila_read_event;
wire debug_ila_udp_event, debug_ila_cmd_rx_event;
wire [15:0] debug_ila_sys_ms, debug_ila_gps_ms, debug_ila_read_ms;
wire [15:0] debug_ila_udp_ms, debug_ila_cmd_rx_ms;

// 以下向量是多个独立单bit状态的同步，不承载需保持原子的多位事务。
(* ASYNC_REG = "TRUE" *) reg [2:0] debug_sys_reset_pipe;
(* ASYNC_REG = "TRUE" *) reg [2:0] debug_reset_s1, debug_reset_s2;
(* ASYNC_REG = "TRUE" *) reg [2:0] debug_cfg_s1, debug_cfg_s2;
wire debug_sys_rst_sync = debug_sys_reset_pipe[2];
wire debug_rst40_sync = debug_reset_s2[0];
wire debug_rsttx_sync = debug_reset_s2[1];
wire debug_rstrx_sync = debug_reset_s2[2];
wire debug_clock_cfg_done_sync = debug_cfg_s2[0];
wire debug_adc1_cfg_done_sync = debug_cfg_s2[1];
wire debug_adc2_cfg_done_sync = debug_cfg_s2[2];
always @(posedge clk_50m or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        debug_sys_reset_pipe <= 3'b000;
        debug_reset_s1 <= 3'b000; debug_reset_s2 <= 3'b000;
        debug_cfg_s1 <= 3'b000; debug_cfg_s2 <= 3'b000;
    end else begin
        debug_sys_reset_pipe <= {debug_sys_reset_pipe[1:0], 1'b1};
        debug_reset_s1 <= {rst_n_gmii_rx, rst_n_gmii_tx, rst_n_40m};
        debug_reset_s2 <= debug_reset_s1;
        debug_cfg_s1 <= {ad_adc2_cfg_done, ad_adc1_cfg_done, ad_clock_cfg_done};
        debug_cfg_s2 <= debug_cfg_s1;
    end
end

// 每个原生域ms[0]每1ms翻转；独立50MHz域在10ms无变化后判静默。
wire [4:0] debug_clock_toggle = {debug_ila_cmd_rx_ms[0], debug_ila_udp_ms[0],
                               mode2_debug_ui_heartbeat, debug_ila_read_ms[0],
                               debug_ila_gps_ms[0]};
(* ASYNC_REG = "TRUE" *) reg [4:0] debug_clock_s1, debug_clock_s2;
reg [4:0] debug_clock_last;
reg [4:0] debug_clk_alive;
reg [18:0] debug_clock_age [0:4];
integer debug_clock_i;
always @(posedge clk_50m or negedge debug_sys_rst_sync) begin
    if (!debug_sys_rst_sync) begin
        debug_clock_s1 <= 5'd0; debug_clock_s2 <= 5'd0;
        debug_clock_last <= 5'd0; debug_clk_alive <= 5'd0;
        for (debug_clock_i=0; debug_clock_i<5; debug_clock_i=debug_clock_i+1)
            debug_clock_age[debug_clock_i] <= 19'd500000;
    end else begin
        debug_clock_s1 <= debug_clock_toggle;
        debug_clock_s2 <= debug_clock_s1;
        debug_clock_last <= debug_clock_s2;
        for (debug_clock_i=0; debug_clock_i<5; debug_clock_i=debug_clock_i+1) begin
            if (debug_clock_s2[debug_clock_i] != debug_clock_last[debug_clock_i]) begin
                debug_clock_age[debug_clock_i] <= 19'd0;
                debug_clk_alive[debug_clock_i] <= 1'b1;
            end else if (debug_clock_age[debug_clock_i] < 19'd500000) begin
                debug_clock_age[debug_clock_i] <= debug_clock_age[debug_clock_i] + 1'b1;
            end else begin
                debug_clk_alive[debug_clock_i] <= 1'b0;
            end
        end
    end
end

wire [18:0] debug_sys_state = {debug_clk_alive, debug_cfg_s2, debug_rstrx_sync,
    debug_rsttx_sync, debug_rst40_sync, debug_sys_rst_sync, ad_startup_done,
    transfer_flag_s3, sys_running_s3, ddr_init_done_s3, rst_n_dco_s3,
    clk_dco_locked_s3, locked_s3};
reg [18:0] debug_sys_state_d;
always @(posedge clk_50m or negedge debug_sys_rst_sync) begin
    if (!debug_sys_rst_sync) debug_sys_state_d <= 19'd0;
    else debug_sys_state_d <= debug_sys_state;
end
ila_capture_control #(.CLK_HZ(50000000)) u_capture_sys (
    .clk(clk_50m), .rst_n(debug_sys_rst_sync),
    .event_in(debug_sys_state != debug_sys_state_d), .wait_active(1'b1),
    .capture_event(debug_ila_sys_event), .timestamp_ms(debug_ila_sys_ms)
);

// gps_events高位是tagged-trigger源通道状态，已不使用旧mask mailbox。
wire [9:0] debug_gps_events = {mode2_debug_gps_events, cmd_valid, gps_debug_events};
wire [15:0] debug_gps_state = {mode2_debug_gps_events[4:1], gps_debug_state};
reg [15:0] debug_gps_state_d;
always @(posedge clk_dco_20m or negedge rst_n_dco) begin
    if (!rst_n_dco) debug_gps_state_d <= 16'd0;
    else debug_gps_state_d <= debug_gps_state;
end
ila_capture_control #(.CLK_HZ(20000000)) u_capture_gps (
    .clk(clk_dco_20m), .rst_n(rst_n_dco),
    .event_in((|debug_gps_events[5:1]) || debug_gps_state != debug_gps_state_d),
    .wait_active(1'b1), .capture_event(debug_ila_gps_event),
    .timestamp_ms(debug_ila_gps_ms)
);

wire [12:0] debug_read_state = {eth_debug_fifo_wr_rst_busy, eth_debug_fifo_drop_reset,
    mode2_debug_read_control[5] && !mode2_debug_read_control[6], data_fifo_drop_sticky, data_fifo_full,
    mode2_debug_read_control[15:13], mode2_debug_read_control[9],
    mode2_debug_read_control[4:1]};
reg [12:0] debug_read_state_d;
always @(posedge clk_40m or negedge rst_n_40m) begin
    if (!rst_n_40m) debug_read_state_d <= 13'd0;
    else debug_read_state_d <= debug_read_state;
end
ila_capture_control #(.CLK_HZ(40000000)) u_capture_read (
    .clk(clk_40m), .rst_n(rst_n_40m),
    .event_in(mode2_debug_read_control[3] || mode2_debug_read_control[14] ||
              debug_read_state != debug_read_state_d ||
              (mode2_debug_read_control[12] && mode2_debug_read_sample[31:16] == 16'd0)),
    .wait_active(mode2_debug_read_control[4]),
    .capture_event(debug_ila_read_event), .timestamp_ms(debug_ila_read_ms)
);

wire [11:0] debug_udp_state = {eth_debug_segment_barrier, segment_done_mailbox_overflow_125m, segment_done_mailbox_busy_125m, eth_debug_fifo_rd_rst_busy, eth_debug_payload_underflow,
                             ts_done_mailbox_overflow_125m, debug_udp_tx_sel};
reg [11:0] debug_udp_state_d;
always @(posedge gmii_tx_clk or negedge rst_n_gmii_tx) begin
    if (!rst_n_gmii_tx) debug_udp_state_d <= 12'd0;
    else debug_udp_state_d <= debug_udp_state;
end
ila_capture_control #(.CLK_HZ(125000000)) u_capture_udp (
    .clk(gmii_tx_clk), .rst_n(rst_n_gmii_tx),
    .event_in(ts_req_125m || ts_pkt_done || data_segment_ready || data_udp_tx_start_en ||
              data_udp_tx_done || data_segment_done || debug_udp_state != debug_udp_state_d),
    .wait_active(ts_pkt_busy || data_tx_enable_125m),
    .capture_event(debug_ila_udp_event), .timestamp_ms(debug_ila_udp_ms)
);

reg cmd_mailbox_overflow_d;
always @(posedge gmii_rx_clk or negedge rst_n_gmii_rx) begin
    if (!rst_n_gmii_rx) cmd_mailbox_overflow_d <= 1'b0;
    else cmd_mailbox_overflow_d <= cmd_mailbox_overflow_125m;
end
ila_capture_control #(.CLK_HZ(125000000)) u_capture_cmd_rx (
    .clk(gmii_rx_clk), .rst_n(rst_n_gmii_rx),
    .event_in(eth_cmd_valid_125m || udp_rec_pkt_done ||
              cmd_mailbox_overflow_125m != cmd_mailbox_overflow_d),
    .wait_active(cmd_mailbox_busy_125m),
    .capture_event(debug_ila_cmd_rx_event), .timestamp_ms(debug_ila_cmd_rx_ms)
);

// Recovery diagnostics run at 40 MHz even while data epochs are held in reset.
wire [24:0] debug_recovery_control = {
    recovery_business_enable,recovery_mig_rst_n,recovery_fifo_request,
    recovery_clear_request,recovery_hold_request,recovery_tx_quiet_40m,
    recovery_calib_ready,recovery_calib_lost,recovery_overwrite,
    recovery_local_ready_40m,recovery_run_seen,recovery_ready_seen,
    recovery_fifo_seen,recovery_clear_seen,recovery_state};
reg [24:0] debug_recovery_control_prev;
reg [1:0] debug_recovery_reason_prev;
wire debug_recovery_event;
wire [15:0] debug_recovery_ms;
always @(posedge clk_40m or negedge rst_n_40m) begin
    if (!rst_n_40m) begin
        debug_recovery_control_prev <= 25'd0;
        debug_recovery_reason_prev <= 2'd0;
    end else begin
        debug_recovery_control_prev <= debug_recovery_control;
        debug_recovery_reason_prev <= recovery_fault_reason;
    end
end
ila_capture_control #(.CLK_HZ(40000000)) u_capture_recovery (
    .clk(clk_40m),.rst_n(rst_n_40m),
    .event_in(debug_recovery_control != debug_recovery_control_prev ||
              recovery_fault_reason != debug_recovery_reason_prev),
    .wait_active(!recovery_business_enable),
    .capture_event(debug_recovery_event),.timestamp_ms(debug_recovery_ms)
);

`ifdef SYNTHESIS
ila_prod_sys_v1 u_ila_prod_sys_v1 (
    .clk(clk_50m),
    .probe0({debug_rstrx_sync, debug_rsttx_sync, debug_rst40_sync, debug_sys_rst_sync, ad_startup_done, transfer_flag_s3, sys_running_s3, ddr_init_done_s3, rst_n_dco_s3, clk_dco_locked_s3, locked_s3}),
    .probe1({debug_adc2_cfg_done_sync, debug_adc1_cfg_done_sync, debug_clock_cfg_done_sync}),
    .probe2(debug_clk_alive),
    .probe3({debug_ila_sys_ms, debug_ila_sys_event})
);

ila_prod_gps_v1 u_ila_prod_gps_v1 (
    .clk(clk_dco_20m),
    .probe0({mode2_debug_gps_events[4], mode2_debug_gps_events[3], mode2_debug_gps_events[2], mode2_debug_gps_events[1], mode2_debug_gps_events[0], cmd_valid, gps_debug_events[3], gps_debug_events[2], gps_debug_events[1], gps_debug_events[0]}),
    .probe1(gps_debug_candidate_date),
    .probe2(gps_debug_candidate_time),
    .probe3(rmc_date),
    .probe4(rmc_time),
    .probe5({gps_debug_state[11:8], gps_debug_state[7:6], gps_debug_state[5:4], gps_debug_state[3:2], gps_debug_state[1:0]}),
    .probe6({gps_debug_counts[31:16], gps_debug_counts[15:0]}),
    .probe7(gps_debug_pps_width),
    .probe8(mode2_debug_time_transaction_seq),
    .probe9({mode2_debug_command_state[25:18], mode2_debug_command_state[17:2], mode2_debug_command_state[1:0], cmd_data}),
    .probe10({debug_ila_gps_ms, debug_ila_gps_event})
);

ila_prod_read_v1 u_ila_prod_read_v1 (
    .clk(clk_40m),
    .probe0({eth_debug_fifo_wr_rst_busy, eth_debug_fifo_drop_reset, eth_debug_fifo_wr_accept, data_fifo_drop_sticky, data_fifo_full, mode2_debug_read_control[15], mode2_debug_read_control[14], mode2_debug_read_control[13], mode2_debug_read_control[12], mode2_debug_read_control[11], mode2_debug_read_control[10], mode2_debug_read_control[9], mode2_debug_read_control[8], mode2_debug_read_control[7], mode2_debug_read_control[6], mode2_debug_read_control[5], mode2_debug_read_control[4], mode2_debug_read_control[3], mode2_debug_read_control[2], mode2_debug_read_control[1], mode2_debug_read_control[0]}),
    .probe1({mode2_debug_read_identity[34:3], mode2_debug_read_identity[2:0]}),
    .probe2(mode2_debug_read_word),
    .probe3({mode2_debug_read_count[36:24], mode2_debug_read_count[23:0]}),
    .probe4({mode2_debug_unpack_level[15:9], mode2_debug_unpack_level[8:0]}),
    .probe5({mode2_debug_read_sample[47:16], mode2_debug_read_sample[15:0]}),
    .probe6({debug_ila_read_ms, debug_ila_read_event})
);

ila_prod_udp_v1 u_ila_prod_udp_v1 (
    .clk(gmii_tx_clk),
    .probe0({eth_debug_payload_underflow, eth_debug_fifo_rd_rst_busy, debug_arbiter_ts_pending, ts_done_mailbox_overflow_125m, ts_done_mailbox_busy_125m, eth_debug_fifo_empty, eth_debug_fifo_rd_en, udp_tx_done, udp_tx_req, udp_tx_start_en, data_udp_tx_done, data_udp_tx_req, data_udp_tx_start_en, data_segment_done, data_tx_busy, data_tx_enable_125m, ts_udp_tx_done, ts_udp_tx_req, ts_udp_tx_start_en, ts_pkt_done, ts_pkt_busy, ts_req_125m, rst_n_gmii_tx}),
    .probe1(debug_ts_seq_125m),
    .probe2(debug_data_seq_125m),
    .probe3({ts_gps_lock_status_125m, ts_work_mode_125m, ts_sec_idx_125m}),
    .probe4({data_pkt_idx, data_sec_idx_125m}),
    .probe5({eth_debug_fifo_rst_cnt, data_fifo_rdusedw}),
    .probe6({udp_tx_byte_num, debug_udp_tx_sel}),
    .probe7({debug_ts_byte_idx, udp_tx_data, eth_debug_fifo_dout, eth_debug_tx_byte_idx}),
    .probe8({debug_ila_udp_ms, debug_ila_udp_event}),
    .probe9({segment_done_mailbox_overflow_125m, segment_done_mailbox_busy_125m, data_segment_done_seq_125m}),
    .probe10({data_segment_ready, eth_debug_segment_barrier[4], eth_debug_segment_barrier[3], eth_debug_segment_barrier[1:0]})
);

ila_prod_cmd_rx_v1 u_ila_prod_cmd_rx_v1 (
    .clk(gmii_rx_clk),
    .probe0({udp_rec_pkt_done, cmd_mailbox_overflow_125m, cmd_mailbox_busy_125m, eth_cmd_valid_125m, rst_n_gmii_rx}),
    .probe1({udp_rec_byte_num, eth_cmd_data_125m}),
    .probe2({debug_ila_cmd_rx_ms, debug_ila_cmd_rx_event})
);

ila_prod_recovery_v1 u_ila_prod_recovery_v1 (
    .clk(clk_40m), .probe0(debug_recovery_control),
    .probe1(recovery_count), .probe2(recovery_wait_age),
    .probe3(recovery_fault_reason), .probe4({debug_recovery_ms,debug_recovery_event})
);

`endif

endmodule
//==============================================================================
// Reset synchronizer: asynchronous assert, synchronous release
//==============================================================================
module reset_sync(
    input   wire    clk,
    input   wire    arst_n,
    output  wire    srst_n
);

(* ASYNC_REG = "TRUE" *) reg [2:0] rst_pipe;

always @(posedge clk or negedge arst_n) begin
    if (!arst_n) begin
        rst_pipe <= 3'b000;
    end
    else begin
        rst_pipe <= {rst_pipe[1:0], 1'b1};
    end
end

assign srst_n = rst_pipe[2];

endmodule
