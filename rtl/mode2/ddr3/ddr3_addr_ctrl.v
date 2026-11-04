`timescale 1ns / 1ps
//******************************************************************************
//  文件名:     ddr3_addr_ctrl.v
//  模块名:     ddr3_addr_ctrl
//  功能描述:   DDR3地址控制模块 - 运行在ui_clk域
//              
//  主要功能:
//  1. PPS信号跨时钟域同步 (clk_50m → ui_clk)
//  2. 分区索引(sec_idx)管理
//  3. 写地址范围动态生成
//  4. 未来扩展: 读地址范围、trig_mask管理
//
//  设计理念:
//  - 所有DDR3地址相关信号统一在ui_clk域生成
//  - 避免跨时钟域的多bit信号同步问题
//  - 与ddr3_rw模块在同一时钟域，无需额外打拍
//------------------------------------------------------------------------------
//  时钟域说明:
//  
//  输入信号:
//  - pps_pulse: 来自clk_50m域，需要同步到ui_clk域
//  - init_calib_complete: ui_clk域信号，无需同步
//  
//  输出信号:
//  - 所有输出均为ui_clk域同步信号
//  - 可直接被ddr3_rw使用，无需打拍
//------------------------------------------------------------------------------

//******************************************************************************

module ddr3_addr_ctrl(
    //==========================================================================
    // 时钟与复位
    //==========================================================================
    input   wire                ui_clk,             // MIG用户时钟 (~200MHz)
    input   wire                ui_clk_sync_rst,    // MIG同步复位 (高有效)
    
    //==========================================================================
    // 跨时钟域输入 (来自clk_50m域)
    //==========================================================================
    input   wire                pps_pulse_async,    // PPS脉冲 (clk_50m域)
                                                    // 需要同步到ui_clk域
    
    //==========================================================================
    // ui_clk域输入
    //==========================================================================
    input   wire                init_calib_complete,// DDR3初始化完成 (ui_clk域)

    // Tagged boundary transaction (ui_clk域)
    input   wire                boundary_req_valid,
    input   wire                wr_boundary_commit,
    
    //==========================================================================
    // 读分区索引输入 (来自read_ctrl，需同步)
    //==========================================================================
    input   wire    [2:0]       rd_sec_idx_async,   // mailbox bundled payload (ui_clk域)
    input   wire                rd_sec_idx_valid,   // mailbox destination valid pulse (ui_clk域)
    //==========================================================================
    // 写地址输出 (ui_clk域同步信号)
    //==========================================================================
    output  reg     [28:0]      app_addr_wr_min,    // 写起始地址
    output  reg     [28:0]      app_addr_wr_max,    // 写结束地址
    output  reg     [28:0]      pending_wr_base,
    output  reg     [28:0]      pending_wr_max,
    output  reg                 pending_wr_base_valid,
    output  reg     [2:0]       boundary_completed_idx,
    
    //==========================================================================
    // 读地址输出 (ui_clk域同步信号，Step5启用)
    //==========================================================================
    output  reg     [28:0]      app_addr_rd_min,    // 读起始地址
    output  reg     [28:0]      app_addr_rd_max,    // 读结束地址
    
    //==========================================================================
    // 状态输出
    //==========================================================================
    output  reg     [2:0]       sec_idx,            // 当前分区索引 (0~3)
    output  reg     [2:0]       sec_idx_prev,       // 上一个分区索引 (PPS前的值)
    output  wire                pps_pulse_ui        // 同步后的PPS脉冲 (ui_clk域)
                                                    // 供外部模块使用 (如ILA观测)
);

//==============================================================================
// 参数定义
//==============================================================================

//------------------------------------------------------------------------------
// 分区地址参数
// 
// 地址单位: 64bit (8字节)
// 每分区大小: 64MB = 0x0400_0000 字节 = 0x0080_0000 个64bit地址
// 
// 注意: MIG地址是字节地址右移3位 (因为64bit=8字节)
// 实际字节地址 = app_addr × 8
//------------------------------------------------------------------------------
localparam [28:0] SEC0_BASE = 29'h0000_0000;
localparam [28:0] SEC1_BASE = 29'h0080_0000;
localparam [28:0] SEC2_BASE = 29'h0100_0000;
localparam [28:0] SEC3_BASE = 29'h0180_0000;
localparam [28:0] SEC4_BASE = 29'h0200_0000;
localparam [28:0] SEC5_BASE = 29'h0280_0000;
localparam [28:0] SEC6_BASE = 29'h0300_0000;
localparam [28:0] SEC7_BASE = 29'h0380_0000;
localparam [28:0] SEC_SIZE  = 29'h0080_0000;    // 64MB = 8M 64bit addresses
//------------------------------------------------------------------------------
// 读取数据量参数
// 
// 每秒采集: 20,000,000 个16bit = 40,000,000 字节
// 地址单位: 64bit (8字节)
// 因此读取地址跨度(64bit地址数) = 40,000,000 / 8 = 5,000,000
// 注意：本工程内部 app_addr_* 统一使用"64bit对齐地址"口径（不是字节地址），
//       因此这里不要再做"×8"的换算。
//------------------------------------------------------------------------------
localparam [28:0] RD_DATA_SIZE = 29'd5_000_000; // 读取地址跨度(单位: 64bit地址)

//==============================================================================
// 内部信号定义
//==============================================================================

// 复位信号 (低有效，与其他模块一致)
wire rst_n = ~ui_clk_sync_rst;

//------------------------------------------------------------------------------
// PPS跨时钟域同步寄存器
// 
// 同步方法: 2-FF同步器 + 边沿检测
// 
// 为什么需要边沿检测?
// - pps_pulse在DCO 20MHz域是单周期脉冲 (50ns宽)
// - ui_clk约200MHz，周期约5ns
// - 同步后的电平信号会持续多个ui_clk周期
// - 需要检测上升沿，确保只触发一次
//------------------------------------------------------------------------------
(* ASYNC_REG = "TRUE" *) reg pps_sync_d0;    // 第一级同步 (可能亚稳态)
(* ASYNC_REG = "TRUE" *) reg pps_sync_d1;    // 第二级同步 (稳定)
reg pps_sync_d2;    // 第三级 (用于边沿检测)

//------------------------------------------------------------------------------
// 分区基地址
//------------------------------------------------------------------------------
reg [2:0]  pending_sec_idx;
reg [28:0] rd_sec_base_addr;    // 读分区基地址

function [28:0] write_partition_base;
    input [2:0] index_value;
    begin
        case (index_value)
            3'd0: write_partition_base = SEC0_BASE;
            3'd1: write_partition_base = SEC1_BASE;
            3'd2: write_partition_base = SEC2_BASE;
            3'd3: write_partition_base = SEC3_BASE;
            3'd4: write_partition_base = SEC4_BASE;
            3'd5: write_partition_base = SEC5_BASE;
            3'd6: write_partition_base = SEC6_BASE;
            3'd7: write_partition_base = SEC7_BASE;
            default: write_partition_base = SEC0_BASE;
        endcase
    end
endfunction

//------------------------------------------------------------------------------
// 读分区索引同步寄存器 (clk_50m → ui_clk)
//------------------------------------------------------------------------------
//==============================================================================
// PPS跨时钟域同步
// 
// 工作流程:
// 1. DCO 20MHz域: pps_pulse拉高1个周期 (50ns)
// 2. ui_clk第1个上升沿: pps_sync_d0采样 (可能亚稳态)
// 3. ui_clk第2个上升沿: pps_sync_d1稳定
// 4. ui_clk第3个上升沿: pps_sync_d2用于边沿检测
// 5. 边沿检测: pps_pulse_ui = d1 & ~d2 (上升沿)
// 
// 延迟分析:
// - 同步延迟: 2~3个ui_clk周期 ≈ 10~15ns
// - 对于1Hz的PPS信号，这个延迟完全可以接受
//==============================================================================
always @(posedge ui_clk or negedge rst_n) begin
    if (!rst_n) begin
        pps_sync_d0 <= 1'b0;
        pps_sync_d1 <= 1'b0;
        pps_sync_d2 <= 1'b0;
    end
    else begin
        pps_sync_d0 <= pps_pulse_async;     // 第一级：捕获异步信号
        pps_sync_d1 <= pps_sync_d0;         // 第二级：稳定输出
        pps_sync_d2 <= pps_sync_d1;         // 第三级：边沿检测用
    end
end

// 边沿检测：产生单周期脉冲
assign pps_pulse_ui = pps_sync_d1 & ~pps_sync_d2;

//==============================================================================
// 分区索引管理
// 
// 工作逻辑:
// - DDR3初始化完成后，每个PPS脉冲切换到下一个分区
// - sec_idx: 0 → 1 → 2 → 3 → 0 循环
// - sec_idx_prev: 保存PPS到来时的旧值（即上一秒写入的分区）
// 
// 时序说明:
// - sec_idx在pps_pulse_ui的下一个ui_clk周期更新
// - sec_idx_prev同时保存旧的sec_idx值
//==============================================================================
always @(posedge ui_clk or negedge rst_n) begin
    if (!rst_n) begin
        sec_idx                 <= 3'd0;
        sec_idx_prev            <= 3'd0;
        pending_sec_idx         <= 3'd0;
        boundary_completed_idx  <= 3'd0;
        app_addr_wr_min         <= SEC0_BASE;
        app_addr_wr_max         <= SEC0_BASE + SEC_SIZE;
        pending_wr_base         <= SEC0_BASE;
        pending_wr_max          <= SEC0_BASE + SEC_SIZE;
        pending_wr_base_valid   <= 1'b0;
    end else if (!init_calib_complete) begin
        // MIG重新初始化时建立新的分区和事件epoch。
        sec_idx                 <= 3'd0;
        sec_idx_prev            <= 3'd0;
        pending_sec_idx         <= 3'd0;
        boundary_completed_idx  <= 3'd0;
        app_addr_wr_min         <= SEC0_BASE;
        app_addr_wr_max         <= SEC0_BASE + SEC_SIZE;
        pending_wr_base         <= SEC0_BASE;
        pending_wr_max          <= SEC0_BASE + SEC_SIZE;
        pending_wr_base_valid   <= 1'b0;
    end else begin
        // raw PPS只经pps_pulse_ui快速停写；tagged request才锁存旧索引和
        // 下一分区地址，active index/base在commit前保持不变。
        if (boundary_req_valid && !pending_wr_base_valid) begin
            boundary_completed_idx <= sec_idx;
            pending_sec_idx        <= sec_idx + 1'b1;
            pending_wr_base        <= write_partition_base(sec_idx + 1'b1);
            pending_wr_max         <= write_partition_base(sec_idx + 1'b1) +
                                      SEC_SIZE;
            pending_wr_base_valid  <= 1'b1;
        end

        if (wr_boundary_commit && pending_wr_base_valid) begin
            sec_idx               <= pending_sec_idx;
            sec_idx_prev          <= boundary_completed_idx;
            app_addr_wr_min       <= pending_wr_base;
            app_addr_wr_max       <= pending_wr_max;
            pending_wr_base_valid <= 1'b0;
        end
    end
end

//==============================================================================
// 读分区索引跨时钟域同步 (clk_50m → ui_clk)
// 
// 注意: rd_sec_idx 是2bit信号，普通2-FF同步器对多bit不完全安全
// 但由于:
// 1. rd_sec_idx 只在PPS时刻变化（1Hz）
// 2. 变化后会保持稳定至少1秒
// 3. 同步延迟只有几个ui_clk周期（约10-15ns），远小于信号稳定时间
// 所以这里使用简单的2-FF同步是安全的
//------------------------------------------------------------------------------

//==============================================================================
// 读分区基地址生成
//==============================================================================
always @(posedge ui_clk or negedge rst_n) begin
    if (!rst_n) begin
        rd_sec_base_addr <= SEC0_BASE;
    end
    else if (rd_sec_idx_valid) begin
        // The payload is consumed only with the destination mailbox valid
        // pulse; no independent bit-wise synchronizers are used here.
        rd_sec_base_addr <= write_partition_base(rd_sec_idx_async);
    end
end

//==============================================================================
// 读地址范围生成
//==============================================================================
always @(posedge ui_clk or negedge rst_n) begin
    if (!rst_n) begin
        app_addr_rd_min <= SEC0_BASE;
        app_addr_rd_max <= SEC0_BASE + RD_DATA_SIZE;
    end
    else begin
        app_addr_rd_min <= rd_sec_base_addr;
        app_addr_rd_max <= rd_sec_base_addr + RD_DATA_SIZE;
    end
end

endmodule
