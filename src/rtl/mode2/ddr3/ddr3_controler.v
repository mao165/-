
module ddr3_controler(
    input wire recover_hold_request, recover_clear_request, recover_fifo_request,
    input wire mig_sys_rst_n,
    input wire recover_hold_wr, recover_hold_rd,
    input wire data_rst_n_wr, data_rst_n_rd,
    input wire capture_started_clk,
    output wire recover_ui_clear_seen_40m, recover_ui_fifo_seen_40m,
    output wire recover_ui_ready_40m, recover_ui_run_40m,
    output wire recover_calib_40m, recover_calib_lost_40m,
    output wire recover_wr_ready, recover_rd_ready,
    inout   [63:0]      ddr3_dq          ,   //ddr3 数据
    inout   [7:0]       ddr3_dqs_n       ,   //ddr3 dqs负
    inout   [7:0]       ddr3_dqs_p       ,   //ddr3 dqs正  
    output  [14:0]      ddr3_addr        ,   //ddr3 地址   
    output  [2:0]       ddr3_ba          ,   //ddr3 banck 选择
    output              ddr3_ras_n       ,   //ddr3 行选择
    output              ddr3_cas_n       ,   //ddr3 列选择
    output              ddr3_we_n        ,   //ddr3 读写选择
    output              ddr3_reset_n     ,   //ddr3 复位
    output  [0:0]       ddr3_ck_p        ,   //ddr3 时钟正
    output  [0:0]       ddr3_ck_n        ,   //ddr3 时钟负
    output  [0:0]       ddr3_cke         ,   //ddr3 时钟使能
    output  [0:0]       ddr3_cs_n        ,   //ddr3 片选
    output  [7:0]       ddr3_dm          ,   //ddr3_dm
    output  [0:0]       ddr3_odt         ,   //ddr3_odt
    output              init_calib_complete,
    output              init_calib_complete_wr, // unique DCO-domain calibration level
    output  [11:0]      device_temp       ,   // MIG internal XADC temperature code
    input               sys_clk_i        ,   //系统时钟
    input               clk_ref_i        ,   //参考时钟
    input               rst_n            ,   //复位信号

    //PPS输入 (用于地址控制模块)
    input               pps_pulse        ,   //PPS脉冲 (clk_50m域，内部同步)

    //突发长度参数
    input   [7:0]       rd_bust_len      ,   //读突发长度
    input   [7:0]       wr_bust_len      ,   //写突发长度

    //读使能
    input               ddr3_read_valid  ,   //DDR3 读使能

    //读分区索引输入 (来自read_ctrl)
    input   [2:0]       rd_sec_idx       ,   //读分区索引 (clk_50m域)
    input               rd_sec_idx_valid ,   // bundled read-index event in rd_clk domain

    //写分区索引输出 (给read_ctrl)
    output  [2:0]       wr_sec_idx       ,   //当前写分区索引
    output  [2:0]       wr_sec_idx_prev  ,   //上一个写分区索引 (PPS前的值，即上一秒数据所在分区)
    output              wr_commit_valid_clk,
    output  [31:0]      wr_commit_seq_clk,
    output  [2:0]       wr_sec_idx_40m,
    output  [2:0]       wr_sec_idx_prev_40m,

    //调试用：读写地址输出
    output  [28:0]      debug_app_addr   ,   //当前DDR3访问地址
    output  [28:0]      debug_app_addr_rd_min,  //读起始地址
    output  [28:0]      debug_app_addr_wr_min,  //写起始地址
    output  [28:0]      debug_app_addr_rd,      //实际读地址
    output  [28:0]      debug_app_addr_wr,      //实际写地址
    output              debug_wr_sec_switch,    //写分区切换标志

    //用户FIFO接口
    input               wr_clk           ,   //写fifo写时钟
    input               rd_clk           ,   //读fifo读时钟
    input               rd_req           ,   //读数据请求使能                                                                    
    input               wr_en            ,   //写数据使能信号
    input   [63:0]      wrdata           ,   //写有效数据 
    input   [1:0]       wr_pack_count    ,   //64bit打包器中已缓存的16bit样本数
    input               fifo_flush       ,   //FIFO刷新请求（50M样本完成后触发）
    output  [63:0]      rddata           ,   //读有效数据 
    output              wfifo_wr_rst_busy,   //写fifo复位忙标志
    output              rfifo_rd_rst_busy,   //读fifo复位忙标志
    output              rd_data_valid,       //读数据有效标志
    output  [5:0]       debug_wfifo_rcount,  //写FIFO读侧数据计数（调试用）
    output              debug_app_rd_en,     //DDR3实际读使能（调试用）
    output              debug_app_wr_en,     //DDR3实际写使能（调试用）
    output  [12:0]      debug_rfifo_rcount,  //读FIFO读侧数据计数（调试用，13位）
    output              wr_boundary_busy,    //PPS边界前数据尚未写完
    output              pps_event_pulse,     // PPS event in wr_clk domain
    output  [31:0]      pps_event_seq,       // PPS event sequence
    output              rd_idx_mailbox_busy,
    output              rd_idx_mailbox_overflow,
    output              boundary_mailbox_busy,
    output              boundary_mailbox_overflow,
    output              wr_boundary_commit_40m,
    output  [31:0]      wr_boundary_commit_seq_40m,
    output  [2:0]       wr_boundary_commit_idx_40m,
    output              index_mailbox_busy,
    output              index_mailbox_overflow,
    output              boundary_protocol_error,
    output              wr_boundary_commit_error_40m,
    output              wfifo_overflow_latched, //写FIFO溢出锁存

    // 读地址完成标志（新增）
    output              rd_addr_done,         //读地址已达到目标

    // 新增：DDR3内部状态调试信号
    output  [23:0]      debug_rd_addr_cnt,    //读Burst计数器（0-24）
    output  [3:0]       debug_state_cnt,      //DDR3_RW状态机状态
    output  [9:0]       debug_rfifo_wcount    //读FIFO写侧数据量 (10位)
    ,output wire [3:0] debug_adc_boundary_control,
    output wire [2:0] debug_adc_word_mod8,
    output wire [31:0] debug_adc_group_target,
    output wire [31:0] debug_adc_group_accepted,
    output wire debug_ila_ui_heartbeat
    );

//wire define
wire                  app_rdy              ;
wire                  app_wdf_wren         ;
wire                  rfifo_wren           ;
wire                  rfifo_wr_rst_busy    ;
wire                  app_wdf_rdy          ;
wire                  app_rd_data_valid    ;
wire [28:0]           app_addr             ;
wire [2:0]            app_cmd              ;
wire                  app_en               ;
wire [511:0]          app_rd_data          ;
wire                  app_rd_data_end      ;
wire [511:0]          app_wdf_data         ;
wire                  app_wdf_end          ;
wire [63:0]           app_wdf_mask         ;
wire                  app_sr_active        ;
wire                  app_ref_ack          ;
wire                  app_zq_ack           ;
wire                  ui_clk               ;
wire [2:0] rd_sec_idx_ui;
wire       rd_sec_idx_ui_valid;
wire                  ui_clk_sync_rst      ;
wire [11:0]           mig_device_temp      ;
wire [5:0]            wfifo_rcount         ;
wire [9:0]            rfifo_wcount         ;  // 修正：写侧深度512，需要10位
wire [12:0]           rfifo_rcount         ;  // 修正：读侧深度4096，需要13位
wire                  rfifo_empty          ;
wire                  rfifo_valid_from_fifo;  // 读FIFO valid (FWFT: dout有效)
wire                  wfifo_full           ;
wire [31:0]           wr_boundary_group_target_src;
wire                  wr_boundary_send_src;
wire [63:0]           wr_boundary_payload_src;
wire [63:0]           wr_boundary_payload_ui;
wire debug_boundary_pending_src, debug_word_accept_src;
wire [31:0]           debug_wr_group_seq_src;
wire [2:0]            debug_wr_word_mod8_src;
(* ASYNC_REG = "TRUE" *) reg                   init_calib_wr_d0     ;
(* ASYNC_REG = "TRUE" *) reg                   init_calib_wr_d1     ;
wire                  wr_boundary_req_ui   ;
wire                  wr_boundary_commit_ui;
wire [31:0]           wr_boundary_commit_seq_ui;
wire [2:0]            wr_boundary_commit_idx_ui;
wire [35:0]           wr_boundary_commit_payload_40m;
wire                  wr_boundary_commit_error_ui;
(* ASYNC_REG = "TRUE" *) reg [1:0] boundary_error_ui_sync1;
(* ASYNC_REG = "TRUE" *) reg [1:0] boundary_error_ui_sync2;
wire                  wr_boundary_error_ui;
reg [11:0]            device_temp_ui_shadow;
reg [11:0]            device_temp_cdc_bus  ;
reg                   device_temp_cdc_toggle_ui;
(* ASYNC_REG = "TRUE" *) reg [2:0] device_temp_toggle_rd;
reg [11:0]            device_temp_rd_clk   ;

//地址控制模块输出信号
wire [28:0]           app_addr_wr_min      ;
wire [28:0]           app_addr_wr_max      ;
wire [28:0]           app_addr_rd_min      ;
wire [28:0]           app_addr_rd_max      ;
wire [28:0]           pending_wr_base      ;
wire [28:0]           pending_wr_max       ;
wire                  pending_wr_base_valid;
wire [2:0]            boundary_completed_idx;
wire [2:0]            debug_sec_idx        ;   // 调试用：分区索引
reg                   wr_idx_pair_pending_ui;
reg                   wr_idx_pair_send_ui;
wire [37:0]            wr_idx_pair_payload_clk;
wire [5:0]             wr_idx_pair_payload_40m;
wire [37:0]            wr_idx_pair_payload_ui_wr;
reg [31:0]             wr_idx_pair_seq_ui;
wire                   wr_idx_pair_valid_clk;
wire                   wr_idx_pair_valid_40m;
wire                   wr_idx_pair_busy_clk;
wire                   wr_idx_pair_overflow_clk;
wire                   wr_idx_pair_busy_40m;
wire                   wr_idx_pair_overflow_40m;
wire [2:0]            debug_sec_idx_prev   ;   // 调试用：上一个分区索引
wire                  debug_pps_pulse_ui   ;   // 调试用：ui_clk域PPS脉冲
wire [28:0]           debug_app_addr_rd_internal;  // 实际读地址
wire [28:0]           debug_app_addr_wr_internal;  // 实际写地址
wire                  debug_wr_sec_switch_internal; // 写分区切换标志
wire                  rd_addr_done_internal;        // 读地址完成标志（新增）
wire    [23:0]        debug_rd_addr_cnt_internal;  // 读地址计数器（新增）
wire    [3:0]         debug_state_cnt_internal;    // 状态机状态（新增）
wire    [9:0]         debug_rfifo_wcount_internal; // rd_fifo写侧数据量（新增）

assign debug_adc_boundary_control = {wr_boundary_send_src, debug_boundary_pending_src,
                                     debug_word_accept_src, wfifo_full};
assign debug_adc_word_mod8 = debug_wr_word_mod8_src;
assign debug_adc_group_target = wr_boundary_group_target_src;
assign debug_adc_group_accepted = debug_wr_group_seq_src;

//*****************************************************
//**                    main code
//*****************************************************

//==============================================================================
// rd_data_valid 信号生成 (FWFT)
//
// 因此：
// - 当 valid=1 时，rddata 上始终是当前队头的有效64bit
// - rd_req 只负责 pop(弹出) 队头，让下一条更新到 rddata
// 当前 rd_fifo 已配置为 FWFT + valid输出(高有效)，且已取消 output registers。
//
// 20 MHz reset may assert asynchronously, but release belongs to each
// destination clock. MIG UI reset also participates in the UI-domain epoch.
// The recovery endpoint and its ACKs survive MIG/data epoch resets.
(* ASYNC_REG = "TRUE" *) reg [2:0] global_ui_reset_pipe;
(* ASYNC_REG = "TRUE" *) reg [2:0] global_rd_reset_pipe;
always @(posedge ui_clk or negedge rst_n) begin
    if (!rst_n) global_ui_reset_pipe <= 3'b000;
    else global_ui_reset_pipe <= {global_ui_reset_pipe[1:0],1'b1};
end
always @(posedge rd_clk or negedge rst_n) begin
    if (!rst_n) global_rd_reset_pipe <= 3'b000;
    else global_rd_reset_pipe <= {global_rd_reset_pipe[1:0],1'b1};
end
wire global_rst_n_ui = global_ui_reset_pipe[2];
wire global_rst_n_rd = global_rd_reset_pipe[2];
wire recover_hold_ui, recover_clear_ui, recover_fifo_ui;
wire ui_clear_seen, ui_fifo_seen, ui_ready_seen, ui_run_seen, data_rst_n_ui;
wire wfifo_rd_rst_busy, wfifo_reset_active, rfifo_reset_active;
wire wfifo_reset_active_ui;
reg calib_seen_ui, calib_lost_ui;
wire ui_calibrated = init_calib_complete && !ui_clk_sync_rst;
wire app_hold_ui = recover_hold_ui || calib_lost_ui || !ui_calibrated;
wire ui_local_ready = ui_calibrated && !calib_lost_ui &&
    !wfifo_rd_rst_busy && !rfifo_wr_rst_busy &&
    !wfifo_reset_active_ui && !rfifo_reset_active &&
    !wr_idx_pair_busy_clk && !wr_idx_pair_busy_40m && !index_mailbox_busy;

recovery_domain_endpoint u_recovery_ui_endpoint (
    .clk(ui_clk), .rst_n(global_rst_n_ui),
    .hold_request(recover_hold_request), .clear_request(recover_clear_request),
    .fifo_request(recover_fifo_request), .local_ready(ui_local_ready),
    .hold_local(recover_hold_ui), .clear_local(recover_clear_ui),
    .fifo_local(recover_fifo_ui), .clear_seen(ui_clear_seen), .fifo_seen(ui_fifo_seen),
    .ready_seen(ui_ready_seen), .run_seen(ui_run_seen), .data_rst_n(data_rst_n_ui)
);
always @(posedge ui_clk or negedge global_rst_n_ui) begin
    if (!global_rst_n_ui) begin
        calib_seen_ui <= 1'b0;
        calib_lost_ui <= 1'b0;
    end else if (recover_clear_ui) begin
        calib_seen_ui <= 1'b0;
        calib_lost_ui <= 1'b0;
    end else begin
        if (ui_calibrated) calib_seen_ui <= 1'b1;
        if (calib_seen_ui && !ui_calibrated) calib_lost_ui <= 1'b1;
    end
end
// Independent status levels, not an atomic multi-bit transaction.
(* ASYNC_REG = "TRUE" *) reg [5:0] recovery_status_s1, recovery_status_s2;
always @(posedge rd_clk or negedge global_rst_n_rd) begin
    if (!global_rst_n_rd) begin
        recovery_status_s1 <= 6'd0;
        recovery_status_s2 <= 6'd0;
    end else begin
        recovery_status_s1 <= {calib_lost_ui,ui_calibrated,ui_run_seen,
                               ui_ready_seen,ui_fifo_seen,ui_clear_seen};
        recovery_status_s2 <= recovery_status_s1;
    end
end
assign {recover_calib_lost_40m,recover_calib_40m,recover_ui_run_40m,
        recover_ui_ready_40m,recover_ui_fifo_seen_40m,recover_ui_clear_seen_40m}
       = recovery_status_s2;
assign recover_wr_ready = data_rst_n_wr && !wfifo_reset_active &&
                          !wfifo_wr_rst_busy && !boundary_mailbox_busy;
assign recover_rd_ready = data_rst_n_rd && !rfifo_reset_active &&
                          !rfifo_rd_rst_busy && !rd_idx_mailbox_busy;
wire ui_reset_request_n = data_rst_n_ui && !ui_clk_sync_rst;
wire rd_reset_request_n = rst_n && data_rst_n_rd;
(* ASYNC_REG = "TRUE" *) reg [2:0] ui_reset_pipe;
(* ASYNC_REG = "TRUE" *) reg [2:0] rd_reset_pipe;
wire rst_n_ui = ui_reset_pipe[2];
wire rst_n_rd = rd_reset_pipe[2];
always @(posedge ui_clk or negedge ui_reset_request_n) begin
    if (!ui_reset_request_n) ui_reset_pipe <= 3'b000;
    else ui_reset_pipe <= {ui_reset_pipe[1:0], 1'b1};
end
always @(posedge rd_clk or negedge rd_reset_request_n) begin
    if (!rd_reset_request_n) rd_reset_pipe <= 3'b000;
    else rd_reset_pipe <= {rd_reset_pipe[1:0], 1'b1};
end

// Queue the post-commit index snapshot until both destination mailboxes are ready.
always @(posedge ui_clk or negedge rst_n_ui) begin
    if (!rst_n_ui) begin
        wr_idx_pair_pending_ui <= 1'b0;
        wr_idx_pair_send_ui    <= 1'b0;
        wr_idx_pair_seq_ui     <= 32'd0;
    end else begin
        wr_idx_pair_send_ui <= 1'b0;
        if (wr_boundary_commit_ui) begin
            wr_idx_pair_pending_ui <= 1'b1;
            wr_idx_pair_seq_ui <= wr_boundary_commit_seq_ui;
        end
        if (!app_hold_ui && wr_idx_pair_pending_ui && !wr_idx_pair_busy_clk &&
            !wr_idx_pair_busy_40m) begin
            wr_idx_pair_send_ui    <= 1'b1;
            wr_idx_pair_pending_ui <= 1'b0;
        end
    end
end

assign wr_idx_pair_payload_ui_wr = {wr_idx_pair_seq_ui, debug_sec_idx_prev, debug_sec_idx};

cdc_event_mailbox #(
    .PAYLOAD_WIDTH (38)
) u_wr_idx_to_wr_clk_mailbox (
    .src_clk       (ui_clk),
    .src_rst_n     (rst_n_ui),
    .src_send      (wr_idx_pair_send_ui),
    .src_payload   (wr_idx_pair_payload_ui_wr),
    .src_busy      (wr_idx_pair_busy_clk),
    .src_overflow  (wr_idx_pair_overflow_clk),
    .dst_clk       (wr_clk),
    .dst_rst_n     (data_rst_n_wr),
    .dst_valid     (wr_idx_pair_valid_clk),
    .dst_payload   (wr_idx_pair_payload_clk)
);

cdc_event_mailbox #(
    .PAYLOAD_WIDTH (6)
) u_wr_idx_to_rd_clk_mailbox (
    .src_clk       (ui_clk),
    .src_rst_n     (rst_n_ui),
    .src_send      (wr_idx_pair_send_ui),
    .src_payload   (wr_idx_pair_payload_ui_wr[5:0]),
    .src_busy      (wr_idx_pair_busy_40m),
    .src_overflow  (wr_idx_pair_overflow_40m),
    .dst_clk       (rd_clk),
    .dst_rst_n     (rst_n_rd),
    .dst_valid     (wr_idx_pair_valid_40m),
    .dst_payload   (wr_idx_pair_payload_40m)
);

assign wr_commit_valid_clk = wr_idx_pair_valid_clk;
assign wr_commit_seq_clk = wr_idx_pair_payload_clk[37:6];
assign wr_sec_idx        = wr_idx_pair_payload_clk[2:0];
assign wr_sec_idx_prev   = wr_idx_pair_payload_clk[5:3];
assign wr_sec_idx_40m    = wr_idx_pair_payload_40m[2:0];
assign wr_sec_idx_prev_40m = wr_idx_pair_payload_40m[5:3];

// 注：read_ctrl 只用 rd_data_valid 做计数，因此这里用 valid 更符合“有可读数据”的语义。
//==============================================================================
assign rd_data_valid = rfifo_valid_from_fifo;
assign init_calib_complete_wr = init_calib_wr_d1;

//==============================================================================
// PPS数据边界跟踪
//==============================================================================
always @(posedge wr_clk or negedge rst_n) begin
    if (!rst_n) begin
        init_calib_wr_d0 <= 1'b0;
        init_calib_wr_d1 <= 1'b0;
    end
    else begin
        init_calib_wr_d0 <= init_calib_complete;
        init_calib_wr_d1 <= init_calib_wr_d0;
    end
end

always @(posedge ui_clk or negedge rst_n_ui) begin
    if (!rst_n_ui) begin
        boundary_error_ui_sync1 <= 2'b00;
        boundary_error_ui_sync2 <= 2'b00;
    end else begin
        boundary_error_ui_sync1 <= {wfifo_overflow_latched, boundary_mailbox_overflow};
        boundary_error_ui_sync2 <= boundary_error_ui_sync1;
    end
end
assign wr_boundary_error_ui = boundary_error_ui_sync2[1] ||
                            boundary_error_ui_sync2[0] ||
                            boundary_protocol_error ||
                            index_mailbox_overflow;

wr_boundary_event_source u_wr_boundary_event_source (
    .wr_clk                  (wr_clk),
    .rst_n                   (data_rst_n_wr),
    .recover_hold            (recover_hold_wr),
    .capture_started         (capture_started_clk),
    .enable                  (init_calib_wr_d1),
    .pps_pulse               (pps_pulse),
    .wr_en                   (wr_en && !wfifo_wr_rst_busy && !wfifo_reset_active),
    .wfifo_full              (wfifo_full),
    .pack_count              (wr_pack_count),
    .boundary_mailbox_busy   (boundary_mailbox_busy),
    .pps_event_pulse         (pps_event_pulse),
    .pps_event_seq           (pps_event_seq),
    .boundary_group_target   (wr_boundary_group_target_src),
    .boundary_send           (wr_boundary_send_src),
    .boundary_payload        (wr_boundary_payload_src),
    .overflow_latched        (wfifo_overflow_latched),
    .debug_group_seq         (debug_wr_group_seq_src),
    .debug_word_mod8         (debug_wr_word_mod8_src),
    .debug_word_accept       (debug_word_accept_src),
    .debug_boundary_pending  (debug_boundary_pending_src)
);

cdc_event_mailbox #(
    .PAYLOAD_WIDTH (64)
) u_wr_boundary_mailbox (
    .src_clk       (wr_clk),
    .src_rst_n     (data_rst_n_wr),
    .src_send      (wr_boundary_send_src),
    .src_payload   (wr_boundary_payload_src),
    .src_busy      (boundary_mailbox_busy),
    .src_overflow  (boundary_mailbox_overflow),
    .dst_clk       (ui_clk),
    .dst_rst_n     (rst_n_ui),
    .dst_valid     (wr_boundary_req_ui),
    .dst_payload   (wr_boundary_payload_ui)
);

//==============================================================================
// DDR3地址控制模块例化
//==============================================================================
cdc_event_mailbox #(
    .PAYLOAD_WIDTH (3)
) u_rd_idx_mailbox (
    .src_clk       (rd_clk),
    .src_rst_n     (rst_n_rd),
    .src_send      (rd_sec_idx_valid),
    .src_payload   (rd_sec_idx),
    .src_busy      (rd_idx_mailbox_busy),
    .src_overflow  (rd_idx_mailbox_overflow),
    .dst_clk       (ui_clk),
    .dst_rst_n     (rst_n_ui),
    .dst_valid     (rd_sec_idx_ui_valid),
    .dst_payload   (rd_sec_idx_ui)
);
ddr3_addr_ctrl u_ddr3_addr_ctrl(
    .ui_clk             (ui_clk),
    .ui_clk_sync_rst    (~rst_n_ui),
    .pps_pulse_async    (pps_event_pulse),
    .init_calib_complete(init_calib_complete),
    .boundary_req_valid (wr_boundary_req_ui),
    .wr_boundary_commit (wr_boundary_commit_ui),

    .rd_sec_idx_async   (rd_sec_idx_ui),
    .rd_sec_idx_valid   (rd_sec_idx_ui_valid),
    .app_addr_wr_min    (app_addr_wr_min),
    .app_addr_wr_max    (app_addr_wr_max),
    .pending_wr_base    (pending_wr_base),
    .pending_wr_max     (pending_wr_max),
    .pending_wr_base_valid(pending_wr_base_valid),
    .boundary_completed_idx(boundary_completed_idx),
    .app_addr_rd_min    (app_addr_rd_min),
    .app_addr_rd_max    (app_addr_rd_max),
    .sec_idx            (debug_sec_idx),
    .sec_idx_prev       (debug_sec_idx_prev),
    .pps_pulse_ui       (debug_pps_pulse_ui)
);

// 写分区索引输出给read_ctrl使用

// 调试地址输出
assign debug_app_addr = app_addr;
assign debug_app_addr_rd_min = app_addr_rd_min;
assign debug_app_addr_wr_min = app_addr_wr_min;

//==============================================================================
// 读写模块
//==============================================================================
ddr3_rw u_ddr3_rw(
    .recover_hold         (app_hold_ui),
    .debug_commit_mailbox_errors({wr_idx_pair_overflow_40m,wr_idx_pair_overflow_clk,index_mailbox_overflow}),
    .debug_selected_read_index(rd_sec_idx_ui),
    .debug_ila_ui_heartbeat(debug_ila_ui_heartbeat),
    .ui_clk               (ui_clk),
    .ui_clk_sync_rst      (~rst_n_ui),
    .init_calib_complete  (init_calib_complete),
    .app_rdy              (app_rdy),
    .app_wdf_rdy          (app_wdf_rdy),
    .app_rd_data_valid    (app_rd_data_valid),
    .app_addr             (app_addr),
    .app_en               (app_en),
    .app_wdf_wren         (app_wdf_wren),
    .app_wdf_end          (app_wdf_end),
    .app_cmd              (app_cmd),
    .app_addr_rd_min      (app_addr_rd_min),
    .app_addr_rd_max      (app_addr_rd_max),
    .rd_bust_len          (rd_bust_len),
    .app_addr_wr_min      (app_addr_wr_min),
    .app_addr_wr_max      (app_addr_wr_max),
    .wr_bust_len          (wr_bust_len),
    .rfifo_wren           (rfifo_wren),
    .ddr3_read_valid      (ddr3_read_valid),
    .fifo_flush_async     (fifo_flush),           // 新增：用于清零rd_addr_done
    .wr_boundary_start    (debug_pps_pulse_ui && init_calib_complete),
    .wr_boundary_req      (wr_boundary_req_ui),
    .wr_boundary_event_seq(wr_boundary_payload_ui[63:32]),
    .wr_boundary_group_target(wr_boundary_payload_ui[31:0]),
    .wr_boundary_completed_idx(boundary_completed_idx),
    .next_wr_base         (pending_wr_base),
    .next_wr_max          (pending_wr_max),
    .next_wr_base_valid   (pending_wr_base_valid),
    .wr_boundary_commit_sink_busy(index_mailbox_busy),
    .wr_boundary_busy     (wr_boundary_busy),
    .wr_boundary_commit   (wr_boundary_commit_ui),
    .wr_boundary_commit_seq(wr_boundary_commit_seq_ui),
    .wr_boundary_commit_idx(wr_boundary_commit_idx_ui),
    .wr_boundary_error     (wr_boundary_error_ui),
    .wr_boundary_commit_error(wr_boundary_commit_error_ui),
    .boundary_protocol_error(boundary_protocol_error),
    .wfifo_rcount         (wfifo_rcount),
    .rfifo_wcount         (rfifo_wcount),
    .rfifo_wr_rst_busy    (rfifo_wr_rst_busy),
    .rd_addr_done         (rd_addr_done_internal),
    .debug_app_addr_rd    (debug_app_addr_rd_internal),
    .debug_app_addr_wr    (debug_app_addr_wr_internal),
    .debug_wr_sec_switch  (debug_wr_sec_switch_internal),
    .debug_app_rd_en      (debug_app_rd_en),
    .debug_app_wr_en      (debug_app_wr_en),
    // 新增调试信号
    .debug_rd_addr_cnt    (debug_rd_addr_cnt_internal),
    .debug_state_cnt      (debug_state_cnt_internal),
    .debug_rfifo_wcount   (debug_rfifo_wcount_internal)
);

cdc_event_mailbox #(
    .PAYLOAD_WIDTH (36)
) u_wr_boundary_commit_mailbox (
    .src_clk       (ui_clk),
    .src_rst_n     (rst_n_ui),
    .src_send      (wr_boundary_commit_ui),
    .src_payload   ({wr_boundary_commit_error_ui,
                     wr_boundary_commit_seq_ui,
                     wr_boundary_commit_idx_ui}),
    .src_busy      (index_mailbox_busy),
    .src_overflow  (index_mailbox_overflow),
    .dst_clk       (rd_clk),
    .dst_rst_n     (rst_n_rd),
    .dst_valid     (wr_boundary_commit_40m),
    .dst_payload   (wr_boundary_commit_payload_40m)
);

assign wr_boundary_commit_error_40m = wr_boundary_commit_payload_40m[35];
assign wr_boundary_commit_seq_40m = wr_boundary_commit_payload_40m[34:3];
assign wr_boundary_commit_idx_40m = wr_boundary_commit_payload_40m[2:0];

// 实际读写地址输出
assign debug_app_addr_rd = debug_app_addr_rd_internal;
assign debug_app_addr_wr = debug_app_addr_wr_internal;
assign debug_wr_sec_switch = debug_wr_sec_switch_internal;

// 写FIFO计数输出（调试用）
assign debug_wfifo_rcount = wfifo_rcount;

// 读FIFO读侧计数输出（调试用）
assign debug_rfifo_rcount = rfifo_rcount;

// 读地址完成标志输出
assign rd_addr_done = rd_addr_done_internal;

// 新增：DDR3内部状态调试信号输出
assign debug_rd_addr_cnt = debug_rd_addr_cnt_internal;
assign debug_state_cnt = debug_state_cnt_internal;
assign debug_rfifo_wcount = debug_rfifo_wcount_internal;
assign device_temp = device_temp_rd_clk;

// MIG device_temp is generated in ui_clk. Export a rd_clk-domain copy for
// timestamp temperature metadata.
always @(posedge ui_clk or negedge rst_n_ui) begin
    if (!rst_n_ui) begin
        device_temp_ui_shadow     <= 12'd0;
        device_temp_cdc_bus       <= 12'd0;
        device_temp_cdc_toggle_ui <= 1'b0;
    end else if (ui_clk_sync_rst) begin
        device_temp_ui_shadow     <= 12'd0;
        device_temp_cdc_bus       <= 12'd0;
        device_temp_cdc_toggle_ui <= 1'b0;
    end else if (mig_device_temp != device_temp_ui_shadow) begin
        device_temp_ui_shadow     <= mig_device_temp;
        device_temp_cdc_bus       <= mig_device_temp;
        device_temp_cdc_toggle_ui <= ~device_temp_cdc_toggle_ui;
    end
end

always @(posedge rd_clk or negedge rst_n_rd) begin
    if (!rst_n_rd) begin
        device_temp_toggle_rd <= 3'd0;
        device_temp_rd_clk    <= 12'd0;
    end else begin
        device_temp_toggle_rd <= {device_temp_toggle_rd[1:0], device_temp_cdc_toggle_ui};
        if (device_temp_toggle_rd[2] ^ device_temp_toggle_rd[1]) begin
            device_temp_rd_clk <= device_temp_cdc_bus;
        end
    end
end

//==============================================================================
// MIG IP核模块
//==============================================================================
mig_7series_0 u_mig_7series_0 (
    .ddr3_addr                      (ddr3_addr),  // output [14:0]		ddr3_addr
    .ddr3_ba                        (ddr3_ba),  // output [2:0]		ddr3_ba
    .ddr3_cas_n                     (ddr3_cas_n),  // output			ddr3_cas_n
    .ddr3_ck_n                      (ddr3_ck_n),  // output [0:0]		ddr3_ck_n
    .ddr3_ck_p                      (ddr3_ck_p),  // output [0:0]		ddr3_ck_p
    .ddr3_cke                       (ddr3_cke),  // output [0:0]		ddr3_cke
    .ddr3_ras_n                     (ddr3_ras_n),  // output			ddr3_ras_n
    .ddr3_reset_n                   (ddr3_reset_n),  // output			ddr3_reset_n
    .ddr3_we_n                      (ddr3_we_n),  // output			ddr3_we_n
    .ddr3_dq                        (ddr3_dq),  // inout [63:0]		ddr3_dq
    .ddr3_dqs_n                     (ddr3_dqs_n),  // inout [7:0]		ddr3_dqs_n
    .ddr3_dqs_p                     (ddr3_dqs_p),  // inout [7:0]		ddr3_dqs_p
    .init_calib_complete            (init_calib_complete),  // output          init_calib_complete
	.ddr3_cs_n                      (ddr3_cs_n),  // output [0:0]		ddr3_cs_n
    .ddr3_dm                        (ddr3_dm),  // output [7:0]		ddr3_dm
    .ddr3_odt                       (ddr3_odt),  // output [0:0]		ddr3_odt
    // Application interface ports
    .app_addr                       (app_addr),  // input [28:0]		app_addr
    .app_cmd                        (app_cmd),  // input [2:0]		app_cmd
    .app_en                         (app_en),  // input				app_en
    .app_wdf_data                   (app_wdf_data),  // input [511:0]		app_wdf_data
    .app_wdf_end                    (app_wdf_end),  // input				app_wdf_end
    .app_wdf_wren                   (app_wdf_wren),  // input				app_wdf_wren
    .app_rd_data                    (app_rd_data),  // output [511:0]		app_rd_data
    .app_rd_data_end                (app_rd_data_end),  // output			app_rd_data_end
    .app_rd_data_valid              (app_rd_data_valid),  // output			app_rd_data_valid
    .app_rdy                        (app_rdy),  // output			app_rdy
    .app_wdf_rdy                    (app_wdf_rdy),  // output			app_wdf_rdy
    .app_sr_req                     (1'b0),  // input			app_sr_req
    .app_ref_req                    (1'b0),  // input			app_ref_req
    .app_zq_req                     (1'b0),  // input			app_zq_req
    .app_sr_active                  (app_sr_active),  // output			app_sr_active
    .app_ref_ack                    (app_ref_ack),  // output			app_ref_ack
    .app_zq_ack                     (app_zq_ack),  // output			app_zq_ack
    .ui_clk                         (ui_clk),  // output			ui_clk
    .ui_clk_sync_rst                (ui_clk_sync_rst),  // output			ui_clk_sync_rst
    .app_wdf_mask                   (64'b0),  // input [63:0]		app_wdf_mask
    // System Clock Ports
    .sys_clk_i                       (sys_clk_i),
    // Reference Clock Ports
    .clk_ref_i                      (clk_ref_i),
    .device_temp                    (mig_device_temp),
    .sys_rst                        (mig_sys_rst_n) // input sys_rst
    );

ddr3_fifo_ctrl u_ddr3_fifo_ctrl (
    .recover_fifo_request(recover_fifo_request),
    .global_rst_n_ui(global_rst_n_ui),
    .wfifo_reset_active_ui(wfifo_reset_active_ui),
    .recover_hold_wr(recover_hold_wr || !capture_started_clk),
    .recover_hold_rd(recover_hold_rd), .recover_hold_ui(app_hold_ui),
    .wfifo_rd_rst_busy(wfifo_rd_rst_busy),
    .wfifo_reset_active(wfifo_reset_active), .rfifo_reset_active(rfifo_reset_active),
    .rst_n               (rst_n)          ,
    // comment
    .wr_clk              (wr_clk)         ,
    .rd_clk              (rd_clk)         ,
    .ui_clk              (ui_clk)         ,
    .wr_en               (wr_en)          ,
    .wrdata              (wrdata )        ,
    .rfifo_din           (app_rd_data   ) ,
    .rdata_req           (rd_req     )    ,
    .rfifo_wren          (rfifo_wren  )   ,
    .wfifo_rden          (app_wdf_wren  ) ,
    .fifo_flush          (fifo_flush)     ,  // 新增：FIFO刷新请求
    .wfifo_dout          (app_wdf_data  ) ,
    .wfifo_rcount        (wfifo_rcount)   ,
    .wfifo_full          (wfifo_full)     ,
    .rfifo_wcount        (rfifo_wcount)   ,
    .rfifo_rcount        (rfifo_rcount)   ,
    .rddata              (rddata      )   ,
    .wfifo_wr_rst_busy   (wfifo_wr_rst_busy),
    .rfifo_wr_rst_busy   (rfifo_wr_rst_busy),
    .rfifo_rd_rst_busy   (rfifo_rd_rst_busy),
    .rfifo_empty         (rfifo_empty),
    .rfifo_valid         (rfifo_valid_from_fifo)
    );

endmodule
