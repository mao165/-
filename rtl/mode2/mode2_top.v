`timescale 1ns / 1ps

module mode2_top(
    //==========================================================================
    // 系统时钟与复位
    //==========================================================================
    input   wire                clk,                // 用户时钟（DCO域）
    input   wire                clk_50m,             // ILA observation clock
    input   wire                clk_200m,
    input   wire                clk_40m,             // 40MHz readout clock
    input   wire                rst_n,
    input   wire                rst_n_40m,
    
    //==========================================================================
    // ADC数据接口
    //==========================================================================
    input   wire    [15:0]      adc_data,
    input   wire    [15:0]      adc_data_raw_debug,
    
    //==========================================================================
    // PPS接口
    //==========================================================================
    input   wire                pps_pos_edge_sync,

    //==========================================================================
    // GPS timestamp interface
    //==========================================================================
    input   wire    [1:0]       gps_lock_status,
    input   wire    [23:0]      rmc_time,
    input   wire    [23:0]      rmc_date,
    input   wire                ts_send_done,
    input   wire                segment_send_done,
    input   wire    [31:0]      segment_send_done_seq,
    
    //==========================================================================
    // 命令接口
    //==========================================================================
    input   wire                cmd_valid,
    input   wire    [23:0]      cmd_data,
    
    //==========================================================================
    // 数据输出接口
    //==========================================================================
    output  wire    [15:0]      data_out,
    output  wire                data_out_valid,
    output  wire                transfer_flag,
    output  wire                data_stream_flag,

    //==========================================================================
    // Timestamp packet request output
    //==========================================================================
    output  wire                ts_req,
    output  wire    [2:0]       ts_sec_idx,
    output  wire    [1:0]       ts_work_mode,
    output  wire    [1:0]       ts_gps_lock_status,
    output  wire    [23:0]      ts_date,
    output  wire    [23:0]      ts_time,
    output  wire    [15:0]      ts_temp_raw,
    output  wire    [7:0]       ts_temp_flags,
    output  reg     [15:0]      ts_threshold,
    //==========================================================================
    // DDR3物理接口
    //==========================================================================
    inout   wire    [63:0]      ddr3_dq,
    inout   wire    [7:0]       ddr3_dqs_n,
    inout   wire    [7:0]       ddr3_dqs_p,
    output  wire    [14:0]      ddr3_addr,
    output  wire    [2:0]       ddr3_ba,
    output  wire                ddr3_ras_n,
    output  wire                ddr3_cas_n,
    output  wire                ddr3_we_n,
    output  wire                ddr3_reset_n,
    output  wire    [0:0]       ddr3_ck_p,
    output  wire    [0:0]       ddr3_ck_n,
    output  wire    [0:0]       ddr3_cke,
    output  wire    [0:0]       ddr3_cs_n,
    output  wire    [7:0]       ddr3_dm,
    output  wire    [0:0]       ddr3_odt,
    
    //==========================================================================
    // 状态输出
    //==========================================================================
    output  wire                ddr_init_done,
    output  wire                sys_running,
    input wire ts_mailbox_busy_40m,
    input wire ts_mailbox_overflow_40m,
    output wire [31:0] ts_event_seq,
    output wire [15:0] debug_ila_read_control,
    output wire [34:0] debug_ila_read_identity,
    output wire [63:0] debug_ila_read_word,
    output wire [36:0] debug_ila_read_count,
    output wire [15:0] debug_ila_unpack_level,
    output wire [47:0] debug_ila_read_sample,
    output wire [4:0] debug_ila_gps_events,
    output wire [31:0] debug_ila_time_transaction_seq,
    output wire [25:0] debug_ila_command_state,
    output wire debug_ila_ui_heartbeat,
    input wire recover_hold_request,
    input wire recover_clear_request,
    input wire recover_fifo_request,
    input wire recover_mig_rst_n,
    input wire recover_hold_40m,
    input wire data_rst_n_40m,
    output wire recover_dco_clear_seen_40m,
    output wire recover_dco_fifo_seen_40m,
    output wire recover_dco_ready_40m,
    output wire recover_dco_run_40m,
    output wire recover_ui_clear_seen_40m,
    output wire recover_ui_fifo_seen_40m,
    output wire recover_ui_ready_40m,
    output wire recover_ui_run_40m,
    output wire recover_calib_40m,
    output wire recover_calib_lost_40m,
    output wire recover_overwrite_40m,
    output wire recover_local_ready_40m
);

//==============================================================================
// 内部信号定义
//==============================================================================

// DDR3控制器接口
wire                init_calib_complete;
wire                init_calib_complete_wr;
wire    [63:0]      rd_data;
wire                wfifo_wr_rst_busy;
wire                rfifo_rd_rst_busy;
wire                rd_data_valid;
wire    [2:0]       wr_sec_idx;
wire    [2:0]       wr_sec_idx_prev;
wire    [2:0]       wr_sec_idx_40m_mailbox;
wire    [2:0]       wr_sec_idx_prev_40m_mailbox;
wire                rd_idx_mailbox_busy;
wire                rd_idx_mailbox_overflow;
wire                wr_boundary_busy;
wire                pps_event_pulse_ddr;
wire    [31:0]      pps_event_seq_ddr;
wire                boundary_mailbox_busy;
wire                boundary_mailbox_overflow;
wire                wr_commit_valid_clk;
wire [31:0]         wr_commit_seq_clk;
reg  [51:0]         trigger_event_payload_hold;
reg                 trigger_event_send;
reg                 trigger_event_pending;
reg                 trigger_event_overflow;
wire                trigger_event_mailbox_busy;
wire                trigger_event_mailbox_overflow;
wire                trigger_event_valid_40m;
wire [51:0]         trigger_event_payload_40m;
wire [15:0] completed_trigger_threshold;
wire [15:0] task_trigger_threshold;
wire                wr_boundary_commit_40m_event;
wire    [31:0]      wr_boundary_commit_seq_40m;
wire    [2:0]       wr_boundary_commit_idx_40m;
wire                wr_boundary_commit_error_40m;
wire                index_mailbox_busy;
wire                index_mailbox_overflow;
wire                boundary_protocol_error;
wire                wfifo_overflow_latched;

// 数据打包信号
wire    [63:0]      wr_data;
wire                wr_en;

// 控制信号
wire                pack_enable;
wire                ddr_init_done_sync;

// 命令解析信号
wire    [1:0]       work_mode;
wire    [15:0]      threshold;
wire                cmd_valid_40m;
wire    [23:0]      cmd_data_40m;
wire    [56:0]      cmd_payload_40m;
wire    [56:0]      cmd_payload_clk;
wire                cmd_mailbox_busy_20m;
wire                test_cmd_clk;
wire                test_cmd_accept_clk;
wire                test_cmd_cutoff_valid_clk;
wire    [1:0]       work_mode_40m;
wire    [15:0]      threshold_40m;
wire                pps_pos_edge_40m;
wire                pps_sync_dummy;
wire                ddr_init_done_40m;



wire    [7:0]       trig_mask_40m;
wire    [2:0]       wr_sec_idx_40m;
wire    [2:0]       wr_sec_idx_prev_40m;
wire                wr_boundary_busy_40m;

// Step3 versioned timestamp transaction path (clk -> clk_40m).
wire [97:0] timestamp_time_payload_clk;
reg  [97:0] timestamp_time_payload_hold;
reg         timestamp_time_pending;
reg         timestamp_time_send;
reg         timestamp_time_overflow;
wire        timestamp_time_busy;
wire        timestamp_time_mailbox_overflow;
wire        timestamp_time_valid_40m;
wire [97:0] timestamp_time_payload_40m;
wire [15:0] timestamp_metadata_threshold_40m;
wire [15:0] timestamp_barrier_threshold;
wire        timestamp_metadata_valid_40m;
wire [31:0] timestamp_metadata_seq_40m;
wire [2:0]  timestamp_metadata_index_40m;
wire [23:0] timestamp_metadata_date_40m;
wire [23:0] timestamp_metadata_time_40m;
wire [1:0]  timestamp_metadata_status_40m;
wire        timestamp_join_overflow_40m;
wire        timestamp_join_mismatch_40m;
wire        timestamp_join_timeout_40m;
wire        timestamp_barrier_req;
wire [2:0]  timestamp_barrier_index;
wire [1:0]  timestamp_barrier_work_mode;
wire [31:0] timestamp_barrier_seq;
wire [23:0] timestamp_barrier_date;
wire [23:0] timestamp_barrier_time;
wire [1:0]  timestamp_barrier_status;
wire        timestamp_barrier_pending;
wire        timestamp_barrier_overflow;
wire        timestamp_barrier_invalid;
wire        timestamp_barrier_timeout;
wire        timestamp_barrier_failed;
wire        timestamp_request_cancel;
wire [31:0] timestamp_expected_seq;
wire        timestamp_expected_seq_valid;
wire [31:0] read_active_seq;
wire        read_abort_sticky;
wire [2:0]  timestamp_pending_index;
wire [31:0] timestamp_pending_seq;

// 触发检测信号 (Step7新增)
wire    [7:0]       trig_mask;              // 触发掩码
wire    [2:0]       trig_clr_idx;           // 触发清除索引 (来自read_ctrl)
wire                trig_clr_pulse;         // 触发清除脉冲 (来自read_ctrl)
wire    [2:0]       trig_clr_idx_40m;        // read_ctrl clear index in 40MHz domain
wire                trig_clr_pulse_40m;      // read_ctrl clear pulse in 40MHz domain
wire                debug_trig_detected;
wire                cmd_mailbox_overflow_20m;
wire                pps_mailbox_overflow_20m;
wire                trig_clr_mailbox_overflow_40m = 1'b0;


// 读控制信号
wire                ddr3_read_valid;
wire    [2:0]       rd_sec_idx;
wire                rd_sec_idx_valid;
wire                transfer_flag_int;
wire                ts_req_int;
wire    [2:0]       ts_sec_idx_int;
wire    [1:0]       ts_work_mode_int;
reg                 ts_req_d1;
reg                 ts_transaction_busy;
reg     [1:0]       ts_work_mode_d1;
wire    [11:0]      mig_device_temp;
wire    [15:0]      current_temp_raw;
wire                current_temp_valid;
wire                current_temp_update_pulse;
wire                current_temp_stale;
wire                current_temp_alarm;
wire    [2:0]       debug_state;
wire    [23:0]      debug_rd_data_cnt;

// 解包模块信号
wire    [15:0]      data_out_int;
wire                data_out_valid_int;
wire                rd_req;
wire                data_stream_flag_int;
wire    [1:0]       debug_idx;
wire    [31:0]      debug_sample_cnt;
wire                fifo_flush;             // 新增：FIFO刷新请求（50M样本完成后触发）

// 新增：解包模块分层排空调试信号
wire                debug_unpack_transfer_active;
wire                debug_unpack_transfer_start;
wire                debug_unpack_done;
wire                debug_unpack_fifo_wr_en;
wire                debug_unpack_fifo_rd_en;
wire                debug_unpack_fifo_full;
wire                debug_unpack_fifo_empty;
wire                debug_unpack_fifo_valid;
wire    [8:0]       debug_unpack_fifo_rd_count;
wire    [6:0]       debug_unpack_fifo_wr_count;

// 新增：read_ctrl分层排空调试信号
wire                debug_rd_addr_done_sync;
wire                debug_test_cmd_pending;
wire    [7:0]       debug_pending_mask;
wire                debug_pending_valid;
wire    [2:0]       debug_pending_rd_idx;

// DDR3调试信号
wire    [28:0]      debug_app_addr;
wire    [28:0]      debug_app_addr_rd_min;
wire    [28:0]      debug_app_addr_wr_min;
wire    [28:0]      debug_app_addr_rd;
wire    [28:0]      debug_app_addr_wr;
wire                debug_wr_sec_switch;
wire    [5:0]       debug_wfifo_rcount;
wire                debug_app_rd_en;
wire                debug_app_wr_en;
wire    [12:0]      debug_rfifo_rcount;     // 读FIFO读侧数据计数（13位）
wire    [1:0]       debug_pack_cnt;
wire                rd_addr_done;           // 读地址完成标志（新增）

// 新增：DDR3内部状态调试信号
wire    [23:0]      debug_rd_addr_cnt;      // 读Burst计数器（0-24）
wire    [3:0]       debug_state_cnt;        // DDR3_RW状态机状态
wire    [9:0]       debug_rfifo_wcount;     // 读FIFO写侧数据量

// 固定参数
localparam [7:0] WR_BURST_LEN = 8'd25;
localparam [7:0] RD_BURST_LEN = 8'd25;

//------------------------------------------------------------------------------
// ILA observation synchronizers (clk_50m domain only)
//------------------------------------------------------------------------------
(* ASYNC_REG = "TRUE" *) reg        wr_boundary_busy_40m_s1, wr_boundary_busy_40m_s2;
(* ASYNC_REG = "TRUE" *) reg        ddr_init_done_40m_s1, ddr_init_done_40m_s2;



//==============================================================================
// 输出信号赋值
//==============================================================================
// Recovery handshakes survive data-epoch clear. Each returned bit is an
// independent status level, never a sampled multi-bit transaction.
wire recover_hold_clk, recover_clear_clk, recover_fifo_clk, data_rst_n_clk;
wire recover_wr_ready, recover_rd_ready, pps_mailbox_busy;
wire dco_clear_seen, dco_fifo_seen, dco_ready_seen, dco_run_seen;
(* ASYNC_REG = "TRUE" *) reg [3:0] dco_recovery_s1, dco_recovery_s2;
// The recovery ready barrier also drains the CONTROL mailbox. Its ACK
// proves all pre-clear cutoff payloads were consumed before RELEASE; no
// elapsed-time assumption or independently synchronized sequence is used.
wire dco_local_ready = recover_wr_ready && !timestamp_time_busy &&
    !trigger_event_mailbox_busy && !pps_mailbox_busy && !boundary_mailbox_busy &&
    !cmd_mailbox_busy_20m;
recovery_domain_endpoint u_recovery_dco (
    .clk(clk), .rst_n(rst_n),
    .hold_request(recover_hold_request), .clear_request(recover_clear_request),
    .fifo_request(recover_fifo_request), .local_ready(dco_local_ready),
    .hold_local(recover_hold_clk), .clear_local(recover_clear_clk),
    .fifo_local(recover_fifo_clk), .clear_seen(dco_clear_seen),
    .fifo_seen(dco_fifo_seen), .ready_seen(dco_ready_seen),
    .run_seen(dco_run_seen), .data_rst_n(data_rst_n_clk)
);
always @(posedge clk_40m or negedge rst_n_40m) begin
    if (!rst_n_40m) begin
        dco_recovery_s1 <= 4'd0;
        dco_recovery_s2 <= 4'd0;
    end else begin
        dco_recovery_s1 <= {dco_run_seen,dco_ready_seen,dco_fifo_seen,dco_clear_seen};
        dco_recovery_s2 <= dco_recovery_s1;
    end
end
assign {recover_dco_run_40m,recover_dco_ready_40m,
        recover_dco_fifo_seen_40m,recover_dco_clear_seen_40m} = dco_recovery_s2;
assign recover_local_ready_40m = data_rst_n_40m && recover_rd_ready && !rd_idx_mailbox_busy;

assign data_out         = data_out_int;
assign data_out_valid   = data_out_valid_int && !recover_hold_40m && data_rst_n_40m;
assign transfer_flag    = transfer_flag_int && !recover_hold_40m;
assign data_stream_flag = data_stream_flag_int && !recover_hold_40m;
// Register only the status that leaves DCO; immediate producer hold remains local.
reg ddr_init_done_observe_clk;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) ddr_init_done_observe_clk <= 1'b0;
    else ddr_init_done_observe_clk <= ddr_init_done_sync;
end
assign ddr_init_done = ddr_init_done_observe_clk;
assign ts_req           = ts_req_d1 && !recover_hold_40m && data_rst_n_40m;
assign ts_work_mode     = ts_work_mode_d1;

//==============================================================================
// ILA observation synchronizers
//==============================================================================

//==============================================================================
// 命令解析模块
//==============================================================================
//==============================================================================
// 20MHz -> 40MHz readout-domain synchronizers
//==============================================================================
// Trig mask is a bundled snapshot, not an independent multi-bit synchronizer.
// Keep one pending snapshot while the mailbox is busy so a burst of ADC
// triggers cannot overwrite or lose the final mask state.

// Every accepted real TEST clears old trigger marks, including a repeated
// TEST in the same service round. Only the number of TEST tasks is coalesced.
// The mailbox samples the pre-edge PPS sequence atomically with the command.
assign test_cmd_clk = cmd_valid && (cmd_data[23:16] == 8'h01);
assign test_cmd_accept_clk = test_cmd_clk && !cmd_mailbox_busy_20m;
assign test_cmd_cutoff_valid_clk = test_cmd_clk && data_rst_n_clk &&
                                   ddr_init_done_sync && !recover_hold_clk;
assign cmd_payload_clk = {test_cmd_cutoff_valid_clk, pps_event_seq_ddr, cmd_data};
assign cmd_data_40m = cmd_payload_40m[23:0];

cdc_event_mailbox #(
    .PAYLOAD_WIDTH      (57)
) u_cmd_sync_20m_to_40m (
    .src_clk            (clk),
    .src_rst_n          (rst_n),
    .src_send           (cmd_valid),
    .src_payload        (cmd_payload_clk),
    .src_busy           (cmd_mailbox_busy_20m),
    .src_overflow       (cmd_mailbox_overflow_20m),
    .dst_clk            (clk_40m),
    .dst_rst_n          (rst_n_40m),
    .dst_valid          (cmd_valid_40m),
    .dst_payload        (cmd_payload_40m)
);

cdc_event_mailbox #(
    .PAYLOAD_WIDTH      (1)
) u_pps_sync_20m_to_40m (
    .src_clk            (clk),
    .src_rst_n          (data_rst_n_clk),
    .src_send           (pps_pos_edge_sync && sys_running && !recover_hold_clk),
    .src_payload        (1'b0),
    .src_busy           (pps_mailbox_busy),
    .src_overflow       (pps_mailbox_overflow_20m),
    .dst_clk            (clk_40m),
    .dst_rst_n          (data_rst_n_40m),
    .dst_valid          (pps_pos_edge_40m),
    .dst_payload        (pps_sync_dummy)
);



wire [3:0] debug_adc_boundary_control;
wire [2:0] debug_adc_word_mod8;
wire [31:0] debug_adc_group_target, debug_adc_group_accepted;
wire join_time_pending, join_index_pending;
wire [31:0] join_time_pending_seq, join_index_pending_seq, join_timeout_count;
wire history_seq_valid, history_slot_valid;
wire [31:0] history_slot_seq, history_timeout_count;
wire [7:0] read_enqueued_mask, read_committed_mask;
wire [3:0] read_wait_flags;
wire read_segment_done_seen;

always @(posedge clk_40m or negedge rst_n_40m) begin
    if (!rst_n_40m) begin
        wr_boundary_busy_40m_s1 <= 1'b0;
        wr_boundary_busy_40m_s2 <= 1'b0;
        ddr_init_done_40m_s1      <= 1'b0;
        ddr_init_done_40m_s2      <= 1'b0;






    end
    else begin
        wr_boundary_busy_40m_s1 <= wr_boundary_busy;
        wr_boundary_busy_40m_s2 <= wr_boundary_busy_40m_s1;
        ddr_init_done_40m_s1      <= ddr_init_done_observe_clk;
        ddr_init_done_40m_s2      <= ddr_init_done_40m_s1;






    end
end

assign trig_mask_40m       = 8'd0; // compatibility input; functional trigger events are tagged
assign wr_sec_idx_40m       = wr_sec_idx_40m_mailbox;
assign wr_sec_idx_prev_40m = wr_sec_idx_prev_40m_mailbox;
assign wr_boundary_busy_40m = wr_boundary_busy_40m_s2;
assign ddr_init_done_40m    = ddr_init_done_40m_s2;



cmd_parse u_cmd_parse(
    .clk                (clk),
    .rst_n              (rst_n),
    .cmd_valid          (cmd_valid),
    .cmd_data           (cmd_data),
    .work_mode          (work_mode),
    .threshold          (threshold)
);
cmd_parse u_cmd_parse_40m(
    .clk                (clk_40m),
    .rst_n              (rst_n_40m),
    .cmd_valid          (cmd_valid_40m),
    .cmd_data           (cmd_data_40m),
    .work_mode          (work_mode_40m),
    .threshold          (threshold_40m)
);

//==============================================================================
// 控制封装模块
//==============================================================================
ctrl_wrapper u_ctrl_wrapper(
    .clk                    (clk),
    .rst_n                  (data_rst_n_clk),
    .ddr_init_done_native   (init_calib_complete_wr && !recover_hold_clk),
    .wfifo_wr_rst_busy      (wfifo_wr_rst_busy),
    .pps_pulse              (pps_pos_edge_sync),
    .pack_enable            (pack_enable),
    .ddr_init_done_sync     (ddr_init_done_sync),
    .sys_running            (sys_running)
);

//==============================================================================
// ADC数据打包模块
//==============================================================================
adc_data_pack u_adc_data_pack(
    .clk            (clk),
    .rst_n          (data_rst_n_clk),
    .enable         (pack_enable && !recover_hold_clk),
    .din            (adc_data),
    .dout           (wr_data),
    .dout_valid     (wr_en),
    .debug_pack_cnt (debug_pack_cnt)
);

//==============================================================================
// 触发检测模块 (Step7新增)
//==============================================================================
trig_detect u_trig_detect(
    .clk                (clk),
    .rst_n              (data_rst_n_clk),
    .work_mode          ((sys_running && !recover_hold_clk) ? work_mode : 2'b00),
    .adc_data           (adc_data),
    .threshold          (threshold),
    .threshold_rd_idx   (wr_sec_idx_prev),
    .trigger_threshold  (completed_trigger_threshold),
    .wr_sec_idx         (wr_sec_idx),
    .wr_partition_start (wr_commit_valid_clk),
    .trig_mask          (trig_mask),
    .trig_clr_idx       (3'd0),
    .trig_clr_pulse     (1'b0),
    .trig_clr_all       (test_cmd_accept_clk),
    .debug_trig_detected(debug_trig_detected)
);

//==============================================================================
// 读控制模块
//==============================================================================
// Step3 source-side RMC time transaction. The event sequence is sampled
// only after the DDR controller has accepted the corresponding PPS.
always @(posedge clk or negedge data_rst_n_clk) begin
    if (!data_rst_n_clk || !ddr_init_done_sync) begin
        timestamp_time_payload_hold <= 98'd0;
        timestamp_time_pending      <= 1'b0;
        timestamp_time_send         <= 1'b0;
        timestamp_time_overflow     <= 1'b0;
    end else if (!recover_hold_clk) begin
        timestamp_time_send <= 1'b0;
        if (pps_event_pulse_ddr) begin
            if (timestamp_time_pending) begin
                timestamp_time_overflow <= 1'b1;
            end else begin
                // pps_event_pulse is itself a registered one-cycle event.
                // The consumer sees it on the following clk edge, after
                // wr_boundary_event_source has already updated the sequence.
                // Use the presented sequence directly so time and commit carry
                // the same PPS tag.
                timestamp_time_payload_hold <= {threshold, pps_event_seq_ddr,
                                                 rmc_date, rmc_time,
                                                 gps_lock_status};
                timestamp_time_pending <= 1'b1;
            end
        end
        if (timestamp_time_pending && !timestamp_time_busy) begin
            timestamp_time_send    <= 1'b1;
            timestamp_time_pending <= 1'b0;
        end
    end
end
assign timestamp_time_payload_clk = timestamp_time_payload_hold;

cdc_event_mailbox #(
    .PAYLOAD_WIDTH (98)
) u_timestamp_time_mailbox (
    .src_clk       (clk),
    .src_rst_n     (data_rst_n_clk),
    .src_send      (timestamp_time_send),
    .src_payload   (timestamp_time_payload_clk),
    .src_busy      (timestamp_time_busy),
    .src_overflow  (timestamp_time_mailbox_overflow),
    .dst_clk       (clk_40m),
    .dst_rst_n     (data_rst_n_40m),
    .dst_valid     (timestamp_time_valid_40m),
    .dst_payload   (timestamp_time_payload_40m)
);

timestamp_event_join #(
    .JOIN_TIMEOUT_CYCLES (20000000)
) u_timestamp_event_join (
    .clk             (clk_40m),
    .rst_n           (data_rst_n_40m),
    .time_valid      (timestamp_time_valid_40m),
    .time_seq        (timestamp_time_payload_40m[81:50]),
    .time_date       (timestamp_time_payload_40m[49:26]),
    .time_value      (timestamp_time_payload_40m[25:2]),
    .time_status     (timestamp_time_payload_40m[1:0]),
    .time_threshold  (timestamp_time_payload_40m[97:82]),
    .index_valid     (wr_boundary_commit_40m_event),
    .index_seq       (wr_boundary_commit_seq_40m),
    .index_value     (wr_boundary_commit_idx_40m),
    .index_error     (wr_boundary_commit_error_40m),
    .metadata_valid  (timestamp_metadata_valid_40m),
    .metadata_seq    (timestamp_metadata_seq_40m),
    .metadata_index  (timestamp_metadata_index_40m),
    .metadata_date   (timestamp_metadata_date_40m),
    .metadata_time   (timestamp_metadata_time_40m),
    .metadata_status (timestamp_metadata_status_40m),
    .metadata_threshold(timestamp_metadata_threshold_40m),
    .time_pending    (join_time_pending),
    .index_pending   (join_index_pending),
    .time_pending_seq(join_time_pending_seq),
    .index_pending_seq(join_index_pending_seq),
    .overflow_sticky (timestamp_join_overflow_40m),
    .mismatch_sticky (timestamp_join_mismatch_40m),
    .timeout_sticky  (timestamp_join_timeout_40m),
    .debug_timeout_count(join_timeout_count)
);

// A tagged completed-partition result replaces the unversioned mask as
// the functional trigger queue input. The old mask remains diagnostic only.
always @(posedge clk or negedge data_rst_n_clk) begin
    if (!data_rst_n_clk) begin
        trigger_event_send <= 1'b0;
        trigger_event_pending <= 1'b0;
        trigger_event_payload_hold <= 52'd0;
        trigger_event_overflow <= 1'b0;
    end else if (!ddr_init_done_sync) begin
        trigger_event_send <= 1'b0;
        trigger_event_pending <= 1'b0;
        trigger_event_payload_hold <= 52'd0;
        trigger_event_overflow <= 1'b0;
    end else begin
        trigger_event_send <= 1'b0;
        if (trigger_event_pending && !trigger_event_mailbox_busy && !trigger_event_send) begin
            trigger_event_send <= 1'b1;
            trigger_event_pending <= 1'b0;
        end
        if (wr_commit_valid_clk) begin
            if (trigger_event_pending || trigger_event_send)
                trigger_event_overflow <= 1'b1;
            else begin
                trigger_event_payload_hold <= {completed_trigger_threshold, wr_commit_seq_clk, wr_sec_idx_prev,
                                               trig_mask[wr_sec_idx_prev]};
                trigger_event_pending <= 1'b1;
            end
        end
    end
end

cdc_event_mailbox #(.PAYLOAD_WIDTH(52)) u_trigger_event_mailbox (
    .src_clk(clk), .src_rst_n(data_rst_n_clk),
    .src_send(trigger_event_send), .src_payload(trigger_event_payload_hold),
    .src_busy(trigger_event_mailbox_busy), .src_overflow(trigger_event_mailbox_overflow),
    .dst_clk(clk_40m), .dst_rst_n(data_rst_n_40m),
    .dst_valid(trigger_event_valid_40m), .dst_payload(trigger_event_payload_40m)
);

timestamp_history_ctrl #(
    .REQUEST_TIMEOUT_CYCLES (20000000),
    .MODE_TEST              (2'b01)
) u_timestamp_history_ctrl (
    .clk                 (clk_40m),
    .rst_n               (data_rst_n_40m),
    .metadata_valid      (timestamp_metadata_valid_40m),
    .metadata_seq        (timestamp_metadata_seq_40m),
    .metadata_index      (timestamp_metadata_index_40m),
    .metadata_date       (timestamp_metadata_date_40m),
    .metadata_time       (timestamp_metadata_time_40m),
    .metadata_status     (timestamp_metadata_status_40m),
    .metadata_threshold  (timestamp_metadata_threshold_40m),
    .request_valid       (ts_req_int),
    .request_cancel      (timestamp_request_cancel),
    .request_seq         (timestamp_expected_seq),
    .request_seq_valid   (timestamp_expected_seq_valid),
    .request_index       (ts_sec_idx_int),
    .request_work_mode   (ts_work_mode_int),
    .ts_req              (timestamp_barrier_req),
    .ts_index            (timestamp_barrier_index),
    .ts_work_mode        (timestamp_barrier_work_mode),
    .ts_seq              (timestamp_barrier_seq),
    .ts_date             (timestamp_barrier_date),
    .ts_time             (timestamp_barrier_time),
    .ts_status           (timestamp_barrier_status),
    .ts_threshold        (timestamp_barrier_threshold),
    .request_pending     (timestamp_barrier_pending),
    .request_overflow    (timestamp_barrier_overflow),
    .invalid_read_sticky (timestamp_barrier_invalid),
    .timeout_sticky      (timestamp_barrier_timeout),
    .request_failed      (timestamp_barrier_failed),
    .debug_pending_index (timestamp_pending_index),
    .debug_pending_seq   (timestamp_pending_seq),
    .debug_selected_slot_valid(history_slot_valid), .debug_selected_slot_seq(history_slot_seq),
    .debug_pending_seq_valid(history_seq_valid), .debug_timeout_count(history_timeout_count)
);

//==============================================================================
// A TEST intention belongs to a complete service round, independently of
// the latest mode. Preserve it in the control reset domain across data clear.
wire read_test_task_active;
wire read_test_task_done;
wire real_test_cmd_40m = cmd_valid_40m && (cmd_data_40m[23:16] == 8'h01);
wire read_command_ready = data_rst_n_40m && ddr_init_done_40m && !recover_hold_40m;
reg test_intent_live;
reg test_retry_pending;
wire test_retry_send = test_retry_pending && read_command_ready &&
                       !cmd_valid_40m && !read_test_task_done;

always @(posedge clk_40m or negedge rst_n_40m) begin
    if (!rst_n_40m) begin
        test_intent_live <= 1'b0;
        test_retry_pending <= 1'b0;
    end else begin
        if (read_test_task_done) begin
            test_intent_live <= 1'b0;
            test_retry_pending <= 1'b0;
        end
        // A real TEST exactly on retirement starts the next service round.
        if (real_test_cmd_40m) begin
            test_intent_live <= 1'b1;
            test_retry_pending <= !read_command_ready;
        end else if (!read_command_ready &&
                     (test_intent_live || debug_test_cmd_pending || read_test_task_active)) begin
            test_retry_pending <= 1'b1;
        end else if (test_retry_send) begin
            test_retry_pending <= 1'b0;
        end
    end
end

// During hold, source cutoff_valid is zero. A pre-hold valid command must
// return its mailbox ACK before the DCO ready barrier permits RELEASE.
// Thus accepting a valid cutoff only after hold_request falls excludes old
// data epochs, even when a control command crosses the data-reset interval.
// A real command can arrive during RELEASE while business is still held:
// retain that NEW-epoch cutoff and deliver it when read_ctrl can latch it.
wire real_cutoff_valid_40m = real_test_cmd_40m && cmd_payload_40m[56] &&
                             !recover_hold_request && data_rst_n_40m;
reg test_cutoff_pending;
reg [31:0] test_cutoff_pending_seq;
wire read_cmd_cutoff_valid = read_command_ready &&
                             (real_cutoff_valid_40m || test_cutoff_pending);
wire [31:0] read_cmd_cutoff_seq = real_cutoff_valid_40m ?
                                cmd_payload_40m[55:24] : test_cutoff_pending_seq;

always @(posedge clk_40m or negedge data_rst_n_40m) begin
    if (!data_rst_n_40m) begin
        test_cutoff_pending <= 1'b0;
        test_cutoff_pending_seq <= 32'd0;
    end else if (recover_hold_request) begin
        test_cutoff_pending <= 1'b0;
    end else if (real_cutoff_valid_40m) begin
        test_cutoff_pending_seq <= cmd_payload_40m[55:24];
        test_cutoff_pending <= !read_command_ready;
    end else if (read_command_ready) begin
        test_cutoff_pending <= 1'b0;
    end
end

read_ctrl u_read_ctrl(
    .clk                (clk_40m),
    .rst_n              (data_rst_n_40m),
    .recover_hold       (recover_hold_40m),
    .recover_overwrite  (recover_overwrite_40m),
    .cmd_valid          (read_command_ready && (cmd_valid_40m || test_retry_send)),
    .cmd_data           (cmd_valid_40m ? cmd_data_40m : 24'h010000),
    .cmd_cutoff_valid   (read_cmd_cutoff_valid),
    .cmd_cutoff_seq     (read_cmd_cutoff_seq),
    .test_task_active   (read_test_task_active),
    .test_task_done     (read_test_task_done),
    .pps_pulse          (pps_pos_edge_40m),
    .ddr_init_done      (ddr_init_done_40m),
    .wr_boundary_busy   (wr_boundary_busy_40m),
    .wr_boundary_commit (wr_boundary_commit_40m_event),
    .wr_boundary_commit_idx (wr_boundary_commit_idx_40m),
    .wr_boundary_commit_seq (wr_boundary_commit_seq_40m),
    .trigger_event_valid(trigger_event_valid_40m),
    .trigger_event_seq(trigger_event_payload_40m[35:4]),
    .trigger_event_index(trigger_event_payload_40m[3:1]),
    .trigger_event_flag(trigger_event_payload_40m[0]),
    .trigger_event_threshold(trigger_event_payload_40m[51:36]),
    .ts_metadata_valid  (timestamp_barrier_req),
    .ts_metadata_index  (timestamp_barrier_index),
    .ts_metadata_seq    (timestamp_barrier_seq),
    .ts_request_failed  (timestamp_barrier_failed),
    .ts_request_cancel  (timestamp_request_cancel),
    .rd_index_busy      (rd_idx_mailbox_busy),
    // Step8新增：工作模式和触发掩码输入
    .work_mode          (work_mode_40m),
    .ts_send_done       (ts_send_done),
    .ts_channel_busy    (timestamp_barrier_pending || ts_transaction_busy || ts_mailbox_busy_40m || ts_req_d1),
    .segment_send_done    (segment_send_done),
    .segment_send_done_seq(segment_send_done_seq),
    .trig_mask          (trig_mask_40m),
    // FIFO状态
    .rfifo_rd_rst_busy  (rfifo_rd_rst_busy),
    .rd_data_valid      (rd_data_valid),
    .rd_req             (rd_req),
    .rfifo_rcount       (debug_rfifo_rcount),
    .fifo_flush         (fifo_flush),
    // 分区索引
    .wr_sec_idx         (wr_sec_idx_40m),
    .wr_sec_idx_prev    (wr_sec_idx_prev_40m),
    .rd_addr_done       (rd_addr_done),
    // 输出
    .ddr3_read_valid    (ddr3_read_valid),
    .rd_sec_idx         (rd_sec_idx),
    .rd_sec_idx_valid        (rd_sec_idx_valid),
    .transfer_flag      (transfer_flag_int),
    .ts_req             (ts_req_int),
    .ts_sec_idx         (ts_sec_idx_int),
    .ts_work_mode       (ts_work_mode_int),
    .ts_trigger_threshold(task_trigger_threshold),
    .ts_expected_seq    (timestamp_expected_seq),
    .ts_expected_seq_valid(timestamp_expected_seq_valid),
    .debug_active_seq   (read_active_seq),
    .debug_segment_done_seen(read_segment_done_seen),
    .debug_wait_flags(read_wait_flags), .debug_enqueued_mask(read_enqueued_mask),
    .debug_committed_mask(read_committed_mask),
    .debug_abort_sticky (read_abort_sticky),
    // Step8新增：触发清除输出
    .trig_clr_idx       (trig_clr_idx_40m),
    .trig_clr_pulse     (trig_clr_pulse_40m),
    // 调试
    .debug_state        (debug_state),
    .debug_rd_data_cnt  (debug_rd_data_cnt),
    .debug_rd_addr_done_sync (debug_rd_addr_done_sync),
    .debug_test_cmd_pending  (debug_test_cmd_pending),
    .debug_pending_mask      (debug_pending_mask),
    .debug_pending_valid     (debug_pending_valid),
    .debug_pending_rd_idx    (debug_pending_rd_idx)
);

//==============================================================================
// Timestamp latch module
//==============================================================================
// Step3 timestamp outputs are emitted only after the history sequence barrier.
assign ts_sec_idx         = timestamp_barrier_index;
assign ts_date            = timestamp_barrier_date;
assign ts_time            = timestamp_barrier_time;
assign ts_gps_lock_status = timestamp_barrier_status;

//==============================================================================
// FPGA die temperature metadata latch module
//==============================================================================
mig_temp_monitor u_mig_temp_monitor(
    .clk                 (clk_40m),
    .rst_n               (rst_n_40m),
    .init_calib_complete (init_calib_complete),
    .mig_device_temp     (mig_device_temp),
    .temp_raw            (current_temp_raw),
    .temp_valid          (current_temp_valid),
    .temp_update_pulse   (current_temp_update_pulse),
    .temp_stale          (current_temp_stale),
    .temp_alarm          (current_temp_alarm)
);

temp_history_ctrl u_temp_history_ctrl(
    .clk                (clk_40m),
    .rst_n              (data_rst_n_40m),
    // Align temperature capture with the tagged DDR completion event.
    // Raw PPS may precede drain completion and cannot identify the completed slot.
    .pps_pulse          (wr_boundary_commit_40m_event),
    .wr_sec_idx_prev    (wr_boundary_commit_idx_40m),
    .temp_raw           (current_temp_raw),
    .temp_valid         (current_temp_valid),
    .temp_stale         (current_temp_stale),
    .temp_alarm         (current_temp_alarm),
    .rd_sec_idx         (timestamp_barrier_index),
    .rd_sec_valid       (timestamp_barrier_req),
    .ts_temp_raw        (ts_temp_raw),
    .ts_temp_flags      (ts_temp_flags)
);
always @(posedge clk_40m or negedge data_rst_n_40m) begin
    if (!data_rst_n_40m) begin
        ts_req_d1       <= 1'b0;
        ts_work_mode_d1 <= 2'd0;
        ts_threshold <= 16'hffff;
        ts_transaction_busy <= 1'b0;
    end
    else begin
        ts_req_d1       <= timestamp_barrier_req && !recover_hold_40m;
        ts_work_mode_d1 <= timestamp_barrier_work_mode;
        if (timestamp_barrier_req && !recover_hold_40m) begin
            // Bind the sideband to the same emitted transaction as temperature.
            ts_threshold <= (timestamp_barrier_work_mode == 2'b10) ?
                            task_trigger_threshold : timestamp_barrier_threshold;
        end
        // mailbox ACK only means payload delivery, not completion of the frame.
        // Hold preserves an emitted timestamp until TX quiesces.
        // Wait for the old actual TX done to return before selecting a new task.
        if (ts_req_d1) ts_transaction_busy <= 1'b1;
        else if (ts_send_done) ts_transaction_busy <= 1'b0;
    end
end
//==============================================================================
// 数据解包模块
//==============================================================================
data_out_unpack u_data_out_unpack(
    .clk                (clk_40m),
    .rst_n              (data_rst_n_40m),
    .transfer_flag      (transfer_flag_int && !recover_hold_40m),
    .rd_addr_done_sync  (debug_rd_addr_done_sync),  // 新增：DDR3读地址完成（已同步）
    .din                (rd_data),
    .din_valid          (rd_data_valid && !recover_hold_40m),
    .dout               (data_out_int),
    .dout_valid         (data_out_valid_int),
    .rd_req             (rd_req),
    .data_stream_flag   (data_stream_flag_int),
    .fifo_flush         (fifo_flush),               // 新增：FIFO刷新请求输出
    .debug_idx          (debug_idx),
    .debug_sample_cnt   (debug_sample_cnt),
    // 新增：分层排空调试信号
    .debug_transfer_active  (debug_unpack_transfer_active),
    .debug_transfer_start   (debug_unpack_transfer_start),
    .debug_done             (debug_unpack_done),
    .debug_fifo_wr_en       (debug_unpack_fifo_wr_en),
    .debug_fifo_rd_en       (debug_unpack_fifo_rd_en),
    .debug_fifo_full        (debug_unpack_fifo_full),
    .debug_fifo_empty       (debug_unpack_fifo_empty),
    .debug_fifo_valid       (debug_unpack_fifo_valid),
    .debug_fifo_rd_count    (debug_unpack_fifo_rd_count),
    .debug_fifo_wr_count    (debug_unpack_fifo_wr_count)
);

//==============================================================================
// DDR3控制器
//==============================================================================
ddr3_controler u_ddr3_controler(
    .recover_hold_request(recover_hold_request),
    .recover_clear_request(recover_clear_request),
    .recover_fifo_request(recover_fifo_request), .mig_sys_rst_n(recover_mig_rst_n),
    .recover_hold_wr(recover_hold_clk), .recover_hold_rd(recover_hold_40m),
    .data_rst_n_wr(data_rst_n_clk), .data_rst_n_rd(data_rst_n_40m),
    .capture_started_clk(sys_running),
    .recover_ui_clear_seen_40m(recover_ui_clear_seen_40m),
    .recover_ui_fifo_seen_40m(recover_ui_fifo_seen_40m),
    .recover_ui_ready_40m(recover_ui_ready_40m),
    .recover_ui_run_40m(recover_ui_run_40m),
    .recover_calib_40m(recover_calib_40m),
    .recover_calib_lost_40m(recover_calib_lost_40m),
    .recover_wr_ready(recover_wr_ready), .recover_rd_ready(recover_rd_ready),
    // DDR3物理接口
    .ddr3_dq                (ddr3_dq),
    .ddr3_dqs_n             (ddr3_dqs_n),
    .ddr3_dqs_p             (ddr3_dqs_p),
    .ddr3_addr              (ddr3_addr),
    .ddr3_ba                (ddr3_ba),
    .ddr3_ras_n             (ddr3_ras_n),
    .ddr3_cas_n             (ddr3_cas_n),
    .ddr3_we_n              (ddr3_we_n),
    .ddr3_reset_n           (ddr3_reset_n),
    .ddr3_ck_p              (ddr3_ck_p),
    .ddr3_ck_n              (ddr3_ck_n),
    .ddr3_cke               (ddr3_cke),
    .ddr3_cs_n              (ddr3_cs_n),
    .ddr3_dm                (ddr3_dm),
    .ddr3_odt               (ddr3_odt),
    
    .init_calib_complete    (init_calib_complete),
    .init_calib_complete_wr (init_calib_complete_wr),
    .device_temp            (mig_device_temp),
    
    .sys_clk_i              (clk_200m),
    .clk_ref_i              (clk_200m),
    .rst_n                  (rst_n),
    
    .pps_pulse              (pps_pos_edge_sync),
    
    .rd_bust_len            (RD_BURST_LEN),
    .wr_bust_len            (WR_BURST_LEN),
    
    .ddr3_read_valid        (ddr3_read_valid),
    .rd_sec_idx             (rd_sec_idx),
    .rd_sec_idx_valid        (rd_sec_idx_valid),
    .wr_sec_idx             (wr_sec_idx),
    .wr_sec_idx_prev        (wr_sec_idx_prev),
    .wr_commit_valid_clk    (wr_commit_valid_clk),
    .wr_commit_seq_clk      (wr_commit_seq_clk),
    .debug_adc_boundary_control(debug_adc_boundary_control),
    .debug_adc_word_mod8(debug_adc_word_mod8), .debug_adc_group_target(debug_adc_group_target),
    .debug_adc_group_accepted(debug_adc_group_accepted),
    .debug_ila_ui_heartbeat(debug_ila_ui_heartbeat),
    .wr_sec_idx_40m         (wr_sec_idx_40m_mailbox),
    .wr_sec_idx_prev_40m    (wr_sec_idx_prev_40m_mailbox),
    
    .debug_app_addr         (debug_app_addr),
    .debug_app_addr_rd_min  (debug_app_addr_rd_min),
    .debug_app_addr_wr_min  (debug_app_addr_wr_min),
    .debug_app_addr_rd      (debug_app_addr_rd),
    .debug_app_addr_wr      (debug_app_addr_wr),
    .debug_wr_sec_switch    (debug_wr_sec_switch),
    
    .wr_clk                 (clk),
    .rd_clk                 (clk_40m),
    .wr_en                  (wr_en && !recover_hold_clk),
    .wrdata                 (wr_data),
    .wr_pack_count          (debug_pack_cnt),
    .rd_req                 (rd_req && !recover_hold_40m),
    .fifo_flush             (fifo_flush),            // 新增：FIFO刷新请求
    .rddata                 (rd_data),
    .wfifo_wr_rst_busy      (wfifo_wr_rst_busy),
    .rfifo_rd_rst_busy      (rfifo_rd_rst_busy),
    .rd_data_valid          (rd_data_valid),
    .debug_wfifo_rcount     (debug_wfifo_rcount),
    .debug_app_rd_en        (debug_app_rd_en),
    .debug_app_wr_en        (debug_app_wr_en),
    .debug_rfifo_rcount     (debug_rfifo_rcount),
    .wr_boundary_busy       (wr_boundary_busy),
    .pps_event_pulse        (pps_event_pulse_ddr),
    .pps_event_seq          (pps_event_seq_ddr),
    .rd_idx_mailbox_busy     (rd_idx_mailbox_busy),
    .rd_idx_mailbox_overflow (rd_idx_mailbox_overflow),
    .boundary_mailbox_busy  (boundary_mailbox_busy),
    .boundary_mailbox_overflow(boundary_mailbox_overflow),
    .wr_boundary_commit_40m (wr_boundary_commit_40m_event),
    .wr_boundary_commit_seq_40m(wr_boundary_commit_seq_40m),
    .wr_boundary_commit_idx_40m(wr_boundary_commit_idx_40m),
    .wr_boundary_commit_error_40m(wr_boundary_commit_error_40m),
    .index_mailbox_busy     (index_mailbox_busy),
    .index_mailbox_overflow (index_mailbox_overflow),
    .boundary_protocol_error(boundary_protocol_error),
    .wfifo_overflow_latched (wfifo_overflow_latched),
    .rd_addr_done           (rd_addr_done),          // 新增
    .debug_rd_addr_cnt      (debug_rd_addr_cnt),     // 新增
    .debug_state_cnt        (debug_state_cnt),       // 新增
    .debug_rfifo_wcount     (debug_rfifo_wcount)     // 新增
);

//==============================================================================
// 临时 ILA-A：ADC两级采样与16位到64位打包边界（20MHz ADC采集域）
//==============================================================================

//==============================================================================
// 临时 ILA-B：DDR读出与64位到16位解包边界（40MHz读出域）
//==============================================================================

//==============================================================================
// 固定 ILA：ADC 原生 20 MHz；元数据关联与任务控制原生 40 MHz
// 事件采样保留事务脉冲、握手变化和等待状态，避免观测跨域多位快照。
//==============================================================================

wire [15:0] debug_ila_adc_ms, debug_ila_join_ms, debug_ila_task_ms;
wire debug_ila_adc_event, debug_ila_join_event, debug_ila_task_event;
reg [31:0] debug_task_wait_age;
reg [2:0] debug_task_state_prev;
reg [7:0] debug_task_errors_prev;
reg [6:0] debug_task_handshake_prev;
wire [6:0] debug_task_handshake = {ts_transaction_busy,rd_idx_mailbox_busy,read_segment_done_seen,read_wait_flags};
reg [2:0] debug_join_errors_prev;
reg [1:0] debug_join_pending_prev;
wire [7:0] debug_task_errors = {ts_mailbox_overflow_40m, rd_idx_mailbox_overflow,
    read_abort_sticky, timestamp_barrier_timeout, timestamp_barrier_invalid,
    timestamp_barrier_overflow, timestamp_barrier_failed, timestamp_request_cancel};
wire [2:0] debug_join_errors = {timestamp_join_timeout_40m,
    timestamp_join_mismatch_40m, timestamp_join_overflow_40m};
assign ts_event_seq = timestamp_barrier_seq;
assign debug_ila_read_control = {data_stream_flag_int,fifo_flush,debug_rd_addr_done_sync,
    data_out_valid_int,debug_unpack_fifo_valid,debug_unpack_fifo_empty,
    debug_unpack_fifo_full,debug_unpack_fifo_rd_en,debug_unpack_fifo_wr_en,
    rd_data_valid,rd_req,debug_unpack_transfer_active,debug_unpack_transfer_start,
    transfer_flag_int,rfifo_rd_rst_busy,rst_n_40m};
assign debug_ila_read_identity = {read_active_seq,rd_sec_idx};
assign debug_ila_read_word = rd_data;
assign debug_ila_read_count = {debug_rfifo_rcount,debug_rd_data_cnt};
assign debug_ila_unpack_level = {debug_unpack_fifo_wr_count,debug_unpack_fifo_rd_count};
assign debug_ila_read_sample = {debug_sample_cnt,data_out_int};
assign debug_ila_gps_events = {trigger_event_overflow | trigger_event_mailbox_overflow,
    trigger_event_mailbox_busy,pps_mailbox_overflow_20m,cmd_mailbox_overflow_20m,
    debug_trig_detected};
assign debug_ila_time_transaction_seq = timestamp_time_payload_hold[81:50];
assign debug_ila_command_state = {trig_mask,threshold,work_mode};
always @(posedge clk_40m or negedge rst_n_40m) begin
    if (!rst_n_40m) begin
        debug_task_wait_age <= 0;
        debug_task_state_prev <= 0;
        debug_task_errors_prev <= 0;
        debug_task_handshake_prev <= 0;
        debug_join_errors_prev <= 0;
        debug_join_pending_prev <= 0;
    end else begin
        if ((debug_state != 3'd6 && debug_state != 3'd5 && !ts_transaction_busy) || !ddr_init_done_40m ||
            debug_state != debug_task_state_prev) debug_task_wait_age <= 0;
        else if (debug_task_wait_age != 32'hffffffff)
            debug_task_wait_age <= debug_task_wait_age + 1'b1;
        debug_task_state_prev <= debug_state;
        debug_task_errors_prev <= debug_task_errors;
        debug_task_handshake_prev <= debug_task_handshake;
        debug_join_errors_prev <= debug_join_errors;
        debug_join_pending_prev <= {join_index_pending,join_time_pending};
    end
end
ila_capture_control #(.CLK_HZ(20000000)) u_ila_adc_capture (
    .clk(clk), .rst_n(rst_n),
    .event_in(pps_event_pulse_ddr || debug_adc_boundary_control[3] ||
              timestamp_time_send),
    .wait_active(1'b1), .capture_event(debug_ila_adc_event), .timestamp_ms(debug_ila_adc_ms));
ila_capture_control #(.CLK_HZ(40000000)) u_ila_join_capture (
    .clk(clk_40m), .rst_n(rst_n_40m),
    .event_in(timestamp_time_valid_40m || wr_boundary_commit_40m_event ||
              timestamp_metadata_valid_40m || debug_join_errors != debug_join_errors_prev ||
              {join_index_pending,join_time_pending} != debug_join_pending_prev),
    .wait_active(join_time_pending || join_index_pending),
    .capture_event(debug_ila_join_event), .timestamp_ms(debug_ila_join_ms));
ila_capture_control #(.CLK_HZ(40000000)) u_ila_task_capture (
    .clk(clk_40m), .rst_n(rst_n_40m),
    .event_in(rd_sec_idx_valid || segment_send_done || ts_req_int || timestamp_barrier_req || ts_send_done ||
              trigger_event_valid_40m || debug_state != debug_task_state_prev ||
              debug_task_errors != debug_task_errors_prev ||
              debug_task_handshake != debug_task_handshake_prev),
    .wait_active(ts_transaction_busy || debug_state == 3'd6 || debug_state == 3'd5),
    .capture_event(debug_ila_task_event), .timestamp_ms(debug_ila_task_ms));
`ifdef SYNTHESIS
ila_prod_adc_v1 u_ila_prod_adc_v1 (
    .clk(clk),
    .probe0({timestamp_time_mailbox_overflow,timestamp_time_overflow,timestamp_time_busy,
        timestamp_time_send,timestamp_time_pending,boundary_mailbox_overflow,
        wfifo_overflow_latched,boundary_mailbox_busy,debug_adc_boundary_control[3:2],
        pps_event_pulse_ddr,debug_adc_boundary_control[1],wfifo_wr_rst_busy,
        debug_adc_boundary_control[0],wr_en,pack_enable,ddr_init_done_sync,rst_n}),
    .probe1(adc_data), .probe2(wr_data), .probe3({debug_adc_word_mod8,debug_pack_cnt}),
    .probe4(pps_event_seq_ddr), .probe5(debug_adc_group_target),
    .probe6(debug_adc_group_accepted), .probe7({debug_ila_adc_ms,debug_ila_adc_event}));
ila_prod_join_v1 u_ila_prod_join_v1 (
    .clk(clk_40m),
    .probe0({timestamp_join_timeout_40m,timestamp_join_mismatch_40m,timestamp_join_overflow_40m,
        timestamp_metadata_valid_40m,join_index_pending,join_time_pending,
        wr_boundary_commit_error_40m,wr_boundary_commit_40m_event,timestamp_time_valid_40m,
        ddr_init_done_40m,rst_n_40m}),
    .probe1(timestamp_time_payload_40m[81:50]), .probe2(wr_boundary_commit_seq_40m),
    .probe3(join_time_pending_seq), .probe4(join_index_pending_seq), .probe5(join_timeout_count),
    .probe6(timestamp_metadata_seq_40m),
    .probe7({timestamp_metadata_status_40m,timestamp_time_payload_40m[1:0],
             timestamp_metadata_index_40m,wr_boundary_commit_idx_40m}),
    .probe8(timestamp_metadata_date_40m), .probe9(timestamp_metadata_time_40m),
    .probe10({debug_ila_join_ms,debug_ila_join_event}));
ila_prod_task_v1 u_ila_prod_task_v1 (
    .clk(clk_40m),
    .probe0({ts_transaction_busy,ts_mailbox_overflow_40m,ts_mailbox_busy_40m,ts_req,debug_pending_valid,
        debug_test_cmd_pending,trigger_event_valid_40m,rd_idx_mailbox_overflow,
        rd_idx_mailbox_busy,wr_boundary_busy_40m,ts_send_done,read_wait_flags,
        timestamp_request_cancel,read_abort_sticky,timestamp_barrier_timeout,
        timestamp_barrier_invalid,timestamp_barrier_overflow,timestamp_barrier_failed,
        timestamp_barrier_req,history_seq_valid,timestamp_barrier_pending,
        timestamp_expected_seq_valid,ts_req_int,ddr_init_done_40m,rst_n_40m}),
    .probe1({timestamp_expected_seq,ts_work_mode_int,ts_sec_idx_int}),
    .probe2({timestamp_pending_seq,timestamp_pending_index}),
    .probe3({history_slot_seq,history_slot_valid}),
    .probe4({timestamp_barrier_seq,timestamp_barrier_status,timestamp_barrier_index}),
    .probe5({rd_sec_idx_valid,read_active_seq,rd_sec_idx}),
    .probe6({read_enqueued_mask,read_committed_mask,debug_pending_rd_idx,debug_pending_mask,debug_state}),
    .probe7(history_timeout_count), .probe8(debug_task_wait_age),
    .probe9({debug_ila_task_ms,debug_ila_task_event}),
    .probe10(trigger_event_payload_40m[35:0]),
    .probe11({read_segment_done_seen,segment_send_done,segment_send_done_seq}));
`endif
endmodule
