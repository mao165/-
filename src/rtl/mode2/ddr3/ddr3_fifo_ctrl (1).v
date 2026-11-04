
`timescale 1ns / 1ps
module ddr3_fifo_ctrl(
    input wire recover_fifo_request,
    input wire global_rst_n_ui,
    input wire recover_hold_wr, recover_hold_rd, recover_hold_ui,
    output wire wfifo_rd_rst_busy,
    output wire wfifo_reset_active, rfifo_reset_active,
    output wire wfifo_reset_active_ui,
    input               rst_n            ,  //复位信号
    input               wr_clk           ,  //写fifo写时钟
    input               rd_clk           ,  //读fifo读时钟
    input               ui_clk           ,  //用户时钟
    input               wr_en            ,  //数据有效使能信号
    input  [63:0]       wrdata           ,  //写有效数据
    input  [511:0]      rfifo_din        ,  //用户读数据
    input               rdata_req        ,  //读数据请求信号
    input               rfifo_wren       ,  //从ddr3读出数据的有效使能
    input               wfifo_rden       ,  //wfifo读使能
    input               fifo_flush       ,  //FIFO刷新请求（20M样本完成后触发，复位rd_fifo）

    output [511:0]      wfifo_dout       ,  //用户写数据
    output [9:0]        rfifo_wcount     ,  //rfifo写侧数据计数 (XCI输出9位，接口零扩展为10位)
    output [12:0]       rfifo_rcount     ,  //rfifo读侧数据计数 (XCI输出12位，接口零扩展为13位)
    output [5:0]        wfifo_rcount     ,  //wfifo写进数据计数
    output              wfifo_full       ,  //写FIFO满，边界计数只统计实际接收数据
    output [63:0]       rddata           ,  //读有效数据
    output              wfifo_wr_rst_busy,
    output              rfifo_wr_rst_busy,
    output              rfifo_rd_rst_busy,
    output              rfifo_empty      ,  //读FIFO空标志
    output              rfifo_valid         //读FIFO valid标志（FWFT：dout有效）

    );

// rd_fifo 复位信号：系统复位 或 fifo_flush 触发
// The FIFO Generator safety circuit requires reset >= 3 slow-clock periods.
// Assert asynchronously; release after four rd_clk edges, even if rd_clk stops.
wire rfifo_reset_req = ~rst_n || fifo_flush || recover_fifo_request;
(* ASYNC_REG = "TRUE" *) reg [3:0] rfifo_reset_hold;
always @(posedge rd_clk or posedge rfifo_reset_req) begin
    if (rfifo_reset_req)
        rfifo_reset_hold <= 4'b1111;
    else
        rfifo_reset_hold <= {rfifo_reset_hold[2:0], 1'b0};
end
wire rfifo_rst = rfifo_reset_hold[3];
wire rfifo_write_enable = rfifo_wren && !rfifo_rst && !rfifo_wr_rst_busy && !recover_hold_ui;
wire rfifo_read_enable = rdata_req && !rfifo_rst && !rfifo_rd_rst_busy && !recover_hold_rd;

// FIFO IP 端口
wire [11:0] rfifo_rcount_ip;
wire [8:0] rfifo_wcount_ip;
// Preserve the existing wrapper widths; the controlled XCI exports 12/9 bits.
assign rfifo_rcount = {1'b0, rfifo_rcount_ip};
assign rfifo_wcount = {1'b0, rfifo_wcount_ip};
wire valid;
assign rfifo_valid = valid && !rfifo_rst && !rfifo_rd_rst_busy && !recover_hold_rd;

rd_fifo u_rd_fifo (
  .rst               (rfifo_rst),         // 慢时钟保持后释放
  .wr_clk            (ui_clk),
  .rd_clk            (rd_clk),
  .din               (rfifo_din),
  .wr_en             (rfifo_write_enable),
  .rd_en             (rfifo_read_enable),
  .dout              (rddata),
  .full              (),
  .empty             (rfifo_empty),
  .valid             (valid),
  .rd_data_count     (rfifo_rcount_ip),
  .wr_data_count     (rfifo_wcount_ip),
  .wr_rst_busy       (rfifo_wr_rst_busy),
  .rd_rst_busy       (rfifo_rd_rst_busy)
);


// Write FIFO reset is separate from the data epoch clear and spans at
// least three complete periods of the slow DCO clock (four rising edges).
wire wfifo_reset_req = ~rst_n || recover_fifo_request;
(* ASYNC_REG = "TRUE" *) reg [3:0] wfifo_reset_hold;
always @(posedge wr_clk or posedge wfifo_reset_req) begin
    if (wfifo_reset_req) wfifo_reset_hold <= 4'b1111;
    else wfifo_reset_hold <= {wfifo_reset_hold[2:0],1'b0};
end
wire wfifo_rst = wfifo_reset_hold[3];
assign wfifo_reset_active = wfifo_rst;
assign rfifo_reset_active = rfifo_rst;
wire wfifo_write_enable = wr_en && !wfifo_rst && !wfifo_wr_rst_busy && !recover_hold_wr;
// Physical reset deasserts in DCO; ordinary UI logic consumes only this
// single-bit status synchronizer. Global UI reset asserts asynchronously and
// releases synchronously. Recovery hold is already high before FIFO reset;
// ready stays blocked until this status and the IP read-domain busy release.
(* ASYNC_REG = "TRUE" *) reg wfifo_reset_ui_s1, wfifo_reset_ui_s2;
always @(posedge ui_clk or negedge global_rst_n_ui) begin
    if (!global_rst_n_ui) begin
        wfifo_reset_ui_s1 <= 1'b1;
        wfifo_reset_ui_s2 <= 1'b1;
    end else begin
        wfifo_reset_ui_s1 <= wfifo_rst;
        wfifo_reset_ui_s2 <= wfifo_reset_ui_s1;
    end
end
assign wfifo_reset_active_ui = wfifo_reset_ui_s2;
wire wfifo_read_enable = wfifo_rden && !wfifo_reset_ui_s2 &&
                         !wfifo_rd_rst_busy && !recover_hold_ui;

wr_fifo u_wr_fifo (
  .rst               (wfifo_rst),
  .wr_clk            (wr_clk),
  .rd_clk            (ui_clk),
  .din               (wrdata),
  .wr_en             (wfifo_write_enable),
  .rd_en             (wfifo_read_enable),
  .dout              (wfifo_dout ),
  .full              (wfifo_full),
  .empty             (),
  .rd_data_count     (wfifo_rcount),
  .wr_data_count     (),
  .wr_rst_busy       (wfifo_wr_rst_busy),
  .rd_rst_busy       (wfifo_rd_rst_busy)
);

endmodule
