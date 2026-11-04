`timescale 1ns / 1ps
//******************************************************************************
//  File Name:   eth_cmd_parse.v
//  Module:      eth_cmd_parse
//  Description: UDP command parser
//
//  External UDP payload protocol:
//  - 1 byte:
//      0x01       test mode
//      0x02       trigger mode
//  - 3 bytes:
//      0x80 HH LL threshold update, threshold = {HH, LL} * 10
//
//  Internal command format:
//      cmd_data[23:16] = cmd_id
//      cmd_data[15:0]  = cmd_arg
//
//  Clock domain: gmii_rx_clk (125MHz)
//******************************************************************************

module eth_cmd_parse(
    // system signals
    input   wire                clk,
    input   wire                rst_n,

    // UDP receive interface
    input   wire                udp_rec_pkt_done,
    input   wire                udp_rec_en,
    input   wire    [7:0]       udp_rec_data,
    input   wire    [15:0]      udp_rec_byte_num,

    // command output, 125MHz domain
    output  reg                 cmd_valid,
    output  reg     [23:0]      cmd_data
);

localparam CMD_TEST      = 8'h01;
localparam CMD_TRIG      = 8'h02;
localparam CMD_THRESHOLD = 8'h80;

reg     [7:0]       rx_byte0;
reg     [7:0]       rx_byte1;
reg     [7:0]       rx_byte2;
reg     [1:0]       rx_byte_cnt;
reg     [15:0]      pkt_len_hold;
reg                 parse_pending;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cmd_valid     <= 1'b0;
        cmd_data      <= 24'h00_0000;
        rx_byte0      <= 8'h00;
        rx_byte1      <= 8'h00;
        rx_byte2      <= 8'h00;
        rx_byte_cnt   <= 2'd0;
        pkt_len_hold  <= 16'd0;
        parse_pending <= 1'b0;
    end
    else begin
        cmd_valid     <= 1'b0;
        parse_pending <= 1'b0;

        if (udp_rec_en) begin
            case (rx_byte_cnt)
                2'd0: rx_byte0 <= udp_rec_data;
                2'd1: rx_byte1 <= udp_rec_data;
                2'd2: rx_byte2 <= udp_rec_data;
                default: ;
            endcase

            if (rx_byte_cnt != 2'd3) begin
                rx_byte_cnt <= rx_byte_cnt + 1'b1;
            end
        end

        // Parse one cycle after packet done so the final payload byte has been
        // registered even when udp_rec_pkt_done overlaps the last udp_rec_en.
        if (udp_rec_pkt_done) begin
            pkt_len_hold  <= udp_rec_byte_num;
            parse_pending <= 1'b1;
            rx_byte_cnt   <= 2'd0;
        end

        if (parse_pending) begin
            case (pkt_len_hold)
                16'd1: begin
                    case (rx_byte0)
                        CMD_TEST,
                        CMD_TRIG: begin
                            cmd_valid <= 1'b1;
                            cmd_data  <= {rx_byte0, 16'h0000};
                        end
                        default: ;
                    endcase
                end

                16'd3: begin
                    if (rx_byte0 == CMD_THRESHOLD) begin
                        cmd_valid <= 1'b1;
                        cmd_data  <= {CMD_THRESHOLD, rx_byte1, rx_byte2};
                    end
                end

                default: ;
            endcase
        end
    end
end

endmodule
