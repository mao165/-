`timescale 1ns/1ps

// One-entry bundled-data CDC mailbox.
//
// The source payload is held from request acceptance until the synchronized
// ACK returns. Only request/ACK/arming flags pass through two-flop
// synchronizers; the multi-bit payload is sampled in the destination after the
// request delay. A source send while busy is rejected and latched as overflow.
module cdc_event_mailbox #(
    parameter integer PAYLOAD_WIDTH = 1
) (
    input  wire                     src_clk,
    input  wire                     src_rst_n,
    input  wire                     src_send,
    input  wire [PAYLOAD_WIDTH-1:0] src_payload,
    output wire                     src_busy,
    output reg                      src_overflow,

    input  wire                     dst_clk,
    input  wire                     dst_rst_n,
    output reg                      dst_valid,
    output reg  [PAYLOAD_WIDTH-1:0] dst_payload
);

reg [PAYLOAD_WIDTH-1:0] payload_hold;
reg                     req_toggle;
reg                     src_armed;
(* ASYNC_REG = "TRUE" *) reg ack_sync_1;
(* ASYNC_REG = "TRUE" *) reg ack_sync_2;
(* ASYNC_REG = "TRUE" *) reg dst_ready_sync_1;
(* ASYNC_REG = "TRUE" *) reg dst_ready_sync_2;

(* ASYNC_REG = "TRUE" *) reg req_sync_1;
(* ASYNC_REG = "TRUE" *) reg req_sync_2;
(* ASYNC_REG = "TRUE" *) reg src_armed_sync_1;
(* ASYNC_REG = "TRUE" *) reg src_armed_sync_2;
reg ack_toggle;
reg dst_ready;
reg last_req_toggle;
reg [1:0] arm_count;

assign src_busy = !dst_ready_sync_2 || (req_toggle != ack_sync_2);

always @(posedge src_clk or negedge src_rst_n) begin
    if (!src_rst_n) begin
        payload_hold     <= {PAYLOAD_WIDTH{1'b0}};
        req_toggle       <= 1'b0;
        src_armed        <= 1'b0;
        ack_sync_1       <= 1'b0;
        ack_sync_2       <= 1'b0;
        dst_ready_sync_1 <= 1'b0;
        dst_ready_sync_2 <= 1'b0;
        src_overflow     <= 1'b0;
    end else begin
        src_armed        <= 1'b1;
        ack_sync_1       <= ack_toggle;
        ack_sync_2       <= ack_sync_1;
        dst_ready_sync_1 <= dst_ready;
        dst_ready_sync_2 <= dst_ready_sync_1;

        if (src_send) begin
            if (src_busy) begin
                src_overflow <= 1'b1;
            end else begin
                payload_hold <= src_payload;
                req_toggle   <= ~req_toggle;
            end
        end
    end
end

always @(posedge dst_clk or negedge dst_rst_n) begin
    if (!dst_rst_n) begin
        req_sync_1       <= 1'b0;
        req_sync_2       <= 1'b0;
        src_armed_sync_1 <= 1'b0;
        src_armed_sync_2 <= 1'b0;
        ack_toggle       <= 1'b0;
        dst_ready        <= 1'b0;
        last_req_toggle  <= 1'b0;
        arm_count        <= 2'd0;
        dst_valid        <= 1'b0;
        dst_payload      <= {PAYLOAD_WIDTH{1'b0}};
    end else begin
        req_sync_1       <= req_toggle;
        req_sync_2       <= req_sync_1;
        src_armed_sync_1 <= src_armed;
        src_armed_sync_2 <= src_armed_sync_1;
        dst_valid        <= 1'b0;

        if (!src_armed_sync_2) begin
            // Rebuild a request baseline while the source is in reset.
            dst_ready       <= 1'b0;
            last_req_toggle <= req_sync_2;
            ack_toggle      <= req_sync_2;
            arm_count       <= 2'd0;
        end else if (!dst_ready) begin
            // Require a stable armed indication before accepting toggles.
            last_req_toggle <= req_sync_2;
            ack_toggle      <= req_sync_2;
            if (arm_count == 2'd2) begin
                dst_ready <= 1'b1;
            end else begin
                arm_count <= arm_count + 1'b1;
            end
        end else if (req_sync_2 != last_req_toggle) begin
            dst_payload     <= payload_hold;
            dst_valid       <= 1'b1;
            last_req_toggle <= req_sync_2;
            ack_toggle      <= req_sync_2;
        end
    end
end

endmodule

