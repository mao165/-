`timescale 1ns / 1ps

// GPS timestamp guard
//
// All inputs and outputs belong to the same 20 MHz GPS clock domain.  An RMC
// commit is kept as a pending UTC candidate and never changes public time.
// Only a rising physical PPS qualified as LOCKED creates guard_tick and a
// public timestamp boundary.  The pending candidate describes the previous
// PPS, so its predicted timestamp for this boundary is UTC+8 plus OFFSET.
// Date values use the public protocol order {year, month, day}, matching
// GPS_top, RMC output, and UDP metadata.
module gps_timestamp_guard #(
    parameter integer RESYNC_VALID_FRAMES = 3,
    parameter integer RMC_TO_NEXT_PPS_OFFSET = 0,
    parameter integer RMC_TO_COMPLETED_SEC_OFFSET = RMC_TO_NEXT_PPS_OFFSET
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        pps_pulse,
    input  wire [1:0]  physical_status,
    input  wire        candidate_commit,
    input  wire [23:0] candidate_utc_time,
    input  wire [23:0] candidate_utc_date,
    output reg  [23:0] local_time,
    output reg  [23:0] local_date,
    output reg  [1:0]  time_quality,
    output reg  [1:0]  guard_state,
    output reg  [15:0] holdover_count,
    output reg  [1:0]  resync_count,
    output reg         guard_tick
);

localparam [1:0] PHYSICAL_UNLOCKED = 2'b00;
localparam [1:0] PHYSICAL_LOCKED   = 2'b01;
localparam [1:0] PHYSICAL_ABNORMAL = 2'b10;

localparam [1:0] QUALITY_UNLOCKED = 2'b00;
localparam [1:0] QUALITY_LOCKED = 2'b01;
localparam [1:0] QUALITY_ERROR  = 2'b10;

localparam [1:0] STATE_UNINITIALIZED = 2'b00;
localparam [1:0] STATE_TRACKING      = 2'b01;
localparam [1:0] STATE_HOLDOVER      = 2'b10;
localparam [1:0] STATE_RESYNC        = 2'b11;

// The frozen two-bit resync_count interface supports thresholds 1..4.  Values
// outside that documented range are safely clamped without widening ports.
// RMC_TO_COMPLETED_SEC_OFFSET is the canonical signed elaboration-time offset.
// RMC_TO_NEXT_PPS_OFFSET is retained as a compatibility alias for older
// instantiations.  The production default is 0 because the public timestamp
// is bound to the completed DDR second.  Explicit positive values remain
// available for a measured next-PPS phase, and -1 is supported when the
// measured RMC phase reports the next boundary second.
localparam integer RESYNC_FRAMES_EFFECTIVE =
    (RESYNC_VALID_FRAMES < 1) ? 1 :
    ((RESYNC_VALID_FRAMES > 4) ? 4 : RESYNC_VALID_FRAMES);
localparam integer PPS_OFFSET_EFFECTIVE = RMC_TO_COMPLETED_SEC_OFFSET;

reg        initialized;
reg        pps_pulse_d;
reg        pending_present;
reg        pending_valid;
reg [23:0] pending_time;
reg [23:0] pending_date;
reg [23:0] resync_anchor_time;
reg [23:0] resync_anchor_date;
reg        physical_fault_seen;
reg [1:0]  pending_fault_age;

// Years are encoded as 00..99 for 2000..2099.  Gregorian leap-year rules
// reduce to divisibility by four over this bounded interval.
function is_leap_year;
    input [7:0] year_value;
    begin
        is_leap_year = (year_value[1:0] == 2'b00);
    end
endfunction

function [7:0] days_in_month;
    input [7:0] month_value;
    input [7:0] year_value;
    begin
        case (month_value)
            8'd1, 8'd3, 8'd5, 8'd7, 8'd8, 8'd10, 8'd12:
                days_in_month = 8'd31;
            8'd4, 8'd6, 8'd9, 8'd11:
                days_in_month = 8'd30;
            8'd2:
                days_in_month = is_leap_year(year_value) ? 8'd29 : 8'd28;
            default:
                days_in_month = 8'd0;
        endcase
    end
endfunction

function timestamp_is_valid;
    input [23:0] time_value;
    input [23:0] date_value;
    reg [7:0] hour_value;
    reg [7:0] minute_value;
    reg [7:0] second_value;
    reg [7:0] month_value;
    reg [7:0] year_value;
    reg [7:0] day_value;
    reg [7:0] month_days;
    begin
        hour_value   = time_value[23:16];
        minute_value = time_value[15:8];
        second_value = time_value[7:0];
        year_value   = date_value[23:16];
        month_value  = date_value[15:8];
        day_value    = date_value[7:0];
        month_days   = days_in_month(month_value, year_value);

        timestamp_is_valid = (hour_value < 8'd24) &&
                             (minute_value < 8'd60) &&
                             (second_value < 8'd60) &&
                             (month_value >= 8'd1) &&
                             (month_value <= 8'd12) &&
                             (year_value <= 8'd99) &&
                             (day_value >= 8'd1) &&
                             (day_value <= month_days);
    end
endfunction

// Return {overflow, date}.  Overflow means the next day would be 2100-01-01;
// in that case the original date is retained.
function [24:0] increment_date;
    input [23:0] date_value;
    reg [7:0] month_value;
    reg [7:0] year_value;
    reg [7:0] day_value;
    reg [7:0] month_days;
    reg [23:0] next_date;
    reg overflow_value;
    begin
        year_value  = date_value[23:16];
        month_value = date_value[15:8];
        day_value   = date_value[7:0];
        month_days  = days_in_month(month_value, year_value);
        next_date   = date_value;
        overflow_value = 1'b0;

        if (day_value < month_days) begin
            next_date[7:0] = day_value + 8'd1;
        end else if (month_value < 8'd12) begin
            next_date[15:8]  = month_value + 8'd1;
            next_date[7:0]   = 8'd1;
        end else if (year_value < 8'd99) begin
            next_date[23:16] = year_value + 8'd1;
            next_date[15:8]  = 8'd1;
            next_date[7:0]   = 8'd1;
        end else begin
            overflow_value = 1'b1;
        end

        increment_date = {overflow_value, next_date};
    end
endfunction

// Return {underflow, date}.  Underflow means the input is the first
// representable date 2000-01-01; the original date is retained.
function [24:0] decrement_date;
    input [23:0] date_value;
    reg [7:0] month_value;
    reg [7:0] year_value;
    reg [7:0] day_value;
    reg [23:0] previous_date;
    reg underflow_value;
    begin
        year_value  = date_value[23:16];
        month_value = date_value[15:8];
        day_value   = date_value[7:0];
        previous_date = date_value;
        underflow_value = 1'b0;

        if (day_value > 8'd1) begin
            previous_date[7:0] = day_value - 8'd1;
        end else if (month_value > 8'd1) begin
            previous_date[15:8] = month_value - 8'd1;
            previous_date[7:0]   = days_in_month(
                month_value - 8'd1, year_value);
        end else if (year_value > 8'd0) begin
            previous_date[23:16] = year_value - 8'd1;
            previous_date[15:8]  = 8'd12;
            previous_date[7:0]   = 8'd31;
        end else begin
            underflow_value = 1'b1;
        end

        decrement_date = {underflow_value, previous_date};
    end
endfunction

// Return {overflow, date, time}.  On 2099 overflow the input is retained.
function [48:0] increment_timestamp;
    input [23:0] time_value;
    input [23:0] date_value;
    reg [23:0] next_time;
    reg [23:0] next_date;
    reg [24:0] date_step;
    reg overflow_value;
    begin
        next_time = time_value;
        next_date = date_value;
        overflow_value = 1'b0;

        if (time_value[7:0] < 8'd59) begin
            next_time[7:0] = time_value[7:0] + 8'd1;
        end else begin
            next_time[7:0] = 8'd0;
            if (time_value[15:8] < 8'd59) begin
                next_time[15:8] = time_value[15:8] + 8'd1;
            end else begin
                next_time[15:8] = 8'd0;
                if (time_value[23:16] < 8'd23) begin
                    next_time[23:16] = time_value[23:16] + 8'd1;
                end else begin
                    next_time[23:16] = 8'd0;
                    date_step = increment_date(date_value);
                    if (date_step[24]) begin
                        overflow_value = 1'b1;
                        next_time = time_value;
                    end else begin
                        next_date = date_step[23:0];
                    end
                end
            end
        end

        increment_timestamp = {overflow_value, next_date, next_time};
    end
endfunction

// Return {underflow, previous_date, previous_time}.  Underflow means the
// input is 2000-01-01 00:00:00; the original timestamp is retained.
function [48:0] decrement_timestamp;
    input [23:0] time_value;
    input [23:0] date_value;
    reg [23:0] previous_time;
    reg [23:0] previous_date;
    reg [24:0] date_step;
    reg underflow_value;
    begin
        previous_time = time_value;
        previous_date = date_value;
        underflow_value = 1'b0;

        if (time_value[7:0] > 8'd0) begin
            previous_time[7:0] = time_value[7:0] - 8'd1;
        end else begin
            previous_time[7:0] = 8'd59;
            if (time_value[15:8] > 8'd0) begin
                previous_time[15:8] = time_value[15:8] - 8'd1;
            end else begin
                previous_time[15:8] = 8'd59;
                if (time_value[23:16] > 8'd0) begin
                    previous_time[23:16] = time_value[23:16] - 8'd1;
                end else begin
                    previous_time[23:16] = 8'd23;
                    date_step = decrement_date(date_value);
                    if (date_step[24]) begin
                        underflow_value = 1'b1;
                        previous_time = time_value;
                    end else begin
                        previous_date = date_step[23:0];
                    end
                end
            end
        end

        decrement_timestamp = {underflow_value, previous_date, previous_time};
    end
endfunction

// Convert UTC to local time and advance by the RMC-to-next-PPS offset.
// Return {overflow, local_date, local_time}.
function [48:0] candidate_prediction;
    input [23:0] utc_time_value;
    input [23:0] utc_date_value;
    reg [23:0] converted_time;
    reg [23:0] converted_date;
    reg [8:0] local_hour;
    reg [24:0] date_step;
    reg [48:0] prediction_step;
    integer offset_index;
    begin
        converted_time = utc_time_value;
        converted_date = utc_date_value;
        local_hour = {1'b0, utc_time_value[23:16]} + 9'd8;
        prediction_step = {1'b0, converted_date, converted_time};

        if (local_hour >= 9'd24) begin
            converted_time[23:16] = local_hour - 9'd24;
            date_step = increment_date(utc_date_value);
            if (date_step[24]) begin
                prediction_step = {1'b1, utc_date_value, utc_time_value};
            end else begin
                converted_date = date_step[23:0];
                prediction_step = {1'b0, converted_date, converted_time};
            end
        end else begin
            converted_time[23:16] = local_hour[7:0];
            prediction_step = {1'b0, converted_date, converted_time};
        end

        if (PPS_OFFSET_EFFECTIVE >= 0) begin
            for (offset_index = 0;
                 offset_index < PPS_OFFSET_EFFECTIVE;
                 offset_index = offset_index + 1) begin
                if (!prediction_step[48]) begin
                    prediction_step = increment_timestamp(
                        prediction_step[23:0], prediction_step[47:24]);
                end
            end
        end else begin
            for (offset_index = 0;
                 offset_index > PPS_OFFSET_EFFECTIVE;
                 offset_index = offset_index - 1) begin
                if (!prediction_step[48]) begin
                    prediction_step = decrement_timestamp(
                        prediction_step[23:0], prediction_step[47:24]);
                end
            end
        end

        candidate_prediction = prediction_step;
    end
endfunction

wire        candidate_input_valid;
wire [48:0] candidate_prediction_value;
wire [48:0] local_plus_one;
wire [48:0] anchor_plus_one;
wire        qualified_pps_event;

assign candidate_input_valid = timestamp_is_valid(
    candidate_utc_time, candidate_utc_date);
assign candidate_prediction_value = candidate_prediction(
    candidate_utc_time, candidate_utc_date);
assign local_plus_one = increment_timestamp(local_time, local_date);
assign anchor_plus_one = increment_timestamp(
    resync_anchor_time, resync_anchor_date);
assign qualified_pps_event = pps_pulse && !pps_pulse_d &&
                             (physical_status == PHYSICAL_LOCKED);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        local_time          <= 24'd0;
        local_date          <= 24'd0;
        time_quality        <= QUALITY_ERROR;
        guard_state         <= STATE_UNINITIALIZED;
        holdover_count      <= 16'd0;
        resync_count        <= 2'd0;
        guard_tick          <= 1'b0;
        initialized         <= 1'b0;
        pps_pulse_d         <= 1'b0;
        pending_present     <= 1'b0;
        pending_valid       <= 1'b0;
        pending_time        <= 24'd0;
        pending_date        <= 24'd0;
        resync_anchor_time  <= 24'd0;
        resync_anchor_date  <= 24'd0;
        physical_fault_seen <= 1'b0;
        pending_fault_age   <= 2'd0;
    end else begin
        // Defaults make guard_tick a one-clock indication even if downstream
        // logic stalls; pps_pulse_d additionally rejects a stretched input.
        guard_tick  <= 1'b0;
        pps_pulse_d <= pps_pulse;

        // Physical loss never advances time.  It invalidates any candidate
        // already waiting for the next PPS, including before the first lock;
        // otherwise a stale pre-lock RMC could be published as LOCKED when a
        // later physical PPS finally qualifies.
        if (physical_status != PHYSICAL_LOCKED) begin
            if (initialized) begin
                time_quality <= (physical_status == PHYSICAL_UNLOCKED) ?
                                QUALITY_UNLOCKED : QUALITY_ERROR;
                guard_state  <= STATE_HOLDOVER;
            end else begin
                time_quality <= QUALITY_ERROR;
                guard_state  <= STATE_UNINITIALIZED;
            end
            resync_count        <= 2'd0;
            physical_fault_seen <= 1'b1;
            // A candidate may arrive just before the physical lock state is
            // reported.  Keep a very short grace window for that normal
            // pipeline skew, but never let a pre-lock candidate survive an
            // unqualified interval indefinitely.
            if (pending_present && !candidate_commit) begin
                if (pending_fault_age >= 2'd2) begin
                    pending_present <= 1'b0;
                    pending_valid   <= 1'b0;
                    pending_time    <= 24'd0;
                    pending_date    <= 24'd0;
                    pending_fault_age <= 2'd0;
                end else begin
                    pending_fault_age <= pending_fault_age + 2'd1;
                end
            end else if (!pending_present) begin
                pending_fault_age <= 2'd0;
            end
        end else begin
            pending_fault_age <= 2'd0;
        end

        if (qualified_pps_event) begin
            guard_tick      <= 1'b1;
            pending_present <= 1'b0;
            pending_valid   <= 1'b0;
            pending_fault_age <= 2'd0;

            if (!initialized) begin
                if (pending_present && pending_valid) begin
                    local_time          <= pending_time;
                    local_date          <= pending_date;
                    time_quality        <= QUALITY_LOCKED;
                    guard_state         <= STATE_TRACKING;
                    holdover_count      <= 16'd0;
                    resync_count        <= 2'd0;
                    initialized         <= 1'b1;
                    physical_fault_seen <= 1'b0;
                end else begin
                    local_time     <= 24'd0;
                    local_date     <= 24'd0;
                    time_quality   <= QUALITY_ERROR;
                    guard_state    <= STATE_UNINITIALIZED;
                    holdover_count <= 16'd0;
                    resync_count   <= 2'd0;
                end
            end else if (local_plus_one[48]) begin
                // 2099-12-31 23:59:59 is the last representable value.
                time_quality        <= QUALITY_ERROR;
                guard_state         <= STATE_HOLDOVER;
                resync_count        <= 2'd0;
                physical_fault_seen <= 1'b0;
                if (holdover_count != 16'hFFFF) begin
                    holdover_count <= holdover_count + 16'd1;
                end
            end else if (!pending_present || !pending_valid) begin
                // A missing or malformed candidate cannot stop the physical
                // second boundary: advance locally, but expose ERROR.
                local_time   <= local_plus_one[23:0];
                local_date   <= local_plus_one[47:24];
                time_quality <= QUALITY_ERROR;
                if (pending_present ||
                    (guard_state == STATE_RESYNC) ||
                    physical_fault_seen) begin
                    guard_state <= STATE_RESYNC;
                end else begin
                    guard_state <= STATE_HOLDOVER;
                end
                resync_count        <= 2'd0;
                physical_fault_seen <= 1'b0;
                if (holdover_count != 16'hFFFF) begin
                    holdover_count <= holdover_count + 16'd1;
                end
            end else if ((guard_state == STATE_TRACKING) &&
                         !physical_fault_seen &&
                         (pending_time == local_plus_one[23:0]) &&
                         (pending_date == local_plus_one[47:24])) begin
                local_time          <= local_plus_one[23:0];
                local_date          <= local_plus_one[47:24];
                time_quality        <= QUALITY_LOCKED;
                guard_state         <= STATE_TRACKING;
                holdover_count      <= 16'd0;
                resync_count        <= 2'd0;
                physical_fault_seen <= 1'b0;
            end else begin
                // Once continuity is lost, public time remains on the local
                // +1 path until three consecutive candidate predictions prove
                // a new timeline.  The third corresponding PPS performs the
                // only allowed correction.
                local_time          <= local_plus_one[23:0];
                local_date          <= local_plus_one[47:24];
                time_quality        <= QUALITY_ERROR;
                guard_state         <= STATE_RESYNC;
                physical_fault_seen <= 1'b0;
                if (holdover_count != 16'hFFFF) begin
                    holdover_count <= holdover_count + 16'd1;
                end

                if ((guard_state == STATE_RESYNC) &&
                    (resync_count != 2'd0) &&
                    !anchor_plus_one[48] &&
                    (pending_time == anchor_plus_one[23:0]) &&
                    (pending_date == anchor_plus_one[47:24])) begin
                    if (resync_count >= (RESYNC_FRAMES_EFFECTIVE - 1)) begin
                        local_time          <= pending_time;
                        local_date          <= pending_date;
                        time_quality        <= QUALITY_LOCKED;
                        guard_state         <= STATE_TRACKING;
                        holdover_count      <= 16'd0;
                        resync_count        <= 2'd0;
                        physical_fault_seen <= 1'b0;
                    end else begin
                        resync_count       <= resync_count + 2'd1;
                        resync_anchor_time <= pending_time;
                        resync_anchor_date <= pending_date;
                    end
                end else if (RESYNC_FRAMES_EFFECTIVE <= 1) begin
                    local_time          <= pending_time;
                    local_date          <= pending_date;
                    time_quality        <= QUALITY_LOCKED;
                    guard_state         <= STATE_TRACKING;
                    holdover_count      <= 16'd0;
                    resync_count        <= 2'd0;
                    physical_fault_seen <= 1'b0;
                end else begin
                    resync_count       <= 2'd1;
                    resync_anchor_time <= pending_time;
                    resync_anchor_date <= pending_date;
                end
            end
        end

        // This block is intentionally after PPS consumption.  Nonblocking
        // assignment ordering implements the frozen same-clock contract: the
        // PPS consumes the old pending value, and this commit is kept for the
        // next PPS.
        if (candidate_commit &&
            !(pps_pulse && (physical_status != PHYSICAL_LOCKED))) begin
            pending_present <= 1'b1;
            pending_valid   <= candidate_input_valid &&
                               !candidate_prediction_value[48];
            pending_time    <= candidate_prediction_value[23:0];
            pending_date    <= candidate_prediction_value[47:24];
            pending_fault_age <= 2'd0;
        end
    end
end

endmodule
