////////////////////////////////////////////////////////////////////////////////
// Author: Saber Mahmoud
//
// Description: Lane Detection Decision Module for Edge-Based Lane Detection System.
// This module processes the edge-detected grayscale image data received from 
// the Sobel filter and determines the number of detected lanes in the image. 
// It utilizes a column-wise analysis approach, counting pixel clusters and gaps 
// to estimate lane boundaries.
////////////////////////////////////////////////////////////////////////////////

module decision #(
    parameter PIXEL_GAP_THRESHOLD =5,
    parameter IMG_WIDTH =640,
    parameter IMG_LENGTH =640
) 
(
    input clk,                                        // The Clk signal of the Design which used in the Synchronization with the other modules.
    input rst_n,                                      // A reset signal to reset the whole signals of the design to nulls.

    //interface shared with decision and sobel
    input sobel_out_valid,
    input first_threshold_sobel_result,
    input second_threshold_sobel_result,

    //speed control unit interface
    output logic [4-1:0]                   number_of_lanes, //indicates the number of lanes in the image
    output logic [4-1:0]                   current_lane, //indicates the current  lane the car is in
    output logic                           decision_out_valid, //indicates that the output of the decision module is valid
    output logic [$clog2(IMG_WIDTH):0]     current_lane_left_boundry, //the left  boundary of the current lane in pixels
    output logic [$clog2(IMG_WIDTH):0]     current_lane_right_boundry //the right boundary of the current lane in pixels
);
    
localparam IMAGE_CENTER = (IMG_WIDTH >> 1);
localparam LANE_SPACING = PIXEL_GAP_THRESHOLD;
localparam [3-1:0] LINE_WIDTH = 3'b111;

logic start_flag,start_filtering;

logic first_threshold_sobel_result_r,second_threshold_sobel_result_r;

logic [$clog2(IMG_WIDTH):0] last_cluster_detected; //number of ones in each row_counter


logic [$clog2(IMG_WIDTH + 1)-1:0] column_counter; //counter to track the current pixel we are processing
logic [$clog2(IMG_LENGTH + 1)-1:0] row_counter;   //counter to track the current pixel we are processing

logic [$clog2(IMG_WIDTH):0] lane_boundaries [5-1:0];
logic [$clog2(IMG_WIDTH):0] lane_boundaries_filtered [5-1:0];

logic [$clog2(IMG_WIDTH):0] lane_boundarie_accurence [5-1:0];

logic row_counter_en;

logic [$clog2(LINE_WIDTH)-1:0] consecutive_ones;

logic cluster_detected;

logic [5-1:0] lines_number;
logic [5-1:0] right_lines_number;

// Sampled input storage
logic [11:0] lane_boundaries_filtered_r[4:0];
logic [2:0] out_stage_counter;


/********************************************************************************************************************
********************************************************************************************************************/

assign row_counter_en = (column_counter == IMG_WIDTH-1) ? 1 : 0;

//choose which fifo to write data into

// Instantiate the counter to count columns 
    counter #(
        .MAX_COUNT(IMG_WIDTH-1)
    ) col_counter (
        .clk    (clk),
        .rst_n  (rst_n),
        .en     (start_flag),
        .count  (column_counter)
    );

// Instantiate the counter to count columns 
    counter #(
        .MAX_COUNT(IMG_LENGTH-1)
    ) r_counter (
        .clk    (clk),
        .rst_n  (rst_n),
        .en     (row_counter_en),
        .count  (row_counter)
    );

    shift_register #(
        .DEPTH($clog2(LINE_WIDTH))
    ) shift_inst (
        .clk(clk),
        .rst_n(rst_n),
        .enable(start_flag),
        .din(first_threshold_sobel_result_r),
        .dout(consecutive_ones)
    );

    remove_close_duplicates #(
        .IMG_WIDTH(IMG_WIDTH)
    ) u_remove_close_duplicates (
        .clk(clk),
        .rst_n(rst_n),
        .start(start_filtering),
        .in_array(lane_boundaries),
        .importance(lane_boundarie_accurence),
        .out_array(lane_boundaries_filtered)
    );


always_ff @(posedge clk or negedge rst_n) begin : samble_sobel_results
    if(!rst_n)begin
        start_flag <= 1'b0;
        first_threshold_sobel_result_r <= 1'b0;
        second_threshold_sobel_result_r <= 1'b0;
    end else if(sobel_out_valid)begin
        start_flag <= 1'b1;
        first_threshold_sobel_result_r <= first_threshold_sobel_result;
        second_threshold_sobel_result_r <= second_threshold_sobel_result;
    end else begin
        start_flag <= 1'b0;
        first_threshold_sobel_result_r <= 1'b0;
        second_threshold_sobel_result_r <= 1'b0;
    end
end
    
/********************************************************************************************************************
calculate lanes and find boundries
********************************************************************************************************************/

always_ff @(posedge clk or negedge rst_n) begin : calc_number_of_lanes
if(!rst_n)begin
        for (int i = 0; i < 5; i = i + 1) begin
            lane_boundarie_accurence[i] <= 0;
            lane_boundaries[i] <= 0;
        end
    end else if(start_flag)begin
        if (cluster_detected) begin
            if (((lane_boundarie_accurence[0] > 0) &&
            (column_counter + LANE_SPACING >= lane_boundaries[0]) &&
            (column_counter <= lane_boundaries[0] + LANE_SPACING)) || lane_boundarie_accurence[0] == 0) begin
                lane_boundaries[0] <= column_counter;
                lane_boundarie_accurence[0] <= (lane_boundarie_accurence[0] < 1000) ? lane_boundarie_accurence[0] + 2 : lane_boundarie_accurence[0] ;
        end else if (((lane_boundarie_accurence[1] > 0) &&
            (column_counter + LANE_SPACING >= lane_boundaries[1]) &&
            (column_counter <= lane_boundaries[1] + LANE_SPACING)) || lane_boundarie_accurence[1] == 0) begin
                lane_boundaries[1] <= column_counter;
                lane_boundarie_accurence[1] <= (lane_boundarie_accurence[1] < 1000) ? lane_boundarie_accurence[1] + 2 : lane_boundarie_accurence[1] ;
        end else if (((lane_boundarie_accurence[2] > 0) &&
            (column_counter + LANE_SPACING >= lane_boundaries[2]) &&
            (column_counter <= lane_boundaries[2] + LANE_SPACING)) || lane_boundarie_accurence[2] == 0) begin
                lane_boundaries[2] <= column_counter;
                lane_boundarie_accurence[2] <= (lane_boundarie_accurence[2] < 1000) ? lane_boundarie_accurence[2] + 2 : lane_boundarie_accurence[2] ;
        end else if (((lane_boundarie_accurence[3] > 0) &&
            (column_counter + LANE_SPACING >= lane_boundaries[3]) &&
            (column_counter <= lane_boundaries[3] + LANE_SPACING)) || lane_boundarie_accurence[3] == 0) begin
                lane_boundaries[3] <= column_counter;
                lane_boundarie_accurence[3] <= (lane_boundarie_accurence[3] < 1000) ? lane_boundarie_accurence[3] + 2 : lane_boundarie_accurence[3] ;
        end else if (((lane_boundarie_accurence[4] > 0) &&
            (column_counter + LANE_SPACING >= lane_boundaries[4]) &&
            (column_counter <= lane_boundaries[4] + LANE_SPACING)) || lane_boundarie_accurence[4] == 0) begin
                lane_boundaries[4] <= column_counter;
                lane_boundarie_accurence[4] <= (lane_boundarie_accurence[4] < 1000) ? lane_boundarie_accurence[4] + 2 : lane_boundarie_accurence[4] ;
        end 
    end else begin
           if ((lane_boundaries[0] == column_counter) && (lane_boundarie_accurence[0] > 0) ) begin
                lane_boundarie_accurence[0] <= lane_boundarie_accurence[0] - 1;
            end else if ((lane_boundaries[1] == column_counter) && (lane_boundarie_accurence[1] > 0)) begin
                lane_boundarie_accurence[1] <= lane_boundarie_accurence[1] - 1;
            end else if ((lane_boundaries[2] == column_counter) && (lane_boundarie_accurence[2] > 0)) begin
                lane_boundarie_accurence[2] <= lane_boundarie_accurence[2] - 1;
            end else if ((lane_boundaries[3] == column_counter) && (lane_boundarie_accurence[3] > 0)) begin
                lane_boundarie_accurence[3] <= lane_boundarie_accurence[3] - 1;
            end else if ((lane_boundaries[4] == column_counter) && (lane_boundarie_accurence[4] > 0)) begin
                lane_boundarie_accurence[4] <= lane_boundarie_accurence[4] - 1;
            end 
        end
    end 
end

always_ff @(posedge clk or negedge rst_n) begin : the_cluster_estimation
    if (!rst_n) begin
        cluster_detected <= 1'b0;
        last_cluster_detected <= 1'b0;
    end else begin
        // Check for cluster of 5 consecutive 1s in the window
        if ((consecutive_ones == LINE_WIDTH) && ((row_counter >= 3) && (row_counter <= IMG_LENGTH - 3)) && (column_counter-last_cluster_detected >= 5)) begin
            cluster_detected <= 1'b1;
            last_cluster_detected <= column_counter;
        end else begin
            cluster_detected <= 1'b0;
        end
        if(column_counter >= IMG_WIDTH - 1)begin
            last_cluster_detected <= 1'b0;
        end
    end
end


/********************************************************************************************************************
********************************************************************************************************************/

    // Sample filtered boundaries 
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (int i = 0; i < 5; i++) lane_boundaries_filtered_r[i] <= 0;
        start_filtering <= 0;
    end else if(column_counter == 0 && row_counter == IMG_LENGTH-3)begin
        start_filtering <= 1;
    end else if (column_counter == IMG_WIDTH-1 && row_counter == IMG_LENGTH-2) begin
        for (int i = 0; i < 5; i++) begin
            lane_boundaries_filtered_r[i] <= lane_boundaries_filtered[i];
        end
    end else begin
        start_filtering <= 0;
    end
end

    /*find the number of lanes and current lane*/
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        lines_number        <= 0;
        right_lines_number  <= 0;
        out_stage_counter   <= 0;
    end else if (column_counter >= IMG_WIDTH-5 && row_counter == IMG_LENGTH-1) begin
        if (lane_boundaries_filtered_r[out_stage_counter] > 0)begin
            lines_number <= lines_number + 1;
        end

        if (lane_boundaries_filtered_r[out_stage_counter] > IMAGE_CENTER)begin
            right_lines_number <= right_lines_number + 1;
        end

        out_stage_counter <= out_stage_counter + 1;
    end else if(row_counter == 1) begin
        lines_number        <= 0;
        right_lines_number  <= 0;
        out_stage_counter   <= 0;
    end
end

    /*find the current lane boundaries*/
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        current_lane_left_boundry  <= 0;
        current_lane_right_boundry <= IMG_WIDTH;
    end else if (column_counter >= IMG_WIDTH-5 && row_counter == IMG_LENGTH-1) begin
        if ((lane_boundaries_filtered_r[out_stage_counter] < IMAGE_CENTER) && (current_lane_left_boundry < lane_boundaries_filtered_r[out_stage_counter]))begin
            current_lane_left_boundry = lane_boundaries_filtered_r[out_stage_counter];
        end

        if ((lane_boundaries_filtered_r[out_stage_counter] > IMAGE_CENTER) && (current_lane_right_boundry > lane_boundaries_filtered_r[out_stage_counter]))begin
            current_lane_right_boundry = lane_boundaries_filtered_r[out_stage_counter];
        end
    end else if(row_counter == 1) begin
        current_lane_left_boundry  <= 0;
        current_lane_right_boundry <= IMG_WIDTH;
    end
end


always_ff @(posedge clk or negedge rst_n) begin : drive_the_valid_signal
if(!rst_n)begin
    decision_out_valid <= 1'b0;
end else if(column_counter == IMG_WIDTH-1 && row_counter == IMG_LENGTH-1)begin
    decision_out_valid <= 1'b1;
end else begin
    decision_out_valid <= 1'b0;
end
end

// Number of lanes = number of boundaries - 1
assign    number_of_lanes = (lines_number > 1) ? (lines_number - 1) : 1;
assign    current_lane = (lines_number > right_lines_number) ? (lines_number - right_lines_number) : 1;


endmodule