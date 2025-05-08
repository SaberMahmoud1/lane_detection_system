////////////////////////////////////////////////////////////////////////////////
// Author: Saber Mahmoud
//
// Description: Lane Detection Decision Module for Edge-Based Lane Detection System.
// This module processes the edge-detected grayscale image data received from 
// the Sobel filter and determines the number of detected lanes in the image. 
// It utilizes a column-wise analysis approach, counting pixel clusters and gaps 
// to estimate lane boundaries.

// The decision module operates by reading the binary edge-detected image data, 
// analyzing the pixel clusters, and applying a threshold-based approach to 
// count lanes based on the spacing between detected edges.

// The module maintains a state machine to control the reading and processing 
// of incoming data, ensuring synchronized operation with other pipeline stages.

// The processed lane count data is output for use in further lane tracking 
// and vehicle control decision-making processes.

// Parameters:
// - PIXEL_GAP_THRESHOLD : Defines the maximum allowed gap between detected edges 
//                         before registering a new lane (default: 10).
// - IMG_WIDTH          : The width of the input image in pixels (default: 640).
// - IMG_LENGTH         : The height of the input image in pixels (default: 640).
// - PIXEL_SIZE         : The bit-width of each grayscale pixel (default: 8).
//
// Inputs:
// - clk          : Clock signal for synchronization with other modules.
// - rst_n        : Active-low reset signal to initialize the module.
// - decision_sobel_fifo_rd_data : Binary pixel data (1 = edge, 0 = non-edge).
// - decision_sobel_fifo_full    : FIFO full indicator.
// - decision_sobel_fifo_empty   : FIFO empty indicator.
// - decision_sobel_wr_ack       : Acknowledgment for successful write operations.
// - decision_sobel_rd_ack       : Acknowledgment for successful read operations.
// - decision_sobel_fifo_data_count : Number of data elements in the FIFO.
//
// Outputs:
// - decision_sobel_fifo_rd_en : Read enable signal to retrieve data from FIFO.
// - number_of_lanes           : Estimated number of detected lanes in the frame.
//
// States:
// - IDLE       : Waiting for valid edge-detected data.
// - WRITE_DATA : Processing pixel data and updating lane count.
//
// Internal Processing:
// - Maintains a counter to track column positions within each row.
// - Counts clusters of ones (edges) to determine lane candidates.
// - Uses a threshold-based method to differentiate between distinct lanes.
// - Stores frequency data to determine the most common lane count.
// - Resets counters and updates results at the end of each frame.
//
// FIFO Interface:
// - Reads pixel data from the Sobel output FIFO for processing.
// - Writes processed lane count results to the decision pipeline.
//
// The module efficiently processes image frames in real-time and provides lane 
// count information crucial for lane detection and vehicle navigation.
////////////////////////////////////////////////////////////////////////////////

module decision #(
    parameter PIXEL_GAP_THRESHOLD =10,
    parameter IMG_WIDTH =640,
    parameter IMG_LENGTH =640,
    parameter PIXEL_SIZE=8
) 
(
    input clk,                                        // The Clk signal of the Design which used in the Synchronization with the other modules.
    input rst_n,                                      // A reset signal to reset the whole signals of the design to nulls.

    //FIFO interface shared with decition
    output logic                           decision_sobel_fifo_rd_en  ,   //when enabled the fifo gives data out
    input  logic                           decision_sobel_fifo_rd_data,   //the data to be read from the fifo
    input  logic                           decision_sobel_fifo_full    ,   //fifo full indicator    
    input  logic                           decision_sobel_fifo_empty   ,   //fifo empty indicator
    input  logic                           decision_sobel_wr_ack        ,   //ack signal to make sure the write operations is done right.
    input  logic                           decision_sobel_rd_ack         ,   //ack signal to make sure the read operations is done right. 
    input  logic [$clog2(IMG_WIDTH):0]     decision_sobel_fifo_data_count ,
    
    //speed control unit interface
    output logic [4-1:0]                   number_of_lanes, //indicates the number of lanes in the image
    output logic [4-1:0]                   current_lane, //indicates the current  lane the car is in
    output logic                           decision_out_valid, //indicates that the output of the decision module is valid
    output logic [$clog2(IMG_WIDTH):0]     current_lane_left_boundary, //the left  boundary of the current lane in pixels
    output logic [$clog2(IMG_WIDTH):0]     current_lane_right_boundary //the right boundary of the current lane in pixels
);
    
    localparam MIDDLE_OF_THE_IMG = (IMG_WIDTH >> 1);

    reg decision_read_ready;

typedef enum logic  {
    IDLE,
    WRITE_DATA
} buffer_write_t;        

buffer_write_t w_buff_cs,w_buff_ns;


logic [$clog2(IMG_WIDTH):0] column_counter; //counter to track the current pixel we are processing

logic [$clog2(IMG_WIDTH):0] ones_counter,last_one_position; //number of ones in each row

logic [$clog2(IMG_LENGTH):0] row;   //counter to track the current pixel we are processing

logic [$clog2(IMG_WIDTH):0] lines_array [16-1:0]; //array to store then number of auccurence of each number of lines from 0 lines to 15 lines the most one accures is most like to be the true number of lines


logic prossing_last_row; //indicates that we are currently processing the last row in the image and no other rows left
logic [$clog2(IMG_WIDTH):0] most_frequent; //most frequent number of lines auccured in the image

logic save_lane_boundaries,save_current_lane;
logic [$clog2(IMG_WIDTH):0] lane_boundaries [16-1:0];
logic [$clog2(IMG_WIDTH):0] most_frequent_lane_boundaries [16-1:0];
logic [$clog2(IMG_WIDTH):0] lane_counter; 


/********************************************************************************************************************
********************************************************************************************************************/
// State transation logic

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        w_buff_cs <= IDLE; 
    end else begin 
        w_buff_cs <= w_buff_ns;
    end
end


always_comb begin : Next_State_logic      
    case (w_buff_cs)
             IDLE : begin
            if(!decision_sobel_fifo_empty) begin
                w_buff_ns = WRITE_DATA;
            end else begin
                w_buff_ns = IDLE;
            end
        end
        WRITE_DATA: begin
            if((decision_sobel_fifo_data_count <= 1))begin
             w_buff_ns = IDLE;
            end else begin
             w_buff_ns = WRITE_DATA;
            end
        end
        default: w_buff_ns = IDLE;
    endcase
end  


always_comb begin : output_logic
    case (w_buff_cs)
        IDLE:         decision_read_ready = 0;
        WRITE_DATA:   decision_read_ready = 1;
        default:      decision_read_ready = 0;
    endcase
end



/********************************************************************************************************************
***********************************************************************************************************************/


//choose which fifo to write data into

always_comb begin : demux_the_rd_en
    decision_sobel_fifo_rd_en = decision_read_ready && !decision_sobel_fifo_empty;
end


always_ff @(posedge clk or negedge rst_n) begin : calc_number_of_lanes
if(!rst_n)begin
    row <= 1'b0 ;
    column_counter <= 0;
    ones_counter <= 0;
    last_one_position <= 0;
    number_of_lanes <=0;
    most_frequent <= 0;
    lane_counter <= 0;
    save_lane_boundaries <= 0;
    save_current_lane <= 0;
    for (int i = 0; i < 16; i = i + 1) begin
            lines_array[i] <= 'd0;  // Initialize all elements to zero
    end
end else if(decision_sobel_rd_ack)begin
    
    column_counter<=column_counter+1;

    if(decision_sobel_fifo_rd_data == 1)begin
        
        last_one_position <= column_counter;
        
        if(column_counter-last_one_position >= PIXEL_GAP_THRESHOLD || ones_counter == 0)begin
            ones_counter <= ones_counter + 1;
            lane_boundaries[lane_counter] <= column_counter;
            lane_counter <= lane_counter + 1;
        end
    end
        
    if(column_counter >= IMG_WIDTH-1)begin
        if(!prossing_last_row)begin
            row<=row+1;
        end
        column_counter<=0;
        ones_counter <= 0;
        last_one_position <= 0;
        lane_counter <= 0;
        
        if(ones_counter < 16)begin
            lines_array[ones_counter] <= lines_array[ones_counter] + 1;
            if(lines_array[ones_counter] > most_frequent)begin
                most_frequent <= ones_counter;
                if(row <= IMG_LENGTH - 5)begin
                save_lane_boundaries <= 1;
                end
            end 
        end
    end

    if(row >= IMG_LENGTH-1)begin
            prossing_last_row <=1;
            row <= 0;
            number_of_lanes <= most_frequent - 1;
            save_current_lane <= 1;
        end

end else begin
    prossing_last_row <= 0;
    save_lane_boundaries <= 0;
    save_current_lane <= 0;
end
end


always_ff @(posedge clk or negedge rst_n) begin : calc_lane_boundaries
if(!rst_n)begin
    for (int i = 0; i < 16; i = i + 1) begin
            most_frequent_lane_boundaries[i] <= 1'b0;  // Initialize all elements to zero
    end
end else if(save_lane_boundaries)begin
    for (int i = 0; i < 16; i = i + 1) begin
            most_frequent_lane_boundaries[i] <= lane_boundaries[i];  // Initialize all elements to zero
    end
    
end 
end

always_ff @(posedge clk or negedge rst_n) begin : calc_current_lane_and_position
if(!rst_n)begin
    current_lane <= 0;
    current_lane_left_boundary <= 0;
    current_lane_right_boundary<= 0;
    decision_out_valid <= 0;
end else if(save_current_lane)begin
    for (int i = 0; i < 15; i = i + 1) begin
            if(most_frequent_lane_boundaries[15-i] > MIDDLE_OF_THE_IMG)begin
                current_lane <= 15-i;
                current_lane_left_boundary  <= most_frequent_lane_boundaries[14-i];
                current_lane_right_boundary <= most_frequent_lane_boundaries[15-i];
            end
    end
    decision_out_valid <= 1;
end else begin
    decision_out_valid <= 0;
end
end



endmodule