////////////////////////////////////////////////////////////////////////////////
// Author: Saber Mahmoud
//
// Description: Lane Detection Wrapper for Grayscale Image Processing Pipeline.
// This module wraps the entire lane detection processing pipeline, which includes 
// the averaging filter, Sobel filter, and decision modules. The input grayscale 
// image data is processed in stages to detect lanes in the image. The wrapper 
// connects all the modules and controls the flow of image data through the pipeline.
//
// The wrapper performs the following operations:
// 1. **Averaging Filter**: Applies an averaging filter to smooth the image and 
//    reduce noise before edge detection.
// 2. **Sobel Filter**: Detects edges in the image by calculating gradients and 
//    highlighting areas of significant intensity change.
// 3. **Decision Module**: Based on the edges detected by the Sobel filter, the 
//    decision module identifies potential lane markers and outputs a decision.
//
// The module interfaces with FIFOs to manage the flow of image data across the stages 
// of processing. It processes rows of pixels from the image, applies the necessary 
// filtering and edge detection, and then outputs the decision about lane presence.
//
// Instantiations:
// - **Averaging Filter:** Smooths the image by applying an averaging filter to 
//   the pixel data. It reduces noise in the image before edge detection.
// - **Sobel Filter:** Detects edges in the smoothed image to highlight potential lane markers.
// - **Decision Module:** Analyzes the edge-detected image to make decisions about the presence of lanes.
////////////////////////////////////////////////////////////////////////////////
module LD_Wrapper # (
    parameter THRESHOLD1      = 10000,
    parameter THRESHOLD2      = 50000,
    parameter IMG_LENGTH     = 150  ,
    parameter IMG_WIDTH      = 416  , 
    parameter PIXEL_SIZE     = 8   ,
    parameter AXI_WIDTH      = 24  ,    
    parameter RGB_WIDTH      =24   ,
    parameter PIXEL_GAP_THRESHOLD = 5,
    parameter FRAME_START = 250


) (
    //system interface
    input clk,                                        // The Clk signal of the Design which used in the Synchronization with the other modules.
    input rst_n,                                      // A reset signal to reset the whole signals of the design to nulls.
    
    //speed control unit interface
    output logic [4-1:0] number_of_lanes, //indicates the number of lanes on the road
    output logic [4-1:0] current_lane, //indicates the current  lane the car is in
    output logic         decision_out_valid, //indicates that the output of the decision module is valid
    output logic [$clog2(IMG_WIDTH):0]     current_lane_left_boundry, //the left  boundary of the current lane in pixels
    output logic [$clog2(IMG_WIDTH):0]     current_lane_right_boundry, //the right boundary of the current lane in pixels

    /* AXI Stream Interface */
    input  logic  [AXI_WIDTH-1:0] s_axi_video_tdata,
    input  logic  s_axi_video_tvalid,
    output logic  s_axi_video_ready,
    input  logic  s_axi_video_tlast
);

    //FIFO interface
    logic                           cnv_avr_fifo_wr_en   ;   //when enabled the fifo takes data in
    logic  [PIXEL_SIZE-1:0]         cnv_avr_fifo_wr_data ;   //the data to be written to the fifo
    logic                           cnv_avr_fifo_full    ;   //fifo full indicator    
    logic                           cnv_avr_fifo_empty   ;   //fifo empty indicator
    logic                           cnv_avr_fifo_rd_en  ;   //when enabled the fifo gives data out
    logic [PIXEL_SIZE-1:0]          cnv_avr_fifo_rd_data ;  //the data to be read from the fifo
    logic                           cnv_avr_wr_ack      ;   //ack signal to make sure the write operations is done right.
    logic                           cnv_avr_rd_ack       ;   //ack signal to make sure the read operations is done right. 
    logic [$clog2(IMG_WIDTH):0]     cnv_avr_fifo_data_count ;


        //FIFO interface
    logic                           avr_data_valid   ;   //when enabled the fifo takes data in
    logic  [PIXEL_SIZE-1:0]         avr_data_out ;   //the data to be written to the fifo

        //interface shared with decision and sobel
    logic sobel_out_valid;
    logic first_threshold_sobel_result;
    logic second_threshold_sobel_result;

    logic valid_cnvrtr;


   /* The Instntiation of the rgb converter Module */
imgcrop #(
    .FRAME_HEIGHT(IMG_LENGTH),
    .FRAME_START(FRAME_START)  // Set the desired cropped frame height
) u_imgcrop (
    .clk                (clk),                  // System clock
    .rst_n              (rst_n),                // Active-low reset
    .s_axi_video_tvalid(s_axi_video_tvalid),   // AXI-Stream valid signal for incoming image data
    .s_axi_video_tlast (s_axi_video_tlast),    // AXI-Stream signal indicating end of line
    .s_axi_video_ready (s_axi_video_ready),    // AXI-Stream ready signal for flow control
    .valid_cnvrtr      (valid_cnvrtr)          // Output valid signal to RGB converter (cropped rows)
);

   /* The Instntiation of the rgb converter Module */ 
rgb_converter_axi #(
    .IMG_WIDTH(IMG_WIDTH),
    .AXI_WIDTH(AXI_WIDTH),      // Set the AXI width to 48 bits.
    .RGB_WIDTH(RGB_WIDTH),      // Set the RGB width to 24 bits (8 bits per color channel).
    .GRAY_WIDTH(PIXEL_SIZE)       // Set the grayscale width to 8 bits.
) rgb_converter_inst (
    .clk(clk),                       // The clock signal for synchronization with other modules.
    .rst_n(rst_n),                   // The active-low reset signal to reset the module.

    /* AXI Stream Interface */
    .s_axi_video_tdata(s_axi_video_tdata),  // AXI stream data input for video.
    .s_axi_video_tvalid(valid_cnvrtr), // AXI stream valid signal for video.
    .s_axi_video_ready(s_axi_video_ready),   // AXI stream ready signal for video output.

    /* Outputs of the converter to the next stage */
    //FIFO interface
    .cnv_avr_fifo_wr_en(cnv_avr_fifo_wr_en)  ,   //when enabled the FIFO takes data in
    .cnv_avr_fifo_wr_data(cnv_avr_fifo_wr_data),   //the data to be written to the FIFO
    .cnv_avr_fifo_full(cnv_avr_fifo_full)    ,   //FIFO full indicator    
    .cnv_avr_fifo_empty(cnv_avr_fifo_empty)   ,   //FIFO empty indicator
    .cnv_avr_wr_ack     (cnv_avr_wr_ack)   ,   //ack signal to make sure the write operation is done right.
    .cnv_avr_rd_ack      (cnv_avr_rd_ack)   ,   //ack signal to make sure the read operation is done right. 
    .cnv_avr_fifo_data_count(cnv_avr_fifo_data_count)     //the number of data stored in the FIFO

);

   
/* The Instntiation of the AVG Filter Module */ 
Avg_filter #(
    .IMG_LENGTH(IMG_LENGTH), 
    .IMG_WIDTH(IMG_WIDTH), 
    .PIXEL_SIZE(PIXEL_SIZE)
) avg_filter_inst (
    .clk(clk),
    .rst_n(rst_n),
    
     //FIFO interface for the gray pixels
    .cnv_avr_fifo_rd_en(cnv_avr_fifo_rd_en),   //when enabled the fifo gives data out
    .cnv_avr_fifo_rd_data(cnv_avr_fifo_rd_data),   //the data to be read from the fifo
    .cnv_avr_fifo_full(cnv_avr_fifo_full),   //fifo full indicator    
    .cnv_avr_fifo_empty(cnv_avr_fifo_empty),   //fifo empty indicator
    .cnv_avr_wr_ack(cnv_avr_wr_ack),   //ack signal to make sure the write operations is done right.
    .cnv_avr_rd_ack (cnv_avr_rd_ack),   //ack signal to make sure the read operations is done right. 
    .cnv_avr_fifo_data_count(cnv_avr_fifo_data_count),
    
    //sobel interface
    .avr_data_valid(avr_data_valid),
    .avr_data_out(avr_data_out)
);

/* The Instntiation of the sobel Filter Module */ 
sobel_filter #(
    .THRESHOLD1(THRESHOLD1),
    .THRESHOLD2(THRESHOLD2),
    .IMG_LENGTH(IMG_LENGTH), 
    .IMG_WIDTH(IMG_WIDTH), 
    .PIXEL_SIZE(PIXEL_SIZE)
) sobel_filter_inst (
    .clk(clk),
    .rst_n(rst_n),
    
    //average filter interface
    .avr_data_valid(avr_data_valid),
    .avr_data_out(avr_data_out),
    
    //interface shared with decision and sobel
    .sobel_out_valid(sobel_out_valid),
    .first_threshold_sobel_result(first_threshold_sobel_result),
    .second_threshold_sobel_result(second_threshold_sobel_result)

);

// Instantiate the decision module
    decision #(
        .PIXEL_GAP_THRESHOLD(PIXEL_GAP_THRESHOLD),
        .IMG_WIDTH(IMG_WIDTH),
        .IMG_LENGTH(IMG_LENGTH)
    ) decision_inst (
        .clk(clk),
        .rst_n(rst_n),
        
        //interface shared with decision and sobel
        .sobel_out_valid(sobel_out_valid),
        .first_threshold_sobel_result(first_threshold_sobel_result),
        .second_threshold_sobel_result(second_threshold_sobel_result),

        //speed control unit interface
        .number_of_lanes(number_of_lanes),
        .decision_out_valid(decision_out_valid),
        .current_lane(current_lane),
        .current_lane_left_boundry(current_lane_left_boundry),
        .current_lane_right_boundry(current_lane_right_boundry)
    );

/* FIFO Line-Buffers to store rows of pixels */

        FIFO #(
            .FIFO_WIDTH(PIXEL_SIZE),  
            .FIFO_DEPTH(IMG_WIDTH)   
        ) cnv_avr_fifo_inst (
            .clk(clk),                
            .rst_n(rst_n),            
            .wr_en(cnv_avr_fifo_wr_en),    
            .rd_en(cnv_avr_fifo_rd_en),    
            .data_in(cnv_avr_fifo_wr_data),  
            .data_out(cnv_avr_fifo_rd_data),
            .full(cnv_avr_fifo_full),      
            .empty(cnv_avr_fifo_empty),    
            .wr_ack(cnv_avr_wr_ack),       
            .rd_ack(cnv_avr_rd_ack),
            .count(cnv_avr_fifo_data_count)        
        );
    
endmodule