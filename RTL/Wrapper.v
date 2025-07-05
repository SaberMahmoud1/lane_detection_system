module Wrapper # (
    parameter THRESHOLD1      = 9500,
    parameter THRESHOLD2      = 55000,
    parameter IMG_LENGTH     = 416  ,
    parameter IMG_WIDTH      = 416  , 
    parameter PIXEL_SIZE     = 8   ,
    parameter AXI_WIDTH      = 24  ,    
    parameter RGB_WIDTH      =24   ,
    parameter PIXEL_GAP_THRESHOLD = 10,
    parameter FRAME_START = 0


) (
    //system interface
    input clk,                                        // The Clk signal of the Design which used in the Synchronization with the other modules.
    input rst_n,                                      // A reset signal to reset the whole signals of the design to nulls.
    
    //speed control unit interface
    output wire [4-1:0] number_of_lanes, //indicates the number of lanes on the road
    output wire [4-1:0] current_lane, //indicates the current  lane the car is in
    output wire         decision_out_valid, //indicates that the output of the decision module is valid
    output wire [$clog2(IMG_WIDTH):0]     current_lane_left_boundry, //the left  boundary of the current lane in pixels
    output wire [$clog2(IMG_WIDTH):0]     current_lane_right_boundry, //the right boundary of the current lane in pixels

    /* AXI Stream Interface */
    input  wire  [AXI_WIDTH-1:0] s_axi_video_tdata,
    input  wire  s_axi_video_tvalid,
    output wire  s_axi_video_ready,
    input  wire  s_axi_video_tlast
);

// DUT Instantiation
    LD_Wrapper #(
    .THRESHOLD1(THRESHOLD1),          // Set the threshold for lane detection to 1600.
    .THRESHOLD2(THRESHOLD2),          // Set the threshold for lane detection to 1600.
    .IMG_LENGTH(IMG_LENGTH),           // Set the image height to 10 pixels.
    .IMG_WIDTH(IMG_WIDTH),            // Set the image width to 10 pixels.
    .PIXEL_SIZE(PIXEL_SIZE),            // Set the pixel size to 8 bits.
    .AXI_WIDTH(24),            // Set the AXI stream data width to 48 bits.
    .RGB_WIDTH(24),             // Set the RGB width to 24 bits (8 bits per channel).
    .PIXEL_GAP_THRESHOLD(PIXEL_GAP_THRESHOLD),
    .FRAME_START(FRAME_START)
) ld_wrapper_inst (
     //system interface
    .clk(clk),// The Clk signal of the Design which used in the Synchronization with the other modules.
    .rst_n(rst_n),// A reset signal to reset the whole signals of the design to nulls.
    
    //speed control unit interface
    .number_of_lanes(number_of_lanes), //indicates the number of lanes on the road
    .current_lane(current_lane), //indicates the current  lane the car is in
    .decision_out_valid(decision_out_valid), //indicates that the output of the decision module is valid
    .current_lane_left_boundry(current_lane_left_boundry), //the left  boundary of the current lane in pixels
    .current_lane_right_boundry(current_lane_right_boundry), //the right boundary of the current lane in pixels

    /* AXI Stream Interface */
    .s_axi_video_tdata(s_axi_video_tdata),
    .s_axi_video_tvalid(s_axi_video_tvalid),
    .s_axi_video_ready(s_axi_video_ready),
    .s_axi_video_tlast(s_axi_video_tlast)
);

endmodule
