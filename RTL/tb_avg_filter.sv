`timescale 1ns / 1ps

module tb_LD_Wrapper;

    // Parameters
    parameter NUMBER_OF_FRAMES = 1;
    parameter IMG_LENGTH_ORIGINAL = 416;  //image Hight should be multibles of 4 the true image hight
    parameter IMG_WIDTH  = 416;  //image width could be any value biger than 4
    parameter PIXEL_SIZE = 8;
    parameter THRESHOLD1  = 10000;
    parameter THRESHOLD2  = 50000;
    parameter TOTAL_SIZE = IMG_WIDTH * IMG_LENGTH_ORIGINAL*NUMBER_OF_FRAMES;
    parameter AXI_WIDTH  = 24;

    parameter FRAME_START = 150;
    parameter IMG_LENGTH_CROP = 150;   //the length of the croped image    
    parameter PIXEL_GAP_THRESHOLD = 5;

    parameter EXPECTED_LANE_COUNT = 2;

    // Testbench signals
    reg clk;
    reg rst_n;
    reg converter_data_valid;
    reg [AXI_WIDTH-1:0] pixel_in;
    wire valid_out;
    wire [PIXEL_SIZE-1:0] pixel_out;
    wire converter_read_ready;
    wire sof;
    wire eol;
    logic avr_filter_en;
    logic [4-1:0]                   number_of_lanes; //indicates the number of lanes in the image
    logic [4-1:0]                   current_lane; //indicates the current  lane the car is in
    logic                           decision_out_valid; //indicates that the output of the decition module is valid
    logic [$clog2(IMG_WIDTH):0]     current_lane_left_boundry; //the left  boundary of the current lane in pixels
    logic [$clog2(IMG_WIDTH):0]     current_lane_right_boundry; //the right boundary of the current lane in pixels

    logic [$clog2(IMG_WIDTH):0]     current_lane_left_boundary_array[NUMBER_OF_FRAMES]; //the left  boundary of the current lane in pixels
    logic [$clog2(IMG_WIDTH):0]     current_lane_right_boundary_array[NUMBER_OF_FRAMES]; //the right boundary of the current lane in pixels
    
    logic sobel_result1;
    logic sobel_result2;
    logic sobel_result_valid;
    logic start_of_line,end_of_line;
    logic s_axi_video_tlast;

    reg [AXI_WIDTH-1:0] pixel_array_out [TOTAL_SIZE]; // 2D array for pixels

    reg sobel_out_array [IMG_WIDTH * IMG_LENGTH_CROP * NUMBER_OF_FRAMES]; // 2D array for pixels
    
    class PixelGen;
    rand bit [7:0] pixel_in;  // 8-bit random pixel value

    constraint c1 {
        pixel_in == 0 dist {0 := 90, [1:255] := 10};  
    }
    endclass

    reg [24-1:0] pixel_array [TOTAL_SIZE]; // 2D array for pixels

    // Clock Generation
    always #5 clk = ~clk;  // 100MHz Clock (Period = 10ns)

    // DUT Instantiation
    LD_Wrapper #(
    .THRESHOLD1(THRESHOLD1),          // Set the threshold for lane detection to 1600.
    .THRESHOLD2(THRESHOLD2),          // Set the threshold for lane detection to 1600.
    .IMG_LENGTH(IMG_LENGTH_CROP),           // Set the image height to 10 pixels.
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
    .s_axi_video_tdata(pixel_in),
    .s_axi_video_tvalid(converter_data_valid),
    .s_axi_video_ready(converter_read_ready),
    .s_axi_video_tlast(s_axi_video_tlast)
);



    logic [$clog2(IMG_LENGTH_ORIGINAL * IMG_WIDTH * NUMBER_OF_FRAMES):0] i = 0;
    reg [9:0] counter=0;

    int correct_counter = 0,error_counter = 0;


    // Test Sequence
    initial begin
       $readmemh("C:/Users/saber/Desktop/GP/task 11/sim/lane_2.txt", pixel_array);
        // Initialize signals
        clk = 0;
        rst_n = 0;
        counter=0;

        // Reset the design
        @(posedge clk);

        rst_n = 1;
       
    //    // Test case: Send a single pixel
    //     while(i < IMG_LENGTH_ORIGINAL * IMG_WIDTH)begin
    //     converter_data_valid = 1;
    //     // pixel_in = pixel_array[i];
    //     @(posedge clk);
    //     if(converter_read_ready)begin
    //     pixel_in =$random;
    //     start_of_line = 1;
    //     end_of_line = 0;
    //     i=i+1;
    //     repeat(IMG_WIDTH-1)begin
    //     @(posedge clk);
    //     //  pixel_in = pixel_array[i];
    //      pixel_in =$random;
    //      i=i+1;
    //     end
    //     end_of_line = 1;

    //     @(posedge clk);
    //     end_of_line = 0;
    //     converter_data_valid = 0;
    //     end
    //     end
       
    repeat (NUMBER_OF_FRAMES)begin
      #1749130;
    end
        
        $writememh("memory_dump.txt", sobel_out_array);
        // $writememh("gray_out.txt", ld_wrapper_inst.cnv_avr_fifo_inst.mem);
        $writememh("left_boundary.txt", current_lane_left_boundary_array);
        $writememh("right_boundary.txt", current_lane_right_boundary_array);

        $display("lane count correct = %d \n lane count wrong = %d",correct_counter,error_counter);
       
         $stop;
    end
assign pixel_in = pixel_array[i];

always_ff @( posedge clk ) begin : blockName
            converter_data_valid <= 1;            
        if(converter_read_ready && i < TOTAL_SIZE)begin
            i<=i+1;
            s_axi_video_tlast <= 0;
             // tlast is high at the last pixel in each row
            if ((i % IMG_WIDTH) == IMG_WIDTH - 1) begin
            s_axi_video_tlast <= 1;
            end 
        end
        else begin
            converter_data_valid <= 0;
        end
    end

int k =0;
always_ff @(posedge clk) begin
    if(decision_out_valid)begin
       k <= k+1;
       current_lane_left_boundary_array[k] <= current_lane_left_boundry;
       current_lane_right_boundary_array[k] <= current_lane_right_boundry;
       if(number_of_lanes == EXPECTED_LANE_COUNT)begin
        correct_counter++;
       end else begin
        error_counter++;
       end
    end
    end
int l =0;

always_ff @(posedge clk) begin
    if(ld_wrapper_inst.decision_inst.start_flag)begin
        sobel_out_array[l] <= ld_wrapper_inst.decision_inst.first_threshold_sobel_result;
        l <= l+1;
    end
end

// logic [20:0] ccounter = 1;
// always_ff @( posedge clk && start_of_line ) begin : blockName
//         converter_data_valid <= 0;
//         if(converter_read_ready && i < IMG_LENGTH_ORIGINAL * IMG_WIDTH)begin
//             i<=i+1;
//             converter_data_valid <= 1;
//             pixel_in <= pixel_array[i]; 
//             ccounter<=ccounter+1;
//             if(s_axi_video_tlast)begin
//                 converter_data_valid <= 0;
//                 i<=i;
//             end
//         end
//     end

// always @(posedge clk or negedge rst_n) begin
//     if(!rst_n) begin
//         s_axi_video_tlast<=0;
//     end
//     else begin
//         if(((ccounter+1)%416)==0&&ccounter) begin
//             s_axi_video_tlast<=1;
//             $display("%d",ccounter);
//         end
//         else begin
//             s_axi_video_tlast<=0;
//     end
//    end
// end

endmodule

