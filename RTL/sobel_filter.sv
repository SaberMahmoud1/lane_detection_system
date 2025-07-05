////////////////////////////////////////////////////////////////////////////////
// Author: Saber Mahmoud
//
// Description: Sobel Filter Design for Lane Detection. This module applies the Sobel 
// edge detection filter to grayscale image data to detect edges, which are key 
// components in lane detection systems. It reads filtered image data from the 
// averaging filter module, applies the Sobel filter, and writes the processed 
// edge data to a decision module that determines the number of lanes based on 
// detected edges.
//
// The module uses multiple FIFO interfaces to manage the input image data, intermediate
// results, and the output data for lane decision-making.
//
// Local FIFO Interfaces (for row-wise data storage during processing):
// 1. **Previous Row FIFO:** Stores the previous row of image data for further processing.
//    - local_fifo_* signals (wr_en, wr_data, rd_en, rd_data, full, empty, ack) track FIFO status.
//
// 2. **Next Row FIFO:** Stores the next row of image data to be used in calculations.
//    - local_fifo_* signals (wr_en, wr_data, rd_en, rd_data, full, empty, ack) track FIFO status.
//
// 3. **Current Row FIFO:** Stores the current row of image data for Sobel filtering.
//    - local_fifo_* signals (wr_en, wr_data, rd_en, rd_data, full, empty, ack) track FIFO status.
//
////////////////////////////////////////////////////////////////////////////////

module sobel_filter #(
    parameter THRESHOLD1 = 9500,
    parameter THRESHOLD2 = 55000,
    parameter IMG_WIDTH =640,
    parameter IMG_LENGTH =640,
    parameter PIXEL_SIZE=8
) 
(
    input clk,                                        // The Clk signal of the Design which used in the Synchronization with the other modules.
    input rst_n,                                      // A reset signal to reset the whole signals of the design to nulls.

    //interface shared with average filter
    input  logic                           avr_data_valid    ,    
    input  logic       [PIXEL_SIZE-1:0]    avr_data_out   ,  

    //interface shared between decision and sobel
    output sobel_out_valid,
    output first_threshold_sobel_result,
    output second_threshold_sobel_result 
);

//local FIFO interface for storing the last row in the previous operation as it is needed in further processing
    logic                           local_fifo_wr_en_prv   ;   //when enabled the fifo takes data in
    logic [PIXEL_SIZE-1:0]          local_fifo_wr_data_prv ;   //the data to be written to the fifo
    logic                           local_fifo_full_prv    ;   //fifo full indicator    
    logic                           local_fifo_empty_prv   ;   //fifo empty indicator
    logic                           local_fifo_rd_en_prv   ;   //when enabled the fifo gives data out
    logic [PIXEL_SIZE-1:0]          local_fifo_rd_data_prv ;  //the data to be read from the fifo
    logic                           local_wr_ack_prv       ;   //ack signal to make sure the write operations is done right.
    logic                           local_rd_ack_prv       ;   //ack signal to make sure the read operations is done right. 
    logic [$clog2(IMG_WIDTH):0]   local_fifo_data_count_prv;

//local FIFO interface for storing the next row as needed in calc
    logic                           local_fifo_wr_en_nxt   ;   //when enabled the fifo takes data in
    logic [PIXEL_SIZE-1:0]          local_fifo_wr_data_nxt ;   //the data to be written to the fifo
    logic                           local_fifo_full_nxt    ;   //fifo full indicator    
    logic                           local_fifo_empty_nxt   ;   //fifo empty indicator
    logic                           local_fifo_rd_en_nxt   ;   //when enabled the fifo gives data out
    logic [PIXEL_SIZE-1:0]          local_fifo_rd_data_nxt ;  //the data to be read from the fifo
    logic                           local_wr_ack_nxt       ;   //ack signal to make sure the write operations is done right.
    logic                           local_rd_ack_nxt       ;   //ack signal to make sure the read operations is done right. 
    logic [$clog2(IMG_WIDTH):0]     local_fifo_data_count_nxt;

//local FIFO interface for storing the current row to do calculations in calc
    logic                           local_fifo_wr_en_crnt   ;   //when enabled the fifo takes data in
    logic [PIXEL_SIZE-1:0]          local_fifo_wr_data_crnt ;   //the data to be written to the fifo
    logic                           local_fifo_full_crnt    ;   //fifo full indicator    
    logic                           local_fifo_empty_crnt   ;   //fifo empty indicator
    logic                           local_fifo_rd_en_crnt   ;   //when enabled the fifo gives data out
    logic [PIXEL_SIZE-1:0]          local_fifo_rd_data_crnt ;  //the data to be read from the fifo
    logic                           local_wr_ack_crnt       ;   //ack signal to make sure the write operations is done right.
    logic                           local_rd_ack_crnt       ;   //ack signal to make sure the read operations is done right. 
    logic [$clog2(IMG_WIDTH):0]   local_fifo_data_count_crnt;    
    

            
    logic [2:0] Buffer_index;
    logic start_flag;

    logic [PIXEL_SIZE-1:0] sobel_window_out [0:2][0:2]; // 3x3 output window
    logic                 pixel_valid;  // New column available
    logic [$clog2(IMG_WIDTH):0] shift_count;

    logic [$clog2(IMG_LENGTH)-1:0] row;

    logic last_column,before_last_column; //to indicate the last column and the column before it to control fifos


    logic [PIXEL_SIZE-1:0] prv_fifo_r,crnt_fifo_r,nxt_fifo_r;
    logic signed [11:0] global_fifo_Gx_r,global_fifo_Gy_r;
    
    logic first_threshold_sobel_result_r,second_threshold_sobel_result_r;

    logic                           avr_data_valid_r    ;   //fifo full indicator    
    logic       [PIXEL_SIZE-1:0]    avr_data_out_r   ;   //fifo empty indicator


/*samble last stage output */
always_ff @(posedge clk or negedge rst_n) begin : samble_avr_results
    if(!rst_n)begin
        avr_data_valid_r <= 1'b0;
        avr_data_out_r <= 0;
    end else if(avr_data_valid)begin
        avr_data_valid_r <= 1'b1;
        avr_data_out_r <= avr_data_out;
    end else begin
        avr_data_valid_r <= 1'b0;
        avr_data_out_r <= 1'b0;
    end
end


/********************************************************************************************************************
                                  buffers filling with data
********************************************************************************************************************/

//the data that will be stored in the buffer is the input pixels and the prv fifo takes data from fifo 4 always
assign local_fifo_wr_data_prv = local_fifo_rd_data_crnt;
assign local_fifo_wr_data_crnt = (Buffer_index == 1) ? avr_data_out_r :local_fifo_rd_data_nxt;
assign local_fifo_wr_data_nxt = avr_data_out_r;

assign first_threshold_sobel_result  = first_threshold_sobel_result_r;
assign second_threshold_sobel_result = second_threshold_sobel_result_r;

//flags for the calculations
assign last_column = (shift_count >= IMG_WIDTH+2) ? 1 : 0; //detect the last column of the image
assign before_last_column = (shift_count >= IMG_WIDTH) ? 1 : 0; //detect the before last column of the image

assign sobel_out_valid = (pixel_valid && shift_count > 3);

//choose which fifo to write data into
always_comb begin : demux_the_wr_en
    local_fifo_wr_en_crnt=( Buffer_index == 1) ? avr_data_valid_r : local_rd_ack_nxt;
    local_fifo_wr_en_prv=(row == IMG_LENGTH - 1) ? 0 : local_rd_ack_crnt;
    local_fifo_wr_en_nxt = (Buffer_index == 2) ? avr_data_valid_r : 0;
end

assign Buffer_index = (start_flag == 0 && !local_fifo_full_crnt) ? 1 : 
                (!local_fifo_full_nxt ? 2 : 0);


/********************************************************************************************************************
                                  averiging the data in buffers
***********************************************************************************************************************/

assign prv_fifo_r = (shift_count >= IMG_WIDTH+1) ? 0 : local_fifo_rd_data_prv;
assign crnt_fifo_r =(shift_count >= IMG_WIDTH+1) ? 0 : local_fifo_rd_data_crnt;
assign nxt_fifo_r = (shift_count >= IMG_WIDTH+1) ? 0 : local_fifo_rd_data_nxt;


assign pixel_valid = (local_rd_ack_crnt) || before_last_column;
  // Instantiate the shift register module
   shift_reg_3x3 #(
    .PIXEL_SIZE(PIXEL_SIZE)
) shift_register_inst (
    .clk(clk),
    .rst_n(rst_n),
    .pixel_valid(pixel_valid),
    .fifo0_in(prv_fifo_r),
    .fifo1_in(crnt_fifo_r),
    .fifo2_in(nxt_fifo_r),
    .window(sobel_window_out)
);


/*instantiate local buffer to store the last row in the previous run*/

    FIFO #(
    .FIFO_WIDTH(PIXEL_SIZE),  
    .FIFO_DEPTH(IMG_WIDTH)   
 ) prev_data_buffer_inst (
    .clk(clk),                
    .rst_n(rst_n),            
    .wr_en(local_fifo_wr_en_prv),    
    .rd_en(local_fifo_rd_en_prv),    
    .data_in(local_fifo_wr_data_prv),  
    .data_out(local_fifo_rd_data_prv),
    .full(local_fifo_full_prv),      
    .empty(local_fifo_empty_prv),    
    .wr_ack(local_wr_ack_prv),       
    .rd_ack(local_rd_ack_prv),
    .count(local_fifo_data_count_prv)        
);


/*instantiate local buffer to store next row in the run for calc*/

    FIFO #(
    .FIFO_WIDTH(PIXEL_SIZE),  
    .FIFO_DEPTH(IMG_WIDTH)   
 ) nxt_data_buffer_inst (
    .clk(clk),                
    .rst_n(rst_n),            
    .wr_en(local_fifo_wr_en_nxt),    
    .rd_en(local_fifo_rd_en_nxt),    
    .data_in(local_fifo_wr_data_nxt),  
    .data_out(local_fifo_rd_data_nxt),
    .full(local_fifo_full_nxt),      
    .empty(local_fifo_empty_nxt),    
    .wr_ack(local_wr_ack_nxt),       
    .rd_ack(local_rd_ack_nxt),
    .count(local_fifo_data_count_nxt)        
);

/*instantiate local buffer to store current row in the run for calc*/

    FIFO #(
    .FIFO_WIDTH(PIXEL_SIZE),  
    .FIFO_DEPTH(IMG_WIDTH)   
    ) crnt_data_buffer_inst (
    .clk(clk),                
    .rst_n(rst_n),            
    .wr_en(local_fifo_wr_en_crnt),    
    .rd_en(local_fifo_rd_en_crnt),    
    .data_in(local_fifo_wr_data_crnt),  
    .data_out(local_fifo_rd_data_crnt),
    .full(local_fifo_full_crnt),      
    .empty(local_fifo_empty_crnt),    
    .wr_ack(local_wr_ack_crnt),       
    .rd_ack(local_rd_ack_crnt),
    .count(local_fifo_data_count_crnt)        
);


// Instantiate the edge detection module
    edge_detection #(
        .THRESHOLD1(THRESHOLD1),
        .THRESHOLD2(THRESHOLD2)
    ) ed1_inst (
        .clk(clk),
        .rst_n(rst_n),
        .Gx(global_fifo_Gx_r),
        .Gy(global_fifo_Gy_r),
        .result1(first_threshold_sobel_result_r),
        .result2(second_threshold_sobel_result_r)
    );



//choose which fifo to write data into

always_comb begin : demux_the_rd_en
    local_fifo_rd_en_prv  = 0;
    local_fifo_rd_en_crnt = 0;
    local_fifo_rd_en_nxt  = 0;

    if(start_flag && !before_last_column)begin
        local_fifo_rd_en_crnt = 1;
        local_fifo_rd_en_nxt  = 1;
        local_fifo_rd_en_prv =  (row == 0) ? 0 : 1 ;
    end
end


always_ff @(posedge clk or negedge rst_n) begin : calc
if(!rst_n)begin
    row <= 1'b0 ;
    shift_count <= 0;
    start_flag <= 0;
    global_fifo_Gx_r <=0;
    global_fifo_Gy_r <=0;

end
else if(start_flag)begin
    if (shift_count > 2) begin
    if (row == 0) begin
        global_fifo_Gx_r <=  -(sobel_window_out[1][0] << 1) + (sobel_window_out[1][2] << 1) +
                             -sobel_window_out[2][0] + sobel_window_out[2][2];

        global_fifo_Gy_r <=  sobel_window_out[2][0] + (sobel_window_out[2][1] << 1) + sobel_window_out[2][2];
    
    end else if (row == IMG_LENGTH - 1) begin
        global_fifo_Gx_r <=  -sobel_window_out[0][0] + sobel_window_out[0][2] +
                             -(sobel_window_out[1][0] << 1) + (sobel_window_out[1][2] << 1);

        global_fifo_Gy_r <=  -sobel_window_out[0][0] - (sobel_window_out[0][1] << 1) - sobel_window_out[0][2];

    end else begin
        global_fifo_Gx_r <=  -sobel_window_out[0][0] + sobel_window_out[0][2] +
                             -(sobel_window_out[1][0] << 1) + (sobel_window_out[1][2] << 1) +
                             -sobel_window_out[2][0] + sobel_window_out[2][2];

        global_fifo_Gy_r <=  -sobel_window_out[0][0] - (sobel_window_out[0][1] << 1) - sobel_window_out[0][2] +
                              sobel_window_out[2][0] + (sobel_window_out[2][1] << 1) + sobel_window_out[2][2];
    end
end

    shift_count<=shift_count+1;
        
    if(shift_count==IMG_WIDTH+3)begin
        row<=row+1;
        shift_count<=0;
    end


    if((local_fifo_empty_nxt && local_fifo_empty_crnt && shift_count==IMG_WIDTH+3))begin
        start_flag <= 0;
        if(local_fifo_empty_nxt && local_fifo_empty_crnt && shift_count==IMG_WIDTH+3)begin
            row <= 0 ;
        end
    end
end else begin
    if(local_fifo_data_count_nxt > 3)begin
        start_flag <= 1;
    end
end
  
end


endmodule