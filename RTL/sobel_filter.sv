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
// Parameters:
// - THRESHOLD    : Edge detection threshold to classify strong edges (default: 22500).
// - IMG_WIDTH    : Width of the input image in pixels (default: 640).
// - IMG_LENGTH   : Height of the input image in pixels (default: 640).
// - PIXEL_SIZE   : Bit-width of each grayscale pixel (default: 8).
//
// Inputs:
// - clk          : Clock signal for synchronization with other modules.
// - rst_n        : Active-low reset signal to initialize the module signals.
//
// FIFO Interfaces:
// 1. **Averaging Filter FIFO Interface:** 
//    - avr_sobel_fifo_full    : FIFO full indicator (high when FIFO is full).
//    - avr_sobel_fifo_empty   : FIFO empty indicator (high when FIFO is empty).
//    - avr_sobel_fifo_rd_en   : Enables FIFO read operation when high.
//    - avr_sobel_fifo_rd_data : Data read from the FIFO (filtered pixel).
//    - avr_sobel_wr_ack       : Acknowledgment for successful write operations.
//    - avr_sobel_rd_ack       : Acknowledgment for successful read operations.
//    - avr_sobel_fifo_data_count : Number of data elements currently in the FIFO.
//
// 2. **Decision FIFO Interface (for lane detection):** 
//    - decision_sobel_fifo_wr_en : Enables FIFO write operation when high.
//    - decision_sobel_fifo_wr_data : Data written to the decision FIFO (edge data).
//    - decision_sobel_fifo_full   : FIFO full indicator (high when FIFO is full).
//    - decision_sobel_fifo_empty  : FIFO empty indicator (high when FIFO is empty).
//    - decision_sobel_wr_ack      : Acknowledgment for successful write operations.
//    - decision_sobel_rd_ack      : Acknowledgment for successful read operations.
//    - decision_sobel_fifo_data_count : Number of data elements currently in the FIFO.
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
// States:
// - IDLE       : The module is in a waiting state with no active processing.
// - WRITE_DATA : The module is processing and writing data to the output FIFO for decision-making.
////////////////////////////////////////////////////////////////////////////////

module sobel_filter #(
    parameter THRESHOLD =22500,
    parameter IMG_WIDTH =640,
    parameter IMG_LENGTH =640,
    parameter PIXEL_SIZE=8
) 
(
    input clk,                                        // The Clk signal of the Design which used in the Synchronization with the other modules.
    input rst_n,                                      // A reset signal to reset the whole signals of the design to nulls.

    //FIFO interface shared with average filter
    input  logic                           avr_sobel_fifo_full    ,   //fifo full indicator    
    input  logic                           avr_sobel_fifo_empty   ,   //fifo empty indicator
    output logic                           avr_sobel_fifo_rd_en   ,   //when enabled the fifo gives data out
    input  logic [PIXEL_SIZE-1:0]          avr_sobel_fifo_rd_data ,  //the data to be read from the fifo
    input  logic                           avr_sobel_wr_ack       ,   //ack signal to make sure the write operations is done right.
    input  logic                           avr_sobel_rd_ack       ,   //ack signal to make sure the read operations is done right. 
    input  logic [$clog2(IMG_WIDTH):0]     avr_sobel_fifo_data_count,

     //FIFO interface shared with decition
    output logic                           decision_sobel_fifo_wr_en  ,   //when enabled the fifo takes data in
    output logic                           decision_sobel_fifo_wr_data,   //the data to be written to the fifo
    input  logic                           decision_sobel_fifo_full   ,   //fifo full indicator    
    input  logic                           decision_sobel_fifo_empty  ,   //fifo empty indicator
    input  logic                           decision_sobel_wr_ack      ,   //ack signal to make sure the write operations is done right.
    input  logic                           decision_sobel_rd_ack      ,   //ack signal to make sure the read operations is done right. 
    input  logic [$clog2(IMG_WIDTH):0]     decision_sobel_fifo_data_count 

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
    logic [$clog2(IMG_WIDTH):0]   local_fifo_data_count_nxt;

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
    
    reg sobel_read_ready;

typedef enum logic  {
    IDLE,
    WRITE_DATA
} buffer_write_t;        

buffer_write_t w_buff_cs,w_buff_ns;
            
 logic [2:0] Buffer_index;
 logic en_index_sample;
 logic start_flag;

 logic [PIXEL_SIZE-1:0] sobel_window_out [0:2][0:2]; // 3x3 output window
 logic                 pixel_valid;  // New column available
 logic [$clog2(IMG_WIDTH):0] shift_count;

 logic [$clog2(IMG_LENGTH)-1:0] row;

logic last_column,before_last_column; //to indicate the last column and the column before it to control fifos


logic [PIXEL_SIZE-1:0] prv_fifo_r,crnt_fifo_r,nxt_fifo_r;
logic signed [PIXEL_SIZE+7:0] global_fifo_Gx_r,global_fifo_Gy_r;
logic global_fifo_G_r;

logic frame_done;


/********************************************************************************************************************
                                  buffers filling with data
********************************************************************************************************************/

//the data that will be stored in the buffer is the input pixels and the prv fifo takes data from fifo 4 always
assign local_fifo_wr_data_prv=local_fifo_rd_data_crnt;
assign local_fifo_wr_data_crnt=(Buffer_index == 1) ? avr_sobel_fifo_rd_data :local_fifo_rd_data_nxt;
assign local_fifo_wr_data_nxt=avr_sobel_fifo_rd_data;

assign decision_sobel_fifo_wr_data = global_fifo_G_r;

//flags for the calculations
assign last_column = (shift_count >= IMG_WIDTH+2) ? 1 : 0; //detect the last column of the image
assign before_last_column = (shift_count >= IMG_WIDTH) ? 1 : 0; //detect the before last column of the image

//choose which fifo to write data into

always_comb begin : demux_the_wr_en
    local_fifo_wr_en_crnt=( Buffer_index == 1) ? avr_sobel_rd_ack : local_rd_ack_nxt;
    local_fifo_wr_en_prv=(row == IMG_LENGTH - 1) ? 0 : local_rd_ack_crnt;
    local_fifo_wr_en_nxt = (Buffer_index == 2) ? avr_sobel_rd_ack : 0;
    decision_sobel_fifo_wr_en = (!decision_sobel_fifo_full && pixel_valid && shift_count > 3);
end

// State transation logic

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        w_buff_cs <= IDLE; 
    end else begin 
        w_buff_cs <= w_buff_ns;
    end
end


always_comb begin : Next_State_logic      
    en_index_sample = 0;
    case (w_buff_cs)
             IDLE : begin
            if(!local_fifo_full_nxt) begin
                en_index_sample = 1;
                w_buff_ns = WRITE_DATA;
            end else begin
                w_buff_ns = IDLE;
            end
        end
        WRITE_DATA: begin
            if((local_fifo_data_count_nxt == IMG_WIDTH-1) || (local_fifo_data_count_crnt  == IMG_WIDTH-1 && Buffer_index == 1))begin
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
        IDLE:         sobel_read_ready = 0;
        WRITE_DATA:   sobel_read_ready = 1;
        default:      sobel_read_ready = 0;
    endcase
end



/*sample the index of the buffer that the data is going to be writen into */
  always_ff @(posedge clk or negedge rst_n) begin : line_index_sample
    if (!rst_n) begin
        Buffer_index <= 0;
    end else if (en_index_sample) begin
        Buffer_index <= local_fifo_empty_crnt ? 1 : 
                (!local_fifo_full_nxt ? 2 : 0);
    end
end


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
        .THRESHOLD(THRESHOLD)
    ) ed1_inst (
        .Gx(global_fifo_Gx_r),
        .Gy(global_fifo_Gy_r),
        .result(global_fifo_G_r)
    );



//choose which fifo to write data into

always_comb begin : demux_the_rd_en
    
    local_fifo_rd_en_prv  = 0;
    local_fifo_rd_en_crnt = 0;
    local_fifo_rd_en_nxt  = 0;
    avr_sobel_fifo_rd_en = sobel_read_ready;

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
    if(shift_count>2)begin
        if(row == 0)begin
            
        global_fifo_Gx_r <=  -2* sobel_window_out[1][0]  + 2* sobel_window_out[1][2] +
                             -1* sobel_window_out[2][0]  + sobel_window_out[2][2] ;
        
        global_fifo_Gy_r <=   sobel_window_out[2][0] + 2* sobel_window_out[2][1] + sobel_window_out[2][2]   ;                                              
        
        end else if(row == IMG_LENGTH-1) begin

        global_fifo_Gx_r <=   -1* sobel_window_out[0][0] +    sobel_window_out[0][2] +
                              -2* sobel_window_out[1][0] + 2* sobel_window_out[1][2] ;
        
        global_fifo_Gy_r <=   -1* sobel_window_out[0][0] + -2*  sobel_window_out[0][1] + -1* sobel_window_out[0][2]   ;         

        end else begin

        global_fifo_Gx_r <=   -1* sobel_window_out[0][0] +   sobel_window_out[0][2] +
                              -2* sobel_window_out[1][0] +2* sobel_window_out[1][2] +
                              -1* sobel_window_out[2][0] +   sobel_window_out[2][2] ;
        
        global_fifo_Gy_r <=   -1* sobel_window_out[0][0] + -2* sobel_window_out[0][1] + -1* sobel_window_out[0][2] +
                                  sobel_window_out[2][0] +  2* sobel_window_out[2][1] +     sobel_window_out[2][2]   ;
        end
        
    end

    shift_count<=shift_count+1;
        
    if(shift_count==IMG_WIDTH+3)begin
        row<=row+1;
        shift_count<=0;
    end


    if(decision_sobel_fifo_full || (local_fifo_empty_nxt && local_fifo_empty_crnt && shift_count==IMG_WIDTH+3))begin
        start_flag <= 0;
        if(local_fifo_empty_nxt && local_fifo_empty_crnt && shift_count==IMG_WIDTH+3)begin
            row <= 0 ;
        end
    end
end else begin
    if(local_fifo_data_count_nxt > 3 && !decision_sobel_fifo_full)begin
        start_flag <= 1;
    end
end
  
end


endmodule