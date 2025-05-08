////////////////////////////////////////////////////////////////////////////////
// Author: Saber Mahmoud
//
// Description: Averaging Filter for Preprocessing Grayscale Image Data in Lane Detection System.
// This module applies an averaging filter to grayscale pixel data, smoothing the image 
// and reducing noise. It reads grayscale pixel data from the RGB to Grayscale Converter 
// module, applies the averaging filter, and writes the processed data to the Sobel filter 
// for edge detection. The output from the Sobel filter will be passed to a decision module 
// for lane detection based on the edges identified.
//
// The averaging filter reduces pixel variations by averaging neighboring pixels, 
// which is essential for preparing the data before applying edge detection.
//
// The module uses FIFO interfaces to manage the image data flow during processing, 
// storing previous, current, and next row data for the filter operation.
//
// Additionally, the module contains a 3x3 shift register, which combines data 
// from three rows (previous, current, and next rows) of pixels to form a 3x3 window. 
// This window is used for calculating the average pixel value in the current clock cycle.
//
// Parameters:
// - IMG_WIDTH    : Width of the input image in pixels (default: 640).
// - IMG_LENGTH   : Height of the input image in pixels (default: 640).
// - PIXEL_SIZE   : Bit-width of each grayscale pixel (default: 8).
//
// Inputs:
// - clk          : Clock signal for synchronization with other modules.
// - rst_n        : Active-low reset signal to initialize the module signals.
// - converter_data_valid : Indicates that the grayscale pixel data is valid and ready for processing.
// - pixel_in     : A grayscale pixel input from the RGB to grayscale converter module.
//
// FIFO Interfaces:
// 1. **Sobel FIFO Interface:**
//    - avr_sobel_fifo_wr_en    : Enables FIFO write operation when high.
//    - avr_sobel_fifo_wr_data  : Data to be written to the Sobel FIFO (filtered pixel).
//    - avr_sobel_fifo_full     : FIFO full indicator (high when FIFO is full).
//    - avr_sobel_fifo_empty    : FIFO empty indicator (high when FIFO is empty).
//    - avr_sobel_wr_ack        : Acknowledgment for successful write operations to Sobel FIFO.
//    - avr_sobel_rd_ack        : Acknowledgment for successful read operations from Sobel FIFO.
//    - avr_sobel_fifo_data_count : Number of data elements in the FIFO.
//
// 2. **Previous Row FIFO (for the previous row's pixel data):**
//    - local_fifo_* signals (wr_en, wr_data, rd_en, rd_data, full, empty, ack) track FIFO status.
//
// 3. **Next Row FIFO (for the next row's pixel data):**
//    - local_fifo_* signals (wr_en, wr_data, rd_en, rd_data, full, empty, ack) track FIFO status.
//
// 4. **Current Row FIFO (for the current row's pixel data being processed):**
//    - local_fifo_* signals (wr_en, wr_data, rd_en, rd_data, full, empty, ack) track FIFO status.
//
// States:
// - IDLE       : The module is waiting for valid data to process.
// - WRITE_DATA : The module is processing and writing the data to the Sobel filter FIFO.
//
// Shift Register Module (3x3 Window) Instantiation:
// - The shift register collects data from the previous, current, and next rows 
//   to form a 3x3 window for averaging. The resulting window is used in the 
//   current clock cycle to compute the average pixel value.
////////////////////////////////////////////////////////////////////////////////

module Avg_filter #(
    parameter IMG_WIDTH =640,
    parameter IMG_LENGTH =640,
    parameter PIXEL_SIZE=8
) 
(
    input clk,                                        // The Clk signal of the Design which used in the Synchronization with the other modules.
    input rst_n,                                      // A reset signal to reset the whole signals of the design to nulls.
    input converter_data_valid,                                   // A signal from the memory that indicates it valided saving the image.
    input [PIXEL_SIZE-1:0] pixel_in,                   // The input Greyscale Pixel from the memory.
    output reg converter_read_ready,                                 // Signal to indicate when the Converter should send it's Pixel.

    //FIFO interface
    output logic                           avr_sobel_fifo_wr_en  ,   //when enabled the FIFO takes data in
    output logic [PIXEL_SIZE-1:0]          avr_sobel_fifo_wr_data,   //the data to be written to the FIFO
    input  logic                           avr_sobel_fifo_full    ,   //FIFO full indicator    
    input  logic                           avr_sobel_fifo_empty   ,   //FIFO empty indicator
    input  logic                           avr_sobel_wr_ack        ,   //ack signal to make sure the write operation is done right.
    input  logic                           avr_sobel_rd_ack         ,   //ack signal to make sure the read operation is done right. 
    input  logic [$clog2(IMG_WIDTH):0]   avr_sobel_fifo_data_count     //the number of data stored in the FIFO

);

//local FIFO interface for storing the last row in the previous operation as it is needed in further processing
    logic                           local_fifo_wr_en_prv   ;   //when enabled the FIFO takes data in
    logic [PIXEL_SIZE-1:0]          local_fifo_wr_data_prv ;   //the data to be written to the FIFO
    logic                           local_fifo_full_prv    ;   //FIFO full indicator    
    logic                           local_fifo_empty_prv   ;   //FIFO empty indicator
    logic                           local_fifo_rd_en_prv   ;   //when enabled the FIFO gives data out
    logic [PIXEL_SIZE-1:0]          local_fifo_rd_data_prv ;  //the data to be read from the FIFO
    logic                           local_wr_ack_prv       ;   //ack signal to make sure the write operations is done right.
    logic                           local_rd_ack_prv       ;   //ack signal to make sure the read operations is done right. 
    logic [$clog2(IMG_WIDTH):0]   local_fifo_data_count_prv;

//local FIFO interface for storing the next row as needed in calc
    logic                           local_fifo_wr_en_nxt   ;   //when enabled the FIFO takes data in
    logic [PIXEL_SIZE-1:0]          local_fifo_wr_data_nxt ;   //the data to be written to the FIFO
    logic                           local_fifo_full_nxt    ;   //FIFO full indicator    
    logic                           local_fifo_empty_nxt   ;   //FIFO empty indicator
    logic                           local_fifo_rd_en_nxt   ;   //when enabled the FIFO gives data out
    logic [PIXEL_SIZE-1:0]          local_fifo_rd_data_nxt ;  //the data to be read from the FIFO
    logic                           local_wr_ack_nxt       ;   //ack signal to make sure the write operations is done right.
    logic                           local_rd_ack_nxt       ;   //ack signal to make sure the read operations is done right. 
    logic [$clog2(IMG_WIDTH):0]   local_fifo_data_count_nxt;

//local FIFO interface for storing the current row to do calculations in calc
    logic                           local_fifo_wr_en_crnt   ;   //when enabled the FIFO takes data in
    logic [PIXEL_SIZE-1:0]          local_fifo_wr_data_crnt ;   //the data to be written to the FIFO
    logic                           local_fifo_full_crnt    ;   //FIFO full indicator    
    logic                           local_fifo_empty_crnt   ;   //FIFO empty indicator
    logic                           local_fifo_rd_en_crnt   ;   //when enabled the FIFO gives data out
    logic [PIXEL_SIZE-1:0]          local_fifo_rd_data_crnt ;  //the data to be read from the FIFO
    logic                           local_wr_ack_crnt       ;   //ack signal to make sure the write operations is done right.
    logic                           local_rd_ack_crnt       ;   //ack signal to make sure the read operations is done right. 
    logic [$clog2(IMG_WIDTH):0]   local_fifo_data_count_crnt;    
    

typedef enum logic  {
    IDLE,
    WRITE_DATA
} buffer_write_t;       

buffer_write_t w_buff_cs,w_buff_ns;
            
 logic [2:0] Buffer_index;
 logic en_index_sample;
 logic start_flag;
 logic [PIXEL_SIZE+8:0] global_fifo_r;

 logic [PIXEL_SIZE-1:0] avr_window_out [0:2][0:2]; // 3x3 output window
 logic                 pixel_valid;  // New column available
 logic [$clog2(IMG_WIDTH):0] shift_count;

 logic [$clog2(IMG_LENGTH):0] row;

logic last_column,before_last_column; //to indicate the last column and the column before it to control fifos


logic [PIXEL_SIZE-1:0] prv_fifo_r,crnt_fifo_r,nxt_fifo_r;




/********************************************************************************************************************
                                  buffers filling with data
********************************************************************************************************************/

//the data that will be stored in the buffer is the input pixels and the prv FIFO takes data from FIFO 4 always
assign local_fifo_wr_data_prv = local_fifo_rd_data_crnt;
assign local_fifo_wr_data_crnt=(Buffer_index == 1) ? pixel_in :local_fifo_rd_data_nxt;
assign local_fifo_wr_data_nxt=pixel_in;

assign avr_sobel_fifo_wr_data = global_fifo_r;

//flags for the calculations
assign last_column = (shift_count >= IMG_WIDTH+2) ? 1 : 0; //detect the last column of the image
assign before_last_column = (shift_count >= IMG_WIDTH) ? 1 : 0; //detect the before last column of the image

//choose which FIFO to write data into

always_comb begin : demux_the_wr_en
    local_fifo_wr_en_crnt=(Buffer_index == 1) ? converter_data_valid : local_rd_ack_nxt;
    local_fifo_wr_en_prv=(row == IMG_LENGTH - 1 ) ? 0 : local_rd_ack_crnt;
    local_fifo_wr_en_nxt = (Buffer_index == 2) ? converter_read_ready && converter_data_valid || w_buff_ns==IDLE && converter_data_valid  : 0;
    avr_sobel_fifo_wr_en = (!avr_sobel_fifo_full && pixel_valid && shift_count > 3);
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
        IDLE:         converter_read_ready = 0;
        WRITE_DATA:   converter_read_ready = (w_buff_ns == IDLE) ? 0 : 1;
        default:      converter_read_ready = 0;
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
    .window(avr_window_out)
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

//choose which FIFO to write data into

always_comb begin : demux_the_rd_en
    
    local_fifo_rd_en_prv  = 0;
    local_fifo_rd_en_crnt = 0;
    local_fifo_rd_en_nxt  = 0;

    if(start_flag && !before_last_column)begin
        local_fifo_rd_en_crnt = 1;
        local_fifo_rd_en_nxt  = 1;
        local_fifo_rd_en_prv =  (row == 0 ) ? 0 : 1 ;
    end

end

always_ff @(posedge clk or negedge rst_n) begin : calc
if(!rst_n)begin
    row <= 1'b0 ;
    global_fifo_r<=0;
    shift_count <= 0;
    start_flag <= 0;
end
else if(start_flag)begin
    if(shift_count>2)begin
        if(row == 0)begin
            
        global_fifo_r <= ((avr_window_out[1][0] + avr_window_out[1][1] + avr_window_out[1][2] +
                           avr_window_out[2][0] + avr_window_out[2][1] + avr_window_out[2][2] )*57)>>9;                           
        
        end else if(row == IMG_LENGTH-1) begin

        global_fifo_r <= ((avr_window_out[0][0] + avr_window_out[0][1] + avr_window_out[0][2] +
                           avr_window_out[1][0] + avr_window_out[1][1] + avr_window_out[1][2])*57)>>9;             

        end else begin

        global_fifo_r <=  ((avr_window_out[0][0] + avr_window_out[0][1] + avr_window_out[0][2]  +
                            avr_window_out[1][0] + avr_window_out[1][1] + avr_window_out[1][2]  +
                            avr_window_out[2][0] + avr_window_out[2][1] + avr_window_out[2][2] )*57)>>9;                
        end     
    end
    
    shift_count<=shift_count+1;
        
    if(shift_count==IMG_WIDTH+3)begin
        row<=row+1;
        shift_count<=0;
    end


    if(avr_sobel_fifo_full || (local_fifo_empty_nxt && local_fifo_empty_crnt && shift_count==IMG_WIDTH+3))begin
        start_flag <= 0;
        if(local_fifo_empty_nxt && local_fifo_empty_crnt && shift_count==IMG_WIDTH+3)begin
            row <= 0 ;
        end
    end
end else begin
    if(local_fifo_data_count_nxt > 3 && !avr_sobel_fifo_full)begin
        start_flag <= 1;
    end
end
  
end


endmodule