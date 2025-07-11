module rgb_converter_axi #(
    parameter  IMG_WIDTH  =416,
    parameter  AXI_WIDTH  = 24,
    parameter  RGB_WIDTH  = 24,
    parameter  GRAY_WIDTH = 8
)(
  
    input wire clk,                // The Clk signal of the Design which used in the Synchronization with the other modules.
    input wire rst_n,              // A reset signal to reset the whole signals of the design to nulls. 
    
    /* AXI Stream Interface */

    input  wire  [AXI_WIDTH-1:0] s_axi_video_tdata,
    input  wire  s_axi_video_tvalid,
    output reg  s_axi_video_ready,

    /* Outputs of the converter to the nextstage   */  
    //FIFO interface
    output wire                           cnv_avr_fifo_wr_en  ,   //when enabled the FIFO takes data in
    output reg [GRAY_WIDTH-1:0]           cnv_avr_fifo_wr_data,   //the data to be written to the FIFO
    input  wire                           cnv_avr_fifo_full    ,   //FIFO full indicator    
    input  wire                           cnv_avr_fifo_empty   ,   //FIFO empty indicator
    input  wire                           cnv_avr_wr_ack        ,   //ack signal to make sure the write operation is done right.
    input  wire                           cnv_avr_rd_ack         ,   //ack signal to make sure the read operation is done right. 
    input  wire [$clog2(IMG_WIDTH):0]     cnv_avr_fifo_data_count     //the number of data stored in the FIFO

);

    //conversion values
    localparam RED_VALUE = 8'b000_01011;
    localparam GREEN_VALUE = 8'b00_100101;
    localparam BLUE_VALUE = 8'b0_0001111;

    //counters and registers and flags
    reg [AXI_WIDTH-1:0] rgb_buffer;
    reg buffer_has_data,read_ready;
    
    wire ready;

    assign ready = (cnv_avr_fifo_data_count <= IMG_WIDTH- 2) ? 1 : 0;
    assign cnv_avr_fifo_wr_en = ((ready)&&(buffer_has_data||(s_axi_video_tvalid&&read_ready))) ? 1 : 0;
    //axis_ready
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            s_axi_video_ready<=0;
            read_ready<=0;
        end
        else begin
           s_axi_video_ready<=ready;
           read_ready<=ready;
        end
    end

    //buffer & conversion logic
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            rgb_buffer<=0;
            buffer_has_data<=0;
            cnv_avr_fifo_wr_data<=0;
        end
        else begin
            if((s_axi_video_tvalid&&read_ready)) begin
                rgb_buffer<=s_axi_video_tdata;
                if(ready) begin
                    if(buffer_has_data) begin
                        cnv_avr_fifo_wr_data<=graycnvrtr(rgb_buffer[RGB_WIDTH-1:0]);
                        buffer_has_data<=0;
                    end
                    else begin
                        cnv_avr_fifo_wr_data<=graycnvrtr(s_axi_video_tdata[RGB_WIDTH-1:0]);
                        buffer_has_data<=0;                        
                    end
                end
                else begin
                   buffer_has_data<=1; 
                end
            end
        end
    end




//red component
function [7:0] div_3;
    input reg [7:0] a;
    input reg [7:0] b;
    reg[15:0] ab;
    begin
        ab = a*b;
        div_3= ab[12:5]; 
    end
endfunction
//green component
function [7:0] div_58;
    input reg [7:0] a;
    input reg [7:0] b;
    reg[15:0] ab;
    begin
        ab = a*b;
        div_58= ab[13:6]; 
    end
endfunction
//blue component
function [7:0] div_11;
    input reg [7:0] a;
    input reg [7:0] b;
    reg[15:0] ab;
    begin
        ab = a*b;
        div_11= ab[14:7]; 
    end
endfunction
//RGB2Gray
function [7:0] graycnvrtr;
    input [23:0] rgb;
    reg [8:0] tmp;
    begin 
    tmp = div_3(rgb[7:0],RED_VALUE) + div_58(rgb[15:8],GREEN_VALUE) + div_11(rgb[23:16],BLUE_VALUE);
    if(tmp[8]==1)begin
        graycnvrtr = 255;
    end 
    else begin
        graycnvrtr = tmp;
    end
    end
endfunction
endmodule