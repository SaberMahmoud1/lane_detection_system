module imgcrop #(parameter FRAME_HEIGHT = 416 ,parameter FRAME_START = 0) (clk, rst_n, s_axi_video_tvalid, s_axi_video_tlast, s_axi_video_ready, valid_cnvrtr);
//declaring paramters
    localparam  AXI_IMG_HIGHT = 416;
    // localparam  FRAME_START = AXI_IMG_HIGHT - FRAME_HEIGHT;//number of skipped rows
    localparam  FRAME_END = FRAME_START + FRAME_HEIGHT;//last bypassed row before skipping again

//declaring ports
    input clk, rst_n, s_axi_video_tlast, s_axi_video_tvalid, s_axi_video_ready;
    output reg valid_cnvrtr;
//counter for the tlast signal (bypass only rows under concern)
    reg [$clog2(AXI_IMG_HIGHT):0] row_counter;
//counter logic
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            row_counter<=0;
        end
        else begin
            if(row_counter==AXI_IMG_HIGHT) begin
                row_counter<=0;
            end
            else if(s_axi_video_tvalid&&s_axi_video_tlast&&s_axi_video_ready) begin
                row_counter<=row_counter+1;
            end
        end
    end
    always @(*) begin
        if(s_axi_video_tvalid&&(row_counter>=FRAME_START&&row_counter<FRAME_END)) begin
            valid_cnvrtr = s_axi_video_tvalid;
        end
        else begin
            valid_cnvrtr = 0;
        end
    end

endmodule