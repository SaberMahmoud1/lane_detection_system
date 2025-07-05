add wave -position insertpoint  \
sim:/tb_LD_Wrapper/clk
add wave -position insertpoint  \
{sim:/tb_LD_Wrapper/ld_wrapper_inst/fifo_inst_gen[1]/fifo_inst/mem}
add wave -position insertpoint  \
{sim:/tb_LD_Wrapper/ld_wrapper_inst/fifo_inst_gen[2]/fifo_inst/mem}
add wave -position insertpoint  \
sim:/tb_LD_Wrapper/ld_wrapper_inst/avg_filter_inst/crnt_data_buffer_inst/mem
add wave -position insertpoint  \
sim:/tb_LD_Wrapper/ld_wrapper_inst/avg_filter_inst/nxt_data_buffer_inst/mem
add wave -position insertpoint  \
sim:/tb_LD_Wrapper/ld_wrapper_inst/avg_filter_inst/prev_data_buffer_inst/mem

add wave -position insertpoint  \
sim:/tb_LD_Wrapper/ld_wrapper_inst/rgb_converter_inst/gray_pixel

add wave -position insertpoint  \
sim:/tb_LD_Wrapper/ld_wrapper_inst/avg_filter_inst/avr_window_out

add wave -position insertpoint  \
sim:/tb_LD_Wrapper/ld_wrapper_inst/avg_filter_inst/global_fifo_r



add wave -position insertpoint  \
sim:/tb_LD_Wrapper/ld_wrapper_inst/avg_filter_inst/local_fifo_data_count_crnt \
sim:/tb_LD_Wrapper/ld_wrapper_inst/avg_filter_inst/local_fifo_data_count_nxt \
sim:/tb_LD_Wrapper/ld_wrapper_inst/avg_filter_inst/local_fifo_data_count_prv
add wave -position insertpoint  \
sim:/tb_LD_Wrapper/ld_wrapper_inst/avg_filter_inst/shift_count
add wave -position insertpoint  \
sim:/tb_LD_Wrapper/ld_wrapper_inst/avg_filter_inst/start_flag
add wave -position insertpoint  \
sim:/tb_LD_Wrapper/ld_wrapper_inst/avg_filter_inst/w_buff_cs
add wave -position insertpoint  \
sim:/tb_LD_Wrapper/ld_wrapper_inst/avg_filter_inst/w_buff_ns


add wave -position insertpoint  \
sim:/tb_LD_Wrapper/ld_wrapper_inst/avr_sobel_fifo_data_count
add wave -position insertpoint  \
sim:/tb_LD_Wrapper/ld_wrapper_inst/converter_data_valid \
sim:/tb_LD_Wrapper/ld_wrapper_inst/converter_read_ready

add wave -position insertpoint  \
sim:/tb_LD_Wrapper/ld_wrapper_inst/sobel_filter_inst/local_fifo_data_count_crnt
add wave -position insertpoint  \
sim:/tb_LD_Wrapper/ld_wrapper_inst/sobel_filter_inst/local_fifo_data_count_nxt
add wave -position insertpoint  \
sim:/tb_LD_Wrapper/ld_wrapper_inst/sobel_filter_inst/local_fifo_data_count_prv
add wave -position insertpoint  \
sim:/tb_LD_Wrapper/ld_wrapper_inst/sobel_filter_inst/local_fifo_rd_en_crnt \
sim:/tb_LD_Wrapper/ld_wrapper_inst/sobel_filter_inst/local_fifo_rd_en_nxt \
sim:/tb_LD_Wrapper/ld_wrapper_inst/sobel_filter_inst/local_fifo_rd_en_prv
add wave -position insertpoint  \
sim:/tb_LD_Wrapper/ld_wrapper_inst/sobel_filter_inst/start_flag

add wave -position insertpoint  \
sim:/tb_LD_Wrapper/ld_wrapper_inst/avr_sobel_fifo_inst/rd_en \
sim:/tb_LD_Wrapper/ld_wrapper_inst/avr_sobel_fifo_inst/wr_en


add wave -position insertpoint  \
sim:/tb_LD_Wrapper/ld_wrapper_inst/avr_sobel_fifo_inst/count \
sim:/tb_LD_Wrapper/ld_wrapper_inst/avr_sobel_fifo_inst/mem \
sim:/tb_LD_Wrapper/ld_wrapper_inst/avr_sobel_fifo_inst/rd_en \
sim:/tb_LD_Wrapper/ld_wrapper_inst/avr_sobel_fifo_inst/wr_en


add wave -position insertpoint  \
sim:/tb_LD_Wrapper/ld_wrapper_inst/sobel_filter_inst/local_fifo_data_count_crnt \
sim:/tb_LD_Wrapper/ld_wrapper_inst/sobel_filter_inst/local_fifo_data_count_nxt \
sim:/tb_LD_Wrapper/ld_wrapper_inst/sobel_filter_inst/local_fifo_data_count_prv \
sim:/tb_LD_Wrapper/ld_wrapper_inst/sobel_filter_inst/local_fifo_rd_en_crnt \
sim:/tb_LD_Wrapper/ld_wrapper_inst/sobel_filter_inst/local_fifo_rd_en_nxt \
sim:/tb_LD_Wrapper/ld_wrapper_inst/sobel_filter_inst/local_fifo_rd_en_prv \
sim:/tb_LD_Wrapper/ld_wrapper_inst/sobel_filter_inst/local_fifo_wr_en_crnt \
sim:/tb_LD_Wrapper/ld_wrapper_inst/sobel_filter_inst/local_fifo_wr_en_nxt \
sim:/tb_LD_Wrapper/ld_wrapper_inst/sobel_filter_inst/local_fifo_wr_en_prv

add wave -position insertpoint  \
sim:/tb_LD_Wrapper/ld_wrapper_inst/sobel_filter_inst/prev_data_buffer_inst/mem
add wave -position insertpoint  \
sim:/tb_LD_Wrapper/ld_wrapper_inst/sobel_filter_inst/crnt_data_buffer_inst/mem
add wave -position insertpoint  \
sim:/tb_LD_Wrapper/ld_wrapper_inst/sobel_filter_inst/nxt_data_buffer_inst/mem

add wave -position insertpoint  \
sim:/tb_LD_Wrapper/pixel_in


add wave -position insertpoint  \
sim:/tb_LD_Wrapper/ld_wrapper_inst/decision_inst/clk \
sim:/tb_LD_Wrapper/ld_wrapper_inst/decision_inst/current_lane \
sim:/tb_LD_Wrapper/ld_wrapper_inst/decision_inst/current_lane_left_boundary \
sim:/tb_LD_Wrapper/ld_wrapper_inst/decision_inst/current_lane_right_boundary \
sim:/tb_LD_Wrapper/ld_wrapper_inst/decision_inst/decision_out_valid \
sim:/tb_LD_Wrapper/ld_wrapper_inst/decision_inst/lane_boundaries \
sim:/tb_LD_Wrapper/ld_wrapper_inst/decision_inst/lane_counter \
sim:/tb_LD_Wrapper/ld_wrapper_inst/decision_inst/most_frequent_lane_boundaries \
sim:/tb_LD_Wrapper/ld_wrapper_inst/decision_inst/number_of_lanes
add wave -position insertpoint  \
sim:/tb_LD_Wrapper/current_lane \
sim:/tb_LD_Wrapper/current_lane_left_boundry \
sim:/tb_LD_Wrapper/current_lane_right_boundry