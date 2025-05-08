////////////////////////////////////////////////////////////////////////////////
// Author: Saber Mahmoud
//
// Description: FIFO Design with read and write acknowledgments. This module 
// implements a parameterizable FIFO (First-In-First-Out) buffer with configurable 
// width and depth. It supports write and read operations with acknowledgment signals,
// and provides full and empty status outputs.
//
// Parameters:
// - FIFO_WIDTH: Width of the data to be stored in the FIFO (default: 32 bits).
// - FIFO_DEPTH: Depth of the FIFO, i.e., the number of data words the FIFO can hold (default: 8).
//
// Inputs:
// - clk: Clock signal.
// - rst_n: Active-low reset signal.
// - wr_en: Write enable signal. When high, data is written to the FIFO.
// - rd_en: Read enable signal. When high, data is read from the FIFO.
// - data_in: Data input bus of width FIFO_WIDTH.
//
// Outputs:
// - data_out: Data output bus of width FIFO_WIDTH.
// - full: High when FIFO is full.
// - empty: High when FIFO is empty.
// - wr_ack: Write acknowledgment signal. High when data is successfully written.
// - rd_ack: Read acknowledgment signal. High when data is successfully read.
////////////////////////////////////////////////////////////////////////////////

module FIFO #(parameter FIFO_WIDTH = 32,
  parameter FIFO_DEPTH = 8
  )(
  input clk,
  input rst_n,
  input wr_en,
  input rd_en,
  input [FIFO_WIDTH-1:0] data_in,
  output reg [FIFO_WIDTH-1:0] data_out,
  output full,
  output  empty, 
  output reg wr_ack,
  output reg rd_ack,
  output reg [$clog2(FIFO_DEPTH):0] count
  );
  reg [FIFO_WIDTH-1:0] mem [FIFO_DEPTH-1:0];
  reg [$clog2(FIFO_DEPTH)-1:0] wr_ptr, rd_ptr;

  always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
      wr_ptr <= 0;
      wr_ack <= 0;
      count  <= 0;
      rd_ptr <= 0;
      rd_ack <= 0;
    end else if(wr_en  && (rd_en && count != 0))begin
           // FIFO is full but we allow reading and writing simultaneously
      mem[wr_ptr] <= data_in;
      data_out    <= mem[rd_ptr];
      wr_ack      <= 1;
      rd_ack      <= 1;
      wr_ptr      <= (wr_ptr + 1) % FIFO_DEPTH;
      rd_ptr      <= (rd_ptr + 1) % FIFO_DEPTH;
    end
    else if (wr_en && (count < FIFO_DEPTH)) begin
      mem[wr_ptr] <= data_in;
      wr_ack <= 1;
      rd_ack<=0;
      wr_ptr <=(wr_ptr + 1) % FIFO_DEPTH;
      count <= count + 1;
    end else if(rd_en && count != 0) begin 
      data_out <= mem[rd_ptr];
      rd_ack<=1;
      wr_ack<=0;
      rd_ptr <= (rd_ptr + 1) % FIFO_DEPTH;
      count <= count - 1;
    end else begin
      wr_ack <= 0;
      rd_ack<=0;
    end
  end

  assign full = (count == FIFO_DEPTH) ? 1 : 0;
  assign empty = (count == 0) ? 1 : 0;

endmodule
