////////////////////////////////////////////////////////////////////////////////
// Author: Saber Mahmoud
//
// Description: Edge Detection Module for Image Processing in Lane Detection System.
// This module performs edge detection based on the gradient magnitude calculated 
// from the Sobel filter's gradient values in both X and Y directions. The magnitude 
// is squared, and then compared to a predefined threshold to determine if an edge 
// exists at a given pixel. The result is a binary output, where a 1 indicates an edge 
// and 0 indicates no edge.
//
// The edge detection is essential for detecting boundaries and features in the 
// image, which will later be used in the lane detection system.
//
// Parameters:
// - THRESHOLD    : Threshold for detecting edges based on gradient magnitude squared (default: 22500).
// - DATA_WIDTH   : Width of the gradient inputs (default: 16 bits).
//
// Inputs:
// - Gx           : Signed gradient in the X direction (from Sobel filter).
// - Gy           : Signed gradient in the Y direction (from Sobel filter).
//
// Output:
// - result       : 1 if an edge is detected (magnitude > threshold), else 0.
//
// Functionality:
// The module calculates the squared magnitude of the gradient vector (Gx and Gy) 
// at each pixel location. The squared magnitude is compared with a threshold to 
// determine whether the pixel is part of an edge (output 1) or not (output 0). 
//
// The result of this comparison is provided on the `result` output, which is a 
// binary signal indicating whether an edge is detected based on the threshold.
////////////////////////////////////////////////////////////////////////////////
module edge_detection #(
    parameter THRESHOLD1      = 9500,
    parameter THRESHOLD2      = 55000
) (
    input  logic               clk,
    input  logic               rst_n,
    input  logic signed [11:0] Gx,
    input  logic signed [11:0] Gy,
    output logic               result1,
    output logic               result2
);

    // Stage 1: Multipliers (split critical path)
    logic signed [23:0] Gx_sq, Gy_sq;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            Gx_sq <= 0;
            Gy_sq <= 0;
        end else begin
            Gx_sq <= Gx * Gx;
            Gy_sq <= Gy * Gy;
        end
    end

    // Stage 2: Sum and compare (can be combinational or pipelined too)
    logic [24:0] magnitude_squared;
    always_comb begin
        magnitude_squared = Gx_sq + Gy_sq;
        result1 = (magnitude_squared > THRESHOLD1);
        result2 = (magnitude_squared > THRESHOLD2);
    end

endmodule
