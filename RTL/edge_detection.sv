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
    parameter THRESHOLD = 22500
) (
    input  logic signed [15:0] Gx,  // Gradient X
    input  logic signed [15:0] Gy,  // Gradient Y
    output logic result             // 1 if edge detected, else 0
);
    logic signed [31:0] magnitude_squared;  // Signed 32-bit to store the squared magnitude

    // Compute squared magnitude: Gx^2 + Gy^2
    assign magnitude_squared = Gx * Gx + Gy * Gy;

    // Compare with threshold squared
    assign result = (magnitude_squared > THRESHOLD) ? 1'b1 : 1'b0;

endmodule
