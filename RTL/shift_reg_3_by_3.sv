////////////////////////////////////////////////////////////////////////////////
// Author: Saber Mahmoud
//
// Description: 3x3 Shift Register Module for Averaging Filter in Image Processing.
// This module implements a 3x3 shift register that stores and shifts pixel data 
// from three consecutive rows in the image to form a sliding 3x3 window. The window 
// is then used for averaging operations to smooth out the image and reduce noise. 
// It helps prepare the image data for further processing stages like edge detection
// or other image transformations.
//
// The module reads pixel data from three FIFOs representing the previous, current, 
// and next rows in the image, and updates the window every clock cycle when a new
// column is available. It supports smoothing by averaging the neighboring pixels 
// in the 3x3 window.
//
// Parameters:
// - PIXEL_SIZE : The bit-width of each pixel (default: 8 bits).
// - IMG_WIDTH  : The width of the image (default: 640 pixels). (Not currently used in logic)
//
// Inputs:
// - clk         : Clock signal for synchronization with other modules.
// - rst_n       : Active-low reset signal to initialize the module signals.
// - pixel_valid : Indicates when a new column of pixels is available for processing.
// - fifo0_in    : Input pixel data from the previous row (Row 0) of the image.
// - fifo1_in    : Input pixel data from the current row (Row 1) of the image.
// - fifo2_in    : Input pixel data from the next row (Row 2) of the image.
//
// Outputs:
// - window      : A 3x3 matrix representing the current 3x3 sliding window 
//                of pixels, used for averaging operations.
//
// Shift Register Behavior:
// - On every clock cycle when `pixel_valid` is asserted, the module shifts the 
//   pixel data in the `window` left by one column.
// - The new pixel values from the FIFOs are inserted into the rightmost column of the window.
// - The `window` output provides the 3x3 grid of pixels representing the current 
//   neighborhood of the image for further processing.
//
// Reset Behavior:
// - When `rst_n` is low, the `window` is cleared (set to zero).
////////////////////////////////////////////////////////////////////////////////
module shift_reg_3x3 #(
    parameter PIXEL_SIZE = 8,
    parameter IMG_WIDTH = 640
)(
    input  logic                     clk,
    input  logic                     rst_n,
    input  logic                     pixel_valid,  // New column available
    input  logic [PIXEL_SIZE-1:0]     fifo0_in,    // Row 0 FIFO output
    input  logic [PIXEL_SIZE-1:0]     fifo1_in,    // Row 1 FIFO output
    input  logic [PIXEL_SIZE-1:0]     fifo2_in,    // Row 2 FIFO output
    output logic [PIXEL_SIZE-1:0]     window [0:2][0:2]  // 4 rows × 3 columns output
);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            window[0][0] <= 0; window[0][1] <= 0; window[0][2] <= 0;
            window[1][0] <= 0; window[1][1] <= 0; window[1][2] <= 0;
            window[2][0] <= 0; window[2][1] <= 0; window[2][2] <= 0;
        end else if (pixel_valid) begin
            // Shift left
            window[0][0] <= window[0][1]; window[0][1] <= window[0][2];
            window[1][0] <= window[1][1]; window[1][1] <= window[1][2];
            window[2][0] <= window[2][1]; window[2][1] <= window[2][2];

            // Insert new column from FIFO inputs
            window[0][2] <= fifo0_in;
            window[1][2] <= fifo1_in;
            window[2][2] <= fifo2_in;
        end
    end

endmodule
