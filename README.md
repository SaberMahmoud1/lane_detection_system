A complete hardware-compatible RTL lane detection system (tested on PYNQ Z2 and ZCU102 FPGAs) processes road images through RGB-to-grayscale conversion, averaging filtering, Sobel edge detection, and decision logic to identify the lane count, boundaries, and current lane position.

Stage 0: Crop the Region of Interest
This stage extracts the region of interest (ROI) from the input image, focusing only on the relevant part of the road to reduce processing time and eliminate unnecessary background information.

Stage 1: RGB to Grayscale Conversion
This stage interfaces with the input RGB image and converts it into grayscale to reduce computational complexity and facilitate edge detection. The grayscale conversion reduces each pixel from 24 bits (8 bits per color channel) to 8 bits. The block uses an AXI-stream interface for efficient video processing and applies an 8-bit integer fixed-point representation to minimize resource usage, delay, and power consumption. The accuracy of the grayscale output is over 90%, which is sufficient for the next processing stage.

Stage 2: Noise Averaging Filter
A 3×3 averaging filter is applied to the grayscale image with a stride of 1. This low-pass filter reduces noise and minimizes undesired edges by averaging the pixel values in the window, helping to stabilize edge detection in subsequent stages.

Stage 3: Sobel Edge Detection
The Sobel algorithm uses two 3×3 convolution kernels to detect horizontal and vertical gradients. These filters are applied with a stride of 1 using line buffers and a window mechanism. The pixel values are read from an intermediate FIFO that connects the averaging filter to the Sobel module. The result is a gradient map that highlights potential edges in the image.

Stage 4: Sobel Post-Processing and Thresholding
The gradients from the Sobel operation are processed to calculate the magnitude of change at each pixel using a sum-of-squares method. To simplify hardware, the square root is omitted, and a predefined threshold is used to decide whether a pixel represents an edge. Pixels exceeding this threshold are marked as '1', and all others as '0'. This produces a binary edge map of the image, stored in a shared global FIFO.

Stage 5: Lane Identification
The binary edge map is scanned line by line to detect and count lane boundaries. The system identifies clusters of edges and filters out any that are too close together to be actual lanes. It then determines the number of lanes, the left and right boundaries of the current lane, and the index of the lane the vehicle is in. The logic mimics human visual interpretation through a simple yet effective detection algorithm.

