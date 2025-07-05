A complete RTL lane detection system that processes road images through RGB-to-grayscale conversion, averaging filter, Sobel edge detection,
and decision logic to identify lane count, boundaries, and current lane position.
A. Stage 1: RGB to Gray Converter 
This stage is the interface between the system and the input 
image. The input RGB image is converted into grayscale to 
decrease the complexity of the computations and to facilitate 
the edge detection process [5]. The conversion process is 
expressed in Eq. 1. Moreover, this process reduces each pixel 
size from 24 bits (8 bits/color channel) to 8 bits. 
block is characterized by an AXI-stream interface, 
which was chosen due to its high efficiency in video 
applications because of its signals, shown in Table 1. The 
block performs the conversion process using an 8-bit integer 
fixed-point representation to save resources and delay as well 
as to decrease the power needed for the conversion. Fig. 2 
shows the output of the block after the RTL implementation. 
The results' accuracy was over 90%, which is enough for the 
next stage, despite the precision loss arising from the 8-bit 
fixed point representation. 
B. Stage 2: Noise Averaging Filtering 
The averaging filter is applied to the image by sliding a 3x3 
window over the whole frame with stride = 1, where the 
matrix weights are all, as in Eq. 2. Its primary use is to reduce 
the noise in the frames and minimize unwanted edges. It works 
as a low-pass filter.
C. Stage 3: Sobel Edge Detection Technique 
The Sobel algorithm relies on two kernels (filters), Gx and 
Gy, where each one is responsible for being convoluted with 
the image in both horizontal and vertical directions, 
respectively. It uses two 3x3 windows for the gradients and 
components, as in Eqs. 3 and 4. It can be noticed that Gx is 
just the transpose of Gy. This operation aims to produce a 
value for each pixel, which we can subsequently compare to a 
specific threshold chosen after trials on real-case images. The 
stride of the convolution is 1. The filter system architecture 
consists of two main blocks: line buffers and the window. Line 
buffers read the input pixels from the interconnecting FIFO 
(average and Sobel interconnecting FIFO). The FIFO stores 
the pixels before operating on them. The Sobel filter then 
writes these pixels into its line buffers in order to apply the 
convolution process on them.
D. Stage4: Sobel Post-Processing and Thresholding 
The gradient is calculated and compared to a threshold 
value to determine if the pixel is an edge or not. The output of 
the Sobel filter is applied to a non-maximum suppression unit. 
However, before that, the output of both the Gx and Gy goes 
into a sum of squares process to get the resulting magnitude 
corresponding to each pixel in the image, as in Eq. 5. To avoid 
complex hardware implementation, the squared gradient is 
used instead of getting the root of squares, and the threshold 
value is chosen accordingly. The produced result is compared 
to a pre-determined threshold, which is 22500 in the proposed 
implementation. Upon the magnitude comparison, each pixel 
in the image is now mapped to only 1 bit: ‘1’ in case the pixel's 
Sobel output is more than the threshold and ‘0’ otherwise. 
After simulation on real case images, the threshold value is 
determined through trial and error. Edge detection takes place 
whenever the pixel value from the Soble filter output exceeds 
the threshold. These binary-represented pixels are written in 
another global FIFO shared between.
E. Stage 5: Lane Identification Stage 
A non-maximum suppression unit receives the output of 
the Sobel filter. Humans can easily identify the number of 
lanes and their locations in this type of image, but computers 
need an algorithm to do so. This algorithm does not differ 
much from what our brains do. The designed system reads the 
lines of binary pixels while keeping count of the position of 
the pixel in the frame. It detects clusters of edges but only 
counts those that have a gap between them wide enough to be 
a lane. This procedure is done on each line and then generates 
the mentioned outputs: current lane index, current lane right 
boundary, current lane left boundary, and the number of lanes 
in the road. 

