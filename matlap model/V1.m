clc;
clear;
close all;
% Create a VideoReader object
videoFile = 'Driving on Tersa.mp4'; 
vidObj = VideoReader(videoFile);

% Initialize a 3D array to hold frames
numFrames = round(vidObj.Duration * vidObj.FrameRate);
frameHeight = vidObj.Height;
frameWidth = vidObj.Width;
videoData = zeros(frameHeight, frameWidth, numFrames, 'uint8'); % Preallocate 3D array

% Read frames into the 3D array
frameIndex = 1;
while hasFrame(vidObj)
    % Read the next frame
    frame = readFrame(vidObj);
    
    % Convert the frame to grayscale
    grayFrame = rgb2gray(frame);
    
    % Store the grayscale frame in the 3D array
    videoData(:, :, frameIndex) = grayFrame;
    frameIndex = frameIndex + 1;
end

% Save the video data to the workspace
assignin('base', 'videoData', videoData);

%% Filtering

filtersize = 5;
averaging_filter = ones(filtersize, filtersize) / (filtersize^2); % setting filter to N x N matrix of values 1/N^2

% Preallocate Filtered_video with the same dimensions as videoData
Filtered_video = zeros(size(videoData));

for i = 1:numFrames
    Filtered_video(:, :, i) = conv2(videoData(:, :, i), averaging_filter, 'same');
end

% Create a figure for displaying frames side by side
figure('Name', 'Video Playback', 'NumberTitle', 'off');
title('Unfiltered (Left) vs Filtered (Right)');

% Loop through each frame and display it
for frameIndex = 1:numFrames
    % Extract the current frames
    unfilteredFrame = videoData(:, :, frameIndex);
    filteredFrame = Filtered_video(:, :, frameIndex);
    
    % Display the current frames side by side
    subplot(1, 2, 1); % Left side for unfiltered
    imshow(unfilteredFrame, []);
    title('Unfiltered Frame');
    
    subplot(1, 2, 2); % Right side for filtered
    imshow(filteredFrame, []);
    title('Filtered Frame');
    
    % Pause to maintain the specified frame rate
    pause(1 / vidObj.FrameRate);
end

%% Edge Detection

% Create a figure for displaying frames and edge detections
figure('Name', 'Video Playback with Edge Detection', 'NumberTitle', 'off');
title('Unfiltered and Filtered with Sobel Edge Detection');

% Loop through each frame and display it
for frameIndex = 1:numFrames
    % Extract the current frames
    unfilteredFrame = videoData(:, :, frameIndex);
    filteredFrame = Filtered_video(:, :, frameIndex);
    
    % Apply Sobel edge detection
    edgesUnfiltered = edge(double(unfilteredFrame), 'Sobel', 10);
    edgesFiltered = edge(double(filteredFrame), 'Sobel', 10);
    
    subplot(1, 2, 1); % Left for edges of unfiltered
    imshow(edgesUnfiltered, []);
    title('Edges of Unfiltered Frame');
    
    subplot(1, 2, 2); % Right for edges of filtered
    imshow(edgesFiltered, []);
    title('Edges of Filtered Frame');
    
    % Pause to maintain the specified frame rate
    pause(1 / vidObj.FrameRate);
end

%% Convert to uint8 image for final edge-detected frame

edgesFilteredImage = uint8(edgesFiltered) * 255;  % Convert logical to uint8 (white for edges)

% Display the edge-detected final frame
figure;
imshow(edgesFilteredImage, []);  % Display as uint8 image (white for edges)
title('Edges of Filtered Frame');

%% Define and Display ROI in edgesFilteredImage

roiYStart = round(size(edgesFilteredImage, 1) * 0.65);  % Adjust based on approximate vertical location
roiYEnd = round(size(edgesFilteredImage, 1) * 0.75);    % Fine-tune as necessary
roiStartX = round(frameWidth / 4);  % Adjusted for a more reasonable start point
roiWidth = 2 * frameWidth / 5;      % Adjust this to control the width of the ROI

roiHeight = roiYEnd - roiYStart;   % Define the height of the ROI
roiEndX = min(roiStartX + roiWidth, frameWidth);
roiEndY = min(roiYStart + roiHeight, frameHeight);

% Crop the image to show only the updated ROI
roiImage = edgesFilteredImage(roiYStart:roiYEnd, roiStartX:roiEndX);

% Display only the ROI
figure;
imshow(roiImage, []);
title('The ROI Image');

%% Lane Detector with Gapped and Continuous Lane Detection

numLanes = 0;  % Start from 0
prevLaneDetected = false;  % Flag to track if we are in a lane already

minConsecutiveZeros = 25;  % Minimum zeros before a lane marking
maxZeroGap = 5;            % Maximum gap of zeros allowed between 255s in a lane

% Loop over each column of the ROI image
for col = 1:size(roiImage, 2)
    consecutiveZeros = 0;          % Counter for consecutive zero values in the column
    inInitialZeroPhase = true;     % Flag to ensure we start with zeros at the beginning
    gapCounter = 0;                % Counter for zeros within a lane marking
    laneDetectedInColumn = false;  % Flag to prevent multiple lanes in one column

    % Loop through each row in the column from top to bottom
    for row = 1:size(roiImage, 1)
        pixelValue = roiImage(row, col);  % Get the pixel value

        if inInitialZeroPhase
            if pixelValue == 0
                consecutiveZeros = consecutiveZeros + 1;
            elseif pixelValue == 255 && consecutiveZeros >= minConsecutiveZeros
                inInitialZeroPhase = false;  % Exit the initial zero phase
                prevLaneDetected = true;     % Mark that a lane has started
                gapCounter = 0;              % Reset gap counter
            else
                consecutiveZeros = 0;
            end
        else
            if pixelValue == 255
                gapCounter = 0;
            elseif pixelValue == 0
                gapCounter = gapCounter + 1;
                
                if gapCounter > maxZeroGap && prevLaneDetected && ~laneDetectedInColumn
                    numLanes = numLanes + 1;  % Count the lane
                    laneDetectedInColumn = true; % Mark lane detected for this column
                    prevLaneDetected = false;    % Reset the lane detection
                    inInitialZeroPhase = true;   % Go back to initial phase
                    consecutiveZeros = 1;        % Count this zero as part of the next phase
                    gapCounter = 0;              % Reset gap counter
                end
            end
        end
    end
    
    if prevLaneDetected && ~laneDetectedInColumn
        numLanes = numLanes + 1;
    end
end

% Display the number of lanes detected
disp(['Number of lanes detected: ', num2str(numLanes)]);
