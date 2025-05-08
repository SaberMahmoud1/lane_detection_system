clc;
clear;
close all;

% Load the image
imageFile = 'test3.jpg';  % Replace with the correct path to your image
frame = imread(imageFile); % Read the image

% Convert the image to grayscale
grayFrame = rgb2gray(frame);  

% Apply an averaging filter to reduce noise
filterSize = 3;  
averagingFilter = ones(filterSize, filterSize) / 9;  
filteredFrame = conv2(grayFrame, averagingFilter, 'same');  

% Edge Detection using Sobel filter
edgesFiltered = edge(double(filteredFrame), 'Sobel', 20);

% Convert binary edge image to uint8 for visualization
edgesFilteredImage = uint8(edgesFiltered) * 255;

% Display the results
figure;
imshow(filteredFrame, []);
title('Averaged Image (After Noise Reduction)');

figure;
imshow(edgesFilteredImage, []);
title('Sobel Edge Detection Result');

% Parameters
gapThreshold = 90; % Define gap threshold for separating groups

% Initialize array to store number of groups per row
numGroupsPerRow = zeros(size(edgesFilteredImage, 1), 1);
laneBoundaries = [];

% Loop through each row
for row = 1:size(edgesFilteredImage, 1)
    lineData = find(edgesFilteredImage(row, :) == 255);  % Get indices of white pixels (ones)
    
    if isempty(lineData)
        numGroupsPerRow(row) = 0; % No ones in this row
        continue;
    end
    
    numGroups = 1; % At least one group exists if there are ones
    lastOnePos = lineData(1); % First white pixel position
    boundaries = lastOnePos; % Store first boundary

    % Scan row pixels
    for i = 2:length(lineData)
        if (lineData(i) - lastOnePos) > gapThreshold
            numGroups = numGroups + 1; % New group detected
            boundaries = [boundaries, lineData(i)]; % Store new boundary
        end
        lastOnePos = lineData(i); % Update last seen white pixel
    end
    
    numGroupsPerRow(row) = numGroups;
    
    % Save boundaries from the middle row (for stability)
    if row == round(size(edgesFilteredImage, 1) / 2)
        laneBoundaries = boundaries;
    end
end

% Find the most frequent number of groups
mostFrequentGroups = mode(numGroupsPerRow);
numLanes = mostFrequentGroups - 1;

% Determine the current lane
centerX = round(size(edgesFilteredImage, 2) / 2); % Middle of the image
laneIndex = find(laneBoundaries > centerX, 1) - 1; % Find the lane based on boundaries

if isempty(laneIndex)
    laneIndex = numLanes; % If no right boundary, assume the last lane
end

% Display results
fprintf('Most frequent number of groups per row: %d\n', mostFrequentGroups);
fprintf('Number of lanes: %d\n', numLanes);
fprintf('Current lane: %d\n', laneIndex);

% Plot histogram
figure;
histogram(numGroupsPerRow, 'BinMethod', 'auto');
title('Histogram of Edge Groups Per Row');
xlabel('Number of Groups');
ylabel('Frequency');
