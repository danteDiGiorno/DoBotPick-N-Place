% Petra Schulzer 13882129 
% Made for manual calibration 
% Code lets you pick three points (to get average) on an image and gives you colour values

%==============================================

% Load the image
image = imread('colourcalib.jpg'); 

% Display the image
imshow(image);
title('Click on the image to get colour values');
axis on;

% Initialize an array to store the colour values - 3 rows (for R, G, B) and 3 columns (for the 3 points)
colourValues = zeros(3, 3); 

for i = 1:3
    % Get user input from the image
    [x, y] = ginput(1); 

    % Round the coordinates to integers
    x = round(x);
    y = round(y);

    % Get the colour value at the clicked point
    colourSelected = image(y, x, :); 
    colourValues(i, :) = colourSelected;

    % Display the colour value
    fprintf('colour value %d at (%d, %d): R=%d, G=%d, B=%d\n', i, x, y, colourSelected(1), colourSelected(2), colourSelected(3));

    %D Display a marker at the clicked point
    hold on;
    rectangle('Position', [x, y, 1, 1], 'EdgeColor', 'r', 'LineWidth', 2);
    hold off;
end

% Display the colour values
fprintf('Selected Colour Values:\n');
disp(colourValues);
