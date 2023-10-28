% Load the image
image = imread('colourcalib.jpg'); % Replace 'your_image.jpg' with your image file

% Display the image
imshow(image);
title('Click on the image to get color values');
axis on;

% Initialize an array to store the color values
colorValues = zeros(3, 3); % 3 rows (for R, G, B) and 3 columns (for the 3 points)

for i = 1:3
    % Get user input (click) from the image
    [x, y] = ginput(1); % Get one point

    % Round the coordinates to integers
    x = round(x);
    y = round(y);

    % Get the color value at the clicked point
    colorValue = image(y, x, :); % Get RGB color value
    colorValues(i, :) = colorValue;

    % Display the color value
    fprintf('Color value %d at (%d, %d): R=%d, G=%d, B=%d\n', i, x, y, colorValue(1), colorValue(2), colorValue(3));

    % Optionally, you can display a small rectangle at the clicked point
    hold on;
    rectangle('Position', [x, y, 1, 1], 'EdgeColor', 'r', 'LineWidth', 2);
    hold off;
end

% Display the collected color values
fprintf('Selected Color Values:\n');
disp(colorValues);
