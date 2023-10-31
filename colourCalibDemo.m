% Load the image
image = imread('testcolour_Color (1).png');

% Display the image
imshow(image);
title('Please click on the object you wish to pick up.');

% Wait for a single click on the image
[x, y] = ginput(1);

% Convert to integers
x = round(x);
y = round(y);

% Get the RGB color values at the clicked point
selectedColor = double(image(y, x, :));
tolerance = 80;
redThreshold = [selectedColor(1) - tolerance, selectedColor(1) + tolerance]
greenThreshold = [selectedColor(2) - tolerance, selectedColor(2) + tolerance]
blueThreshold = [selectedColor(3) - tolerance, selectedColor(3) + tolerance]


