% Petra Schulzer 13882129 
% Made for automatic calibration 
% Code is integrated with ROS and RealSense camera 
% It lets you pick one point on an image and works out object colour, finds that colour in the point cloud and extracts location coordinates

%==============================================

%% ========================= Cleaning Workspace ========================
clear all;
close all;

%% ================ ROS (Robot Operating System) Initialization ========
% Shut down any previous ROS communication and initalize new communication
rosshutdown;
rosinit("http://localhost:11311");

%% ================= RGBD Camera Setup ==================================
% Subscribes to two ROS topics for RGB and depth data from a camera
rgbSub = rossubscriber('/camera/aligned_depth_to_color/image_raw');
pointsSub = rossubscriber('/camera/depth/color/points');
pause(5);

% Get point cloud data from the first message and plot it in a 3D scatter plot
pointMsg = pointsSub.LatestMessage;
pointMsg.PreserveStructureOnRead = false;
cloudPlot_h = scatter3(pointMsg,'Parent',gca);

% The view of the camera, limited up to dobot and the workspace
% sets the limits for the viewing volume of the 3D scatter plot that displays the point cloud data from the camera. The limits control the range of X, Y, and Z coordinates that are visible in the 3D plot.
xlim([-0.05 0.2]);
ylim([0.01 0.05]);
zlim([0.2 0.4]); 

%% ================= Colour Calibration Through User Input ==================================

% Prompt the user to click on a point in the image to calibrate the colour
disp( 'Please click on the object you wish to pick up.');
figure;

% Show image from latest message
pointMsg = rgbSub.LatestMessage;
image = pointMsg.readImage;
imshow(image);

% User input point 
[x, y] = ginput(1);

% Get the colour at the selected point and adjust thresholds
selectedColour = double(image(round(y), round(x), :));
tolerance = 80;
redThreshold = [selectedColour(1) - tolerance, selectedColour(1) + tolerance];
greenThreshold = [selectedColour(2) - tolerance, selectedColour(2) + tolerance];
blueThreshold = [selectedColour(3) - tolerance, selectedColour(3) + tolerance];

%% ============  Colour Processing & Block Detection ===========================

% Get latest point cloud messages
pointMsg = pointsSub.LatestMessage;
pointMsg.PreserveStructureonRead = false;

% Create a point cloud object
pcobj = pointCloud(readXYZ(pointMsg), 'Colour', uint8(255 * readRGB(pointMsg)));

% Extract RGB data from the point cloud
rgb = pcobj.Colour;

% Define object based on colour thresholds
objectColourRange = (rgb(:, 1) >= redThreshold(1) - tolerance & rgb(:, 1) <= redThreshold(2) + tolerance) & ...
                    (rgb(:, 2) >= greenThreshold(1) - tolerance & rgb(:, 2) <= greenThreshold(2) + tolerance) & ...
                    (rgb(:, 3) >= blueThreshold(1) - tolerance & rgb(:, 3) <= blueThreshold(2) + tolerance);


% Find indices of red objects in the point cloud
colourIndices = find(objectColourRange);


% Check if object is found
if ~isempty(colourIndices)
    disp("Object found:");

    % Calculate the position and colour of the object
    colourIndex = min(colourIndices) + 50;
    blockPosition = pcobj.Location(colourIndex, :);
    RGBVal = pcobj.Colour(colourIndex, :);

    % Display position and colour information
    disp("Position: " + mat2str(blockPosition));
    disp("Colour: " + mat2str(RGBVal));

    % Show coordinates
    blockHeight = -0.0267;  
    blockCoordinates = [((blockPosition(2)*-1) + endEffectorPosition(1) + 0.03),  (blockPosition(1)*-1) + 0.1, blockHeight];
    disp("Block Coordinates in Dobot Frame:");
    disp("X: " + num2str(blockCoordinates(1)));
    disp("Y: " + num2str(blockCoordinates(2)));
    disp("Z: " + num2str(blockCoordinates(3)));
else
    disp("Object not found");
end

