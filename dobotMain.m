
% 41014 Sensors and Control for Mechatronic Systems
% Spring 2023

% Ahmad Syahmi Mohd Nasir - 14034882
% Petra Schulzer - 13882129 [feature extraction]
% Anton Cecire - 13936529 [Movement and End Effector Control]

%%
clc;
clear;
close all;
%% Start ROS and Dobot

rosshutdown;
rosinit;

%%
endEffectorPosition = [0.2,0,0.15];
endEffectorRotation = [0,0,0];

[targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');

targetEndEffectorMsg.Position.X = endEffectorPosition(1);
targetEndEffectorMsg.Position.Y = endEffectorPosition(2);
targetEndEffectorMsg.Position.Z = endEffectorPosition(3);

qua = eul2quat(endEffectorRotation);
targetEndEffectorMsg.Orientation.W = qua(1);
targetEndEffectorMsg.Orientation.X = qua(2);
targetEndEffectorMsg.Orientation.Y = qua(3);
targetEndEffectorMsg.Orientation.Z = qua(4);

send(targetEndEffectorPub,targetEndEffectorMsg);

% Initialize the ROS subscribers for RGBD camera data
rgbSub = rossubscriber('/camera/aligned_depth_to_color/image_raw');
pointsSub = rossubscriber('/camera/depth/color/points');
pause(5);

% Get the latest point cloud message and create a 3D scatter plot
pointMsg = pointsSub.LatestMessage;
pointMsg.PreserveStructureOnRead = false;
figure; % Create a new figure for the point cloud plot
cloudPlot_h = scatter3(pointMsg);

% Set the limits for the camera view
xlim([-0.3 0.3]);
ylim([-0.1 0.2]);
zlim([0 0.5]);

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

% Get latest point cloud messages
pointMsg = pointsSub.LatestMessage;
pointMsg.PreserveStructureonRead = false;

% Create a point cloud object
pcobj = pointCloud(readXYZ(pointMsg), 'Color', uint8(255 * readRGB(pointMsg)));

% Extract RGB data from the point cloud
rgb = pcobj.Color;

% Define object based
objectColourRange = (rgb(:, 1) >= redThreshold(1) - tolerance & rgb(:, 1) <= redThreshold(2) + tolerance) & ...
                    (rgb(:, 2) >= greenThreshold(1) - tolerance & rgb(:, 2) <= greenThreshold(2) + tolerance) & ...
                    (rgb(:, 3) >= blueThreshold(1) - tolerance & rgb(:, 3) <= blueThreshold(2) + tolerance);


% Find indices of red objects in the point cloud
colourIndices = find(objectColourRange);

% Check if red objects are found
if isempty(colourIndices)
    disp("Object not found");
    return;
else
    % Calculate the position and color of the found object
    colourIndex = min(colourIndices) + 50;
    blockPose = pcobj.Location(colourIndex, :);
    RGBVal = pcobj.Color(colourIndex, :);
    disp("Object found:");
    disp("Position: " + mat2str(blockPose));
    disp("Color (RGB): " + mat2str(RGBVal));
end


%% Convert Camera pose to Dobot Coordinate frame

blockHeight = -0.0309929275512695;  %based on coordinate frame of Dobot
blockPos = [((blockPose(2)*-1)+endEffectorPosition(1)+0.03),  (blockPose(1)*-1)+0.1 , blockHeight]; %x = distance from 0,0 to camera

%% Move end effector to location
endEffectorPosition = [blockPos];
endEffectorRotation = [0,0,0];

[targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');

targetEndEffectorMsg.Position.X = endEffectorPosition(1);
targetEndEffectorMsg.Position.Y = endEffectorPosition(2);
targetEndEffectorMsg.Position.Z = endEffectorPosition(3);

qua = eul2quat(endEffectorRotation);
targetEndEffectorMsg.Orientation.W = qua(1);
targetEndEffectorMsg.Orientation.X = qua(2);
targetEndEffectorMsg.Orientation.Y = qua(3);
targetEndEffectorMsg.Orientation.Z = qua(4);

send(targetEndEffectorPub,targetEndEffectorMsg);


pause(5);


%% move to another location
endEffectorPosition = [0.2,0,0.1];
endEffectorRotation = [0,0,0];

[targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');

targetEndEffectorMsg.Position.X = endEffectorPosition(1);
targetEndEffectorMsg.Position.Y = endEffectorPosition(2);
targetEndEffectorMsg.Position.Z = endEffectorPosition(3);

qua = eul2quat(endEffectorRotation);
targetEndEffectorMsg.Orientation.W = qua(1);
targetEndEffectorMsg.Orientation.X = qua(2);
targetEndEffectorMsg.Orientation.Y = qua(3);
targetEndEffectorMsg.Orientation.Z = qua(4);

send(targetEndEffectorPub,targetEndEffectorMsg);
