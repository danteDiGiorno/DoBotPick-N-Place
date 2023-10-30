% 41014 Sensors and Control for Mechatronic Systems
% Spring 2023

% Ahmad Syahmi Mohd Nasir - 14034882

% Look into readme.md for startup and errors

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

% Create a point cloud object
pcobj = pointCloud(readXYZ(pointMsg), 'Color', uint8(255 * readRGB(pointMsg)));

% Extract RGB data from the point cloud
rgb = pcobj.Color;

% Define color range for red objects
redRange = (rgb(:, 1) > 180 & rgb(:, 1) < 225) & ...
           (rgb(:, 2) > 60 & rgb(:, 2) < 120) & ...
           (rgb(:, 3) > 94 & rgb(:, 3) < 120);

% Find indices of red objects in the point cloud
redIndices = find(redRange);

% Check if red objects are found
if isempty(redIndices)
    disp("Red objects not found");
    return;
else
    % Calculate the position and color of the first red object
    redIndex = min(redIndices) + 50;
    redBlockPose = pcobj.Location(redIndex, :);
    redRGBVal = pcobj.Color(redIndex, :);
    disp("Red object found:");
    disp("Position: " + mat2str(redBlockPose));
    disp("Color (RGB): " + mat2str(redRGBVal));
end


%% Convert Camera pose to Dobot Coordinate frame

blockHeight = -0.0309929275512695;  %based on coordinate frame of Dobot
redBlockPos = [((redBlockPose(2)*-1)+endEffectorPosition(1)+0.03),  (redBlockPose(1)*-1)+0.1 , blockHeight]; %x = distance from 0,0 to camera

%% Move end effector to location
endEffectorPosition = [redBlockPos];
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
