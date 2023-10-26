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

%% Start the IntelRealSense RGBD Camera

% Sub to the ideal ros topic from the camera
rgbSub = rossubscriber('/camera/aligned_depth_to_color/image_raw');
pointsSub = rossubscriber('/camera/depth/color/points'); %('/camera/depth/points');
pause(5); 

% Get the first message and plot the pointcloud data as 3D scatter plot
pointMsg = pointsSub.LatestMessage;                
pointMsg.PreserveStructureOnRead = false;  
cloudPlot_h = scatter3(pointMsg,'Parent',gca);

% The view of the camera, limited up to dobot and the workspace
xlim([-0.3 0.3]);
ylim([-0.1 0.2]);
zlim([0 0.5]);

pcobj = pointCloud(readXYZ(pointMsg),'Color',uint8(255*readRGB(pointMsg)));

% Select the RGB data from the pointcloud
rgb = pcobj.Color(:,:,:);
red = pcobj.Color(:,1,:);
green = pcobj.Color(:,2,:);
blue = pcobj.Color(:,3,:);

% Filter the pointcloud by the colour of the blocks
resultRed   =  find(red > 180 & red < 225  & green > 60 & green < 120 & blue > 94 & blue < 120);


drawnow();

cloud = readXYZ(pointMsg);
r = 0; g = 0; b = 0;          % Use to flag if the colour blocks are found


% Red ---------------------------------------------------------------------------------------
IndexR = min((resultRed))+50;           % Find the minimum of the red point but plus 50 index to get to the middle
redBlockPose = [cloud(IndexR,1,:), cloud(IndexR,2,:), cloud(IndexR,3,:)];
redRGBVal = [pcobj.Color(IndexR,1,:), pcobj.Color(IndexR,2,:), pcobj.Color(IndexR,3,:)];

if length(redBlockPose) == 0  || all(redBlockPose) == 0      % If the pose is empty
disp("Red not found");
  return;
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
