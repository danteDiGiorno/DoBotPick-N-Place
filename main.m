% 41014 Sensors and Control for Mechatronic Systems
% Spring 2023

% Ahmad Syahmi Mohd Nasir - 14034882

% Look into readme.md for startup and errors
%% Start ROS and Dobot

rosshutdown;
rosinit;

DoBotControl.MoveXYZ(0.1776,0,0.07,0,0,pi/8);
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
resultGreen =  find(red > 90   & red < 120    & green > 176 & green < 210 & blue > 178 & blue < 210);
resultBlue  =  find(red > 1 & red < 40 & green > 135 & green < 145 & blue > 200 & blue < 255);


drawnow();

cloud = readXYZ(pointMsg);
r = 0; g = 0; b = 0;          % Use to flag if the colour blocks are found


% Red ---------------------------------------------------------------------------------------
IndexR = min((resultRed))+50;           % Find the minimum of the red point but plus 50 index to get to the middle
redBlockPose = [cloud(IndexR,1,:), cloud(IndexR,2,:), cloud(IndexR,3,:)];
redRGBVal = [pcobj.Color(IndexR,1,:), pcobj.Color(IndexR,2,:), pcobj.Color(IndexR,3,:)];

if length(redBlockPose) == 0  || all(redBlockPose) == 0      % If the pose is empty
disp("Red not found");
  r = 1;
end  


% Green -------------------------------------------------------------------------------------
IndexG = min((resultGreen))+50;         % Find the minimum of the green point but plus 50 index to get to the middle
greenBlockPose = [cloud(IndexG,1,:), cloud(IndexG,2,:), cloud(IndexG,3,:)];
greenRGBVal = [pcobj.Color(IndexG,1,:), pcobj.Color(IndexG,2,:), pcobj.Color(IndexG,3,:)];

if length(greenBlockPose) == 0 || all(greenBlockPose) == 0    % If the pose is empty
disp("Green not found");
  g = 1;
end  


% Blue  -------------------------------------------------------------------------------------
IndexB = min((resultBlue))+50;         % Find the minimum of the blue point but plus 50 index to get to the middle
blueBlockPose = [cloud(IndexB,1,:), cloud(IndexB,2,:), cloud(IndexB,3,:)];
blueRGBVal = [pcobj.Color(IndexB,1,:), pcobj.Color(IndexB,2,:), pcobj.Color(IndexB,3,:)];

if length(blueBlockPose) == 0 || all(blueBlockPose) == 0     % If the pose is empty
disp("Blue not found");
  b = 1;
end  

%% Convert Camera pose to Dobot Coordinate frame

blockHeight = -0.0309929275512695;  %based on coordinate frame of Dobot
redBlockPoseDobot = [ 0.3118 , -0.0112 , blockHeight];

DoBotControl.MoveXYZ(redBlockPoseDobot(1),redBlockPoseDobot(2),redBlockPoseDobot(3),0,0,pi/8);
pause(1);
