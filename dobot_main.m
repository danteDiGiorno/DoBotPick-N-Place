%% ========================= Cleaning Workspace ========================
% The code starts by clearing the MATLAB workspace, command window, and closing any open figures
clear all;
close all;

%% ================ ROS (Robot Operating System) Initialization ========
% It shuts down any previous ROS communication initializes new communication
rosshutdown;
rosinit("http://localhost:11311");


% It creates a DobotMagician object, initializing the robot (Dobot Magician function is from the drivers)
dobot = DobotMagician();
pause(5);

%% ================= RGBD Camera Setup ==================================
% It subscribes to two ROS topics for RGB and depth data from a camera
rgbSub = rossubscriber('/camera/aligned_depth_to_color/image_raw');
pointsSub = rossubscriber('/camera/depth/color/points');
pause(5);


% It gets the point cloud data from the first message and plots it in a 3D scatter plot
pointMsg = pointsSub.LatestMessage;
pointMsg.PreserveStructureOnRead = false;
cloudPlot_h = scatter3(pointMsg,'Parent',gca);


% The view of the camera, limited up to dobot and the workspace
% sets the limits for the viewing volume of the 3D scatter plot that displays the point cloud data from the camera. The limits control the range of X, Y, and Z coordinates that are visible in the 3D plot.
xlim([-0.05 0.2]);
ylim([0.01 0.05]);
zlim([0.2 0.4]); 

% Creates a ‘pointCloud’ object with color information based on the data obtained from the ‘pointMsg’ message
pcobj = pointCloud(readXYZ(pointMsg),'Color',uint8(255*readRGB(pointMsg)));


%% ============ Block Detection & Color Processing ===========================
% Code extracts RGB data from the pointcloud object ‘pcobj’ and creates separate arrays for different color channels
rgb = pcobj.Color(:,:,:);
red = pcobj.Color(:,1,:);
green = pcobj.Color(:,2,:);
blue = pcobj.Color(:,3,:);



% Filters and finds points in the point cloud with specific color ranges, representing colored objects.
resultRed = find(red > 100 & red < 200 & green > 5 & green < 70 & blue > 0 & blue < 50);
resultGreen = find(red > 0 & red < 20 & green > 80 & green < 120 & blue > 75 & blue < 120);
resultBlue = find(red > 0 & red < 20 & green > 30 & green < 80 & blue > 70 & blue < 120);
drawnow();


% Checks if a block of each color is found and updates corresponding variables r, g, and b to flag whether each block is found.
cloud = readXYZ(pointMsg);
r = 0;
g = 0;
b = 0;


% Responsible for detecting and capturing information about the position and colors of the red, green and blue blocks within the point cloud data. If a block of a specific color is not found, it sets the corresponding flag (r, g, or b) to 1, indicating that the block is not detected.
% Red Block ------------------------------------------------------------------
IndexR = min((resultRed));
redBlockPosition = [(cloud(IndexR,1,:)*0.01), ((cloud(IndexR,3,:)*-0.01)), (cloud(IndexR,2,:))]
redRGBVal = [pcobj.Color(IndexR,1,:), pcobj.Color(IndexR,2,:), pcobj.Color(IndexR,3,:)];
if length(redBlockPosition) == 0 || all(redBlockPosition) == 0
    r = 1;
end

% Green Block ----------------------------------------------------------------
IndexG = min((resultGreen));
greenBlockPosition = [cloud(IndexG,1,:), cloud(IndexG,2,:), cloud(IndexG,3,:)];
greenRGBVal = [pcobj.Color(IndexG,1,:), pcobj.Color(IndexG,2,:), pcobj.Color(IndexG,3,:)];
if length(greenBlockPosition) == 0 || all(greenBlockPosition) == 0
    g = 1;
end

% Blue Block  ----------------------------------------------------------------
IndexB = min((resultBlue));
blueBlockPosition = [cloud(IndexB,1,:), cloud(IndexB,2,:), cloud(IndexB,3,:)];
blueRGBVal = [pcobj.Color(IndexB,1,:), pcobj.Color(IndexB,2,:), pcobj.Color(IndexB,3,:)];
if length(blueBlockPosition) == 0 || all(blueBlockPosition) == 0
    b = 1;
end



%% ======================= Camera Pose ================================
% Hardcoded values that represent the camera’s position relative to the robot's coordinate system.
cameraPose = [0.2240, 0.2450, 0.0482]


%% =============== Setting up Kinematic Model ===============================
% Defines the kinematic structure of the DoBot using MATLAB’s Robotics System Toolbox. Each line defines a link of the robot arm.
L1 = Link('d',0, 'a',0, 'alpha',-pi/2); % Base
L2 = Link('d',0, 'a',0.135, 'alpha',0); % RearArm
L3 = Link('d',0, 'a',0.147, 'alpha',0); % ForeArm
L4 = Link('d',0, 'a',0.06, 'alpha',pi/2); % End-Effector Bracket
L5 = Link('d',-0.06, 'a',0, 'alpha',0); % Suction Cup

% joint limits
L1.qlim = [-135 135]*pi/180;
L2.qlim = [5 80]*pi/180;
L3.qlim = [15 170]*pi/180;
L4.qlim = [-90 90]*pi/180;
L5.qlim = [-85 85]*pi/180;
L2.offset = -pi/2;
L3.offset = pi/4;
L4.offset = -pi/4;

% Initial joint configuration
q = [0 pi/4 pi/4 0 0];

% Use serial link to connect the links together
DobotSim.model = SerialLink([L1 L2 L3 L4 L5],'name','Dobot');

DobotSim.model.plot(q);


%% =============== Target Block Poses ===============================
% Represents the Z-coordinate at which the robot should approach the detected blocks
blockHeight = -0.267;


% Calculated the forward kinematics transformations for the DoBot’s end effector given a specific joint configuration
Tmask = DobotSim.model.fkine(q);


% Go to default position if red block cannot be found ---------------------------
if r == 1
    % This is a pose above the cubes (Just an empty target pose cause the real one cannot be found)
    redTargetPose = [0.1821, 0.0413, -0.0388];
    disp("Red Block Not Found");
else
    redTargetPose = [(cameraPose(1)+redBlockPosition(1)), cameraPose(2)-redBlockPosition(3), blockHeight]
end

% Go to default position if green block cannot be found  -------------------------
if g == 1
    disp("Green Block Not Found");
    % This is a pose above the cubes (Just an empty target pose cause the real one cannot be found)
    greenTargetPosition = [0.1960, 0.0557, 0.0624];
else
    greenTargetPose = [cameraPose(1)+greenBlockPosition(1), cameraPose(2)-greenBlockPosition(2), blockHeight]
end

% Go to default position if blue block cannot be found  --------------------------
if b == 1
    disp("Blue Block Not Found");
    % This is a pose above the cubes (Just an empty target pose cause the real one cannot be found)
    blueTargetPose = [0.1960, 0.0557, 0.0624];
else
    blueTargetPose = [cameraPose(1)+blueBlockPosition(1), cameraPose(2)+blueBlockPosition(2), blockHeight]
end


% Define target position for block placement
T0 = transl(0,0,0);
Tr = transl(double(redTargetPose));
Tg = transl(double(greenTargetPose));
Tb = transl(double(blueTargetPose));
input("Press Enter to Start Pick Up");



%% =============== Control the Real DoBot ===============================
% Dobot initial joint state (startup position)
q0 = [0 pi/4 pi/4 0 0];
% A point above the three cubes
q1 = [0.2740 0.5173 0.7785 0 0];
% A point above drop off zone
q2 = [-0.1155 0.4652 0.5681 0 0];



% Use the target pose of the blue cube to calculate for the joint state q
qC{1} = DobotSim.model.ikcon(Tb)
% Use the target pose of the red cube to calculate for the joint state q
qC{2} = DobotSim.model.ikcon(Tr)
% Use the target pose of the green cube to calculate for the joint state q
qC{3} = DobotSim.model.ikcon(Tg)


% End Position of Blue Cube
qE{1} = [-0.1377 0.4330 0.8655 0 0];
% End Position of Red Cube
qE{2} = [-0.1377 0.6940 1.1437 0 0]; 
% End Position of Green Cube
qE{3} = [-0.1377 0.2198 0.5835 0 0];

% Set the initial joint state to the dobot's start position
qI = q0;

% Loop through the poses to complete the picking and placing of cubes
for i = 1:1:3
    % Generates trajectory for initial state → above block position
    qMatrix = jtraj(qI,q1,100);
    % Generates trajectory for above block position → above drop off location
    qMatrix2 = jtraj(q1,q2,100);

    % Generates trajectory for above block position → pick up position for current block (i)
    qMatrix1 = jtraj(q1,qC{i},100);
    % Generates trajectory for above drop off location → drop off position for current block (i)
    qMatrix3 = jtraj(q2,qE{i},100);
    % Generates trajectory for drop off position for current block (i) → above drop off location
    qMatrix4 = jtraj(qE{i}, q2, 100);

    % Robot moves to point above blocks
    movement.move(dobot, qMatrix);
    % Robot moves to block to pick up
    movement.move(dobot, qMatrix1);

    % Suction cup on
    dobot.PublishToolState(1,1)

    % Robot moves to point above blocks
    movement.move(dobot, qMatrix);
    % Robot moves to point above drop off location
    movement.move(dobot, qMatrix2);
    % Robot places the block
    movement.move(dobot, qMatrix3);

    % Suction cup off
    dobot.PublishToolState(0,0)

    % Robot moves to point above blocks
    movement.move(dobot, qMatrix4);
    % Above drop off zone becomes new initial position
    qI = q2;
end


%% ======================== Return to Origin ===============================
% Reinitializes the first initial position
q2 = [-0.2370 -0.1474 0.2820 0 0];
% Generates a trajectory back to home position
qMatrixOrigin = jtraj(q2, q0, 50);
% Moves robot to home position
movement.move(dobot, qMatrixOrigin);


