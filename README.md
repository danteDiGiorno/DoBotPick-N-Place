# DoBotPick-N-Place
Sensors and Control for Mechatronic Systems | Project 3 Group 11 - DoBot Pick and Place 

=========================================================================================

Contributions: 
- Petra Schulzer - 13882129 --> 33%
- Anton Cecire - 13936529 --> 33%
- Ahmad Syahmi Mohd Nasir - 14034882 --> 33%

=========================================================================================

Code Overview:
This is code is for Group 11, Project 3. It controls the Dobot Magician and an Intel® RealSense™ D435i to identify and interact with objects of a specified colour. It initializes ROS communication, perceives its environment through the camera, identifies and locates objects based on user-defined colour calibration, and moves its suction cup end effector to pick and place objects from their start location to a drop off zone. 

=========================================================================================

Code Structure: 
1. Initialization and ROS Setup
- Clears the MATLAB workspace and initializes ROS (Robot Operating System).
- Defines the desired end effector position and orientation.
- Sets up a ROS publisher to send end effector target positions to the robot.

2. Camera Setup and Visualization
- Subscribes to RGB-D camera data.
- Creates a 3D scatter plot of the point cloud data.
- Sets the limits for the camera view.

3. User Interaction and Colour Calibration
- Prompts the user to click on an object in the camera image for colour calibration.
- Captures the selected colour and defines colour tolerance thresholds.

4. Point Cloud Processing
- Obtains the latest point cloud data.
- Extracts RGB data from the point cloud.
- Defines a colour range for identifying objects based on the selected colour and tolerance.

5. Object Identification:
- Finds indices of points in the point cloud that match the colour range.
- Checks if objects of the specified colour are found and prints their position and colour.

6. Conversion to Dobot Coordinate Frame
- Converts the position of the identified object to the Dobot Magician's coordinate frame.

7. Moving the End Effector:
- Updates the desired end effector position and orientation.
- Sends a command to the Dobot Magician to move its end effector to the specified location.

=========================================================================================

DoBot Set-Up Links:
https://github.com/gapaul/dobot_magician_driver/wiki/Instructions-For-Native-Linux

https://canvas.uts.edu.au/courses/27375/files/4478549?wrap=1

https://github.com/gapaul/dobot_magician_driver/wiki/MATLAB-Example

https://github.com/CharlieeT/Dobot-D435i-PickObjects

========================================================================================

Errors Encountered:
Error when running

sudo apt-get install ros-$ROS_DISTRO-realsense2-camera

roslaunch realsense2_camera rs_camera.launch align_depth:=true filters:=pointcloud ordered_pc:=true

Error after running "roslaunch dobot_magician_driver dobot_magician.launch"

RLException: [dobot_magician.launch] is neither a launch file in package [dobot_magician_driver] nor is [dobot_magician_driver] a launch file name
The traceback for the exception was written to the log file

Fix: 
cd catkin_ws/
source devel/setup.bash 
and make sure roscore is running
roslaunch dobot_magician_driver dobot_magician.launch


  terminate called after throwing an instance of 'LibSerial::OpenFailed'
    what():  Permission denied
  
  Fix: 
  
  run "gedit ~./bashrc" in terminal
  
  Move "source ~/catkin_ws/devel/setup.bash" to bottom (Line may be written differently for each user)
  
  run "sudo chown 'user' /dev/ttyUSB0" in terminal

Python Error on MATLAB

Fix:

Go to preferences > ROS Toolbox > Open ROS Toolbox Preferences > copy and past python 3.8 install line into terminal
.
