# DoBotPick-N-Place
Sensors and Control for Mechatronic Systems | Project 3 Group 11 - DoBot Pick and Place 

DoBot Set-up Links

https://github.com/gapaul/dobot_magician_driver/wiki/Instructions-For-Native-Linux

https://canvas.uts.edu.au/courses/27375/files/4478549?wrap=1

https://github.com/gapaul/dobot_magician_driver/wiki/MATLAB-Example

https://github.com/CharlieeT/Dobot-D435i-PickObjects

Errors encountered

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
