# DoBotPick-N-Place
Sensors and Control for Mechatronic Systems | Project 3 Group 11 - DoBot Pick and Place 

DoBot Set-up Links

https://github.com/gapaul/dobot_magician_driver/wiki/Instructions-For-Native-Linux

https://canvas.uts.edu.au/courses/27375/files/4478549?wrap=1

https://github.com/gapaul/dobot_magician_driver/wiki/MATLAB-Example

https://github.com/CharlieeT/Dobot-D435i-PickObjects

Errors encountered
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
