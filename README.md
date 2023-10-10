# DoBotPick-N-Place
Sensors and Control for Mechatronic Systems | Project 3 Group 11 - DoBot Pick and Place 

DoBot Set-up Links
https://github.com/gapaul/dobot_magician_driver/wiki/Instructions-For-Native-Linux
https://canvas.uts.edu.au/courses/27375/files/4478549?wrap=1

Errors encountered
terminate called after throwing an instance of 'LibSerial::OpenFailed'
  what():  Permission denied
Fix: 
run "gedit ~./bashrc" in terminal
Move "source ~/catkin_ws/devel/setup.bash" to bottom (Line may be written differently for each user)

run "sudo chown 'user' /dev/ttyUSB0" in terminal
