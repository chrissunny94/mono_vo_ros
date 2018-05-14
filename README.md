# Visual odometry

This package provides basic state (i.e. linear and angular position, velocity, acceleration) estimation functionality using optical flow data from a camera rigidly mounted to the chassis of the robot. 

The camera data can be  fused with IMU data using an Extended Kalman Filter to improve accuracy. Keep reading for specifics.



# Instructions

launch the webcam node , ideally use a usb cam

    roslaunch mono_vo_ros webcam.launch	

to start the visual odometery node 

    roslaunch mono_vo_ros start_vo.launch



##### Visualization:
Running the code should automatically display one cv window. 

[![Demo](https://j.gifs.com/86N5lo.gif)](https://www.youtube.com/watch?v=P0ghKIdzdvM)


##Installation 
	

	git clone https://github.com/chrissunny94/mono_vo_ros
	catkin_make