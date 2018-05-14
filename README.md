# Visual odometry

This package provides basic state (i.e. linear and angular position, velocity, acceleration) estimation functionality using optical flow data from a camera rigidly mounted to the chassis of the robot. 

The camera data can be  fused with IMU data using an Extended Kalman Filter to improve accuracy. Keep reading for specifics.



# Instructions
    roslaunch mono_vo_ros start_vo.launch



##### Visualization:
Running the code should automatically display one cv window. 


##Installation 
	

	git clone https://github.com/chrissunny94/mono_vo_ros
	catkin_make