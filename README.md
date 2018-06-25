# Visual odometry

This package provides basic state (i.e. linear and angular position, velocity, acceleration) estimation functionality using optical flow data from a camera rigidly mounted to the chassis of the robot. 

The camera data can be  fused with IMU data using an Extended Kalman Filter to improve accuracy. Keep reading for specifics.



# Instructions

	

to start the visual odometery hack node 

    roslaunch mono_vo_ros start_vo.launch

to start the visual odometery node using Essential matrix method 

    roslaunch mono_vo_ros start_vo_mono.launch

## Visualization

Running the code should automatically display one cv window. 



## Installation 

clone the repository and catkin_make	

	git clone https://github.com/chrissunny94/mono_vo_ros
	catkin_make
	

	
## Source Code:

The below are the main scripts and what it does individually 

**flow.cpp**

 Extracts features, calculates optical flow, and performs perspective transform on a live camera feed. Publishes a geometry_msgs/Twist message to /optical_flow/twist which contains twist data in the form of velocity in x, y, z, roll, pitch, and yaw.

**twist_data.py**

 Converts output of flow.cpp from a geometry_msgs/Twist message to a nav_msgs/Odometry message which contains measurement covariances in addition to the original Twist data. This message is published to the /optical_flow topic.	


[![Demo](https://j.gifs.com/86N5lo.gif)](https://www.youtube.com/watch?v=P0ghKIdzdvM)
