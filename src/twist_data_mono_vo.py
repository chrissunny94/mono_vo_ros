#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class optical_flow_conversion:
    def __init__(self):
        self.optical_flow_pub = rospy.Publisher('optical_flow', Odometry, queue_size=10)
        self.optical_flow_sub = rospy.Subscriber('optical_flow/twist', Twist, self.callback)
        self.rate = rospy.Rate(30) # 30Hz because 30fps
        self.visual_odom = Odometry()

    def callback(self,data):
        self.current_time = rospy.get_rostime()

        self.visual_odom.twist.twist.linear.x = data.linear.x
        self.visual_odom.twist.twist.linear.y = data.linear.y
        self.visual_odom.twist.twist.linear.z = data.linear.z

        self.visual_odom.twist.twist.angular.x = data.angular.x
        self.visual_odom.twist.twist.angular.y = data.angular.y
        self.visual_odom.twist.twist.angular.z = data.angular.z

        self.visual_odom.header.frame_id = 'base_link'
	#self.visual_odom.header.child_frame_id = 'base_link'
        self.visual_odom.header.stamp = self.current_time
        self.visual_odom.twist.covariance[0] = 0.001
        self.visual_odom.twist.covariance[7] = 0.001
        self.visual_odom.twist.covariance[14] = 0.001
        self.visual_odom.twist.covariance[21] = 100.0
        self.visual_odom.twist.covariance[28] = 100.0
        self.visual_odom.twist.covariance[35] = 0.03
        # self.visual_odom.twist.covariance[0] = 0.001
        # self.visual_odom.twist.covariance[7] = 0.001
        # self.visual_odom.twist.covariance[14] = 0.001
        # self.visual_odom.twist.covariance[21] = 1000000.0
        # self.visual_odom.twist.covariance[28] = 1000000.0
        # self.visual_odom.twist.covariance[35] = 0.03

        self.optical_flow_pub.publish(self.visual_odom)
        self.rate.sleep()

def main():
    rospy.init_node('data_converter', anonymous=True)
    listener = tf.TransformListener()

    ofc = optical_flow_conversion()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
