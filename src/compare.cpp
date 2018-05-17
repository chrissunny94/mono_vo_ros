// ROS and related stuff:
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

// core c++ stuff:
#include <iostream>
#include <vector>
#include <cmath> // sqrt
#include <iomanip> // std::setw
#include <algorithm>

#define WIDTH 12 // column spacing

using namespace std;


class DataGrab
{
private:
  // set up ROS stuff:
  ros::NodeHandle nh;
  ros::Subscriber imu_data_sub;
  ros::Subscriber imu_odometry_sub;
  ros::Subscriber visual_odometry_sub;
  ros::Subscriber magni_odometry_sub;

public:
  // x, y, theta for each of the methods:
  
  geometry_msgs::Pose2D visual_pose;
  geometry_msgs::Pose2D magni_pose;

  // twists from each source:
  
  geometry_msgs::Twist visual_twist;
  geometry_msgs::Twist magni_twist;

  // twists at time t-1 (for deriv calcs):
  
  geometry_msgs::Twist visual_twist_prev;
  geometry_msgs::Twist magni_twist_prev;

  // use geometry_msgs::Twist containers to store relevant accels:
 
  geometry_msgs::Twist visual_accel;
  geometry_msgs::Twist magni_accel;

  // time variables for calculating accels:
  
  double t_prev_visual;
  double t_prev_magni;


public:
  DataGrab(ros::NodeHandle n) // CONSTRUCTOR
  {
    // set up nodehandle & subscriptions to different topics:
    nh = n;
    
    visual_odometry_sub = nh.subscribe("/odometry/visual", 1, &DataGrab::visual_callback, this);
    magni_odometry_sub = nh.subscribe("/odometry/filtered", 1, &DataGrab::magni_callback, this);

    // initialize times:
    
    t_prev_visual = ros::Time::now().toSec();
    t_prev_magni = ros::Time::now().toSec();

  } // END OF CONSTRUCTOR


private:
  

  void visual_callback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    double t_curr = ros::Time::now().toSec();
    double dt = t_curr - t_prev_visual;

    // extract RPY from incoming message
    double R, P, Y;
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q); // transform from geometry_msgs::quaternion to tf::quaternion
    tf::Matrix3x3 m(q); // create 3x3 matrix from quaternion
    m.getRPY(R, P, Y); // extract RPY

    // and store values in Pose2D
    visual_pose.x = msg->pose.pose.position.x;
    visual_pose.y = msg->pose.pose.position.y;
    visual_pose.theta = Y;

    // extract twist data
    visual_twist.linear.x = msg->twist.twist.linear.x;
    visual_twist.linear.y = msg->twist.twist.linear.y;
    visual_twist.linear.z = msg->twist.twist.linear.z;
    visual_twist.angular.x = msg->twist.twist.angular.x;
    visual_twist.angular.y = msg->twist.twist.angular.y;
    visual_twist.angular.z = msg->twist.twist.angular.z;

    // calculate accel data
    visual_accel.linear.x = (visual_twist.linear.x - visual_twist_prev.linear.x) / dt;
    visual_accel.linear.y = (visual_twist.linear.y - visual_twist_prev.linear.y) / dt;
    visual_accel.linear.z = (visual_twist.linear.z - visual_twist_prev.linear.z) / dt;
    visual_accel.angular.x = (visual_twist.angular.x - visual_twist_prev.angular.x) / dt;
    visual_accel.angular.y = (visual_twist.angular.y - visual_twist_prev.angular.y) / dt;
    visual_accel.angular.z = (visual_twist.angular.z - visual_twist_prev.angular.z) / dt;

    // cout << "visual_twist: curr, prev, dt = " << setw(12) << visual_twist.angular.z  << setw(12) << visual_twist_prev.angular.z  << setw(12) << dt << endl;

    visual_twist_prev = visual_twist; // for next round

    t_prev_visual = t_curr; // update time for next call

  } // END OF visual_callback() FUNCTION


  void magni_callback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    double t_curr = ros::Time::now().toSec();
    double dt = t_curr - t_prev_magni;

    // extract RPY from incoming message
    double R, P, Y;
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q); // transform from geometry_msgs::quaternion to tf::quaternion
    tf::Matrix3x3 m(q); // create 3x3 matrix from quaternion
    m.getRPY(R, P, Y); // extract RPY

    // and store values in Pose2D
    magni_pose.x = msg->pose.pose.position.x;
    magni_pose.y = msg->pose.pose.position.y;
    magni_pose.theta = Y;

    // extract twist data
    magni_twist.linear.x = msg->twist.twist.linear.x;
    magni_twist.linear.y = msg->twist.twist.linear.y;
    magni_twist.linear.z = msg->twist.twist.linear.z;
    magni_twist.angular.x = msg->twist.twist.angular.x;
    magni_twist.angular.y = msg->twist.twist.angular.y;
    magni_twist.angular.z = msg->twist.twist.angular.z;

    // calculate accel data
    magni_accel.linear.x = (magni_twist.linear.x - magni_twist_prev.linear.x) / dt;
    magni_accel.linear.y = (magni_twist.linear.y - magni_twist_prev.linear.y) / dt;
    magni_accel.linear.z = (magni_twist.linear.z - magni_twist_prev.linear.z) / dt;
    magni_accel.angular.x = (magni_twist.angular.x - magni_twist_prev.angular.x) / dt;
    magni_accel.angular.y = (magni_twist.angular.y - magni_twist_prev.angular.y) / dt;
    magni_accel.angular.z = (magni_twist.angular.z - magni_twist_prev.angular.z) / dt;

    magni_twist_prev = magni_twist; // for next round

    t_prev_magni = t_curr; // update time for next call

  } // END OF magni_callback() FUNCTION

}; // END OF CLASS DataGrab


int main(int argc, char** argv)
{
  // set up node and instantiate callback class:
  ros::init(argc, argv, "analysis_node");
  ros::NodeHandle nh;
  
  ros::Publisher visual_pub = nh.advertise<geometry_msgs::Twist>("/accel/visual", 1);
  ros::Publisher magni_pub = nh.advertise<geometry_msgs::Twist>("/accel/magni", 1);

  // initialize data handler class:
  DataGrab c(nh);

  // loop to publish and print relevant data:
  int j = 0; // counter
  while(ros::ok())
  {
    
    ros::spinOnce(); // required to invoke callbacks

    visual_pub.publish(c.visual_accel);
    magni_pub.publish(c.magni_accel);

    ros::Duration(0.1).sleep(); // 10 Hz

    ++j %= 30; // reprint column heades every 30 lines
  }

  return 0;
}
