#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <iostream>

#include <stdlib.h>
#include <stdio.h>

#include <sensor_msgs/image_encodings.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "vo_features.h"
using namespace cv;
using namespace std;
//Mat image;

#define MAX_FRAME 1000
#define MIN_NUM_FEAT 50



double scale = 1.00;

Mat prevImage ,prevImage_c ;
Mat currImage_c,currImage;
Mat traj = Mat::zeros(600, 600, CV_8UC3);

double x = 0.0;
double y = 0.0;
double th = 0.0;

double vx = 0.1;
double vy = -0.1;
double vth = 0.1;
double delta_x ,delta_y , delta_th;
Mat img_1, img_2;
Mat R_f, t_f; 
//the final rotation and tranlation vectors containing the 
//double scale = 1.00;


int estimate_vo(Mat img_1_c,Mat img_2_c){
 

  char text[100];
  int fontFace = FONT_HERSHEY_PLAIN;
  double fontScale = 1;
  int thickness = 1;  
  cv::Point textOrg(10, 50);
  if ( !img_1_c.data || !img_2_c.data ) { 
    	std::cout<< " --(!) Error reading images " << std::endl; return -1;
  	}
  cvtColor(img_1_c, img_1, COLOR_BGR2GRAY);
  cvtColor(img_2_c, img_2, COLOR_BGR2GRAY);
  vector<Point2f> points1, points2;        //vectors to store the coordinates of the feature points
  featureDetection(img_1, points1);        //detect features in img_1
  vector<uchar> status;
  featureTracking(img_1,img_2,points1,points2, status); //track those features to img_2
  double focal = 718.8560;
  cv::Point2d pp(607.1928, 185.2157);
  Mat E, R, t, mask;
  E = findEssentialMat(points2, points1, focal, pp, RANSAC, 0.999, 1.0, mask);
  recoverPose(E, points2, points1, R, t, focal, pp, mask);
  Mat prevImage = img_2;
  Mat currImage;
  vector<Point2f> prevFeatures = points2;
  vector<Point2f> currFeatures;
  R_f = R.clone();
  t_f = t.clone();
  clock_t begin = clock();
  //namedWindow( "view", WINDOW_AUTOSIZE );// Create a window for display.
  //namedWindow( "Trajectory", WINDOW_AUTOSIZE );// Create a window for display.
  Mat traj = Mat::zeros(600, 600, CV_8UC3);
  Mat currImage_c = img_1_c;
  cvtColor(currImage_c, currImage, COLOR_BGR2GRAY);
  featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);
  E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
  recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);
  Mat prevPts(2,prevFeatures.size(), CV_64F), currPts(2,currFeatures.size(), CV_64F);
   for(int i=0;i<prevFeatures.size();i++) {   //this (x,y) combination makes sense as observed from the source code of triangulatePoints on GitHub
      prevPts.at<double>(0,i) = prevFeatures.at(i).x;
      prevPts.at<double>(1,i) = prevFeatures.at(i).y;

      currPts.at<double>(0,i) = currFeatures.at(i).x;
      currPts.at<double>(1,i) = currFeatures.at(i).y;
    }
    scale = 2;
    cout << "Scale is " << scale << endl;

    if ((scale>0.1)&&(t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {

      t_f = t_f + scale*(R_f*t);
      R_f = R*R_f;

    }
    
    else {
     cout << "scale below 0.1, or incorrect translation" << endl;
    }
    if (prevFeatures.size() < MIN_NUM_FEAT) {
      cout << "Number of tracked features reduced to " << prevFeatures.size() << endl;
      cout << "trigerring redection" << endl;
      featureDetection(prevImage, prevFeatures);
      featureTracking(prevImage,currImage,prevFeatures,currFeatures, status);
      prevImage = currImage.clone();
      prevFeatures = currFeatures;

      int x = int(t_f.at<double>(0)) ;
      delta_x = x;
      int y = int(t_f.at<double>(2)) ;
      delta_y = y;
      delta_th =0;
    
      circle(traj, Point(x, y) ,1, CV_RGB(255,0,0), 2);

  
    }

  cv::imshow("Trajectory", traj);

  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  cout << "Total time taken: " << elapsed_secs << "s" << endl;

  return 0;
}




void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    //cv::waitKey(30);
    if (prevImage_c.empty()){
          prevImage_c = cv_bridge::toCvCopy(msg, "bgr8")->image;
          cout<<"module started\n";   
          } 
        else{
          prevImage_c = currImage_c;
          currImage_c = cv_bridge::toCvCopy(msg, "bgr8")->image;
          }   
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;
  cv::namedWindow("view");
  cv::namedWindow( "Trajectory", WINDOW_AUTOSIZE );// Create a window for display.
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/image_raw", 1, imageCallback);
  //ros::spin();
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  ros::Rate r(1.0);
  while(nh.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    //double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    //double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    //double delta_th = vth * dt;
     if(!currImage_c.empty() && !prevImage_c.empty())
          try
        {estimate_vo(prevImage_c,currImage_c);}
          catch(int e)
        {cout<<"odometry estimation failed";}

    	x += delta_x;
    	y += delta_y;
    	th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    	geometry_msgs::TransformStamped odom_trans;
    	odom_trans.header.stamp = current_time;
    	odom_trans.header.frame_id = "odom";
    	odom_trans.child_frame_id = "base_link";

   	odom_trans.transform.translation.x = x;
    	odom_trans.transform.translation.y = y;
    	odom_trans.transform.translation.z = 0.0;
    	odom_trans.transform.rotation = odom_quat;

    //send the transform
    	odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    	nav_msgs::Odometry odom;
    	odom.header.stamp = current_time;
    	odom.header.frame_id = "odom";

    //set the position
    	odom.pose.pose.position.x = x;
    	odom.pose.pose.position.y = y;
    	odom.pose.pose.position.z = 0.0;
    	odom.pose.pose.orientation = odom_quat;

    //set the velocity
    	odom.child_frame_id = "base_link";
    	odom.twist.twist.linear.x = vx;
    	odom.twist.twist.linear.y = vy;
    	odom.twist.twist.angular.z = vth;

    //publish the message
    	odom_pub.publish(odom);

    	last_time = current_time;
      r.sleep();
  }
  //cv::destroyWindow("view");
  //cv::destroyWindow("Trajectory");
}
