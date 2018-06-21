// ROS and related stuff:
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

// for subscribing to compressed image topics:
#include <image_transport/image_transport.h>
#include "compressed_image_transport/compressed_subscriber.h"
#include "compressed_image_transport/compression_common.h"

// core OpenCV stuff:
#include <cv_bridge/cv_bridge.h>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/video/tracking.hpp> // calcOpticalFlowPyrLK

// core c++ headers:
#include <iostream>
#include <vector>
#include <cmath> // sqrt
#include <iomanip> // std::setw
#include <algorithm> // a few things


#define PI 3.1416 // mmm, delicious pi
#define MAX_POINTS 300 // max # of points to track w/ optical flow
#define TRAIL_LENGTH 30 // not currently being used for any real purposes
#define CAM_PIX_U 640 // pixels
#define CAM_PIX_V 480 // pixels
#define CAM_HEIGHT 0.238 // meters from lens center to ground
#define CAM_RADIAL 0.2413 // meters radial distance from center of robot
#define CAM_DEG 45 // degrees from horizontal
#define CAM_RAD 0.7854 // radians from horizontal
#define CAM_M_U 0.1334 // meters subtended by camera in u direction (wider of the two)
#define CAM_M_V 0.1334 // meters subtended by camera in v direction (narrower of the two) - THIS IS MAGICALLY THE SAME
#define CAM_DEG_U 23.86 // degrees subtended by camera in u direction (wider of the two)
#define CAM_DEG_V 18.15 // degrees subtended by camera in v direction (narrower of the two)
#define CAM_RAD_U 0.416 // radians subtended by camera in u direction (wider of the two)
#define CAM_RAD_V 0.317 // radians subtended by camera in v direction (narrower of the two)
#define PIX_DEG_U 0.03728 // degrees subtended by each pixel in u direction
#define PIX_DEG_V 0.03781 // degrees subtended by each pixel in v direction
#define PIX_RAD_U 0.00065 // radians subtended by each pixel in u direction
#define PIX_RAD_V 0.00066 // radians subtended by each pixel in v direction


using namespace std;
using namespace cv;


class FlowCalculator
{
private:
  // set up ROS
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub;
  ros::Publisher pose_pub;
  ros::Publisher twist_pub;

  // image structures and indices:
  cv::Mat curr_color;
  cv::Mat curr;
  cv::Mat prev;
  vector<Point2f> curr_track_indices;
  vector<Point2f> prev_track_indices;
  vector<Point2f> prev_track_centered;
  vector<Point2f> curr_track_centered;
  vector<Point2f> curr_track_undistorted;
  vector<Point2f> prev_track_undistorted;

  // optical flow, Fundamental & Essential matrices:
  vector<float> flow_errs;
  vector<unsigned char> flow_status;
  vector<unsigned char> F_indices_mask; // outliers from RANSAC or LMedS when finding F
  cv::Matx33d F; // Fundamental Matrix
  cv::Matx33d E; // Essential Matrix

  // setup and generic stuff:
  int counter;
  string ColorWinName;
  // string GrayWinName;
  Mat out_img; // output image, marked up with flow points and stuff

  // camera calibration data:
  cv::Matx33d camera_matrix; // Camera Matrix (from calibration file)
  cv::Mat distortion_coefficients; // distortion coefficients (from calibration file)
  cv::Mat rectification_matrix; // rectification matrix (from calibration file)
  cv::Mat projection_matrix; // projection matrix (from calibration file)
  // W MUST BE A Matx33d OR THE ROTATION MATRIX WILL COME OUT WRONG (e-280 ISSUE)
  // INVESTIGATE LATER, NO TIME TO DIVE INTO THIS NOW
  Matx33d W; // multiview geometry eqn 9.13, H&Z
  Mat R; // rotation matrix
  Mat t; // translation vector

  // containers for output and motion estimate values:
  Point2f accumulated_xy;
  float accumulated_heading;
  float accumulated_travel;
  Point3f accumulated_motion;
  Point3f derpz;
  geometry_msgs::Pose2D pose_out;
  geometry_msgs::Twist twist_out;

  // testing out Farneback's instead of LK to solve OF:
  Mat curr_track_indices_mat;
  // cv::Mat H; // Perspective Transformation (Homography) Matrix
  // H MUST BE A Matx33d
  // INVESTIGATE LATER, NO TIME TO DIVE INTO THIS NOW
  cv::Matx33d H; // Perspective Transformation (Homography) Matrix

  double t_curr;
  double t_prev;

  geometry_msgs::Twist filter[5];
  int filter_count;


public:
  FlowCalculator()
  : it_(nh_)
  {
    std::cout << "instance of FlowCalculator class instantiated" << std::endl;

    // subscribe to input video stream from camera
    image_sub = it_.subscribe("/raspicam_node/image", 1, &FlowCalculator::img_cb, this, image_transport::TransportHints("compressed"));
    // image_sub = it_.subscribe("/camera/image_color", 1, &FlowCalculator::img_cb, this, image_transport::TransportHints("compressed"));

    // publish output pose estimate to EKF
    pose_pub = nh_.advertise<geometry_msgs::Pose2D>("/optical_flow/pose", 1);
    twist_pub = nh_.advertise<geometry_msgs::Twist>("/optical_flow/twist", 1);

    double camera_matrix_data[9] = {1149.322298, 0.0, 351.778662, 0.0, 1151.593614, 276.459807, 0.0, 0.0, 1.0};
    camera_matrix = cv::Mat(3, 3, CV_64F, camera_matrix_data);

    float distortion_coefficients_data[5] = {-0.073669, 1.170392, 0.000976, -0.00244, 0.0};
    distortion_coefficients = cv::Mat(1, 5, CV_32F, distortion_coefficients_data);

    double Wtmp_data[9] = {0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    W = cv::Mat(3, 3, CV_64F, Wtmp_data);

    float rectification_matrix_data[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    rectification_matrix = cv::Mat(3, 3, CV_32F, rectification_matrix_data);

    float projection_matrix_data[12] = {1160.519653, 0.0, 349.420934, 0.0, 0.0, 1164.307007, 275.445505, 0.0, 0.0, 0.0, 1.0, 0.0};
    projection_matrix = cv::Mat(3, 4, CV_32F, projection_matrix_data);

    double H_data[9] = {0.0002347417933653588, -9.613823951336309e-20, -0.07500000298023225, -7.422126200315807e-19, -0.0002818370786240783, 0.5159999728202818, 1.683477982667922e-19, 5.30242624981192e-18, 1};

    H = cv::Mat(3, 3, CV_64F, H_data);

    // accumulated_xy = Point2f(0.0, 0.0);
    accumulated_motion = Point3f(0.0, 0.0, 0.0);
    // accumulated_heading = 0.0;
    // accumulated_travel = 0.0;

    counter = 0.0;

    t_prev = ros::Time::now().toSec();

    geometry_msgs::Twist blah;
    blah.linear.x = 0;
    blah.linear.y = 0;
    blah.linear.z = 0;
    blah.angular.x = 0;
    blah.angular.y = 0;
    blah.angular.z = 0;

    for(int i = 0; i < 5; ++i)
    {
      filter[i] = blah;
    }
    filter_count = 0;

  } // END OF CONSTRUCTOR ######################################################


  ~FlowCalculator()
  {
    cv::destroyAllWindows();
    std::cout << "Object destructed. The end." << std::endl;
  } // END OF DESTRUCTOR #######################################################


  void img_cb(const sensor_msgs::ImageConstPtr& input)
  {
    // grab current frame from camera stream
    try
    {
      curr_color = (cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::BGR8))->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::cvtColor(curr_color, curr, CV_BGR2GRAY); // create curr (grayscale version of the input image)

    if(!prev.data) // Check for invalid input, also handles 1st time being called
    {
      ROS_WARN("prev image data not available, assigning to curr data");
      curr.copyTo(prev); // deep copy, none of that shared pointer stuff
      return;
    }


// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
// BEGIN LK METHOD >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

    if(prev_track_indices.size() < 0.75*MAX_POINTS) // if enough tracking indices are dropped, calculate a new set
    {
      // create vector of good points to track from previous image
      goodFeaturesToTrack(prev, prev_track_indices, MAX_POINTS, 0.1, 5.0);
    }

    if(prev_track_indices.empty()) // check, even though we shouldn't have this problem
    {
      ROS_WARN("no tracking objects found");
      curr.copyTo(prev); // deep copy, none of that shared pointer stuff
      return;
    }

    // find optical flow between previous and current images
    calcOpticalFlowPyrLK(prev, curr, prev_track_indices, curr_track_indices, flow_status, flow_errs, Size(21,21), 4);

    // NOT SURE WE NEED THIS CHECK, BUT ONE OF MY CUSTOM FUNCTIONS STATES WE HAVE IT:
    if(curr_track_indices.size() != prev_track_indices.size())
    { ROS_ERROR("tracking index data size different between previous and current images"); }

    

    derpz = estimate_motion(prev_track_indices, curr_track_indices);
    accumulated_motion += derpz;
    cout << "ACCUMULATED MOVEMENT = " << accumulated_motion << ", ANGLE = " << accumulated_motion.x / (2 * PI * 0.4485) * 360 * 1.57 << endl;
    
    

    // trying to do twist output instead:
    t_curr = ros::Time::now().toSec();
    twist_out.linear.x = derpz.y / (t_curr - t_prev);
    twist_out.linear.y = derpz.y ;
   // twist_out.linear.z = derpz.z / (t_curr - t_prev);
    twist_out.angular.z = derpz.x / (2 * PI * 0.4485) * 2 * PI * 1.57 / (t_curr - t_prev);

    filter[filter_count] = twist_out;

    twist_out.linear.x = 0;
    twist_out.linear.y = 0;
    twist_out.linear.z = 0;
    twist_out.angular.x = 0;
    twist_out.angular.y = 0;
    twist_out.angular.z = 0;
    for(int i = 0; i < 5; ++i)
    {
      twist_out.linear.x += filter[i].linear.x / 5;
      twist_out.linear.y += filter[i].linear.y / 5;
      twist_out.linear.z += filter[i].linear.z / 5;
      twist_out.angular.x += filter[i].angular.x / 5;
      twist_out.angular.y += filter[i].angular.y / 5;
      twist_out.angular.z += filter[i].angular.z / 5;
    }



    twist_pub.publish(twist_out);

    ++filter_count %= 5;



    curr.copyTo(prev); // deep copy, none of that shared pointer stuff

    prev_track_indices = curr_track_indices; // deep copy, MAKE SURE TO COMMENT OUT IF I REVERT TO CALLING goodFeaturesToTrack EACH LOOP

    // this function emulates std::remove_if, which is technically only available
    // in c++ version 11 or greater, and also does not work with OpenCV types
    vector<Point2f>::iterator first = prev_track_indices.begin();
    vector<Point2f>::iterator new_start = first;
    vector<Point2f>::iterator last = prev_track_indices.end();
    while(first!=last)
    {
      if ((*first).x < 0.0 || (*first).x > CAM_PIX_U || (*first).y < 0.0 || (*first).y > CAM_PIX_V)
      {
        
        prev_track_indices.erase(first);
        
      }
      ++first;
      // return new_start;
    }
    // cout << "made it here" << endl;

    prev_track_indices.begin() = new_start;

    ++counter %= TRAIL_LENGTH;
    t_prev = t_curr;




  } // END OF FUNCTION img_cb() ################################################





// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
// BEGIN MEMBER FUNCTIONS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

  



  Point3f estimate_motion(vector<Point2f> &prev_coords, vector<Point2f> &curr_coords)
  { // function to calculate the rotational & translational motion from frame-to-frame

    int N = prev_coords.size();
    vector<Point3f> curr_homog;
    vector<Point3f> prev_homog;
    convertPointsToHomogeneous(curr_coords, curr_homog);
    convertPointsToHomogeneous(prev_coords, prev_homog);

    Point3f motion_avg = Point3f(0.0, 0.0, 0.0);

    // calc total left/right tracked point movement
    vector<Point3f>::iterator it1 = prev_homog.begin(); // sizes should already
    vector<Point3f>::iterator it2 = curr_homog.begin(); // be verified equal
    for( ; it2 != curr_homog.end(); ++it1, ++it2)
    { // calculates robot coordinates from camera coordinates
      motion_avg += Point3f((H * Mat(*it2) - H * Mat(*it1)) / N); // you can tell it's the average by the way that it is!
    }

    // correct y-axis (fore-aft) sign (so forward motion = positive value)
    // factor for speed of 0.1 (from spreadsheet)
    motion_avg.y *= -1.57 * 4.0 / 3.0;

    // now apply deadband of 0.5 mm or so (so we don't accrue unnecessary noise errors)
    if(fabs(motion_avg.x) > 0.0005 || fabs(motion_avg.y) > 0.0005)
    {
      return motion_avg;
    }
    else
    {
      return Point3f(0.0, 0.0, 0.0);
    }
  } // END OF FUNCTION estimate_motion() #######################################


  


  

}; // END OF CLASS FlowCalculator ##############################################



int main(int argc, char** argv)
{
  ros::init(argc, argv, "visual_odom_node");
  // ros::param::set("_image_transport", "compressed");
  FlowCalculator fc;
  ros::spin();
  return 0;
}
