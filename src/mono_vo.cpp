// ROS and related stuff:
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>

// for subscribing to compressed image topics:
#include <image_transport/image_transport.h>
#include "compressed_image_transport/compressed_subscriber.h"
#include "compressed_image_transport/compression_common.h"

// core OpenCV stuff:
#include <cv_bridge/cv_bridge.h>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/video/tracking.hpp> // calcOpticalFlowPyrLK
#include <opencv2/features2d/features2d.hpp>

// core c++ headers:
#include <iostream>
#include <vector>
#include <cmath> // sqrt
#include <iomanip> // std::setw
#include <algorithm> // a few things


using namespace std;
using namespace cv;


class MonoVo
{
private:
  // set up ROS
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub;
  image_transport::Publisher pub;
  ros::Publisher twist_pub;
  ros::Subscriber scale_sub;
  geometry_msgs::Twist twist_out;
  ros::Subscriber caminfo_sub;			

  cv::Mat curr_color;
  sensor_msgs::ImagePtr msg;	
  cv::Mat curr;
  cv::Mat prev;

    Mat img_1;
    Mat img_2;

    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector; 
    Ptr<DescriptorExtractor> descriptor;
    
    Ptr<DescriptorMatcher> matcher;

    double focal; 
    cv::Point2d pp;
    //recovering the pose and the essential matrix
    Mat E, R, t, mask;
    Mat R_f, t_f; //the final rotation and tranlation vectors containing the	
    std::vector< DMatch > good_matches;	
    std::vector< DMatch > prev_matches;	
    vector<Point2f> prevFeatures ;
    vector<Point2f> currFeatures;		
    int count;

    double t_curr;
    double t_prev;
   bool haveCamInfo;
    cv::Mat cameraMatrix;
    cv::Mat distortionCoeffs;
    int frameNum;
    std::string frameId;
    double scale;
    double prev_scale;		

public:

//Constructor
MonoVo()
  : it_(nh_)
  {	

	// Default parameters of ORB
    int nfeatures=500;
    float scaleFactor=1.2f;
    int nlevels=8;
    int edgeThreshold=15; // Changed default (31);
    int firstLevel=0;
    int WTA_K=2;
    int scoreType=ORB::HARRIS_SCORE;
    int patchSize=31;
    int fastThreshold=20;
	detector = ORB::create(nfeatures,
    		scaleFactor,
    		nlevels,
    		edgeThreshold,
    		firstLevel,
    		WTA_K,
    		scoreType,
    		patchSize,
    		fastThreshold );
	descriptor = ORB::create();
	scale = 0;
	prev_scale =0;
	caminfo_sub = nh_.subscribe("/raspicam_node/camera_info", 1,
			       &MonoVo::camInfoCallback, this);
	nh_.param<double>("errorCorrectionRate", focal , 718.8560);
	
	scale_sub = nh_.subscribe("/ubiquity_velocity_controller/odom",1000, &MonoVo::scaleCallback ,this);
        
	count = 0;
	pp = cv::Point2d(607.1928, 185.2157);
	matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
	std::cout << "instance of MonoVo class instantiated" << std::endl;

    // subscribe to input video stream from camera
    image_sub = it_.subscribe("/raspicam_node/image", 1, &MonoVo::img_cb, this, image_transport::TransportHints("compressed"));
    pub = it_.advertise("mono_vo_output/image", 1);
    twist_pub = nh_.advertise<geometry_msgs::Twist>("/optical_flow/twist", 1);
    t_prev = ros::Time::now().toSec();	
} 
// END OF CONSTRUCTOR ######################################################

//Destructor
  ~MonoVo()
  {
    cv::destroyAllWindows();
    std::cout << "Object destructed. The end." << std::endl;
  } 

// END OF DESTRUCTOR #######################################################


//Image callback
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
   
	
    if(prev.data) // Check for invalid input, also handles 1st time being called
    	{
      	
	
      	Return_Pose(curr , prev);
		
      	curr.copyTo(prev); // deep copy, none of that shared pointer stuff
      	
    	}
    else{
	ROS_WARN("prev image data not available, assigning to curr data");
	curr.copyTo(prev);		
    	 }	
}


void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    if (haveCamInfo) {
        return;
    }

    if (msg->K != boost::array<double, 9>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})) {
        for (int i=0; i<3; i++) {
            for (int j=0; j<3; j++) {
                cameraMatrix.at<double>(i, j) = msg->K[i*3+j];
            }
        }

        for (int i=0; i<5; i++) {
            distortionCoeffs.at<double>(0,i) = msg->D[i];
        }

        haveCamInfo = true;
        frameId = msg->header.frame_id;
    }
    else {
        ROS_WARN("%s", "CameraInfo message has invalid intrinsics, K matrix all zeros");
    }
}


void scaleCallback(const nav_msgs::Odometry::ConstPtr& msg){

	
	scale = sqrt(pow(msg->pose.pose.position.x,2) + pow(msg->pose.pose.position.y,2) +pow(msg->pose.pose.position.z,2));
	scale = scale - prev_scale;
	prev_scale = scale;
	
}

void Return_Pose(Mat img_1 ,Mat img_2){

          
       
          
	  //Oriented FAST 
    	detector->detect ( img_1,keypoints_1 );
    	detector->detect ( img_2,keypoints_2 );

    	//BRIEF 
    	descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    	descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    	
    	//BRIEF Hamming 
    	vector<DMatch> matches;
    	//BFMatcher matcher ( NORM_HAMMING );
    	matcher->match ( descriptors_1, descriptors_2, matches );

    	//-- 
    	double min_dist=10000, max_dist=0;

    	//
    	for ( int i = 0; i < descriptors_1.rows; i++ )
    	{
        	double dist = matches[i].distance;
        	if ( dist < min_dist ) min_dist = dist;
        	if ( dist > max_dist ) max_dist = dist;
    	}

    	//printf ( "-- Max dist : %f \n", max_dist );
    	//printf ( "-- Min dist : %f \n", min_dist );

    	
	
    	for ( int i = 0; i < descriptors_1.rows; i++ )
    	{
        	if ( matches[i].distance <= max ( 2*min_dist, 30.0 ) )
        	{
            	good_matches.push_back ( matches[i] );
		//currFeatures.push_back
        	}
    	}
	KeyPoint::convert(keypoints_2, currFeatures);
        KeyPoint::convert(keypoints_1, prevFeatures);
	
		
	try{
		
		if((currFeatures.size() == prevFeatures.size()) ){	
			E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
			recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);
			if(count >2){
				t_f = t_f + scale*(R_f*t);
      				R_f = R*R_f;
				
				t_curr = ros::Time::now().toSec();
    				twist_out.linear.x = t_f.at<double>(0)/ (t_curr - t_prev);
    				twist_out.linear.y = t_f.at<double>(1)/(t_curr - t_prev) ;
    				twist_out.linear.z = t_f.at<double>(2) / (t_curr - t_prev);
				twist_out.angular.x = R_f.at<double>(0)/ (t_curr - t_prev);
    				twist_out.angular.y = R_f.at<double>(1)/ (t_curr - t_prev);
    				twist_out.angular.z = R_f.at<double>(2)/ (t_curr - t_prev);
				twist_pub.publish(twist_out);
				t_prev = t_curr;
			}
			else{
				t_f = t;
				R_f = R;
			}
			Mat img_goodmatch;
			drawKeypoints(img_2,keypoints_2,img_goodmatch ,Scalar::all(-1), DrawMatchesFlags::DEFAULT );
			//drawMatches ( img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch );
			//imshow ( "good_match", img_goodmatch );
			msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_goodmatch).toImageMsg();
			pub.publish(msg);
			}
		else
			printf("\nSKIPPED\n");
		}
		catch (const std::exception& e1)
		{
    			cerr << e1.what() << endl;
		}		
				
		
		
		
		
		
	count++;
	
    	
    		
         
    	


}

//End of image callback

}; 

// END OF CLASS MonoVo ##############################################


int main(int argc, char** argv)
{
  ros::init(argc, argv, "visual_odom_node");
  MonoVo fc;
  ros::spin();
  return 0;
}
