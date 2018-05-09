

#include "vo_features.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

using namespace cv;
using namespace std;


void imageCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info)
{
    try
    {
        cv::imshow("view", cv_bridge::toCvShare(image, "bgr8")->image);
        // Init logic to ensure two images are captured for initial features
        if (!init)
        {
            // TODO: Visualize Keypoints being tracked.
            init_2_c = cv_bridge::toCvShare(image, "bgr8")->image;

            if (!init_1_c.empty() && !init_2_c.empty()) {
                std::cout << "Initializing..." << std::endl;

                //Convert to grayscale.
                cv::cvtColor(init_1_c, init_1, COLOR_BGR2GRAY);
                cv::cvtColor(init_2_c, init_2, COLOR_BGR2GRAY);

                //feature detection and tracking.
                featureDetection(init_1, points1); //detect features in Img1
                featureTracking(init_1, init_2, points1, points2, status); //track those features to init_2

                //focal_length = cam_info->K[0];
                focal_length = 1200.0;
                std::cout << "Focal Length: " << focal_length << std::endl;
                cv::Point2d ppoint(cam_info->K[2], cam_info->K[5]); //principle point
                std::cout << "Finding essential matrix..." << std::endl;
                E = cv::findEssentialMat(points1, points2, focal_length, ppoint, RANSAC, 0.999, 1.0, mask);
                std::cout << "E = "<< std::endl << " "  << E << std::endl << std::endl;
                cv::recoverPose(E, points1, points2, R, t, focal_length, ppoint, mask);

                /*TODO Stay in init loop until R and t are "different" enough? To ensure good data?
                   as it stands, there are just to consecutive images taken.
                   */

                prevImage = init_2;
                prevFeatures = points1;

                std::cout << "R = "<< std::endl << " "  << R << std::endl << std::endl;
                std::cout << "t = "<< std::endl << " "  << t << std::endl << std::endl;

                R_f = R.clone();
                t_f = t.clone();

                init = true;
            }

            init_1_c = init_2_c;
        }

        if(init) //Main portion of callback right now.
        {
            std::cout << "Visual Odom Publishing..." << std::endl;
            std::cout << "In main loop" << std::endl;
            currImage_c = cv_bridge::toCvShare(image, "bgr8")->image;
            cvtColor(currImage_c,currImage, COLOR_BGR2GRAY);
            std::cout << "In after colorchange" << std::endl;

            featureTracking(prevImage,currImage, prevFeatures, currFeatures, status);
            std::cout << "feat track" << std::endl;

            cv::Point2d ppoint(cam_info->K[2], cam_info->K[5]); //principle point
            std::cout << "principle point" << std::endl;

            E = cv::findEssentialMat(prevFeatures, currFeatures, focal_length, ppoint, RANSAC, 0.999, 1.0, mask);
            cv::recoverPose(E, prevFeatures, currFeatures, R, t, focal_length, ppoint, mask);

            cv::Mat prevPts(2, prevFeatures.size(), CV_64F);
            cv::Mat currPts(2, currFeatures.size(), CV_64F);

            for(int i=0;i<prevFeatures.size();i++)  {   //this (x,y) combination makes sense as observed from the source code of triangulatePoints on GitHub
                prevPts.at<double>(0,i) = prevFeatures.at(i).x;
                prevPts.at<double>(1,i) = prevFeatures.at(i).y;

                currPts.at<double>(0,i) = currFeatures.at(i).x;
                currPts.at<double>(1,i) = currFeatures.at(i).y;
            }

            //TODO: scale = getAbsoluteScale(numFrame, 0, t.at<double>(2));


            if ((scale>0.1)&&(t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {
                //construct trajectory here:

                t_f = t_f + scale*(R_f*t); //translation
                R_f = R*R_f; //rotation

            }

            // try and find more keypoints if enough were found.
            if (prevFeatures.size() < MIN_NUM_FEAT) {
                std::cout << "Number of tracked features reduced to " << prevFeatures.size() << std::endl;
                std::cout << "trigerring redection" << std::endl;
                featureDetection(prevImage, prevFeatures);
                featureTracking(prevImage,currImage,prevFeatures,currFeatures, status);

            }

            prevImage = currImage.clone();
            prevFeatures = currFeatures;

            // sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));

            // std::cout << text << std::endl;
        }
        cv::waitKey(30);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image->encoding.c_str());
    }
}

#define MAX_FRAME 1000
#define MIN_NUM_FEAT 50

/******************************************************************************/
// setup the cameras properly based on OS platform

// 0 in linux gives first camera for v4l
//-1 in windows gives first device or user dialog selection

#ifdef linux
#define CAMERA_INDEX  1
#else
#define CAMERA_INDEX -1
#endif

/******************************************************************************/


// IMP: Change the file directories (4 places) according to where your dataset is saved before running!

double getAbsoluteScale(int frame_id, int sequence_id, double z_cal)	{
  
  string line;
  int i = 0;
  ifstream myfile ("/home/avisingh/Datasets/KITTI_VO/00.txt");
  double x =0, y=0, z = 0;
  double x_prev, y_prev, z_prev;
  if (myfile.is_open())
  {
    while (( getline (myfile,line) ) && (i<=frame_id))
    {
      z_prev = z;
      x_prev = x;
      y_prev = y;
      std::istringstream in(line);
      //cout << line << '\n';
      for (int j=0; j<12; j++)  {
        in >> z ;
        if (j==7) y=z;
        if (j==3)  x=z;
      }
      
      i++;
    }
    myfile.close();
  }

  else {
    cout << "Unable to open file";
    return 0;
  }

  return sqrt((x-x_prev)*(x-x_prev) + (y-y_prev)*(y-y_prev) + (z-z_prev)*(z-z_prev)) ;

}


int main( int argc, char** argv )	{


    VideoCapture cap; // capture object


    if(
            ( argc == 2 && (cap.open(argv[1]) == true )) ||
            ( argc != 2 && (cap.open(CAMERA_INDEX) == true))
            )



        if (!cap.isOpened()) { //check if video device has been initialised
            cout << "cannot open camera";
        }

  Mat img_1, img_2;
  Mat R_f, t_f; //the final rotation and tranlation vectors containing the 

  ofstream myfile;
  myfile.open ("results1_1.txt");

  double scale = 1.00;


  char text[100];
  int fontFace = FONT_HERSHEY_PLAIN;
  double fontScale = 1;
  int thickness = 1;  
  cv::Point textOrg(10, 50);

  //read the first two frames from the dataset
  Mat img_1_c;
  cap.read(img_1_c);
  Mat img_2_c;
  cap.read(img_2_c);

  if ( !img_1_c.data || !img_2_c.data ) { 
    std::cout<< " --(!) Error reading images " << std::endl; return -1;
  }

  // we work with grayscale images
  cvtColor(img_1_c, img_1, COLOR_BGR2GRAY);
  cvtColor(img_2_c, img_2, COLOR_BGR2GRAY);

  // feature detection, tracking
  vector<Point2f> points1, points2;        //vectors to store the coordinates of the feature points
  featureDetection(img_1, points1);        //detect features in img_1
  vector<uchar> status;
  featureTracking(img_1,img_2,points1,points2, status); //track those features to img_2

  //TODO: add a fucntion to load these values directly from KITTI's calib files
  // WARNING: different sequences in the KITTI VO dataset have different intrinsic/extrinsic parameters
  double focal = 718.8560;
  cv::Point2d pp(607.1928, 185.2157);
  //recovering the pose and the essential matrix
  Mat E, R, t, mask;
  E = findEssentialMat(points2, points1, focal, pp, RANSAC, 0.999, 1.0, mask);
  recoverPose(E, points2, points1, R, t, focal, pp, mask);

  Mat prevImage = img_2;
  Mat currImage;
  vector<Point2f> prevFeatures = points2;
  vector<Point2f> currFeatures;

  char filename[100];

  R_f = R.clone();
  t_f = t.clone();

  clock_t begin = clock();

  namedWindow( "Road facing camera", WINDOW_AUTOSIZE );// Create a window for display.
  namedWindow( "Trajectory", WINDOW_AUTOSIZE );// Create a window for display.

  Mat traj = Mat::zeros(600, 600, CV_8UC3);

  for(int numFrame=2; numFrame < MAX_FRAME; numFrame++)	{
  	Mat currImage_c ;
    cap.read(currImage_c);
    cvtColor(currImage_c, currImage, COLOR_BGR2GRAY);
  	vector<uchar> status;
  	featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);

  	E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
  	recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);

    Mat prevPts(2,prevFeatures.size(), CV_64F), currPts(2,currFeatures.size(), CV_64F);


   for(int i=0;i<prevFeatures.size();i++)	{   //this (x,y) combination makes sense as observed from the source code of triangulatePoints on GitHub
  		prevPts.at<double>(0,i) = prevFeatures.at(i).x;
  		prevPts.at<double>(1,i) = prevFeatures.at(i).y;

  		currPts.at<double>(0,i) = currFeatures.at(i).x;
  		currPts.at<double>(1,i) = currFeatures.at(i).y;
    }

  	//scale = getAbsoluteScale(numFrame, 0, t.at<double>(2));
   scale = 2;

    cout << "Scale is " << scale << endl;

    if ((scale>0.1)&&(t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {

      t_f = t_f + scale*(R_f*t);
      R_f = R*R_f;

    }
  	
    else {
     //cout << "scale below 0.1, or incorrect translation" << endl;
    }
    
   // lines for printing results
   // myfile << t_f.at<double>(0) << " " << t_f.at<double>(1) << " " << t_f.at<double>(2) << endl;

  // a redetection is triggered in case the number of feautres being trakced go below a particular threshold
 	  if (prevFeatures.size() < MIN_NUM_FEAT)	{
      cout << "Number of tracked features reduced to " << prevFeatures.size() << endl;
      //cout << "trigerring redection" << endl;
 		  featureDetection(prevImage, prevFeatures);
      featureTracking(prevImage,currImage,prevFeatures,currFeatures, status);

 	  }

    prevImage = currImage.clone();
    prevFeatures = currFeatures;

    int x = int(t_f.at<double>(0)) + 300;
    int y = int(t_f.at<double>(2)) + 100;
    circle(traj, Point(x, y) ,1, CV_RGB(255,0,0), 2);

    rectangle( traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);
    sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
    putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);

    imshow( "Road facing camera", currImage_c );
    imshow( "Trajectory", traj );

    waitKey(1);


  }

  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  cout << "Total time taken: " << elapsed_secs << "s" << endl;

  //cout << R_f << endl;
  //cout << t_f << endl;

  return 0;
}
