void estimate_odometery(){

        Mat R_f, t_f; //the final rotation and tranlation vectors containing the 
        Mat img_1, img_2;
        Mat E, R, t, mask;

         char text[100];
         int fontFace = FONT_HERSHEY_PLAIN;
         double fontScale = 1;
         int thickness = 1;  
         cv::Point textOrg(10, 50);

        


         vector<uchar> status;
         

         double focal = 718.8560;
         cv::Point2d pp(607.1928, 185.2157);
         

         
         

        

         //R_f = R.clone();
         //t_f = t.clone();

         clock_t begin = clock();

         //namedWindow( "Road facing camera", WINDOW_AUTOSIZE );// Create a window for display.
         //namedWindow( "Trajectory", WINDOW_AUTOSIZE );// Create a window for display.
         
         

         vector<Point2f> prevFeatures ;
         vector<Point2f> currFeatures;
         cvtColor(currImage_c, currImage, COLOR_BGR2GRAY);
         cvtColor(prevImage_c, prevImage, COLOR_BGR2GRAY);
         featureDetection(prevImage, prevFeatures);        //detect features in img_1
         featureDetection(currImage, currFeatures);        //detect features in img_1
         
         featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);

         E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
         recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);

         Mat prevPts(2,prevFeatures.size(), CV_64F), currPts(2,currFeatures.size(), CV_64F);
          for(int i=0;i<prevFeatures.size();i++)  {   
                 //this (x,y) combination makes sense as observed from the source code of triangulatePoints on GitHub
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
    
          
    
          // a redetection is triggered in case the number of feautres being trakced go below a particular threshold
          if (prevFeatures.size() < MIN_NUM_FEAT) {
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

          
          imshow( "Trajectory", traj );

          clock_t end = clock();
          double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
          cout << "Total time taken: " << elapsed_secs << "s" << endl;

}