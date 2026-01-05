#include "mono_vo_core.hpp"

using namespace cv;
using namespace std;

MonoVoCore::MonoVoCore()
    : scale_(0.0),
      frame_count_(0)
{
    orb_ = ORB::create(500);
    matcher_ = DescriptorMatcher::create("BruteForce-Hamming");

    R_ = Mat::eye(3, 3, CV_64F);
    t_ = Mat::zeros(3, 1, CV_64F);
}

void MonoVoCore::setCameraIntrinsics(double focal,
                                    const Point2d& pp)
{
    focal_ = focal;
    pp_ = pp;
}

void MonoVoCore::setScale(double scale)
{
    scale_ = scale;
}

bool MonoVoCore::processFrame(const Mat& curr,
                              Mat& vis_output,
                              Vec3d& lin_vel,
                              Vec3d& ang_vel)
{
    if (prev_.empty())
    {
        curr.copyTo(prev_);
        return false;
    }

    vector<KeyPoint> kp1, kp2;
    Mat desc1, desc2;

    orb_->detectAndCompute(prev_, noArray(), kp1, desc1);
    orb_->detectAndCompute(curr, noArray(), kp2, desc2);

    if (desc1.empty() || desc2.empty())
        return false;

    vector<DMatch> matches;
    matcher_->match(desc1, desc2, matches);

    vector<Point2f> pts1, pts2;
    for (auto& m : matches)
    {
        pts1.push_back(kp1[m.queryIdx].pt);
        pts2.push_back(kp2[m.trainIdx].pt);
    }

    Mat mask;
    Mat E = findEssentialMat(pts2, pts1, focal_, pp_,
                             RANSAC, 0.999, 1.0, mask);

    recoverPose(E, pts2, pts1, R_, t_, focal_, pp_, mask);

    if (frame_count_ > 2)
    {
        t_f_ += scale_ * (R_f_ * t_);
        R_f_ = R_ * R_f_;
    }
    else
    {
        t_f_ = t_;
        R_f_ = R_;
    }

    lin_vel = Vec3d(t_f_);
    ang_vel = Vec3d(R_f_.at<double>(0),
                    R_f_.at<double>(1),
                    R_f_.at<double>(2));

    drawKeypoints(curr, kp2, vis_output);

    curr.copyTo(prev_);
    frame_count_++;

    return true;
}
