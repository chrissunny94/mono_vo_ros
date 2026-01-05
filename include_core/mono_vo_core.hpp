#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

class MonoVoCore
{
public:
    MonoVoCore();

    void setCameraIntrinsics(double focal,
                             const cv::Point2d& pp);

    void setScale(double scale);

    // Main API
    bool processFrame(const cv::Mat& curr_gray,
                      cv::Mat& vis_output,
                      cv::Vec3d& linear_vel,
                      cv::Vec3d& angular_vel);

private:
    void computePose(const cv::Mat& curr,
                     const cv::Mat& prev);

    cv::Mat prev_;

    // ORB
    cv::Ptr<cv::ORB> orb_;
    cv::Ptr<cv::DescriptorMatcher> matcher_;

    // VO state
    cv::Mat R_, t_;
    cv::Mat R_f_, t_f_;

    double focal_;
    cv::Point2d pp_;
    double scale_;
    int frame_count_;
};
