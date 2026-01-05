#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "mono_vo/mono_vo_core.hpp"

class MonoVoROS1
{
public:
    MonoVoROS1()
        : it_(nh_)
    {
        vo_.setCameraIntrinsics(718.8560, {607.1928, 185.2157});

        img_sub_ = it_.subscribe("/camera/image", 1,
            &MonoVoROS1::imageCb, this);

        img_pub_ = it_.advertise("mono_vo/debug", 1);
        twist_pub_ = nh_.advertise<geometry_msgs::Twist>("mono_vo/twist", 1);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv::Mat gray =
            cv_bridge::toCvShare(msg, "mono8")->image;

        cv::Mat vis;
        cv::Vec3d lin, ang;

        if (vo_.processFrame(gray, vis, lin, ang))
        {
            geometry_msgs::Twist t;
            t.linear.x = lin[0];
            t.linear.y = lin[1];
            t.linear.z = lin[2];
            t.angular.x = ang[0];
            t.angular.y = ang[1];
            t.angular.z = ang[2];
            twist_pub_.publish(t);

            img_pub_.publish(
                cv_bridge::CvImage(msg->header,
                                   "bgr8", vis).toImageMsg());
        }
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber img_sub_;
    image_transport::Publisher img_pub_;
    ros::Publisher twist_pub_;

    MonoVoCore vo_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mono_vo_ros1");
    MonoVoROS1 node;
    ros::spin();
}
