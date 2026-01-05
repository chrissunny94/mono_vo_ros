#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>

#include "mono_vo_core.hpp"

class MonoVoROS2 : public rclcpp::Node
{
public:
    MonoVoROS2() : Node("mono_vo_ros2")
    {
        vo_.setCameraIntrinsics(718.8560, {607.1928, 185.2157});

        img_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image", 10,
            std::bind(&MonoVoROS2::imageCb, this, std::placeholders::_1));

        twist_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            "mono_vo/twist", 10);
    }

private:
    void imageCb(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat gray =
            cv_bridge::toCvShare(msg, "mono8")->image;

        cv::Mat vis;
        cv::Vec3d lin, ang;

        if (vo_.processFrame(gray, vis, lin, ang))
        {
            geometry_msgs::msg::Twist t;
            t.linear.x = lin[0];
            t.linear.y = lin[1];
            t.linear.z = lin[2];
            t.angular.x = ang[0];
            t.angular.y = ang[1];
            t.angular.z = ang[2];
            twist_pub_->publish(t);
        }
    }

    MonoVoCore vo_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MonoVoROS2>());
    rclcpp::shutdown();
}
