#ifndef IMAGE_SUBSCRIBER_NODE_HPP
#define IMAGE_SUBSCRIBER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class ImageSubscriberNode : public rclcpp::Node
{
public:
    ImageSubscriberNode();

    std::function<void(const cv::Mat&)> image_callback;

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

};

#endif // IMAGE_SUBSCRIBER_NODE_HPP