#ifndef IMAGE_SUBSCRIBER_NODE_HPP
#define IMAGE_SUBSCRIBER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class ImageSubscriberNode : public rclcpp::Node
{
public:
    ImageSubscriberNode();

    std::function<void(const cv::Mat&)> image_callback;

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

#endif // IMAGE_SUBSCRIBER_NODE_HPP