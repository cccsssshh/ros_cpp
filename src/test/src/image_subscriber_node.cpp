#include "test/image_subscriber_node.hpp"

ImageSubscriberNode::ImageSubscriberNode()
: Node("image_subscriber_node")
{
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/webcam_image", 10, std::bind(&ImageSubscriberNode::imageCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Image subscriber node has been initialized.");
}

void ImageSubscriberNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        cv::Mat cv_image = cv_bridge::toCvShare(msg, "bgr8")->image;
        
        if (image_callback) {
            image_callback(cv_image);
        }

        RCLCPP_INFO(this->get_logger(), "Received an image. Size: %dx%d", cv_image.cols, cv_image.rows);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}