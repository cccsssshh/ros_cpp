#ifndef ROBOT_CONTROL_NODE_HPP
#define ROBOT_CONTROL_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp_action/rclcpp_action.hpp>

class RobotControlNode : public rclcpp::Node
{
public:
    RobotControlNode();

    struct RobotPose {
        double x;
        double y;
        double theta;
        double degree;
    };  // 세미콜론 추가

    // RobotPose getCurrentPose() const { return current_pose_; } 뭔지 모르겠음 나중에 뺴기
    std::function<void(const RobotPose&)> pose_callback;

    void navigateToPose(double x, double y, double theta);
    void cancelNavigation();

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    RobotPose current_pose_;

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client_;
     // Navigation2 액션 클라이언트 콜백 함수
    void nav_to_pose_response_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle);
    void nav_to_pose_feedback_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle,
                                       const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);
    void nav_to_pose_result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result);
};


#endif