#include "storagy/robot_control_node.hpp"

RobotControlNode::RobotControlNode()
: Node("robot_control_node")
{
    amcl_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10,
        std::bind(&RobotControlNode::amclPoseCallback, this, std::placeholders::_1));

    nav_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this,
        "navigate_to_pose"
    );

    // 클라이언트가 사용 가능할 때까지 대기
    while (!nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the action server.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
    }

    RCLCPP_INFO(this->get_logger(), "Navigation action server is available.");

    RCLCPP_INFO(this->get_logger(), "Robot node has been initialized.");

}

void RobotControlNode::amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    current_pose_.x = msg->pose.pose.position.x;
    current_pose_.y = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    current_pose_.theta = yaw;
    current_pose_.degree = yaw * 180.0 / M_PI;

    RCLCPP_INFO(this->get_logger(), "Robot Pose: x=%.2f, y=%.2f, theta=%.2f rad, degree=%.2f°", 
                current_pose_.x, current_pose_.y, current_pose_.theta, current_pose_.degree);
    if (pose_callback) {
        RCLCPP_INFO(this->get_logger(), "Calling pose_callback");
        pose_callback(current_pose_);
    }

}

void RobotControlNode::navigateToPose(double x, double y, double theta)
{
    if (!nav_to_pose_client_) {
        RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
        return;
    }

    if (!nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        return;
    }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    goal_msg.pose.pose.orientation.x = q.x();
    goal_msg.pose.pose.orientation.y = q.y();
    goal_msg.pose.pose.orientation.z = q.z();
    goal_msg.pose.pose.orientation.w = q.w();

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&RobotControlNode::nav_to_pose_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&RobotControlNode::nav_to_pose_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&RobotControlNode::nav_to_pose_result_callback, this, std::placeholders::_1);

    nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
}
void RobotControlNode::cancelNavigation()
{
    RCLCPP_INFO(this->get_logger(), "Cancelling all navigation goals");
    nav_to_pose_client_->async_cancel_all_goals();
}

void RobotControlNode::nav_to_pose_response_callback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void RobotControlNode::nav_to_pose_feedback_callback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
    RCLCPP_INFO(this->get_logger(), "Distance remaining: %.2f", feedback->distance_remaining);
}

void RobotControlNode::nav_to_pose_result_callback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(this->get_logger(), "Goal was canceled");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
    }
}