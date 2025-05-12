#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(rclcpp::Node::SharedPtr node, const rclcpp::Logger& logger);
    
  private:
    rclcpp::Logger logger_;
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void controlLoop(); 
    geometry_msgs::msg::PoseStamped findLookaheadPoint(); 
    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped& target); 
    double calculateDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b);
    double extractYaw(const geometry_msgs::msg::Quaternion& q);
    nav_msgs::msg::Path::SharedPtr current_path_; 
    nav_msgs::msg::Path::SharedPtr current_odom_; 
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    double lookahead_distance_ = 1.0; 
    double goal_tolerance_ = 0.1; 
    double linear_speed_ = 0.5; 
};

} 

#endif 
