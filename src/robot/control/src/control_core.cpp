#include "control_core.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

namespace robot
{

ControlCore::ControlCore(rclcpp::Node::SharedPtr node, const rclcpp::Logger& logger) 
  : node_(node), logger_(logger) 
  {
    path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
      "/path", 10, std::bind(&ControlCore::pathCallback, this, std::placeholders::_1));
    odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&ControlCore::odomCallback, this, std::placeholders::_1));
    cmd_pub_ = node->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10);
    control_timer_ = node->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&ControlCore::controlLoop, this));
  }

void ControlCore::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  current_path_ = msg; 
  RCLCPP_INFO(logger_, "Received path with %zu points", msg->poses.size());
}

void ControlCore::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_odom_ = msg; 
}

void ControlCore::controlLoop()
{
  if (!current_path_ || !current_odom_ || current_path_->poses.empty()) return; 
  geometry_msgs::msg::PoseStamped lookahead = findLookaheadPoint();
  geometry_msgs::msg::Twist cmd = computeVelocity(lookahead);
  cmd_pub_->publish(cmd);
}

geometry_msgs::msg::PoseStamped ControlCore::findLookaheadPoint()
{
  const geometry_msgs::msg::Point& current_position = current_odom_->pose.pose.position;
  for (const auto& pose : current_path_->poses)
{
    double distance = calculateDistance(current_position, pose.pose.position);
    if (distance >= lookahead_distance_) {
      return pose;
    }
  }
  return current_path_->poses.back();
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(const geometry_msgs::msg::PoseStamped& target)
{
  geometry_msgs::msg::Twist cmd; 
  const auto& current_pose = current_odom_->pose.pose;
  double yaw = extractYaw(current_pose.orientation);
  double dx = target.pose.position.x - current_pose.position.x;
  double dy = target.pose.position.y - current_pose.position.y;
  double target_angle = std::atan2(dy, dx); 
  double heading_error = target_angle - yaw;
  while(heading_error > M_PI) heading_error -= 2 * M_PI; 
  while(heading_error < -M_PI) heading_error += 2 * M_PI;
  cmd.linear.x = linear_speed_; 
  cmd.angular.z = 2.0 * heading_error; 
  if(calculateDistance(current_position, target.pose.position) < goal_tolerance_) {
    cmd.linear.x = 0.0; 
    cmd.angular.z = 0.0; 
  }

  return cmd;

}

double ControlCore::calculateDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b)
{
  return std::hypot(a.x - b.x, a.y - b.y); 

}  

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& q)
{
  tf2::Quaternion tf2_q(q.x, q.y, q.z, q.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf2_q).getRPY(roll, pitch, yaw);
  return yaw;
}
