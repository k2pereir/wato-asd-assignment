#include "planner_node.hpp"
#include <functional>
#include <memory>
#include <string>

PlannerNode::PlannerNode() : Node("planner") 
{
  planner_ = std::make_shared<robot::PlannerCore>(this->get_logger());
  planner_->initSelfPointer();  
  this->declare_parameter<std::string>("map_frame", "map");
  this->declare_parameter<std::string>("base_frame", "base_link");
  this->declare_parameter<std::string>("global_frame", "odom");
  this->declare_parameter<double>("robot_radius", 0.5);
  this->declare_parameter<double>("robot_height", 0.5);
  this->declare_parameter<double>("robot_width", 0.5);
  
  map_ = nullptr;
  start_ = nullptr;
  goal_ = nullptr;

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  start_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
    "/start", 10, std::bind(&PlannerNode::startCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
    "/goal", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
  RCLCPP_INFO(logger_, "PlannerCore constructed.");
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  map_ = msg;
  tryPlan();
}

void PlannerNode::startCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  start_ = msg;
  tryPlan();
}

void PlannerNode::goalCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
  goal_ = msg;
  tryPlan();
}

void PlannerNode::tryPlan()
{
  if (!map_ || !start_ || !goal_) return; 
  nav_msgs::msg::Path path = planner_.aStar(*map_, *start_, *goal_);
  path.header.stamp = this->now();
  path.header.frame_id = map_->header.frame_id;
  path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}

