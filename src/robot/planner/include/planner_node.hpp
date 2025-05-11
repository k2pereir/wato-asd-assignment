#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    robot::PlannerCore planner_;
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void startCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::Point::SharedPtr msg);
    void tryPlan(); 
    nav_msgs::msg::OccupancyGrid::SharedPtr map_; 
    geometry_msgs::msg::Pose::SharedPtr start_; 
    geometry_msgs::msg::Point::SharedPtr goal_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr start_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

#endif 
