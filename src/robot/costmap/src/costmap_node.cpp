#include <chrono>
#include <memory>
#include <cmath>
#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) 
{
  int width = 100; 
  int height = 100; 
  double resolution = 0.1; 
  double inflation_radius = 1.0; 
  geometry_msgs::msg::Pose origin; 
  origin.position.x = -5.0; 
  origin.position.y = -5.0; 
  origin.position.z = 0.0;
  origin.orientation.w = 1.0;
  costmap_core_.initializeCostmap(width, height, resolution, origin, inflation_radius);
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::processLaserScan, this, std::placeholders::_1));
}

void CostmapNode::processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
{
  RCLCPP_INFO(this->get_logger(), "Received Lidar data");
  costmap_.updateCostmap(msg);
  auto occupancy_grid = costmap_.getOccupancyGrid();
  costmap_pub_->publish(occupancy_grid);
  RCLCPP_INFO(this->get_logger(), "Published costmap data");
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}