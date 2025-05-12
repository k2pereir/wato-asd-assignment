#include "map_memory_node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory_node"), last_x_(0.0), last_y_(0.0), distance_threshold(1.5), map_updated_(false), update_map_(false) 
{
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odometryCallback, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map_memory", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&MapMemoryNode::updateMap, this));
  map_memory_.initializeMap(100, 100, 0.1);
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  map_updated_ = true;
  RCLCPP_INFO(this->get_logger(), "Received costmap: width: %d, height: %d", msg->info.width, msg->info.height);
}

void MapMemoryNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double distance = std::sqrt(std::pow(x - last_x_, 2) + std::pow(y - last_y_, 2));
  if (distance > distance_threshold_) {
    last_x_ = x;
    last_y_ = y;
    update_map_ = true;
  }
}

void MapMemoryNode::updateMap()
{
  if (map_updated_ && update_map_) {
    map_memory_.updateCostmap(latest_costmap_, last_x_, last_y_); 
    map_pub_->publish(map_memory_.getMap());
    update_map_ = false;
    RCLCPP_INFO(this->get_logger(), "Map updated and published");
  }
}

void MapMemoryNode::mergeCostmap() 
{
  for(size_t i = 0; i < latest_costmap_.info.height; ++i) 
  {
    for(size_t j = 0; j < latest_costmap_.info.width; ++j)
    {
      int global_x = j + latest_costmap_.info.origin.position.x / map_memory_.getMap().info.resolution; 
      int global_y = i + latest_costmap_.info.origin.position.y / map_memory_.getMap().info.resolution;
      const auto& memory_map = map_memory_.getMap();
      if(global_x >= 0 && global_x < memory_map.info.width && global_y >= 0 && global_y < memory_map.info.height)
      {
        size_t global_index = global_y * map_.info.width + global_x;
        size_t costmap_index = i * latest_costmap_.info.width + j; 
        if(latest_costmap_.data[costmap_index] != -1) 
        {
          map_memory_.getMap().data[global_index] = latest_costmap_.data[costmap_index];
        }
      }
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}