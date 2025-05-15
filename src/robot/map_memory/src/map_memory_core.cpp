#include "map_memory_core.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace robot
{
MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) : logger_(logger) {}

void MapMemoryCore::initializeMap(int width, int height, double resolution) 
{
  map_.info.width = width;
  map_.info.height = height;
  map_.info.resolution = resolution;
  map_.data.resize(width * height, -1);
}

nav_msgs::msg::OccupancyGrid& MapMemoryCore::getMutableMap() {
  return map_;
}

void MapMemoryCore::updateCostmap(const nav_msgs::msg::OccupancyGrid& costmap, double x, double y)
{
  for(size_t h = 0; h < costmap.info.height; ++h) {
    for(size_t w = 0; w < costmap.info.width; ++w) 
    {
      int global_x, global_y; 
      toGlobal(costmap, w, h, x, y, global_x, global_y);
      if (global_x >= 0 && global_x < static_cast<int>(map_.info.width) &&
          global_y >= 0 && global_y < static_cast<int>(map_.info.height))
      {
        size_t global_index = global_y * map_.info.width + global_x;
        size_t local_index = h * costmap.info.width + w;
        if(costmap.data[local_index] != -1)
      {
        if (map_.data[global_index] == -1 || costmap.data[local_index] > map_.data[global_index])
        {
          map_.data[global_index] = costmap.data[local_index];
        }
        
      }
      }
    }
  }
}

void MapMemoryCore::toGlobal(const nav_msgs::msg::OccupancyGrid& costmap, int local_x, int local_y, double x, double y, int& global_x, int& global_y)
{
  double local_x_m = local_x * costmap.info.resolution + costmap.info.origin.position.x;
  double local_y_m = local_y * costmap.info.resolution + costmap.info.origin.position.y;
  double yaw = tf2::getYaw(costmap.info.origin.orientation);
  double global_x_m = std::cos(yaw) * local_x_m - std::sin(yaw) * local_y_m + x;
  double global_y_m = std::sin(yaw) * local_x_m + std::cos(yaw) * local_y_m + y;
  global_x = static_cast<int>(std::round(global_x_m / map_.info.resolution));
  global_y = static_cast<int>(std::round(global_y_m / map_.info.resolution));
} 

const nav_msgs::msg::OccupancyGrid& MapMemoryCore::getMap() const {
  return map_;
}
}
