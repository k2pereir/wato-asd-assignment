#include "costmap_core.hpp"
#include <cmath> 
#include <algorithm> 
#include <vector>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace robot
{
CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger), inflation_radius_(1.0), resolution_(0.1) {}

    void CostmapCore::initializeCostmap(int width, int height, double resolution, const geometry_msgs::msg::Pose& origin, double inflation_radius) {
        resolution_ = resolution;
        inflation_radius_ = inflation_radius;
        costmap_.info.width = width;
        costmap_.info.height = height;
        costmap_.info.resolution = resolution;
        costmap_.info.origin = origin;
        costmap_.data.assign(width * height, 0); 
        costmap_.header.frame_id = "map";
        costmap_.header.stamp = rclcpp::Clock().now();
        costmap_.info.map_load_time = costmap_.header.stamp;    
        RCLCPP_INFO(logger_, "Costmap initialized with width: %d, height: %d, resolution: %f", width, height, resolution);
    }

void CostmapCore::updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr &scan) {
    std::fill(costmap_.data.begin(), costmap_.data.end(), 0); // Reset costmap
    double angle_min = scan->angle_min;
    for (size_t i = 0; i < scan->ranges.size(); ++i, angle_min += scan-> angle_increment) {
        double range = scan->ranges[i]; 
        if (range < scan->range_min || range > scan-> range_max) continue; 
        double x_pos = range * std::cos(angle_min); 
        double y_pos = range * std::sin(angle_min);
        int grid_x = static_cast<int>((x_pos - costmap_.info.origin.position.x) / resolution_); 
        int grid_y = static_cast<int>((y_pos - costmap_.info.origin.position.y) / resolution_);
        if (grid_x < 0 || grid_x >= static_cast<int>(costmap_.info.width) || grid_y < 0 || grid_y >= static_cast<int>(costmap_.info.height)) continue;
        int index = grid_y * costmap_.info.width + grid_x;
        costmap_.data[index] = 100;
        inflateObstacles(costmap_, grid_x, grid_y);
    }

}

void CostmapCore::inflateObstacles(nav_msgs::msg::OccupancyGrid& costmap_, int origin_x, int origin_y)
{
    int width = costmap_.info.width; 
    int height = costmap_.info.height;
    int cell_radius = static_cast<int>(std::ceil(inflation_radius_ / resolution_));
    int max_cost = 100; 
    for(int dy = -inflation_radius; dy <= inflation_radius; ++dy) {
        for(int dx = -inflation_radius; dx <= inflation_radius; ++dx) {
            int nx = origin_x + dx;
            int ny = origin_y + dy;
            if(nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
            double distance = std::hypot(dx, dy) * resolution_;
            if(distance > inflation_radius_) continue; 
            int cost = static_cast<int>(max_cost * (1 - distance / inflation_radius_));
            int index = ny * width + nx;
            costmap_.data[index] = std::max(costmap_.data[index], static_cast<signed char>(cost));
        }
    }
}

}




