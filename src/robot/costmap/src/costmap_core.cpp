#include "costmap_core.hpp"
#include <cmath> costmap 

namespace robot
{
CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

    void CostmapCore::initializeCostmap(int width, int height, double resolution)
    {
        width_ = width;
        height_ = height;
        resolution_ = resolution;
        costmap_.resize(width_ * height_, 0);
        RCLPP_INFO(logger_, "Costmap initialized with width: %d, height: %d, resolution: %f", width_, height_, resolution_);
    }

    void CostmapCore::convertToGridCoordinates(double range, double angle, int& x, int& y)
    {
        double x_pos = range * std::cos(angle);
        double y_pos = range * std::sin(angle);
        x = static_cast<int>((x_pos + (width_ * resolution_) / 2) / resolution_);
        y = static_cast<int>((y_pos + (height_ * resolution_) / 2) / resolution_);
    }

    void CostmapCore::obstacles(int x, int y) 
    {
        if (x >= 0 && x < width_ && y >= 0 && y < height_)
        {
            costmap_[y * width_ + x] = 100; 
        }
    }

    void CostmapCore::processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr &scan) 
    {
        RCLPP_INFO(logger_, "Processing LaserScan data");
        std::fill(costmap_.begin(), costmap_.end(), 0); // Reset costmap
        for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];
        if (range < scan->range_max && range > scan->range_min) {
            // Calculate grid coordinates
            int x_grid, y_grid;
            convertToGrid(range, angle, x_grid, y_grid);
            markObstacle(x_grid, y_grid);
        }
    }
    inflateObstacles(); 
    }

    void CostmapCore::inflateObstacles() 
    {
        std::vector<int8_t> inflated_costmap = costmap_;
        int radius = static_cast<int>(1/ resolution_);
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                if (costmap_[y * width_ + x] > 0) {
                    for (int dy = -radius; dy <= radius; ++dy) {
                        for (int dx = -radius; dx <= radius; ++dx) {
                            if (dx * dx + dy * dy <= radius * radius) {
                                int nx = x + dx;
                                int ny = y + dy;
                                if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                                    inflated_costmap[ny * width_ + nx] = 100;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    costmap = inflated_costmap;
    
    nav_msgs::msg::OccupancyGrid CostmapCore::getOccupancyGrid() 
    {
        nav_msgs::msg::OccupancyGrid; 
        occupancy_grid.header.frame_id = "map";
        occupancy_grid.info.resolution = resolution_;
        occupancy_grid.info.width = width_;
        occupancy_grid.info.height = height_;
        occupancy_grid.info.origin.position.x = -width_ * resolution_ / 2;
        occupancy_grid.info.origin.position.y = -height_ * resolution_ / 2;
        occupancy_grid.data = costmap_;
        return occupancy_grid;
    }
}



