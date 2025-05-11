#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);
    void initializeCostmap(int width, int height, double resoulution, const geometry_msgs::msg::Pose& origin, double inflation_radius);
    void updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr &scan);
    void inflateObstacles(nav_msgs::msg::OccupancyGrid& costmap, int origin_x, int origin_y); 

  private:
    rclcpp::Logger logger_;
    int inflation_radius_;
    double resolution_;
    int width_;
    int height_;
};

}  

#endif  