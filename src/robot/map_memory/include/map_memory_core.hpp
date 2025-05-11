#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>
#include <cmath>

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);
    void initializeMap(int width, int height, double resolution); 
    void updateCostmap(const nav_msgs::msg::OccupancyGrid& costmap, double x, double y);
    const nav_msgs::msg::OccupancyGrid& getMap() const;


  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid map_;
    void toGlobal(const nav_msgs::msg::OccupancyGrid& costmap, int local_x, int local_y, double x, double y, int& global_x, int& global_y);
};

}  

#endif  
