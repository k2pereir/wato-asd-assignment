#include "planner_core.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" 
#include <cmath>
#include <vector>
#include <memory>
#include <functional>
#include <string>

namespace robot
{

robot::PlannerCore::PlannerCore(const rclcpp::Logger& logger) : logger_(logger) {}

CellIndex worldToMap(double x, double y, const nav_msgs::msg::OccupancyGrid& map)
{
  int map_x = static_cast<int>((x - map.info.origin.position.x) / map.info.resolution);
  int map_y = static_cast<int>((y - map.info.origin.position.y) / map.info.resolution);
  return CellIndex(map_x, map_y);
}

geometry_msgs::msg::Pose indexToWorld(CellIndex index, const nav_msgs::msg::OccupancyGrid& map)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = index.x * map.info.resolution + map.info.origin.position.x + map.info.resolution / 2.0;
  pose.position.y = index.y * map.info.resolution + map.info.origin.position.y + map.info.resolution / 2.0;
  pose.position.z = 0.0;
  return pose;
}

nav_msgs::msg::Path PlannerCore::aStar(
  const nav_msgs::msg::OccupancyGrid& map,
  const geometry_msgs::msg::Pose& start,
  const geometry_msgs::msg::Point& goal)
{
  CellIndex start_index = worldToMap(start.position.x, start.position.y, map);
  CellIndex goal_index  = worldToMap(goal.x,            goal.y,            map);

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, double, CellIndexHash>     g_score;

  open_set.emplace(start_index, 0.0);
  g_score[start_index] = 0.0;

  const std::vector<CellIndex> directions{{1,0},{-1,0},{0,1},{0,-1}};

  while (!open_set.empty()) {
    CellIndex current = open_set.top().index;
    open_set.pop();

    if (current == goal_index) {
      nav_msgs::msg::Path path;
      path.header.frame_id = map.header.frame_id;
      path.header.stamp = rclcpp::Clock().now();  

      for (CellIndex idx = current; idx != start_index; idx = came_from[idx]) {
        geometry_msgs::msg::PoseStamped ps;
        ps.pose = indexToWorld(idx, map);
        path.poses.insert(path.poses.begin(), ps);
      }
      {
        geometry_msgs::msg::PoseStamped ps;
        ps.pose = indexToWorld(start_index, map);
        path.poses.insert(path.poses.begin(), ps);
      }
      return path;
    }

    for (const auto & dir : directions) {
      CellIndex neighbor(current.x + dir.x, current.y + dir.y);

      if (neighbor.x < 0 || neighbor.y < 0 ||
          neighbor.x >= static_cast<int>(map.info.width) ||
          neighbor.y >= static_cast<int>(map.info.height))
        continue;

      int idx = neighbor.y * map.info.width + neighbor.x;
      if (map.data[idx] > 50)  
        continue;

      double tentative_g = g_score[current] + 1.0;
      if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]) 
      {
          came_from[neighbor] = current;
          g_score[neighbor]   = tentative_g;
          double h = std::hypot(goal_index.x - neighbor.x, goal_index.y - neighbor.y);
          open_set.emplace(neighbor, tentative_g + h);
      }
    }
  }
  return nav_msgs::msg::Path();
}

} 
