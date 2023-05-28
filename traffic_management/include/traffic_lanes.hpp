#ifndef TRAFFIC_LANES_HPP_
#define TRAFFIC_LANES_HPP_

#include <map>
#include <vector>
#include <mutex>

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_map_server/map_io.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace traffic_lanes_costmap_plugin
{

class TrafficLanes : public nav2_costmap_2d::Layer
{

public:
  
  TrafficLanes();

  virtual void onInitialize();

  virtual void updateBounds(
    double robot_x, 
    double robot_y, 
    double robot_yaw, 
    double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);

  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, 
    int min_j, 
    int max_i, 
    int max_j);

  virtual void reset() { return; }

  virtual void onFootprintChanged();
  
  virtual bool isClearable() { return false; }


protected:

  std::string traffic_lanes_yaml;
  uint8_t non_lane_cost;
  uint8_t max_cost;
  nav_msgs::msg::OccupancyGrid traffic_lanes_layer;

private:

  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

  // Maximum node value:
  int MAXIMUM_NODE_VALUE = nav2_costmap_2d::MAX_NON_OBSTACLE;
};

}  // namespace traffic_lanes_costmap_plugin

#endif  // TRAFFIC_LANES_HPP_