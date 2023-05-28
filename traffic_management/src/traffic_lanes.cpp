#include "traffic_lanes.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace traffic_lanes_costmap_plugin
{

TrafficLanes::TrafficLanes()
: last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max())
{
}

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void TrafficLanes::onInitialize() {
  auto node = node_.lock(); 
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);
  
  declareParameter("traffic_lanes_yaml", rclcpp::PARAMETER_STRING);
  node->get_parameter(name_ + "." + "traffic_lanes_yaml", traffic_lanes_yaml);
  RCLCPP_INFO(rclcpp::get_logger("nav2_costmap_2d"), "Traffic_lanes_yaml: %s", traffic_lanes_yaml.c_str());

  declareParameter("non_lane_cost", rclcpp::PARAMETER_INTEGER);
  node->get_parameter(name_ + "." + "non_lane_cost", non_lane_cost);
  RCLCPP_INFO(rclcpp::get_logger("nav2_costmap_2d"), "non_lane_cost: %d", non_lane_cost);

  declareParameter("max_lane_cost", rclcpp::PARAMETER_INTEGER);
  node->get_parameter(name_ + "." + "max_lane_cost", max_cost);
  RCLCPP_INFO(rclcpp::get_logger("nav2_costmap_2d"), "max_cost: %d", max_cost);

  // only try to load map if parameter was set
  if (traffic_lanes_yaml.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("nav2_costmap_2d"), "No traffic lanes yaml provided for the node. Exiting...");
  }

  nav2_map_server::LOAD_MAP_STATUS status = nav2_map_server::loadMapFromYaml(traffic_lanes_yaml, traffic_lanes_layer);  
  RCLCPP_INFO(rclcpp::get_logger("nav2_costmap_2d"), "Load map status: %d", status);
}

// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void TrafficLanes::updateBounds (
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * /*min_x*/,
  double * /*min_y*/, double * /*max_x*/, double * /*max_y*/)
{
  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "called TrafficLanes::onFootprintChanged() - override not implemented");
}

// The method is called when footprint was changed.
// Here it just resets need_recalculation_ variable.
void TrafficLanes::onFootprintChanged() {
  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "called TrafficLanes::onFootprintChanged() - override not implemented");
}

// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
// Inside this method the costmap gradient is generated and is writing directly
// to the resulting costmap master_grid without any merging with previous layers.
void TrafficLanes::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j) {

  if (!enabled_) {
    return;
  }

  unsigned char * master_array = master_grid.getCharMap();
  
  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      int index = master_grid.getIndex(i, j);

      // Determine if we have to add any cost to the node:
      uint8_t layer_value = traffic_lanes_layer.data.at(index); 
      //e have created our custom planner, we need to export our planner plugin so that it would be visible to the planner server. Plugins are loaded at runtime and if they are not visible, then our planner server won’t be able to load it. In ROS 2, exporting and loading plugins is handled by pluginlib.RCLCPP_INFO(rclcpp::get_logger("nav2_costmap_2d"), "Value: %d", layer_value);

      // Lane has value 100, else where 0
      if (layer_value == 0) {
        // Add cost to node:
        unsigned char cost = master_array[index];

        // Ensure we do not change lethal or inflated obstacels:
        if (cost >= MAXIMUM_NODE_VALUE) {
          continue;
        }

        uint8_t new_cost = std::min(cost + non_lane_cost, (int)max_cost);
        master_array[index] = new_cost;
      }
    }
  }

  current_ = true;
}

}  // traffic_lanes_costmap_plugin

// This is the macro allowing a traffic_lanes_costmap_plugin::TrafficLanes class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(traffic_lanes_costmap_plugin::TrafficLanes, nav2_costmap_2d::Layer)
