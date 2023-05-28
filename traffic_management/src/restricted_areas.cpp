#include "restricted_areas.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace restricted_areas_costmap_plugin
{

// for string delimiter
std::vector<std::string> split(std::string s, std::string delimiter) {
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    std::string token;
    std::vector<std::string> res;

    while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos) {
        token = s.substr (pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back (token);
    }

    res.push_back (s.substr (pos_start));
    return res;
}


RestrictedAreas::RestrictedAreas()
: last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max())
{
}

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void RestrictedAreas::onInitialize() {
  auto node = node_.lock(); 
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_); 

  declareParameter("robot_name", rclcpp::PARAMETER_STRING);
  node->get_parameter(name_ + "." + "robot_name", robot_name); 
  std::string s = split(robot_name, "/").front();
  robot_name = s;
  RCLCPP_INFO(node->get_logger(), "Robot name: %s", robot_name.c_str());

  subscription_ = node->create_subscription<std_msgs::msg::String>(
      "/job_scheduler/goals", 10, std::bind(&RestrictedAreas::jobCallback, this, std::placeholders::_1));

  goal_x = 0.0;
  goal_y = 0.0;
}

void RestrictedAreas::jobCallback(const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_INFO(rclcpp::get_logger("RestrictedAreas"), "msg: %s", msg->data.c_str());

  std::vector<std::string> s = split(msg->data, ",");

  if (s.empty() || s[0] != robot_name) {
    return;
  }
  RCLCPP_INFO(rclcpp::get_logger("RestrictedAreas"), "Got a message for me! %s", msg->data.c_str());

  goal_x = std::stod(s[1]);
  goal_y = std::stod(s[2]);

  std::cout << "goal_x " << goal_x << " goal_y " << goal_y << std::endl;  
}

// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void RestrictedAreas::updateBounds (
  double robot_x, double robot_y, double /*robot_yaw*/, double * /*min_x*/,
  double * /*min_y*/, double * /*max_x*/, double * /*max_y*/)
{
  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "called RestrictedAreas::onFootprintChanged() - override not implemented");

  this->robot_x = robot_x;
  this->robot_y = robot_y;
}

// The method is called when footprint was changed.
// Here it just resets need_recalculation_ variable.
void RestrictedAreas::onFootprintChanged() {
  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "called RestrictedAreas::onFootprintChanged() - override not implemented");
}


bool isInsideRect(int x1, int y1, int x2,
               int y2, int x, int y)
{
    if (x > x1 and x < x2 and y > y1 and y < y2)
        return true;
 
    return false;
}

// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
// Inside this method the costmap gradient is generated and is writing directly
// to the resulting costmap master_grid without any merging with previous layers.
void RestrictedAreas::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j) {

  if (!enabled_) {
    return;
  }

  unsigned int g_x, g_y, r_x, r_y;
  bool res1 = master_grid.worldToMap(goal_x, goal_y, g_x, g_y);
  bool res2 = master_grid.worldToMap(robot_x, robot_y, r_x, r_y);

  if (res1 == false || res2 == false) return;

  // bool isGoalInsideBoxOne = isInsideRect(94, 161, 155, 322, g_x, g_y);
  // bool isRobotInsideBoxOne = isInsideRect(94, 161, 155, 322, r_x, r_y);

  bool isGoalInsideBoxTwo = isInsideRect(156, 161, 193, 322, g_x, g_y);
  bool isRobotInsideBoxTwo = isInsideRect(156, 161, 193, 322, r_x, r_y);

  // bool doNotdrawBoxOne = isGoalInsideBoxOne || isRobotInsideBoxOne;
  bool doNotdrawBoxTwo = isGoalInsideBoxTwo || isRobotInsideBoxTwo;

  //std::cout << "doNotdrawBoxOne: " << doNotdrawBoxOne << ", doNotdrawBoxTwo: " << doNotdrawBoxTwo << std::endl; 

  // if (!doNotdrawBoxOne) {
  //   unsigned char * master_array = master_grid.getCharMap();
  //   for (int j = 94; j < 155; j++) {
  //     for (int i = 161; i < 322; i++) {
  //       master_array[master_grid.getIndex(j, i)] = INFLATED;
  //     }
  //   }
  // }

  if (!doNotdrawBoxTwo) {
    unsigned char * master_array = master_grid.getCharMap();
    for (int j = 156; j < 193; j++) {
      for (int i = 161; i < 322; i++) {
        master_array[master_grid.getIndex(j, i)] = INFLATED;
      }
    }
  }

  // unsigned char * master_array = master_grid.getCharMap();
  
  // for (int j = min_j; j < max_j; j++) {
  //   for (int i = min_i; i < max_i; i++) {
  //     int index = master_grid.getIndex(i, j);

  //     // Determine if we have to add any cost to the node:
  //     uint8_t layer_value = traffic_lanes_layer.data.at(index); 
  //     //e have created our custom planner, we need to export our planner plugin so that it would be visible to the planner server. Plugins are loaded at runtime and if they are not visible, then our planner server won’t be able to load it. In ROS 2, exporting and loading plugins is handled by pluginlib.RCLCPP_INFO(rclcpp::get_logger("nav2_costmap_2d"), "Value: %d", layer_value);

  //     // Lane has value 100, else where 0
  //     if (layer_value == 0) {
  //       // Add cost to node:
  //       unsigned char cost = master_array[index];

  //       // Ensure we do not change lethal or inflated obstacels:
  //       if (cost >= MAXIMUM_NODE_VALUE) {
  //         continue;
  //       }

  //       uint8_t new_cost = std::min(cost + non_lane_cost, MAXIMUM_NODE_VALUE);
  //       master_array[index] = new_cost;
  //     }
  //   }
  // }

  current_ = true;
}

}  // restricted_areas_costmap_plugin

// This is the macro allowing a restricted_areas_costmap_plugin::RestrictedAreas class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(restricted_areas_costmap_plugin::RestrictedAreas, nav2_costmap_2d::Layer)
