# Source code for Traffic Management Solution

This repo contains the source code and packages that is required to compile and run the traffic management solution with ROS 2, Nav2, Gazebo.

The package `traffic_management` contains the source code that implements the traffic management solution. Interesting files are:
* Traffic Lanes Layer is implemented by `traffic_lanes.cpp` ([https://github.com/Slangb17/traffic_management_solution/blob/main/traffic_management/src/traffic_lanes.cpp](https://github.com/Slangb17/traffic_management_solution/blob/main/traffic_management/src/traffic_lanes.cpp))
* Restricted Areas Layer is implemented by `restricted_areas.cpp` ([https://github.com/Slangb17/traffic_management_solution/blob/main/traffic_management/src/restricted_areas.cpp](https://github.com/Slangb17/traffic_management_solution/blob/main/traffic_management/src/restricted_areas.cpp))
* Traffic Directions is implemented by `tm_navfn_planner.cpp` ([https://github.com/Slangb17/traffic_management_solution/blob/main/traffic_management/src/tm_navfn_planner.cpp](https://github.com/Slangb17/traffic_management_solution/blob/main/traffic_management/src/tm_navfn_planner.cpp))

The following packages are 3rd party that have been used for the implementation:
* `ira_laser_tools` ([https://github.com/iralabdisco/ira_laser_tools](https://github.com/iralabdisco/ira_laser_tools))
* `mir_robot` ([https://github.com/relffok/mir_robot](https://github.com/relffok/mir_robot))
* `twist_stamper` ([https://github.com/joshnewans/twist_stamper](https://github.com/joshnewans/twist_stamper))
