#### Route planner

Provides ROS service to generate actual waypoints for the robot from a list of areas.

Advertises `route_planner_service`:
* Takes a list of areas as inputs
* Returns a list of areas with pose information

`route_planner` is implemented as an abstract class so new approaches for generating waypoints can be implemented by implementing `compute_route` function in the derived class.

#### Dependencies
Depends on two action servers: `/get_topology_node` and `/get_shape` provided by `ropod_wm_mediator`.

##### Available route planners
* SimpleOSMRoutePlanner - plans route solely based on local topological nodes in OSM world model 

#### Running `route_planner_Service`
```
rosrun ropod_navigation route_planner_node
```
TODO: Add this node to navigation launch file once tested
