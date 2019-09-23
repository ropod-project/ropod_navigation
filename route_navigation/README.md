# route_navigation

## Summary

A ROS component that manages action calls for waypoint navigation. The component only listens to actions of type `GOTO`, which are exposed through an action server; ignores all other actions types.

The GOTO action is defined in [this PDDL domain](https://github.com/ropod-project/task-planner/blob/master/config/task_domains/agaplesion/hospital_transportation.pddl).

The component is embedded into a [fault-tolerant state machine](https://github.com/ropod-project/ftsm) and has [this specification](https://github.com/ropod-project/component-monitoring/blob/master/component_monitoring/component_config/robot/software/route_navigation.yaml).

## Dependencies

The waypoint navigation depends on the following other components:
* [`ropod_rod_msgs`](https://github.com/ropod-project/ropod_ros_msgs): Contains message and action definitions used within the component
* [`maneuver_navigation`](https://github.com/ropod-project/ros-structured-nav): Handles navigation requests
* [`route_planner`](https://git.ropod.org/ropod/navigation/ropod_navigation/tree/develop/route_planner): Converts OSM areas and sub-areas in a path plan to 2D waypoints

## Launch file parameters

The component expects several parameters to be made available to the ROS parameter server:
* `mn_nav_topic: str` -- name of a topic for sending maneuver navigation goals (default `/route_navigation/goal`)
* `mn_nav_cancel_topic: str` -- name of a topic for cancelling maneuver navigation goals (default `/route_navigation/cancel`)
* `localisation_topic: str` -- name of a topic on which localisation pose estimates are published (default `/amcl_pose`)
* `goto_server_name: str` -- name of the action server exposed by the component (default `/ropod/goto`)
* `route_planner_server: str` -- name of an action server exposed by a route planner (default `/route_planner`)

* `goto_timeout_s: float` -- timeout (in seconds) for the action of going from one waypoint to another (default `120`)
* `route_planner_timeout_s: float` -- timeout (in seconds) for calls to the route planner (default `120`)
* `recovery_timeout_s: float` -- timeout (in seconds) specifying how long to wait for the component to recover (if recovery is necessary) before reporting a failure (default `120`)
* `recovery_waiting_sleep_s: float` -- sleep duration (in seconds) while waiting for the component to recover itself (default `0.1`)

* `pos_tolerance_m: float` -- distance tolerance (in meters) for navigation waypoints (default `0.3`)
* `orientation_tolerance_deg: float` -- orientation tolerance (in degrees) for navigation waypoints (default `10`)
