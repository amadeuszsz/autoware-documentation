# Obstacle avoidance using A*

## Problem overview

## Block diagram

## Env_Perceiver package

### Subscribed topic
    /clock: rosgraph_msgs/msg/Clock
    /localization/kinematic_state: nav_msgs/msg/Odometry
    /map: nav_msgs/msg/OccupancyGrid
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /planning/racing_planner/trajectory: autoware_auto_planning_msgs/msg/Trajectory
    /sensing/lidar/scan: sensor_msgs/msg/LaserScan

### Published topic
    /border_pointcloud: sensor_msgs/msg/PointCloud2
    /goal_point: geometry_msgs/msg/Pose
    /goal_pointcloud: sensor_msgs/msg/PointCloud2
    /local_occupancy_grid: nav_msgs/msg/OccupancyGrid
    /obstacle_alarm: std_msgs/msg/String
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /start_point: geometry_msgs/msg/Pose
    /start_pointcloud: sensor_msgs/msg/PointCloud2

## Traj_Selector package

### Subscribed topic
    /clock: rosgraph_msgs/msg/Clock
    /goal_point: geometry_msgs/msg/Pose
    /local_occupancy_grid: nav_msgs/msg/OccupancyGrid
    /localization/kinematic_state: nav_msgs/msg/Odometry
    /obstacle_alarm: std_msgs/msg/String
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /planning/racing_planner/trajectory: autoware_auto_planning_msgs/msg/Trajectory
    /start_point: geometry_msgs/msg/Pose

### Published topic
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /planning/racing_planner/avoidance/trajectory: autoware_auto_planning_msgs/msg/Trajectory
    /rosout: rcl_interfaces/msg/Log
    /waypoint_pointcloud: sensor_msgs/msg/PointCloud2

## Working system presentation