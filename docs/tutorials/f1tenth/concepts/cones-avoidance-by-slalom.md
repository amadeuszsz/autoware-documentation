# Cones avoidance by slalom

## Problem overview

## System schematic

## Cones detection

## Data fusion

## Planner
To plan a trajectory around the cones, the `cone_planner` node is used.
This node uses the *Informed RRT\** algorithm implemented in `freespace_planning_algorithms` package.

### Subscribed Topics
| Name                                   | Type                                         | Description                                          |
| -------------------------------------- | -------------------------------------------- | ---------------------------------------------------- |
| `/planning/racing_planner/trajectory`  | autoware_auto_planning_msgs::msg::Trajectory | Reference trajectory                                 |
| `/output_map`                          | nav_msgs::msg::OccupancyGrid                 | Occupancy grid map with virtual obstacles            |
| `/localization/cartographer/pose`      | geometry_msgs::msg::PoseStamped              | Pose of the vehicle                                  |
| `/localization/kinematic_state`        | nav_msgs::msg::Odometry                      | Odometry used for checking if the car stopped or not |

### Published Topics
| Name                                            | Type                                         | Description        |
| ----------------------------------------------- | -------------------------------------------- | ------------------ |
| `/planning/racing_planner/avoidance/trajectory` | autoware_auto_planning_msgs::msg::Trajectory | Planned trajectory |

### Parameters
| Name                         | Type   | Description                                                                                                                          |
| ---------------------------- | ------ | ------------------------------------------------------------------------------------------------------------------------------------ |
| `update_rate`                | float  | Update rate of checking if replanning is needed                                                                                      |
| `waypoints_velocity`         | float  | Reference velocity at planned trajectory points                                                                                      |
| `vehicle_shape_margin_m`     | float  | Car perimeter margin used for planning                                                                                               |
| `th_arrived_distance_m`      | float  | Distance between car and goal point to consider that the goal pose has been achieved                                                 |
| `th_stopped_time_sec`        | float  | Time after stopping to consider that the car stopped successfully                                                                    |
| `th_stopped_velocity_mps`    | float  | Velocity threshold to consider that the car stopped                                                                                  |
| `th_course_out_distance_m`   | float  | Distance between car and planned trajectory above which replanning is triggered                                                      |
| `lookahead_distance`         | int    | Number of points on reference trajectory to offset the goal pose from the current one                                                |
| `c_space_margin_m`           | float  | C space margin. When it equals 0 the C space is a rectangle spanned over the current pose and the goal pose                          |
| `replan_when_obstacle_found` | bool   | True if replanning should be triggered when the new obstacle is found                                                                |
| `replan_when_course_out`     | bool   | True if replanning should be triggered when the vehicle is out of the planned trajectory                                             |
| `time_limit`                 | float  | Max time until finding path                                                                                                          |
| `minimum_turning_radius`     | float  | Min car turning radius                                                                                                               |
| `theta_size`                 | int    | Discretization resolution of the heading angle range                                                                                 |
| `obstacle_threshold`         | int    | Cost value threshold to be considered an obstacle                                                                                    |
| `rrt_enable_update`          | bool   | True if planning should be continued after finding the valid path                                                                    |
| `rrt_max_planning_time`      | float  | If the planning time was smaller than this value the planner performs path length optimization until specified amount of time passes |
| `rrt_margin`                 | float  | Additional car perimeter margin used by RRT planner itself                                                                           |
| `rrt_neighbor_radius`        | float  | Radius in which checking if shorter path exist through other neighbor is performed                                                   |

## Demo of the working system
