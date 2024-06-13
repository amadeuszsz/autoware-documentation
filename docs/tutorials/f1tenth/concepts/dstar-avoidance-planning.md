# Obstacle avoidance planning using D* graph-based algorithm

The goal of this project is to allow F1TENTH autonomous car to avoid obstacles that are present on the track. The environment is designed so that the car follows a predefined trajectory (`/planning/racing_planner/trajectory`), which is partially modified when computing obstacle avoidance.

Obstacles are detected using data from laser scanner and added to the occupancy map. The selected A* or D* planner finds avoidance trajectory in the free space beetwen start node and the goal node. Modified trajectory consists of waypoints with the least cost to reach based on heuristics. For real-time operation a partial grid map is extracted to tighten searching space. The goal node is a parameter and is set to be +30 nodes in the original trajectory from the current node when an obstacle is detected.


## Installation

Requirements for this project is a valid instalation of Autoware environment, you can check it under this link [Installation](../../../installation/index.md). 

You can run demo project with 

```bash
ros2 launch f1tenth_launch e2e_simulator.launch.py
```
By default, this command launches package `obstacle_astar` with D* Lite planner. If you want to try A* planner you can specify a parameter `use_dstar` to true as was shown below
```bash
ros2 launch f1tenth_launch e2e_simulator.launch.py use_dstar:=true
```

After that you can launch AWSIM with

```bash
./autoware_awsim/AWSIM_v1.2.0.x86_64
```
To start autonomous driving u need to have a pad. On the Xbox pad you should press
```bash
 X -> A -> B -> left arrow -> B
```

If you don't have a pad u can try to use this command to change gear to DRIVE 

```bash
ros2 topic pub /control/command/gear_cmd autoware_auto_vehicle_msgs/msg/GearCommand {"command: 2"}
```

You can see results of trajectory planning using A* under below link.


[![mq2](https://github.com/amadeuszsz/autoware-documentation/assets/126728897/2a665f9c-b30e-4e0f-a5e9-5ec7838c41a0)](https://youtu.be/mDZgydrJ3Kk)

## Block diagram

A block diagram below shows all used topics and packages included in this project. 

![block_diagram](https://github.com/amadeuszsz/autoware-documentation/assets/126728897/f401ec99-a3d5-4476-8da6-6b9e90f9cfd9)

Inside the `freespace_planning_algorithms` package there are multiple algorithms that can be used for path planning. The name of the algorithm to use can be specified in e.g.  `obstacle_dstar.param.yaml`. In case of D* you should use package `obstacle_dstar` and for A* you should use `obstacle_astar` package.

It is crucial because of differences inside `onTimer` function which is the main loop of the algorithm.

## Packages API

The package can be found in `src` folder.

### obstacle_detection

| Topic                           | Type                         | Function                        | Description                                                       |
| ------------------------------- | ---------------------------- | ------------------------------- | ----------------------------------------------------------------- |
| `/sensing/lidar/scan`           | sensor_msgs::msg::LaserScan  | Input scans from LIDAR          | Scans used to detect obstacles and add them to the occupancy grid |
| `/map`                          | nav_msgs::msg::OccupancyGrid | Input original map              | Generated map from `cartographer`                                 |
| `/localization/kinematic_state` | nav_msgs::msg::Odometry      | Input odometry for localization | Original odometry                                                 |
| `/modified_map`                 | nav_msgs::msg::OccupancyGrid | Output map with obstacles       | Occupancy grid map with obstacles detected by LiDAR.              |

### obstacle_astar

| Topic                                           | Type                                         | Function                        | Description                                                                                   |
| ----------------------------------------------- | -------------------------------------------- | ------------------------------- | --------------------------------------------------------------------------------------------- |
| `/planning/racing_planner/trajectory`           | autoware_auto_planning_msgs::msg::Trajectory | Input reference trajectory      | Trajectory that is originally subscribed                                                      |
| `/modified_map`                                 | nav_msgs::msg::OccupancyGrid                 | Input map with obstacles        | Occupancy grid map with obstacles detected by LIDAR. It is made by obstacle_detection package |
| `/localization/kinematic_state`                 | nav_msgs::msg::Odometry                      | Input odometry for localization | Original odometry                                                                             |
| `/planning/racing_planner/avoidance/trajectory` | autoware_auto_planning_msgs::msg::Trajectory | Output modified trajectory      | Modified trajectory output by planner                                                         |

### obstacle_dstar

| Topic                                           | Type                                         | Function                        | Description                                                                                   |
| ----------------------------------------------- | -------------------------------------------- | ------------------------------- | --------------------------------------------------------------------------------------------- |
| `/planning/racing_planner/trajectory`           | autoware_auto_planning_msgs::msg::Trajectory | Input reference trajectory      | Trajectory that is originally subscribed                                                      |
| `/modified_map`                                 | nav_msgs::msg::OccupancyGrid                 | Input map with obstacles        | Occupancy grid map with obstacles detected by LIDAR. It is made by obstacle_detection package |
| `/localization/kinematic_state`                 | nav_msgs::msg::Odometry                      | Input odometry for localization | Original odometry                                                                             |
| `/planning/racing_planner/avoidance/trajectory` | autoware_auto_planning_msgs::msg::Trajectory | Output modified trajectory      | Modified trajectory output by planner                                                         |  |

## Main components description

### Detecting obstacles
One of the main functionalities is obstacle detection. It is done by `obstacle_detection` package. Based on the input LiDAR scans, odometry and occupancy grid from cartographer it detects map indices where the laser hits an obstacle. If at least `cell_count_th` (currently 2) scans lie in a certain grid cell, it is set as occupied (100). Modified map is then published to `/modified_map` topic and can be used by planning algorithms to find avoidance trajectory.

| Original map                                             | Modified map                                             |
| -------------------------------------------------------- | -------------------------------------------------------- |
| ![original_map_detection](https://github.com/amadeuszsz/autoware-documentation/assets/126728897/8fb1fdc6-cd96-41e8-a8d4-688a8f8245fb) | ![modified_map_detection](https://github.com/amadeuszsz/autoware-documentation/assets/126728897/5aac5c8b-68fb-42a7-a308-f114d73ae741) |


### New trajectory planning - A*
`obstacle_astar` package is partially based on autoware.universe `freespace_planner` package. It periodically does the following steps:
- from the current trajectory it cuts out a partial trajectory of a set length from the current pose onwards,
- checks whether there is an obstacle on this partial trajectory in function `isPlanRequired`,
- if an obstacle is found it extracts partial grid map from original one and sets it as a map for planner,
- performs planning using specified algorithm,
- creates a new trajectory by replacing nodes from original trajectory with `new avoidance trajectory` from planner,
- sets current trajectory as the one that the car will follow,
- avoids the obstacle by following the new trajectory until the car is in the vicinity of the goal node,
- once the car reaches the goal node it switches to referential trajectory,

This loop ensures that if the car moves too close to the obstacle or the shape/position of the obstacle changes, the trajectory can be replanned over and over again to properly avoid collision. 

<p align="center">
  <img src="https://github.com/amadeuszsz/autoware-documentation/assets/126728897/8ec38c99-40f9-4eae-ad61-58d74077166f" alt="first_plan">
</p>
<p align="center">
  <img src="https://github.com/amadeuszsz/autoware-documentation/assets/126728897/769287f6-b5e6-468f-85ea-fe58376dc9ec" alt="second_plan">
</p>
<p align="center">
  <img src="https://github.com/amadeuszsz/autoware-documentation/assets/126728897/17877c26-281c-4e79-8cb4-3d3e06017c3d" alt="third_plan">
</p>



To help the process of implementation and debugging markers have been added to the project. Each marker is placed in the position of every node with regardless of the orienatation. All markers are stored in `MarkerArray`. To view them you need to add `/markers` topic in the rviz. One of the possible outcomes looks like this:
 
![markers](https://github.com/amadeuszsz/autoware-documentation/assets/126728897/3440f205-7ee4-423e-8dc2-b768f02c5dac)


### New trajectory planning - D*Lite
Package `obstacle_dstar` is very similar to `obstacle_astar`, however it has a few modifications inside `onTimer` function. One of the biggest modification is planner initialization. In comparison to A*, which is a classic graph-based algorithm, D*Lite is an incremental algorithm. This implies that in every iteration when an obstacle is detected the planner must have an updated map. 

To ensure that, there is a flag `is_detected` which sets partial grid map and clears nodes and graph the first time an obstacle is detected. It also saves visited nodes that were used to make the avoidance trajectory. 

The planner that was used in this package was added to the autoware.universe `freespace_planning_algorithms`  package. It is placed in the same namespace as `AbstractAlgorithms` to make sure it is compatible with the current API. The only change that was needed to be made was to add `clearNodesDstar` function to the public header file of AbstractAlgorithms.

This planner is still not fully developed as it was tested only for creating nodes in a graph. 

## Conducted Experiments

Performance of the car in this case is based on planner algorithm's speed and it's parameters. Some of the parameters are conditioned by the car hardware such as `vehicle_shape_margin_m` but the others are common for all planners. In this section there were mentioned basic test on different kinds of combinations of planner parameters. 

Below there is a list of parameters that resulted in the best performance during testing.

![params](https://github.com/amadeuszsz/autoware-documentation/assets/126728897/31d26f6b-d5f3-4f7e-b0f3-6075ab5dd82d)

Parameter with the biggest impact was `vehicle_shape_margin_m`. By default  this parameter is set to `1.0` which means that in every position where a collision detection is computed the shape of a car is enlarged by 1 meter of circumference. This is crucial when a reference trajectory goes close to the walls because with a value of `1.0` it always detects collision no matter where the car is. 

The second most important parameters are `minimum/maximum_turning_radius`. Modificating those values from 1 to 9 can change output trajectory from the planner. The lower this values are the sharper are the curves which the car follows. By setting them to the value in the middle e.g. 5 the path should create a smooth line around obstacles while also being fast enough. 

You can also decrease the planning time by setting lower `theta_size` which creates a minimum angular step taken in each iteration of planning. The lower this value is the faster might be our algorithm. 

Depending on the speed of the car and the size of the track you should also consider adjusting `planning_distance` and `collision_check_distance`.  If there is a need to know about obstacles faster you should change `collision_check_distance` to higher value. `Planning_distance` can be set with prior knowledge about possible shape and size of the obstacles. If you know that the obstacle might be relatively small, the algorithm can plan only for short distances behind the detected obstacles. 

In case of the map from cartographer being imperfect a good idea might be to extract a partial occupancy grid map with a fixed window size. It might be helpful in specific places on a map such as open spaces or intersecions, where many nodes might be located and as a result the whole process might take longer. Extracting partial grid map can limit the size of free space to be planned so that the speed of an algorithm increases. The unknown parts in this partial grid map should be set as `-1`. 


![turning_radius_2](https://github.com/amadeuszsz/autoware-documentation/assets/126728897/cf560f59-cb99-4814-b3c9-b23b157d395a)
This trajectory is much sharper than the other because it has lower turning radius.

![turning_radius_5](https://github.com/amadeuszsz/autoware-documentation/assets/126728897/914105ed-660f-403e-b5a2-d807379a4b6f)
This trajectory is much smoother than the other because it has higher turning radius. Output path is also further from obstacles.


## Possible future improvements

- Adding more parameters for testing/debugging purposes,
- Visualization of markers not only for A*,
- Finishing the development of D* Lite algorithm,
- Optimizing D* Lite algorithm,
- Comparing D* and D* Lite performance,
- Merginig `obstacle_dstar` and `obstacle_astar` packages into one common package for all planners,
- Validating in different environmnents.

### Credits

Andrzej Ziemnicki

Jeremiasz Wojciak

Norbert Mostowski

**Poznan University of Technology 2024**
