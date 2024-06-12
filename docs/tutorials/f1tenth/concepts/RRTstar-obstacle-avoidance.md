# Obstacle avoidance using RRT* algorithm
[Screencast from 06-13-2024 01:19:01 AM.webm](https://github.com/Morgaliel/autoware-documentation/assets/58528111/a297c91b-8702-475d-bd4e-dae12c9a17f2)


The goal of this project is to add obstacle avoidance functionality to the F1TENTH autonomous car. Currently, the car follows a predefined trajectory based on the given .csv file. For this project, the input trajectory topic remapping is set to `/planning/racing_planner/avoidance/trajectory` instead of predefined `/planning/racing_planner/trajectory`. When an obstacle is detected, the car should avoid it by computing a new trajectory using the RRT* algorithm. The new trajectory is then published to the `/planning/racing_planner/avoidance/trajectory` topic (in other cases, the predefined path is published by the same topic).

Obstacle detection is based on the data from the laser scanner and the occupancy grid map. The obstacle detection and avoidance algorithm is implemented in the `obstacle_detection` package. The detected obstacles are added to the occupancy grid map and published to the `/modified_map` topic. 

The package uses the RRT* algorithm to find an avoidance trajectory in the free space between the start node and the goal node. The car follows the avoidance trajectory until it reaches the goal node. 


## How to run

The requirement for this project is a valid installation of the Autoware environment, you can check it under this link [Installation](../../../installation/index.md). 

You can launch the AWSIM F1TENTH simulator with the following command:

```bash
./autoware_awsim/AWSIM_v1.2.0.x86_64
```

After that, you can launch the obstacle avoidance package alongside all the Autoware stack with the following command:

```bash
ros2 launch f1tenth_launch e2e_simulator.launch.py
```

and for starting the autonomous mode with the X-box controller, one should make this combination:
```bash
 X -> A -> B -> left arrow -> B
```



## Block diagram

A block diagram below shows all used topics included in this project and the idea behind it.
![image (1)](https://github.com/Morgaliel/autoware-documentation/assets/58528111/c8ae752a-9514-44fc-ac47-c7ce4f5e3b66)



## Packages API

The package can be found in the src folder.

### obstacle_avoidance

| Topic                           | Type                         | Function                        | Description                                                   |
|---------------------------------|------------------------------|---------------------------------|---------------------------------------------------------------|
| `/sensing/lidar/scan`           | sensor_msgs::msg::LaserScan  | Input scans from LIDAR          | Scans used to detect obstacles and add them to the occupancy grid |
| `/map`                          | nav_msgs::msg::OccupancyGrid | Input original map              | Generated map from `cartographer`                               |
| `/map_updated`                          | nav_msgs::msg::OccupancyGrid | Output map              | Updated map after inflation                             |
| `/localization/cartographer/pose` | geometry_msgs::msg::PoseStamped | Vehicle Odometry | Vehicle Position                     |
| `/planning/racing_planner/avoidance/trajectory` | nav_msgs::msg::Path | Output avoidance trajectory     | New trajectory planned by RRT* algorithm.                      |
| `/planning/racing_planner/trajectory` | nav_msgs::msg::Path | Input referential trajectory   | Referential trajectory that the car is following.              |
| `/path_found` | visualization_msgs::msg::Marker | Visualise path markers | Shows path found to goal by RRT* |
| `/goal` | visualization_msgs::msg::Marker | Visualise markers | Shows current goal used by RRT* |
| `/tree_nodes` | visualization_msgs::msg::Marker | Visualise tree nodes markers | Shows points found by RRT* |
| `/tree_branches` | visualization_msgs::msg::Marker | Visualise branches markers | Shows edges to tree nodes made by RRT* |


## Main components description
All components are included in the `obstacle_avoidance` package and the RRT* algorithm is run in the `/localization/cartographer/pose`  callback.

### Detecting obstacles
One of the main functionalities is obstacle detection - based on the input LiDAR scans, odometry, and occupancy grid from the cartographer, it detects trajectory indices where the laser hits an obstacle. 
Between every 2 waypoints (in the `COLLISION_HORIZON` range), the algorithm divides the space into grid cells and checks occupancy. Ranges from the `/sensing/lidar/scan` topic are being cropped to +-60 degrees to increase the performance of the algorithm.

### New trajectory planning - RRT* algorithm
- from preplanned trajectory, it cuts out a partial trajectory of a certain length from the current pose onwards,
- checks whether there is an obstacle on this partial trajectory (`obstacle_detected` flag)
- if yes, it performs planning using the RRT* algorithm to find a new trajectory that avoids the obstacle and publishes it to the `/planning/racing_planner/avoidance/trajectory` topic
  - during following the new trajectory, it repeats the process of checking for obstacles and replanning
  - the needed Yaw angle is calculated based on an angle between the every 2 following points on the replanned trajectory
  - after reaching the goal node, it switches back to a preplanned trajectory
- if no, it publishes the preplanned trajectory as the one that the car is currently following (on the same topic as above)
![Screenshot from 2024-06-13 00-26-59](https://github.com/Morgaliel/autoware-documentation/assets/58528111/24b6c883-7f2c-4622-bb96-83b4a8d70473)


### Tunable parameters

 * int COLLISION_HORIZON - number of waypoints ahead to be checked for obstacles
 * int MAX_ITER - maximum number of iterations for RRT* algorithm
 * int MIN_ITER - minimum number of iterations for RRT* algorithm
 * double STD - standard deviation for random sampling
 * double STEER_RANGE - steering range for RRT* algorithm
 * float NEAR_RANGE - range for finding the nearest node
 * double GOAL_THRESHOLD - the threshold for reaching the goal node
 * float GOAL_AHEAD_DIST - distance from the goal node
 * double X_SAMPLE_RANGE - range for random sampling, X axis
 * double Y_SAMPLE_RANGE - range for random sampling, Y axis
 * float MARGIN - margin for the obstacle detection
 * float MARGIN_2 - margin for the new RRT* node samples
 * float DETECTED_OBS_MARGIN - margin for the detected obstacle
 * float DETECTED_OBS_MARGIN_2 - margin for the new RRT* node samples
 * float VELOCITY - fixed velocity of the car during an obstacle avoidance maneuver



