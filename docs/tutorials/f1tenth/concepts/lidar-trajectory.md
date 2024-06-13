# Lidar Trajectory Planner

This is project page for planning trajectory for the f1teenth bolid locally with lidar data only.

## Running the demo

For installation refer to [Installation](../../../installation/index.md). Remember to change branch to `f1teenth`.

Once installed you can launch autoware with

```bash
ros2 launch f1tenth_launch e2e_simulator.launch.py
```

Then launch AWSIM

```bash
./autoware_awsim/AWSIM_v1.2.0.x86_64
```

If you don't have a pad to change the gear to DRIVE use this command and wait some time until gear changes to DRIVE permamently

```bash
ros2 topic pub /control/command/gear_cmd autoware_auto_vehicle_msgs/msg/GearCommand {"command: 2"}
```

Finally you can press the `AUTO` button in the rviz and observe how the bolid starts driving.

You should get effects like this:

<video src="
https://github.com/amadeuszsz/autoware-documentation/assets/66121303/7d6b27cb-8a29-4fc3-a3ba-ffaef5e96b49" type="video/mp4" width="1000" controls>
    Your browser does not support the video tag.
</video>

Below you can observe how the path is being planned.

<video src="https://github.com/amadeuszsz/autoware-documentation/assets/66121303/3ec7b4c7-4b4c-4261-9189-8988c480cc81" type="video/mp4" width="1000" controls>
    Your browser does not support the video tag.
</video>

## lidar_trajectory package API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

The package can be found in src folder.

<p align="center">
  <img src="https://github.com/amadeuszsz/autoware-documentation/assets/66121303/b6e539ba-bc4e-4e2f-b74d-47a992ae1aac" alt="Block diagram">
</p>

### Input

| Name                            | Type                        | Description     |
| ------------------------------- | --------------------------- | --------------- |
| `/sensing/lidar/scan`           | sensor_msgs::msg::LaserScan | Data from lidar |
| `/localization/kinematic_state` | nav_msgs::msg::Odometry     | Odometry        |

### Output

| Name                                  | Type                                         | Description        |
| ------------------------------------- | -------------------------------------------- | ------------------ |
| `/planning/racing_planner/trajectory` | autoware_auto_planning_msgs::msg::trajectory | Planned trajectory |


## Algorithm

If you want to use a simplified environment to run your ideas and test how this algorithm works, you can find the Python implementation at this [link](https://github.com/MikolajZielinski/Lidar-Trajectory-Planner).

Below images are depicting bolid (green square with blue arrow), lidar points (red points), reference trajectory (pink points).

The first step is to connect all the lidar points together. If the distance between consecutive points is larger then the threshold, divide the line into sections. Then reduce number of points in each section.

![step_1](https://github.com/amadeuszsz/autoware-documentation/assets/66121303/786418fb-0b47-4ca3-a1d4-d705944faa5f)

Find normal vectors to the points.

![step_2](https://github.com/amadeuszsz/autoware-documentation/assets/66121303/7586de88-3eb7-491a-bb53-d9f3c6910207)

Connect the ends of normal vecotrs together.

![step_3](https://github.com/amadeuszsz/autoware-documentation/assets/66121303/2d6de525-da24-42b5-a5bc-02cbcd76cb59)

Use bezier curve to smooth obtained path.

![step_4](https://github.com/amadeuszsz/autoware-documentation/assets/66121303/cb415963-fcce-44fa-b4fa-dae8888f0e33)

Add the ability to properly join two path sections. This is done by finding two closest points on both paths and connecting them together. At the same time rest of the points is ignored.

![step_5](https://github.com/amadeuszsz/autoware-documentation/assets/66121303/3dcf9619-1da2-45f6-904b-0eff4f944ca0)

Next step is to recognize the intersections. To achieve that, every start and end point from each subsection is connected with each other. Then the triangle with biggest perimeter is found.

![step_6](https://github.com/amadeuszsz/autoware-documentation/assets/66121303/09d58f6e-b77b-40d8-8b6b-ab13c19d6231)

Left corner of this triangle is showing the path to the left, and right corner to the right. Here we can see two examples of trajectories planned on intersections.

![step_7](https://github.com/amadeuszsz/autoware-documentation/assets/66121303/f5bcfca9-f27e-4afa-be9a-e2ab1080a914)
![step_8](https://github.com/amadeuszsz/autoware-documentation/assets/66121303/7f0aa924-9531-4d54-91c8-dd9f131bb1e4)

## Future work

There are several features that can be added to this system:
- Improving the stability of the planned path.
- Ability to dynamically select right or left turns.
- Calculating the speed of the bolid based on the trajectory curve. This would lead to the bolid going faster on a straight road and slowing down on curves.