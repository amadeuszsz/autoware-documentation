# Full demo

!!! note

    Running the F1TENTH demo requires some additional steps on top of building and installing Autoware, so make sure that [F1TENTH installation](installation.md) has been completed first before proceeding.

## Running steps

1. Run the container if you have not already done so and update the environment.
   
   ```bash
   cd ~/autoware
   ./run_amd64.sh
   cd autoware
   source install/setup.bash
   sudo apt-get update
   rosdep install --from-paths src --ignore-src -y
   ```

2. Open a few (4-6) new terminal windows/tabs and enter the container with the following command in each of them.
   
   ```bash
   cd ~/autoware
   ./enter.sh
   cd autoware
   source install/setup.bash
   ```

3. Launch the simulator.
   
   ```bash
   ../AWSIM_v1.1.0_F1TENTH/AWSIM_F1TENTH.x86_64
   ```
4. Launch the Autoware.

   ```bash
   ros2 launch f1tenth_launch f1tenth.launch.py
   ```

5. Once you see the RVIZ window with the map loaded, trigger the localization node.

   ```bash
   ros2 service call /localization/trigger_node std_srvs/srv/SetBool "{data: true}"
   ```

Done! Steps 1 and 2 need to be done only once, after the container is created. Step 3 when you want to reset the position of the vehicle. Steps 4 and 5 each time you have made changes to the code. You can terminate the simulation by pressing `Ctrl+C` in the terminal window where you launched the simulator and same for F1TENTH launch.

## Planning

The Autoware Planning component consists of multiple modules responsible for different tasks. In short, these packages handle vehicle behaviors and generate suitable trajectories for current vehicle ego state and environment. For more information about the Planning component, please refer to the [Planning architecture](../../../design/autoware-architecture/planning/). Autonomous racing is a use case with very specific requirements, so reusing the existing component is not best idea in order to achieve the best performance. Instead, we can just few modules from Autoware and combine them with custom ones to achieve the desired behavior. 

### Free space planner

One of worth noting module is the [Free Space Planner](https://autowarefoundation.github.io/autoware.universe/main/planning/freespace_planner/). Based on occupancy grid and given path search algorithm, it generates a set of waypoints formed under defined constraints. By stack concept, it is used as a parking planner but it can be adapted to custom scenarios as well. Currently, two algorithms are supported: A\* and RRT\*.

#### Task

1. Launch the simulator.
   
   ```bash
   ../AWSIM_v1.1.0_F1TENTH/AWSIM_F1TENTH.x86_64
   ```

2. Launch the Autoware.

   ```bash
   ros2 launch f1tenth_launch f1tenth.launch.py
   ```

3. Trigger the localization node.

   ```bash
   ros2 service call /localization/trigger_node std_srvs/srv/SetBool "{data: true}"
   ```

4. Select the 2D Nav Goal tool in RVIZ on the top bar and click on the map to set the goal position. Ensure correct orientation. When you see the trajectory, click AUTO button from Autoware State Panel to start the vehicle motion. Vehicle should follow the trajectory and stop at the goal position.

5. Modify the Free Space Planner and A\* parameters in order to get more aggressive trajectory. The config is located in `autoware/src/launcher/external/f1tenth_launch/config/planning/freespace_planner.param.yaml`. To fully understand the parameters, please refer to the [documentation](https://autowarefoundation.github.io/autoware.universe/main/planning/freespace_planner/). Basic knowledge of inner workings algorithms is required as well. Repeat steps 1-4 and test the new configuration.

6. In same config file change `planning_algorithm` to `rrtstar`. Check the different parameter values as well. Repeat steps 1-4 and test the new configuration.

![type:video](https://www.youtube.com/embed/LtObbLfJzm0)

!!! note

    Updating the parameters requires restarting the Autoware launch. Restarting the simulator is not necessary but it is recommended to reset the vehicle position. Otherwise you need to set the initial pose manually with the 2D Pose Estimate tool in RVIZ. Do not forget to trigger the localization node again.

!!! note
      
    Workspace built with `--symlink-install` does not require building the package after modifying the config and launch files.

!!! note
      
    Due to the fact that Autoware volume is mapped in the container, you can edit the files directly via any text editor on the host machine.

!!! note
      
    For higher vehicle velocity and low machine performance, the simulation time can be slowed down using the slider at the bottom of the simulator window. 

### Static planners

Moving on to autonomous racing and its nature, the required effort for generating optimal race trajectory for a well defined (already mapped) environment could be done offline. In this case, developers are not limited by the real-time constraints and can use more complex algorithms. Indeed, challenging with opponents on track would need possible handling of additional behaviors like overtaking, slowing down behind the opponent (or no) and so on but this is a good starting point. For demonstration purposes, the race trajectory was generated by [global racetrajectory optimization](https://github.com/TUMFTM/global_racetrajectory_optimization).

#### Task

1. Launch the simulator.
   
   ```bash
   ../AWSIM_v1.1.0_F1TENTH/AWSIM_F1TENTH.x86_64
   ```

2. Launch the Autoware.

   ```bash
   ros2 launch f1tenth_launch f1tenth.launch.py use_trajectory_loader:=True
   ```

3. Trigger the localization node.

   ```bash
   ros2 service call /localization/trigger_node std_srvs/srv/SetBool "{data: true}"
   ```

4. Select the 2D Nav Goal tool in RVIZ on the top bar and click on the map anywhere between the track boundaries, just to select a lane. When the lane is focused, click AUTO button from Autoware State Panel to start the vehicle motion.

![type:video](https://www.youtube.com/embed/rj7HJB9bPcE)

## Control

Next to the planners, the Control component stands for taking given trajectory and generating the control commands for the vehicle. For more information about the Control component, please refer to the [Control architecture](../../../design/autoware-architecture/control/). These modules can be used as they are with tiny touch of tuning the parameters.

### Model Predictive Control

Do the recent trajectory affects unstable vehicle motion? The controller might need to be tuned.

#### Task

1. Modify the MPC parameters in order to get more stable behavior. The config is located in `autoware/src/launcher/external/f1tenth_launch/config/control/trajectory_follower/lateral/mpc.param.yaml`. To fully understand the parameters, please refer to the [documentation](https://autowarefoundation.github.io/autoware.universe/main/control/mpc_lateral_controller/). Basic knowledge of inner workings algorithms is required as well.

2. Launch the simulator.
   
   ```bash
   ../AWSIM_v1.1.0_F1TENTH/AWSIM_F1TENTH.x86_64
   ```

3. Launch the Autoware.

   ```bash
   ros2 launch f1tenth_launch f1tenth.launch.py use_trajectory_loader:=True
   ```

4. Trigger the localization node.

   ```bash
   ros2 service call /localization/trigger_node std_srvs/srv/SetBool "{data: true}"
   ```

5. Select the 2D Nav Goal tool in RVIZ on the top bar and click on the map anywhere between the track boundaries, just to select a lane. When the lane is focused, click AUTO button from Autoware State Panel to start the vehicle motion.

6. Repeat steps 1-5 in order to find the best parameters.


### Pure Pursuit

The second option, fortunately with a lower entry threshold. To start working with the Pure Pursuit controller, a change in launch file will be necessary. Open the top-level launch file `autoware/src/launcher/external/f1tenth_launch/launch/f1tenth.launch.py`, find `lateral_controller_mode` within `control_launch` module and change its value to `pure_pursuit`.

#### Task

1. Launch the simulator.
   
   ```bash
   ../AWSIM_v1.1.0_F1TENTH/AWSIM_F1TENTH.x86_64
   ```

2. Launch the Autoware.

   ```bash
   ros2 launch f1tenth_launch f1tenth.launch.py use_trajectory_loader:=True
   ```

3. Trigger the localization node.

   ```bash
   ros2 service call /localization/trigger_node std_srvs/srv/SetBool "{data: true}"
   ```

4. Select the 2D Nav Goal tool in RVIZ on the top bar and click on the map anywhere between the track boundaries, just to select a lane. When the lane is focused, click AUTO button from Autoware State Panel to start the vehicle motion.

5. Modify the Pure Pursuit parameters in order to get more stable behavior. The config is located in `autoware/src/launcher/external/f1tenth_launch/config/control/trajectory_follower/lateral/pure_pursuit.param.yaml`. Repeat steps 1-4 in order to find the best parameters.
