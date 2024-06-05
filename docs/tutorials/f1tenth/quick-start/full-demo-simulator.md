# Simulator full demo

## Running steps

!!! note

    Demo requires Xbox joy controller.


1. Run Docker container:
   
   ```bash
   cd ~/autoware
   ./run.sh
   ```

2. Open a few (4-6) new terminal windows/tabs and enter the container with the following command in each of them:
   
   ```bash
   cd ~/autoware
   ./enter.sh
   cd autoware
   source install/setup.bash
   ```

3. Launch the simulator.
   
   ```bash
   ~/autoware_awsim/AWSIM_v1.2.0_F1TENTH/AWSIM_v1.2.0.x86_64
   ```

4. Launch Autoware.

   ```bash
   ros2 launch f1tenth_launch e2e_simulator.launch.py
   ```

5. Press the `B` button on the pad, then the `X` to activate remote control. Next, change gear using the directional pad (`D-pad`) to the left. To activate autonomous driving mode, press the `B` button after changing the gear.
