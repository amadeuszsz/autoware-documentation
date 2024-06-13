# ML Controller

Project has 3 main repositories:

- [ml-controller](https://github.com/Gandhi444/ml_controller)
- [nc-net](https://github.com/SzymKwiatkowski/nc-net)
- [f1tenth_awsim_data_recorder](https://github.com/SzymKwiatkowski/f1tenth_awsim_data_recorder)

## Controller (ml-controller)

The ML Controller module calculates the steering angle for tracking a desired trajectory using the neural network. This is used as a lateral controller plugin in the `trajectory_follower_node`.

### Inputs

Set the following from the [controller_node](https://github.com/autowarefoundation/autoware.universe/blob/main/control/autoware_trajectory_follower_node/README.md)

- `autoware_auto_planning_msgs/Trajectory` : reference trajectory to follow.
- `nav_msgs/Odometry`: current ego pose and velocity information

### Outputs

Return LateralOutput which contains the following to the controller node

- `autoware_auto_control_msgs/AckermannLateralCommand`: target steering angle
- LateralSyncData
  - steer angle convergence
### Parameters
Used via `ml_controller.param.param.yaml` file in config directory.
| Name         | Type | Description  |
| ------------ | ---- | ------------ |
| converged_steer_rad | double  | Threshold for when steering is considered converged |
| resampling_ds | double | Trajectory resampling distance |
| lookahead_distance | double | Waypoint look ahead distance |
| closest_thr_dist | double | Maximum distance for selecting closest waypoint |
| closest_thr_ang | double | Maximum angle for selecting closest waypoint |
| trajectory_input_points | int | Number of trajectory waypoints passed to the network |
| model_path | string | Path to the onnx model inside resources folder|
| precision | string | Quantisation used by the model |
### Model requirments
Model should be stored as .onnx inside resources folder. As inputs it takes current vechicle x and y position and z and w parts of orientation quaterion followed by same values of n trajectory waypoints. Network output should be steering angle in range 0-1 it is converted to min-max vechicle steering angle in postprocessing.


## Training model (nc-net)

Repository [nc-net](https://github.com/SzymKwiatkowski/nc-net) contains:

- training
- exporting model to onnx format

Training module is using a couple of different selected models:

- Basic neural network - with fully connected layers
- RBF network - with RBF model variation for model calculation
- DenseRBF - fully connected backbone extraction with RBF added on top
- skip connection model - model with skip connection

As well as losses:

- MSE Loss
- Huber Loss
- L1 Loss

All of the training can be logged by using two different loggers specified in configuration:

- Tensorflow board (used by default)
- Neptune ai

During training dataset provides data based on specified count of points which are extracted from trajectory. Then during training all of the data is extracted and provided in batches based on car current position from sample taken from dataset.
It also normalizes outputs to `(0, 1)` values based on vehicle maximum steer parameter provided from ros2 node.

To train model recorded data via using f1tenth_awsim_data_recorder is required (there is an option of using some small recorded data in data directory).
Dataset can be adjusted with `datasets.json` file in data directory. Example configuration is present in repository and looks as follows:

```json
[
  {
    "main_df": "main_df.csv",
    "points_df": "points_df.csv",
    "points_count": 271,
    "train_size": 0.75
  }
]
```

This way multiple datasets can be provided to ensure more data during training is delivered from many different trajectories.

Parameters are:

- main_df: string - path to specified dataframe containing recorded data poses of car as well as control inputs

- points_df: string - path to specified dataframe containing recorded trajectory for

- points_count: int - count of points for specified input trajectory provided in `points_df` param

- train_size: float - size for training dataset `(0, 1)` value.



### Graphs of training progress:

![training_train_loss](./images/training_train_loss.png)
![training_val_loss](./images/training_val_loss.png)\
![training_val_MeanAbsoluteError](./images/training_val_MeanAbsoluteError.png)
![training_val_MeanSquaredError](./images/training_val_MeanSquaredError.png)


## Recording data (f1tenth_awsim_data_recorder)

To use package clone it to src directory of ros2 workspace and then build it. To launch use command:

```bash
ros2 launch f1tenth_awsim_data_recorder f1tenth_awsim_data_recorder.launch.py
```

Package has parameters that can be adjusted in parameters file.

### Parameters

Used via `f1tenth_awsim_data_recorder.param.yaml` file in config directory.

| Name               | Type   | Description                                                                                            |
| ------------------ | ------ | ------------------------------------------------------------------------------------------------------ |
| max_points_count   | int    | Maximum count of points saved (remaining if path is shorter then specified amount it is filled with 0) |
| ackermann_topic    | string | Sample desc.                                                                                           |
| ground_truth_topic | string | Sample desc.                                                                                           |
| trajectory_topic   | string | Sample desc.                                                                                           |

Example configuration can look as follows:

```yaml
/**:
  ros__parameters:
    max_points_count: 271 # Max points count being saved from trajectory topic
    ackermann_topic: "/control/command/control_cmd"
    ground_truth_topic: "/localization/cartographer/pose"
    trajectory_topic:  "/planning/racing_planner/trajectory"
```

## Results

![path](./images/path.png)

[![video](https://img.youtube.com/vi/nCAl3mee5yo/0.jpg)](https://www.youtube.com/watch?v=nCAl3mee5yo)
