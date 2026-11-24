# waiko_ros2_control

WAIKO state observer (`stateObservation::WaikoHumanoid` with the leg odometry of state-observation),
for ROS 2 Jazzy. It estimates the pose and velocity of the IMU of a legged robot from the IMU and the
kinematics of its contacts, as `MCWaiko` does in mc_state_observation.

The estimator receives the contact kinematics, it does not compute them: for each contact, its pose
and linear velocity in the IMU frame, whether it is set, and its weight.

The package provides:

- `waiko_ros2_control/WaikoController`: a ros2_control controller reading these measurements from state
  interfaces;
- `waiko_estimator_node`: the same estimator fed by topics, with `rosbag_publish_waiko` to replay
  recorded inputs.

Both run `WaikoEstimator` (`include/waiko_ros2_control/waiko_estimator.hpp`), which has no ROS dependency.

## ros2_control controller

State interfaces claimed (no command interface):

| Interface | Content |
| --- | --- |
| `<imu_name>/angular_velocity.{x,y,z}` | gyrometer |
| `<imu_name>/linear_acceleration.{x,y,z}` | accelerometer |
| `<contact>/active` | non-zero when the contact is set |
| `<contact>/lambda` | weight of the contact, e.g. its normal force |
| `<contact>/position.{x,y,z}` | position of the contact in the IMU frame |
| `<contact>/orientation.{x,y,z,w}` | orientation of the contact in the IMU frame |
| `<contact>/linear_velocity.{x,y,z}` | linear velocity of the contact in the IMU frame |

The contact interfaces are exported by the hardware interface, or by an upstream controller
computing the forward kinematics and the contact detection.

Published topics:

- `~/odometry` (`nav_msgs/Odometry`): pose of the IMU in `world_frame_id`, twist in `imu_frame_id`;
- `~/estimated_state` (`waiko_ros2_control/WaikoState`): full state of the estimator.

The estimator starts from `initial_imu_position`, `initial_imu_linear_velocity` and
`initial_imu_orientation` at each activation.
See `config/waiko_controller.yaml` for an example, and `src/waiko_controller_parameters.yaml` and
`src/waiko_estimator_parameters.yaml` for all the parameters.

## Estimator node

- Subscribes to `~/init_state` (`WaikoInitialState`), which (re)initializes the estimator, and to
  `~/input` (`WaikoInput`), one sample per message with the sampling period `dt`.
- Publishes `~/estimated_state` (`WaikoState`) for the initial state and for each input, with the
  stamp of that message.

```bash
ros2 launch waiko_ros2_control waiko.launch.py
# Replay a bag of inputs and record the estimates:
ros2 launch waiko_ros2_control waiko_replay.launch.py input_bag:=<input bag> output_bag:=<new bag>
```

## Position anchor

WAIKO estimates the IMU position `p_local` in a frame whose axes are those of the IMU and whose
origin is a point of the world, the position anchor: `position = position_anchor + R p_local`.
Orientation corrections act on the position through the lever arm `p_local`. With
`reset_position_anchor_on_contact_change`, the anchor is moved to the centroid of the contact
references each time the contacts change, which keeps that lever arm short; the estimated world state
is unchanged by the reset (`WaikoEstimator::reset_position_anchor_to_support`).

## Build

state-observation must be found by CMake, e.g. with its install prefix in `CMAKE_PREFIX_PATH`.

```bash
colcon build --packages-select waiko_ros2_control
colcon test --packages-select waiko_ros2_control
```
