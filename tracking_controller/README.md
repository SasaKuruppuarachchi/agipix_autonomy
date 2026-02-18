# tracking_controller (ROS 2)

Low-level tracking and command output package for AgiAUTO.

## Status

- ROS 1 → ROS 2 port completed
- Build and runtime tested in integrated stack

## ROS 2 usage

```bash
ros2 launch tracking_controller tracking_controller.launch.py
```

## Parameters

Primary parameter file:

- `cfg/controller_param.yaml`

Backend selection:

- `controller.backend`: `dds` (default and only supported backend)
- `controller.dds_target_topic`: default `/px4_control_interface/controller_target_state`
- `controller.odom_topic`: default `/drone0/sensor_measurements/odom`
- `controller.imu_topic`: default `/drone0/sensor_measurements/imu`

Notes:

- Publishes full `tracking_controller/msg/Target` (position/velocity/acceleration/yaw + `type_mask`) to the DDS bridge topic.

## Key interfaces

Subscribes:

- `/drone0/sensor_measurements/odom`
- `/drone0/sensor_measurements/imu`
- `/autonomous_flight/target_state`

Publishes:

- `/px4_control_interface/controller_target_state`
- `/tracking_controller/robot_pose`
- `/tracking_controller/trajectory_history`
- `/tracking_controller/target_pose`
- `/tracking_controller/target_trajectory_history`
- `/tracking_controller/vel_and_acc_info`

## Credits

Inspired by the great academic work of Zhefan Xu and collaborators (CERLAB/CMU).

This Agipix ROS 2 version is heavily modified and integrated for AgiAUTO.

- Repo: https://github.com/SasaKuruppuarachchi/agipix_px4_autonomy.git
- Docs: https://sasakuruppuarachchi.github.io/agipix/
The example results of running the above command can be visulized as below:




