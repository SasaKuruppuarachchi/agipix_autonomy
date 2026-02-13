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

## Key interfaces

Subscribes:

- `/mavros/local_position/odom`
- `/mavros/imu/data`
- `/autonomous_flight/target_state`

Publishes:

- `/mavros/setpoint_raw/attitude`
- `/mavros/setpoint_raw/local`
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




