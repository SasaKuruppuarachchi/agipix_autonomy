# autonomous_flight (ROS 2)

Mission-level autonomy package for AgiAUTO.

## Status

- ROS 1 → ROS 2 port completed
- **Build tested** in this repository
- Runtime validation is still in progress

## Modes

- Takeoff + hover
- Takeoff + circle tracking
- Static navigation
- Dynamic navigation
- Dynamic inspection
- Dynamic exploration
- (Octomap) inspection mode

## ROS 2 usage

By default, launch files use DDS controller backend (`controller_backend:=dds`).

```bash
# takeoff and hover
ros2 launch autonomous_flight takeoff_and_hover.launch.py

# takeoff and circle
ros2 launch autonomous_flight takeoff_and_track_circle.launch.py

# static navigation
ros2 launch autonomous_flight navigation.launch.py

# dynamic navigation
ros2 launch autonomous_flight dynamic_navigation.launch.py

# dynamic inspection
ros2 launch autonomous_flight dynamic_inspection.launch.py

# dynamic exploration
ros2 launch autonomous_flight dynamic_exploration.launch.py

# octomap inspection
ros2 launch autonomous_flight inspection.launch.py
```

## Build

- DDS build:

```bash
colcon build --packages-select autonomous_flight
```

## Parameters

Configuration files are under `cfg/` by mode, for example:

- `cfg/navigation/*`
- `cfg/dynamic_navigation/*`
- `cfg/dynamic_inspection/*`
- `cfg/dynamic_exploration/*`

## Notes

This package depends on:

- `tracking_controller`
- `map_manager`
- `onboard_detector`
- `global_planner`
- `trajectory_planner`
- `time_optimizer`

Runtime notes:

- DDS-first mission ownership is handled via `px4_control_interface` + `tracking_controller` DDS backend.

## Credits

Inspired by the great academic work of Zhefan Xu and collaborators (CERLAB/CMU).

This Agipix ROS 2 version is heavily modified and integrated for AgiAUTO.

- Repo: https://github.com/SasaKuruppuarachchi/agipix_px4_autonomy.git
- Docs: https://sasakuruppuarachchi.github.io/agipix/




