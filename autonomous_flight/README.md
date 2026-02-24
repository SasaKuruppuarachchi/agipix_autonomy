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

Mission launch files now publish mission references only (no standalone controller node launch).

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

- `map_manager`
- `onboard_detector`
- `global_planner`
- `trajectory_planner`
- `time_optimizer`

Message interfaces:

- `autonomous_flight/msg/Target` is now the mission reference contract consumed by `px4_control_interface`.

Runtime notes:

- Active runtime path is integrated: `autonomous_flight` mission references -> `px4_control_interface` in-process middle-level controller -> PX4 setpoints.
- Standalone `tracking_controller` runtime launch is deprecated for normal mission execution.

### Dynamic exploration controls

Key runtime parameters in [cfg/dynamic_exploration/flight_base.yaml](cfg/dynamic_exploration/flight_base.yaml):

- `replan_on_finish_or_fail`
- `replan_on_collision_fail`
- `stabilize_before_rotate`
- `min_waypoint_distance`

Dynamic exploration start is service-gated:

```bash
ros2 launch px4_control_interface dds_shadow.launch.py use_sim_time:=true start_legacy_stack:=true mission:=dynamic_exploration
ros2 service call /dynamic_exploration/start std_srvs/srv/Trigger "{}"
```

## Credits

Inspired by the great academic work of Zhefan Xu and collaborators (CERLAB/CMU).

This Agipix ROS 2 version is heavily modified and integrated for AgiAUTO.

- Repo: https://github.com/SasaKuruppuarachchi/agipix_px4_autonomy.git
- Docs: https://sasakuruppuarachchi.github.io/agipix/




