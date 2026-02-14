# px4_control_interface

DDS control bridge package for AgiAUTO.

## Current status

Phase-1 bootstrap implemented:
- External PX4 mode node based on `px4_ros2_interface_lib`
- Subscribes mission/controller target stream (`/autonomous_flight/target_state`)
- Converts ENU setpoints to NED and publishes PX4 trajectory setpoints
- Safe hold fallback when target stream times out

## Run

```bash
ros2 launch px4_control_interface dds_shadow.launch.py
```

Run with a specific legacy mission stack in parallel (shadow mode):

```bash
ros2 launch px4_control_interface dds_shadow.launch.py mission:=dynamic_navigation
```

Available `mission` values:

- `takeoff_and_hover`
- `takeoff_and_track_circle`
- `navigation`
- `dynamic_navigation`
- `dynamic_inspection`
- `dynamic_exploration`
- `inspection`

Run only the PX4 external mode node (no legacy stack include):

```bash
ros2 launch px4_control_interface dds_shadow.launch.py start_legacy_stack:=false
```

> This launch does not force activation; mode activation remains under PX4/QGC control.
