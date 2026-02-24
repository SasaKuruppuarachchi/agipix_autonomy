# px4_control_interface

Integrated PX4 mode + middle-level controller package for AgiAUTO.

## Current status

Integrated control path implemented:
- External PX4 mode executor + mode node based on `px4_ros2_interface_lib`
- Subscribes mission target stream (`/autonomous_flight/target_state`, type: `autonomous_flight/msg/Target`)
- Runs middle-level controller in-process (`middle_level_controller:=pass_through|cascaded_pid`)
- Converts ENU mission references to NED and publishes PX4 trajectory setpoints
- Safe hold fallback when target stream times out
- Executor workflow follows `mode_with_executor` example (takeoff → tracking mode → RTL → disarm wait)

## Run

```bash
ros2 launch px4_control_interface dds_shadow.launch.py
```

Run with a specific legacy mission stack in parallel (shadow mode):

```bash
ros2 launch px4_control_interface dds_shadow.launch.py mission:=dynamic_navigation
```

Select middle-level controller implementation:

```bash
ros2 launch px4_control_interface dds_shadow.launch.py \
	middle_level_controller:=cascaded_pid
```

Available `mission` values:

- `takeoff_and_hover`
- `takeoff_and_track_circle`
- `navigation`
- `dynamic_navigation`
- `dynamic_inspection`
- `dynamic_exploration`
- `inspection`

Executor-native missions (`takeoff_and_hover`, `takeoff_and_track_circle`, `navigation`, `dynamic_navigation`, `dynamic_inspection`, `dynamic_exploration`) are launched without parallel `px4_tracking_mode_node` to avoid controller contention.

Run only the PX4 external mode node (no legacy stack include):

```bash
ros2 launch px4_control_interface dds_shadow.launch.py start_legacy_stack:=false
```

Run only the PX4 external mode node (no legacy mission include):

```bash
ros2 launch px4_control_interface dds_shadow.launch.py \
	start_legacy_stack:=false \
	target_topic:=/autonomous_flight/target_state
```

> Legacy standalone controller runtime path is deprecated in this package flow.
