# px4_control_interface

DDS control bridge package for AgiAUTO.

## Current status

Phase-1 bootstrap implemented:
- External PX4 mode executor + mode node based on `px4_ros2_interface_lib`
- Subscribes mission/controller target stream (`/autonomous_flight/target_state`)
- Converts ENU setpoints to NED and publishes PX4 trajectory setpoints
- Safe hold fallback when target stream times out
- Optional auto-takeoff command on executor activation (`auto_takeoff:=true`)
- Optional auto-arm before takeoff (`auto_arm:=true` to arm automatically, `auto_arm:=false` to wait for manual arm before takeoff)

## Run

```bash
ros2 launch px4_control_interface dds_shadow.launch.py
```

Run with a specific legacy mission stack in parallel (shadow mode):

```bash
ros2 launch px4_control_interface dds_shadow.launch.py mission:=dynamic_navigation
```

Use the controller DDS sink output as input to PX4 mode adapter:

```bash
ros2 launch px4_control_interface dds_shadow.launch.py \
	target_topic:=/px4_control_interface/controller_target_state
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

Run full DDS shadow parity path (include `tracking_controller` in DDS mode and feed its DDS sink into PX4 mode adapter):

```bash
ros2 launch px4_control_interface dds_shadow.launch.py \
	start_tracking_controller:=false \
	tracking_backend:=dds \
	tracking_dds_target_topic:=/px4_control_interface/controller_target_state \
	target_topic:=/px4_control_interface/controller_target_state \
	auto_takeoff:=true \
	auto_arm:=true
```

> Activation remains under PX4/QGC control. After activation, executor can issue takeoff and schedule tracking mode automatically.
