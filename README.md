# AgiAUTO (ROS 2) — Agipix PX4 Autonomy Stack

This repository is the **AgiAUTO module** of **Agipix** and contains a ROS 2 autonomy stack for PX4-based UAV operation in simulation and real-world workflows.

<p align="center"><img src=".media/images/banner.png" alt="AgiAUTO banner"></p>

- Repository: https://github.com/SasaKuruppuarachchi/agipix_px4_autonomy.git
- Platform docs and build instructions: https://sasakuruppuarachchi.github.io/agipix/
- Publication submission (ICUAS 2026): *"Agipix: A Comprehensive Aerial Robotics Platform Bridging Simulation and Reality"*

## Porting and validation status

All packages in this repository are ported from ROS 1 to ROS 2.

- **Build + runtime tested**: `global_planner`, `map_manager`, `onboard_detector`, `time_optimizer`, `tracking_controller`, `trajectory_planner`
- **Build tested (runtime pending full validation)**: `autonomous_flight`

## High-level stack architecture (lossless view)

```text
Sensors (depth / point cloud / RGB / lidar) + MAVROS/PX4 state
		    │
		    ▼
	 onboard_detector (dynamic obstacle detection + tracking)
		    │
		    ▼
	 map_manager (occupancy/dynamic map + collision/raycast)
		    │
	┌─────────┴──────────────────────────────────────┐
	▼                                                ▼
global_planner (RRT/RRT*/DEP)                 trajectory_planner
(global waypoints / exploration path)          (poly/PWL/B-spline trajectories)
	└───────────────┬────────────────────────────────┘
			    ▼
		   time_optimizer (optional)
			    ▼
	autonomous_flight (mission executive/state machine)
			    ▼
     tracking_controller (trajectory tracking + PX4 setpoints)
			    ▼
			 MAVROS → PX4
```

## Package roles

- `autonomous_flight`: mission orchestration (`takeoff`, `navigation`, `dynamic_navigation`, `inspection`, `dynamic_inspection`, `dynamic_exploration`)
- `tracking_controller`: low-level tracking and command publication to PX4 via MAVROS
- `map_manager`: occupancy and dynamic map representations, map services
- `onboard_detector`: dynamic obstacle perception, data association, tracking
- `global_planner`: global path/exploration planners (RRT/RRT*, DEP)
- `trajectory_planner`: smooth local trajectory generation (poly/PWL/B-spline)
- `time_optimizer`: trajectory time allocation optimization utilities

## PX4 ROS 2 interface setup

The stack is validated against **PX4 v1.16.1** messaging.

```bash
mkdir -p ~/workspace/agipix_control/src
cd ~/workspace/agipix_control/src

# PX4 messages + autopilot source (for message sync)
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/PX4-Autopilot.git -b v1.16.1

# Sync message/service definitions
rm -f px4_msgs/msg/*.msg px4_msgs/srv/*.srv
cp PX4-Autopilot/msg/*.msg px4_msgs/msg/
cp PX4-Autopilot/msg/versioned/*.msg px4_msgs/msg/
cp PX4-Autopilot/srv/*.srv px4_msgs/srv/
touch px4_version_synced_v1_16_1
rm -rf PX4-Autopilot

# PX4 ROS 2 interface library
git clone https://github.com/Auterion/px4-ros2-interface-lib -b release/1.16
```

Compatibility check:

```bash
cd ~/workspace/agipix_control/src/px4-ros2-interface-lib
./scripts/check-message-compatibility.py -v path/to/px4_msgs/ path/to/PX4-Autopilot/
```

Build:

```bash
cd /workspaces/agipix_control
colcon build --packages-select px4_msgs
colcon build --packages-select px4_ros2_cpp
```

## Credits

This stack builds on original CERLAB autonomy components and research implementations by **Zhefan Xu** and collaborators.

This repository version is a **heavily modified ROS 2 integration** within Agipix by **Sasa Kuruppuarachchi**.

Please credit both:

1. Original algorithm/software authors (see package-level citations)
2. Agipix ROS 2 integration and platform engineering work in this repository