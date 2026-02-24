# agi_viz

Utility ROS 2 package to launch RViz profiles used in `agipix_autonomy`.

## Launch all RViz profiles

```bash
ros2 launch agi_viz rviz.launch.py
```

## Launch only selected profiles

```bash
ros2 launch agi_viz rviz.launch.py \
  launch_all:=false \
  launch_global_planner_navigation:=true \
  launch_trajectory_planner_bspline_interactive:=true
```

## Migration note (2026-02)

RViz profiles remain compatible with the current DDS migration updates.
