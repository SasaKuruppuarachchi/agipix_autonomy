# trajectory_planner (ROS 2)

Trajectory generation and smoothing library for AgiAUTO.

## Status

- ROS 1 → ROS 2 port completed
- Build and runtime tested in integrated stack

## Features

- Polynomial trajectory generation
- Piecewise-linear trajectory generation
- B-spline trajectory optimization
- Interactive demos for RRT/RRT* + trajectory generation

## ROS 2 usage

```bash
# B-spline interactive
ros2 launch trajectory_planner bspline_interactive.launch.py

# RRT + polynomial trajectory
ros2 launch trajectory_planner poly_RRT_interactive.launch.py

# RRT* + polynomial trajectory
ros2 launch trajectory_planner poly_RRTStar_interactive.launch.py

# Goal-driven variants
ros2 launch trajectory_planner poly_RRT_goal_interactive.launch.py
ros2 launch trajectory_planner poly_RRTStar_goal_interactive.launch.py
```

## Parameters

Planner parameters are under `cfg/`.

## Dependencies

- `global_planner`
- `map_manager`
- `octomap` / `octomap_msgs`

## Credits

Inspired by the great academic work of Zhefan Xu and collaborators (CERLAB/CMU).

This Agipix ROS 2 version is heavily modified and integrated for AgiAUTO.

- Repo: https://github.com/SasaKuruppuarachchi/agipix_px4_autonomy.git
- Docs: https://sasakuruppuarachchi.github.io/agipix/
