# global_planner (ROS 2)

Global planning library for AgiAUTO.

## Status

- ROS 1 → ROS 2 port completed
- Build and runtime tested in integrated stack

## Features

- RRT planning
- RRT* planning
- DEP exploration planning

## ROS 2 usage

```bash
# RRT interactive
ros2 launch global_planner rrt_interactive.launch.py

# RRT* interactive
ros2 launch global_planner rrt_star_interactive.launch.py

# DEP test/demo
ros2 launch global_planner test_dep.launch.py
```

## Parameters

Key parameter files:

- `cfg/rrt_planner_ros2.yaml`
- `cfg/dep_param_ros2.yaml`

## Dependencies

- `map_manager`
- `octomap` / `octomap_ros` / `octomap_msgs`

## Credits and references

Inspired by the great academic work of Zhefan Xu, Christopher Suzuki, and collaborators (CERLAB/CMU).

This Agipix ROS 2 version is heavily modified and integrated for AgiAUTO.

- Repo: https://github.com/SasaKuruppuarachchi/agipix_px4_autonomy.git
- Docs: https://sasakuruppuarachchi.github.io/agipix/
