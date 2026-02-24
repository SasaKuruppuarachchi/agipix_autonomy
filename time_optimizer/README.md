# time_optimizer (ROS 2)

Trajectory time allocation/optimization library for AgiAUTO.

## Status

- ROS 1 → ROS 2 port completed
- Build and runtime tested in integrated stack

## Scope

- Time-optimal reparameterization support for generated trajectories
- Used by higher-level planning/execution modules as a linked library
- No standalone runtime launch flow is required for standard stack operation

## Dependencies

- `trajectory_planner`
- `global_planner`
- `map_manager`

## Notes

Architecture-dependent third-party solver libraries are linked during build (see `CMakeLists.txt`).

## Migration note (2026-02)

No interface-breaking changes were introduced in this package during the recent dynamic mission DDS migration work.

## Credits

Inspired by the great academic work of Zhefan Xu and collaborators (CERLAB/CMU).

This Agipix ROS 2 version is heavily modified and integrated for AgiAUTO.

- Repo: https://github.com/SasaKuruppuarachchi/agipix_px4_autonomy.git
- Docs: https://sasakuruppuarachchi.github.io/agipix/



