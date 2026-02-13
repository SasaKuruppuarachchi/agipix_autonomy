# map_manager (ROS 2)

`map_manager` provides occupancy/dynamic map services for AgiAUTO.

## Role in stack

- Fuses depth/point-cloud sensor data with UAV pose/odom
- Maintains occupancy and inflated occupancy maps
- Exposes map services (collision check, raycast, static obstacles)
- Provides dynamic-map integration with onboard dynamic obstacle detection

## ROS 2 launch examples

```bash
# occupancy map node
ros2 launch map_manager occupancy_map.launch.py

# dynamic map + detector
ros2 launch map_manager dynamic_map.launch.py

# ESDF map node
ros2 launch map_manager esdf_map.launch.py
```

## Key runtime interfaces

- Topics under `/occupancy_map/*` and `/dynamic_map/*`
- Services:
  - `/occupancy_map/check_pos_collision`
  - `/occupancy_map/raycast`
  - `/occupancy_map/get_static_obstacles`

## AgiAUTO context

- Repo: https://github.com/SasaKuruppuarachchi/agipix_px4_autonomy.git
- Docs: https://sasakuruppuarachchi.github.io/agipix/

## Credits

Original core work and algorithms are from CERLAB/CMU packages by Zhefan Xu and collaborators.
This ROS 2 integrated Agipix version is heavily modified for AgiAUTO workflows.
