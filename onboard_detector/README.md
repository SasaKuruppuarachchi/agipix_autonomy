# onboard_detector (ROS 2)

`onboard_detector` provides dynamic obstacle detection, tracking, and obstacle query services for AgiAUTO.

## Role in stack

- Processes depth/lidar/RGB streams
- Produces dynamic obstacle bounding boxes and velocity estimates
- Serves nearby dynamic obstacles to map/planning modules

## ROS 2 launch examples

```bash
# standalone detector launch (if configured)
ros2 launch onboard_detector dynamic_detector.launch.py

# common integrated path via map_manager dynamic map launch
ros2 launch map_manager dynamic_map.launch.py
```

## Key runtime interfaces

- Topics under `/onboard_detector/*`
- Service:
  - `/onboard_detector/get_dynamic_obstacles`

## AgiAUTO context

- Repo: https://github.com/SasaKuruppuarachchi/agipix_px4_autonomy.git
- Docs: https://sasakuruppuarachchi.github.io/agipix/

## Credits

Original core work and algorithms are from CERLAB/CMU packages by Zhefan Xu and collaborators.
This ROS 2 integrated Agipix version is heavily modified for AgiAUTO workflows.
