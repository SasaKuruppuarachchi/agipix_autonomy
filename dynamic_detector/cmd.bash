ros2 launch ros2_yolos_cpp detector.launch.py     model_path:=/workspaces/agipix_control/src/YOLOs-CPP/models/yolo26n.onnx     labels_path:=/workspaces/agipix_control/src/YOLOs-CPP/models/coco.names     use_gpu:=true     image_topic:=/camera/color/image_raw
rm -rf build/ros2_yolos_cpp/
rm -rf install/ros2_yolos_cpp/
colcon build --packages-select ros2_yolos_cpp --cmake-args -DENABLE_GPU=ON
ros2 launch ros2_yolos_cpp detector.launch.py     model_path:=/workspaces/agipix_control/src/YOLOs-CPP/models/yolo26_fp16.onnx     labels_path:=/workspaces/agipix_control/src/YOLOs-CPP/models/coco.names     use_gpu:=true     image_topic:=/camera/color/image_raw

ros2 lifecycle set /yolos_detector configure
ros2 lifecycle set /yolos_detector activate