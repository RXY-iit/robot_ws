ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  pointcloud.enable:=true \
  align_depth.enable:=true
