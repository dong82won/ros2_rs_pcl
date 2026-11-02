ros2 launch realsense2_camera rs_launch.py camera_name:=camera camera_namespace:=camera initial_reset:=false enable_sync:=false align_depth.enable:=false pointcloud.enable:=false enable_depth:=false enable_infra:=false enable_infra1:=false enable_infra2:=false enable_accel:=false enable_gyro:=false rgb_camera.color_profile:=640x480x30 log_level:=error




ros2 launch apriltag_6dof tag_fusion.launch.py use_sim_time:=true
