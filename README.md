# tritonkart
ROS2 Repository for Triton Kart

Editing URDF
- For most accurate results edit the kart.urdf to reflect your Lidar, IMU, Odometry and GPS positions with respect to the base. This is not the most necessary though

Creating a map:
- Before mapping first note the current GPS coordinates and IMU heading. This goes into the tritonkart_localization yaml file as datum later on
- ```ros2 launch tritonkart_bringup tritonkart_mapping.launch.py use_sim_time:= false use_scan:= false use_cam:= false odom0_topic:=<your_odom_topic> imu0_topic:=<your_imu_topic> scan_topic:=<your_lidar_topic>```. Or edit the file with appropriate default topics
- Start moving around. Once you are satisfied with the map run ```ros2 run nav2_map_server map_saver_cli -f map```
- Move the map contents (by default it is kept where you run the above command in the terminal) into an appropriate folder (Preferred location is tritonkart_mapping/maps
- Remember the GPS coordinates and imu heading recorded earlier? Go to tritonkart_localization/config/gps_localization.yaml and add it to datum

Logging:
- ``` ros2 launch tritonkart_bringup tritonkart_logging.launch.py use_sim_time:= false freq:=<freq in hz> imu_topic:=<your_imu_topic> gps_topic:=<your_gps_topic>```
- This saves a path with GPS coordinates in tritonkart_logging/path . You can open it in Python and visualize using ``` plt.plot(df['latitude'], df['longitude'], '--bo') ```. Filter out some points and clean if you want

Navigation
- Edit the tritonkart_navigation/config/nav2_navigation.yaml and add your topics there
- ``` ros2 launch tritonkart_bringup tritonkart_navigation.launch.py use_sim_time:= false use_scan:= false use_cam:= false odom_topic:=<your_odom_topic> imu_topic:=<your_imu_topic> gps_topic:=<your_gps_topic> scan_topic:=<your_lidar_topic>```

