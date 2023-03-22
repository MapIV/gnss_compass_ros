# gnss_compass_ros

ROS1 Package to calculate position and orientation using two gnss receivers .

Note.)
RTK positioning is required for each GNSS receiver.  
In the standard configuration, pose is only published when two antennas are in RTK-FIX.

```
ros2 launch gnss_compass gnss_compass.launch.xml
```

## build

```
cd ~/catkin_ws/src
git clone --recursive https://github.com/MapIV/gnss_compass_ros.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release
```

## pubsub

- input  
/main/mosaic/gga  (nmea_msgs/Gpgga)  (if input_gnss_type == 0)  
/sub/mosaic/gga  (nmea_msgs/Gpgga)  (if input_gnss_type == 0)  
/main/mosaic/fix  (sensor_msgs/NavSatFix)  (if input_gnss_type == 1)  
/sub/mosaic/fix  (sensor_msgs/NavSatFix)  (if input_gnss_type == 1)  
/tf_static(from "base_link" to main gnss's frame)  

- output  
/gnss_compass_pose (geometry_msgs/PoseStamped)  
/gnss_compass_odom  (nav_msgs/Odometry)  
/illigal_gnss_compass_odom  (nav_msgs/Odometry)  
/diagnostics  ([diagnostic_msgs/DiagnosticArray)  
/tf(from "map" to "base_link")  

## Notes on use

Be sure to set gnss_frequency, yaw_bias, and baseline_length in gnss_compass.yaml when using it.  