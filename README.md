# TFM-Landmark-Based-Localization

### How to launch our test

##### Run CARLA environment
```console
~$ cd /opt/carla-simulator/ && ./CarlaUE4.sh
~$ ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py
~$ cd ~/tfm_landmark_based_localization_package/scripts/ && python3 carla_reduce_map.py
```


##### Run our localization method
```console
~$ ros2 launch tfm_landmark_based_localization_package landmark_localitation.launch.py
```

### Ros diagram
![secuencia_2](https://github.com/davidst12/TFM-Landmark-Based-Localization/assets/118628096/f4298b41-e659-495d-8633-eefde5de4c7b)

### Sequence diagram
![node_graph](https://github.com/davidst12/TFM-Landmark-Based-Localization/assets/118628096/1af52464-bd71-4728-8c0b-1f3d1f627b57)

### Dependencies

Add this ROS 2 packages to same workspace as this package

https://github.com/Butakus/detection_msgs

https://github.com/carla-simulator/ros-bridge

Add this ROS 2 packages to ROS2 worksapce

https://wiki.ros.org/g2o
