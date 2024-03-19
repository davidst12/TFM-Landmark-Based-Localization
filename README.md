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
![tfm_diagram_nodes_topics_1 drawio](https://github.com/davidst12/TFM-Landmark-Based-Localization/assets/118628096/0604499f-51a3-4719-b0f3-e05dbda50617)


### Sequence diagram

![tfm_diagram_sequence_1 drawio (1)](https://github.com/davidst12/TFM-Landmark-Based-Localization/assets/118628096/8bd07667-1a63-455a-8e22-c98725353c24)
