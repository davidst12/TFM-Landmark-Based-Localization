import glob
import os
import sys

# Description
# - Reduce al mapa a carretera + acera + semaforos
# - Spawnea landmarks en el mapa
# How to use
# - Ejecuta CARLA: ./CarlaUE4.sh
# - Ejecuta este script: python3 carla_reduce_map.py

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import json

def main():

    f = open('../config/landmark_poses_town02.json')
    data = json.load(f)
    landmarks_array = []
    for i in data['landmarks']:
        landmarks_array.append(i)
    f.close

    # Connect to carla server
    client = carla.Client('localhost', 2000)
    client.set_timeout(12.0)
    print("Cliente conectado, cliente:"+str(client))
    print(client.get_available_maps())

    # Get the world
    world = client.get_world()
    print("Hola")

    #Toggle all buildings off
    world.unload_map_layer(carla.MapLayer.Buildings)
    world.unload_map_layer(carla.MapLayer.Decals)
    world.unload_map_layer(carla.MapLayer.Foliage)
    world.unload_map_layer(carla.MapLayer.ParkedVehicles)
    world.unload_map_layer(carla.MapLayer.Walls)
    world.unload_map_layer(carla.MapLayer.Ground)
    world.unload_map_layer(carla.MapLayer.Props)
    world.unload_map_layer(carla.MapLayer.StreetLights)

    # Eliminar layers 2.0
    # env_objs = world.get_environment_objects(carla.CityObjectLabel.Buildings)

    # # Access individual building IDs and save in a set
    # building_01 = env_objs[0]
    # building_02 = env_objs[1]
    # building_03 = env_objs[2]
    # building_04 = env_objs[3]
    # building_05 = env_objs[4]
    # building_06 = env_objs[5]
    # building_07 = env_objs[6]
    # objects_to_toggle = {building_01.id, building_02.id, building_03.id, building_04.id, building_05.id, building_06.id, building_07.id}

    # # Toggle buildings off
    # world.enable_environment_objects(objects_to_toggle, False)

    # object = world.get_blueprint_library().find("static.prop.bin")
    # world.try_spawn_actor(object, transform)

    # Transform ROS coordinates to CARLA coordinates
    for index in range(len(landmarks_array)):
        y_coor = landmarks_array[index][1]
        landmarks_array[index][1] = -y_coor

    # Spawn landmarks
    object = world.get_blueprint_library().find("static.prop.bin")
    for index in range(len(landmarks_array)):
        world.try_spawn_actor(object, carla.Transform(carla.Location(x=landmarks_array[index][0],z=0.0,y=landmarks_array[index][1])))

    # Move spectator object
    spectator = world.get_spectator()
    transform = carla.Transform(
        location = carla.Location(x=104, z=235.0, y = 205), 
        rotation = carla.Rotation(pitch = -90, yaw = -90, roll = 0)
    )
    spectator.set_transform(transform)

if __name__ == '__main__':

    main()