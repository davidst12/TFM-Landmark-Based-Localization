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

def main():

    #columna1
    c1 = (-120, 37, 58)
    c2 = (-95, 6, 37, 47, 58, 78, 100)
    c3 = (-60, -75, -50, -22, 6, 37, 47, 58, 78, 100, 122, 147)
    c4 = (-35, -75, -50, -22, 6, 37, 47, 58, 78, 100, 122, 147)
    c5 = (0, -75, -50, 6, 37, 58, 78, 122, 147)
    c6 = (33, -75, -50, 6, 37, 47, 58, 78, 122, 147)
    c7 = (91, -70, 6, 37, 47, 58, 78, 100, 142)
    c8 = (125, 37, 58)
    #Matrix
    m = (c1, c2, c3, c4, c4, c5, c6, c7, c8)

    # Connect to carla server
    client = carla.Client('localhost', 2000)
    client.set_timeout(12.0)

    # Get the world
    world = client.get_world()

    #Toggle all buildings off
    world.unload_map_layer(carla.MapLayer.Buildings)
    world.unload_map_layer(carla.MapLayer.Decals)
    world.unload_map_layer(carla.MapLayer.Foliage)
    world.unload_map_layer(carla.MapLayer.ParkedVehicles)
    world.unload_map_layer(carla.MapLayer.Walls)
    world.unload_map_layer(carla.MapLayer.Ground)
    world.unload_map_layer(carla.MapLayer.Props)
    world.unload_map_layer(carla.MapLayer.StreetLights)

    # Move spectator object
    # spectator = world.get_spectator()
    # transform = carla.Transform(carla.Location(x=c4x, z=0.0, y = 122.6))
    # spectator.set_transform(transform)

    # object = world.get_blueprint_library().find("static.prop.bin")
    # world.try_spawn_actor(object, transform)

    # Spawn landmarks
    object = world.get_blueprint_library().find("static.prop.bin")
    for matrix_index in range(len(m)):
        for i in range(len(m[matrix_index])-1):
            world.try_spawn_actor(object, carla.Transform(carla.Location(x=m[matrix_index][0],z=0.0,y=m[matrix_index][i+1])))


if __name__ == '__main__':

    main()