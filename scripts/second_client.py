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
    print("Comenzando script")
    print("Path to egg: "+str('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64')))

    # Connect to carla server
    client = carla.Client('localhost', 2000)
    client.set_timeout(12.0)
    print("Cliente conectado, cliente:"+str(client))

    # Get the world
    world = client.get_world()
    print("MUndo sonseguido")

    weather = carla.WeatherParameters(
    cloudiness=80.0,
    precipitation=30.0,
    sun_altitude_angle=70.0)

    world.set_weather(weather)

    print(world.get_weather())  


if __name__ == '__main__':

    main()