import carla

def main():
    # Connect to carla server
    client = carla.Client('localhost', 2000)
    client.set_timeout(12.0)
    print("Cliente conectado, cliente:"+str(client))
    synchronous_master = False

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