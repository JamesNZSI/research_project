import glob
import os
import sys
from pathlib import Path
import random
import vehicle_helper
from vehicle_helper import VehicleHelper
import carla
import time
from datetime import date
from modules import generator_L3D2 as gen

## 10 scan / second
try:
    sys.path.append(glob.glob('%s/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        "C:/CARLA_0.9.10/WindowsNoEditor" if os.name == 'nt' else str(Path.home()) + "/CARLA_0.9.10",
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# vehicle speed is 5 m/s ~ 18 km/h

## generate ego for 2 seconds then generate sensors
def main():
    start_record_full = time.time()
    ## frame per second
    fps_simu = 100.0
    time_stop = 2.0
    ## point clouds before simulation stops
    nbr_frame = 40000 #MAX = 10000
    # simulation period 70 seconds
    simulation_period = 70
    nbr_walkers = 0#100
    nbr_vehicles = 0#100

    # actor_list = []
    vehicles_list = []
    all_walkers_id = []
    # data_date = date.today().strftime("%Y_%m_%d")

    recording_flag = True
    logging_flag = False
    init_settings = None
    ## use town02 map A small simple town with a mixture of residential and commercial buildings
    # index of map from 1~7 : 7 maps from Town01 to Town07
    i_map = 2
    draw_location = True
    start_record = time.time()
    try:
        client = carla.Client('localhost', 2000)
        init_settings = carla.WorldSettings()

        client.set_timeout(100.0)
        print("Map Town0"+str(i_map))
        ## load world map

        helper = VehicleHelper(client, logging_flag)
        world = client.get_world()

        folder_output = "L3D2_Dataset_CARLA_v%s/%s/generated" % (client.get_client_version(), world.get_map().name)
        # recording
        if recording_flag:
            client.start_recorder(os.path.dirname(os.path.realpath(__file__))+"/"+folder_output+"/recording.log")

        # Spawn vehicles and walkers
        gen.spawn_npc(client, nbr_vehicles, nbr_walkers, vehicles_list, all_walkers_id)

        # All sensors produce first data at the same time (this ts)
        gen.Sensor.initial_ts = world.get_snapshot().timestamp.elapsed_seconds
        
        # print("Start record : ")
        # frame_current = 0
        current_time = world.get_snapshot().timestamp.elapsed_seconds
        ## draw location to see routes
        if draw_location:
            helper.draw_spawn_points()

        # while (frame_current < nbr_frame):
        while (current_time < simulation_period):
            # print("frame %d"% frame_current)
            # frame_current += 1
            current_time = world.get_snapshot().timestamp.elapsed_seconds
            helper.tick()
            ## spector follow ego vehicle
            helper.follow()
            world.tick()    # Pass to the next simulator frame

        if recording_flag:
            client.stop_recorder()
            print("Stop record")

        # Stop walker controllers (list is [controller, actor, controller, actor ...])
        all_actors = world.get_actors(all_walkers_id)
        for i in range(0, len(all_walkers_id), 2):
            all_actors[i].stop()
        print('Destroying %d walkers' % (len(all_walkers_id)//2))
        client.apply_batch([carla.command.DestroyActor(x) for x in all_walkers_id])
        all_walkers_id.clear()

        print("Elapsed time : ", time.time()-start_record)
        print()

        time.sleep(2.0)
    finally:
        print("Elapsed total time : ", time.time()-start_record_full)
        if world:
            world.apply_settings(init_settings)
        
        time.sleep(2.0)
        

if __name__ == '__main__':
    main()
