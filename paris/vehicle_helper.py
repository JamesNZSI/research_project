import random

import carla
import os
import glob
from modules import ply
from modules import generator_L3D2 as gen


## use town02 map A small simple town with a mixture of residential and commercial buildings
# index of map from 1~7 : 7 maps from Town01 to Town07
i_map = 2
# spawn points in town02
vehicle_spawn_points = [47, 50]
# spawn point routes: each spawn point has a list of routes
routes_sp = [
                [
                    # [47, 53, 44, 90, 49, 51],
                    #[47, 53, 82, 80]
                    [53, 44],
                 ],
                [
                    # [52, 91, 50, 46, 54, 48],
                    #[52, 91, 50, 46, 82, 80]
                    [50, 46, 54],
                ]
             ]
#(spawn_points, exist_time)
# noise_sp = [(82, 15*60), (83, 45*60)]
# noise_sp = [(82, 15), (83, 45)]
# Set sensors transformation from lidar
lidar_transform = carla.Transform(carla.Location(x=-2.30, y=0, z=2.30), carla.Rotation(pitch=-45, yaw=180, roll=0))
#
# folder_output = "L3D2_Dataset_CARLA_v%s/%s/generated" % (client.get_client_version(), world.get_map().name)
# need to stop 2 seconds for generating vehicle
time_stop = 2.0
# frame per second
fps_simu = 100.0
# time offset 3 before actions
time_offset = 3.0

# need world.tick() after invocation
def set_path(client, vehicle, path):
    traffic_manager = client.get_trafficmanager()
    traffic_manager.set_path(vehicle, path)
    vehicle.set_autopilot(True)


class VehicleHelper:
    def __init__(self, client, logging):
        self.client = client
        self.world = client.load_world("Town0"+str(i_map))
        self.logging = logging
        self.folder_output = "L3D2_Dataset_CARLA_v%s/%s/generated" % (client.get_client_version(), self.world.get_map().name)
        os.makedirs(self.folder_output) if not os.path.exists(self.folder_output) else [os.remove(f) for f in glob.glob(self.folder_output+"/*") if os.path.isfile(f)]
        # vehicle list
        self.v_list = []
        self.vehicle_no = 0
        self.waiting_vehicles = []
        self.waiting_noises = []
        # [(vehicle, created_time)]
        self.created_vehicles = []
        # [(vehicle, sensor, created_time)]
        self.vehicles_with_sensor = []
        self.waiting_destroy = []
        self.s_list = []
        # actor list
        self.a_list = []
        self.frame_count = 0
        # {sensor:(create_time, alive_period)}
        self.world_setting()
        # sensor alive time 8 seconds
        self.alive_period = 9
        self.speed_perc = 0
        self.traffic_light_list = []
        # (vehicle, create_time, exist_time)
        self.noise_list = []
        self.noise_sp = []

        self.init()

    def init(self):
        self.set_traffic_lights_green()
        self.determine_noise_position()
        self.add_vehicles()
        # order by create time
        self.waiting_vehicles.sort(key=lambda x:x[1])
        self.add_noise()
        # order by create time
        self.waiting_noises.sort(key=lambda x:x[1])
        if self.logging:
            print("noise list:", self.waiting_noises)

    def determine_noise_position(self):
        # determine noises' positions
        noise_1 = self.world.get_map().get_spawn_points()[82]
        noise_2 = self.world.get_map().get_spawn_points()[83]
        n2_loc = noise_2.location
        n2_rot = noise_2.rotation
        noise_2 = carla.Transform(carla.Location(x=n2_loc.x, y=noise_1.location.y, z=n2_loc.z), 
                                  carla.Rotation(yaw=n2_rot.yaw))
        # (82, 15), (83, 45)
        self.noise_sp = [noise_1, noise_2]

    def set_traffic_lights_green(self):
        actors = self.world.get_actors()
        for actr in actors:
            if actr.type_id == "traffic.traffic_light":
                actr.set_state(carla.TrafficLightState.Green)
                actr.freeze(True)

    def world_setting(self):
        # Weather
        self.world.set_weather(carla.WeatherParameters.WetCloudyNoon)

        # Set Synchronous mode
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1.0/fps_simu
        settings.no_rendering_mode = False
        self.world.apply_settings(settings)

    def create_noise(self, sp_idx, end_time):
        current_time = self.world.get_snapshot().timestamp.elapsed_seconds
        start_pose = self.noise_sp[sp_idx]

        v_name = "N" + str(len(self.noise_list))

        bp_v = self.world.get_blueprint_library().find('vehicle.nissan.patrol')
        bp_v.set_attribute('color', '130, 159, 175')
        bp_v.set_attribute('role_name', v_name)
        # spawn position
        if self.logging:
            print("noise position:%s"%start_pose)
        # use try to spawn
        vehicle = self.world.try_spawn_actor(bp_v, start_pose)
        if self.logging:
            if vehicle is not None:
                print('Created noise %s current time %s' % (v_name, current_time))
            else:
                print('Created noise %s failed at %s' % (v_name, start_pose))
        if vehicle is not None:
            self.noise_list.append((vehicle, current_time, end_time))
        return vehicle

    # vehicles:[(vehicle)]
    def destroy_noises(self, vehicles):
        d_list = []
        for vehicle, _, _ in vehicles:
            if self.logging:
                print("destroy noise %s" % vehicle.attributes["role_name"])
            d_list.append(vehicle)
        self.client.apply_batch([carla.command.DestroyActor(x) for x in d_list])

    def create_vehicle(self, sp_idx):
        # sp_idx = random.randint(0, len(vehicle_spawn_points) - 1)
        v_name = "V" + str(self.vehicle_no)
        spawn_position_idx = vehicle_spawn_points[sp_idx]

        bp_v = self.world.get_blueprint_library().find('vehicle.nissan.patrol')
        bp_v.set_attribute('color', '130, 159, 175')
        bp_v.set_attribute('role_name', v_name)
        # spawn position
        start_pose = self.world.get_map().get_spawn_points()[spawn_position_idx]
        # use try to spawn
        vehicle = self.world.try_spawn_actor(bp_v, start_pose)
        if self.logging:
            if vehicle is not None:
                print('Created %s' % v_name)
            else:
                print('Created %s failed at %s' % (v_name, start_pose))
        if vehicle is not None:
            self.set_vehicle_path(vehicle, random.choice(routes_sp[sp_idx]))
            vehicle.set_autopilot(True)
            self.vehicle_no += 1
            traffic_manager = self.client.get_trafficmanager()
            traffic_manager.vehicle_percentage_speed_difference(vehicle, self.speed_perc)

        return vehicle

    # vehicles:[(vehicle, sensor, actor, created_time)]
    def destroy_vehicles(self, vehicles):
        d_list = []
        for vehicle, _, actor, _ in vehicles:
            if self.logging:
                print("destroy %s with lidar" % vehicle.attributes["role_name"])
            d_list.append(vehicle)
            d_list.append(actor)
        self.client.apply_batch([carla.command.DestroyActor(x) for x in d_list])

    def get_route(self, sp_array):
        wsp = self.world.get_map().get_spawn_points()
        route = []
        for ind in sp_array:
            route.append(wsp[ind].location)
        return route

    def draw_spawn_points(self):
        ## draw location to see routes
        for i, spawn_point in enumerate(self.world.get_map().get_spawn_points()):
            self.world.debug.draw_string(spawn_point.location, str(i), life_time=1000)

    def set_vehicle_path(self, vehicle, route_sp):
        route = self.get_route(route_sp)
        traffic_manager = self.client.get_trafficmanager()
        traffic_manager.set_path(vehicle, route)
        vehicle.set_autopilot(True)

    def create_sensor(self, vehicle):
        v_name = vehicle.attributes["role_name"]
        actor_list = []
        init_ts = self.world.get_snapshot().timestamp.elapsed_seconds
        lidar = gen.HDL32E(vehicle, self.world, actor_list, self.folder_output + "/" + v_name, lidar_transform, init_ts)
        if self.logging:
            print("create lidar for %s length is %d" % (v_name, len(actor_list)))
        actor = actor_list[0]
        lidar.init()
        return lidar, actor

    def follow(self):
        if len(self.vehicles_with_sensor) > 0:
            # print("vehicle speed ", self.vehicles_with_sensor[0][0].get_velocity())
            gen.follow(self.vehicles_with_sensor[0][0].get_transform(), self.world)

    # destroy/collect data first, then create sensor with vehicle, then create vehicle
    def tick(self):
        self.frame_count += 1
        current_time = self.world.get_snapshot().timestamp.elapsed_seconds
        current_time -= time_offset
        # if current_time > 50:
        #     print("tick time:", current_time)
        #create noises
        created_noises = []
        for idx, (sp_idx, create_time, end_time) in enumerate(self.waiting_noises):
            if current_time > create_time:
                if self.logging:
                    print("create noise at time ", current_time, " create time ", create_time, " end time ", end_time)
                noise = self.create_noise(sp_idx, end_time)
                if noise is not None:
                    created_noises.append(idx)
                else:
                    continue
            else:
                break
            
        if len(created_noises) != 0:
            created_noises.sort(reverse=True)
            for idx in created_noises:
                del self.waiting_noises[idx]
        # self.create_noise()
        # check noises
        removed_noises = []
        for idx, (vehicle, created_time, end_time) in enumerate(self.noise_list):
            if current_time > end_time:
                removed_noises.append(idx)

        removed_list = []
        if len(removed_noises) != 0:
            removed_noises.sort(reverse=True)
            for idx in removed_noises:
                removed_list.append(self.noise_list[idx])
                del self.noise_list[idx]

        self.destroy_noises(removed_list)

        # collect sensor data and check destroy
        destroy_list = []
        for idx, (vehicle, sensor, actor, created_time) in enumerate(self.vehicles_with_sensor):
            if current_time < created_time + self.alive_period:
                # collect data
                sensor.save()
            else:
                # destroy
                destroy_list.append(idx)
        # destroy actors which are ready to be removed
        if len(destroy_list) != 0:
            destroy_list.sort(reverse=True)
            for idx in destroy_list:
                self.waiting_destroy.append(self.vehicles_with_sensor[idx])
                del self.vehicles_with_sensor[idx]

            self.destroy_vehicles(self.waiting_destroy)

        # create sensor
        created_actors = []
        for idx, (vehicle, created_time) in enumerate(self.created_vehicles):
            if current_time > created_time + time_stop:
                lidar, actor = self.create_sensor(vehicle)
                self.vehicles_with_sensor.append((vehicle, lidar, actor, current_time))
                created_actors.append(idx)
        # remove vehicles with sensor
        if len(created_actors) != 0:
            created_actors.sort(reverse=True)
            for idx in created_actors:
                del self.created_vehicles[idx]

        # spawn vehicle
        created_actors = []
        for idx, (sp_idx, create_time) in enumerate(self.waiting_vehicles):
            if current_time > create_time:
                vehicle = self.create_vehicle(sp_idx)
                if vehicle is not None:
                    self.created_vehicles.append((vehicle, current_time))
                    created_actors.append(idx)
                else:
                    continue
            else:
                break
        if len(created_actors) != 0:
            created_actors.sort(reverse=True)
            for idx in created_actors:
                del self.waiting_vehicles[idx]

        return self.frame_count
    
    def add_noise(self):
        # spawn position index, start time, end time
        # self.waiting_noises.append((0, 5, 20)) # alive 15 seconds
        # self.waiting_noises.append((1, 4, 50)) # alive 45 seconds
        return

    def add_vehicles(self):
        # print("no vehicles with lidar")
        self.waiting_vehicles.append((0, 1))
        # self.waiting_vehicles.append((1, 5))
        # self.waiting_vehicles.append((0, 10))
        # self.waiting_vehicles.append((1, 15))
        # self.waiting_vehicles.append((0, 20))
        # self.waiting_vehicles.append((1, 25))
        # self.waiting_vehicles.append((0, 30))
        # self.waiting_vehicles.append((1, 35))
        # self.waiting_vehicles.append((0, 40))
        # self.waiting_vehicles.append((1, 45))
        # self.waiting_vehicles.append((0, 50))
        # self.waiting_vehicles.append((1, 55))
        # self.waiting_vehicles.append((0, 5))
        # self.waiting_vehicles.append((0, 15))
        # self.waiting_vehicles.append((0, 25))
        # self.waiting_vehicles.append((0, 35))
        # self.waiting_vehicles.append((0, 45))
        # print("no vehicles with lidar")
        # self.waiting_vehicles.append((1, 5))
        # self.waiting_vehicles.append((1, 15))
        # self.waiting_vehicles.append((1, 25))
        # self.waiting_vehicles.append((1, 35))
        # self.waiting_vehicles.append((1, 45))

