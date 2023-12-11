#!/usr/bin/env python

import carla
import random
import numpy as np
import random
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node

from tqdm import tqdm

from carla_bridge.gt_object_detector import GTObjectDetector
from carla_bridge.gt_state_estimator import GTStateEstimator
from carla_bridge.carla_visualizer import CARLAVisualizer
from carla_bridge.controller_bridge import ControllerBridge

from iss_msgs.msg import ControlCommand

class CARLABridgeNode(Node):
    def __init__(self):
        super().__init__("carla_bridge_node")

        carla_host = self.declare_parameter('~carla_host', 'localhost').value
        carla_port = self.declare_parameter('~carla_port', 2000).value
        client = carla.Client(carla_host, carla_port)
        client.set_timeout(5.0)
        map_name = self.declare_parameter('~map_name', 'Town06').value
        client.load_world(map_name)

        self.params =  {
            "fixed_delta_seconds": self.declare_parameter('~fixed_delta_seconds', 0.05).value,
            "num_non_ego_vehicles": self.declare_parameter('~num_non_ego_vehicles', 50).value,
            "graphic_rendering": self.declare_parameter('~graphic_rendering', True).value,
            "simulation_duration": self.declare_parameter('~simulation_duration', 60).value,
            "simple_agent_demo": self.declare_parameter('~simple_agent_demo', False).value,
            "ego_init": self.declare_parameter('~ego_init', 1).value,
            "ego_destination": self.declare_parameter('~ego_destination', 10).value,
            "agent_control_frequency": self.declare_parameter('~agent_control_frequency', 10).value,
        }
        self._world = client.get_world()
        self._original_settings = self._world.get_settings()
        self._traffic_manager = client.get_trafficmanager()
        self._traffic_manager_port = self._traffic_manager.get_port()
        self._traffic_manager.set_random_device_seed(42)
        settings = self._world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.params["fixed_delta_seconds"]
        settings.no_rendering_mode = not self.params["graphic_rendering"]
        self._world.apply_settings(settings)
        self._traffic_manager.set_synchronous_mode(True)
        self._map = self._world.get_map()
        self._spawn_points = self._map.get_spawn_points()
        self._spectator = self._world.get_spectator()
        self._vehicles = {}
        self._add_vehicles()
        self._world.tick()
        

    def run(self):
        self._gt_object_detector = GTObjectDetector(self, self._vehicles["ego_vehicle"].id, self._world)
        self._gt_state_estimator = GTStateEstimator(self, self._vehicles["ego_vehicle"])
        self._controller_bridge = ControllerBridge(self, self._vehicles["ego_vehicle"])
        self._carla_timer = self.create_timer(self.params["fixed_delta_seconds"], self._carla_tick)
        self._total_steps = int(self.params["simulation_duration"] / self.params["fixed_delta_seconds"])
        # self._progress_bar = tqdm(total=self.params["simulation_duration"] + 0.1, unit="sec")
        self._step_cnt = 0

        if self.params["simple_agent_demo"]:
            self._controller_bridge.start_simple_agent(self._spawn_points[self.params["ego_destination"]], self.params["agent_control_frequency"])
        else:
            self._controller_bridge.start_iss_agent(self._spawn_points[self.params["ego_destination"]])
            self._carla_visualizer = CARLAVisualizer(self._world)
        rclpy.spin(self)
    
        
    def _carla_tick(self, event):
        # self._progress_bar.update(self.params["fixed_delta_seconds"])
        self._step_cnt += 1
        self._set_spectator(self._vehicles["ego_vehicle"].get_transform())
        self._controller_bridge.apply_control()
        self._world.tick()
        if self._step_cnt >= self._total_steps:
            self._gt_object_detector.shutdown()
            self._gt_state_estimator.shutdown()
            self._carla_timer.shutdown()
            self._agent_timer.shutdown() if self.params["simple_agent_demo"] else None
            # self._progress_bar.close()
            self.destory()
            self._world.tick()
            rclpy.shutdown("Simulation finished!")
            
    def _add_ego_vehicle(self, spawn_point):
        blueprint_library = self._world.get_blueprint_library()
        vehicle_bp = random.choice(blueprint_library.filter('vehicle.tesla.model3'))
        vehicle = self._world.try_spawn_actor(vehicle_bp, spawn_point)
        if vehicle != None:
            self._vehicles["ego_vehicle"] = vehicle
        else:
            print("Ego vehicle spawn failed")
    
    def _add_non_ego_vehicle(self, spawn_point, role_name):
        blueprint_library = self._world.get_blueprint_library()
        vehicle_bp = random.choice(blueprint_library.filter('vehicle.*'))
        vehicle = self._world.try_spawn_actor(vehicle_bp, spawn_point)
        if vehicle != None:
            self._vehicles[role_name] = vehicle
            vehicle.set_autopilot(True, self._traffic_manager_port)
    
    def _add_vehicles(self):
        ego_spawn_point = self._spawn_points[self.params["ego_init"]]
        nonego_spawn_points = []       
        for p in self._spawn_points:
            if p != ego_spawn_point and \
                np.hypot(p.location.x - ego_spawn_point.location.x,
                         p.location.y - ego_spawn_point.location.y) < 50:
                nonego_spawn_points.append(p)
        self._add_ego_vehicle(ego_spawn_point)
        for i in range(self.params["num_non_ego_vehicles"]):
            random.shuffle(nonego_spawn_points)
            if i < len(nonego_spawn_points):
                self._add_non_ego_vehicle(nonego_spawn_points[i], "non_ego_vehicle_" + str(i))
            else:
                break
            
    def _set_spectator(self, transform):
        new_transform = carla.Transform(transform.location, transform.rotation)
        r = R.from_euler('xyz', [0, 0, new_transform.rotation.yaw], degrees=True)
        loc = r.apply([-15, 0, 20])
        new_transform.location.x += loc[0]
        new_transform.location.y += loc[1]
        new_transform.location.z += loc[2]
        new_transform.rotation.pitch = -40
        self._spectator.set_transform(new_transform)
    
    def destory(self):
        # self._traffic_manager.set_synchronous_mode(False)
        for vehicle in self._vehicles.values():
            vehicle.destroy()
        self._world.tick()
        self._world.apply_settings(self._original_settings)
            
def main():
    rclpy.init()
    simulator = CARLABridgeNode()
    simulator.run()

if __name__ == "__main__":
    main()
