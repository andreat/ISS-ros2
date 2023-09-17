from ISS.algorithms.perception.detection_3d.base import Detection3DBase
from ISS.algorithms.utils.dataexchange.perception.object_detection import ObjectDetectionOutput
from ISS.algorithms.utils.sensorutils.transform import carla_bbox_to_bbox, carla_location_to_location, carla_transform_to_transform, vertices_to_carla_bbox, carla_bbox_trans

import carla
import os
import numpy as np
from typing import List
from multiprocessing import Process
from ISS.algorithms.utils.vehicleutils.vehicleutils import CollisionChecker

class Detection3Dgt(Detection3DBase):
    def _preprocess(self, detection_3d_input):
        pass

    def detect(self, carla_world, vehicle_id=None):
        # directly return all actors in the world
        res = []
        for actor in carla_world.get_actors():
            if vehicle_id != None and vehicle_id == actor.id:
                continue
            output = ObjectDetectionOutput()
            output._score = 1
            # "static", "vehicle", "walker", "traffic"
            if actor.type_id.startswith("vehicle"):
                output._label = "vehicle"
                # print("VEHICLE: " + actor.type_id)
                output._bbox = carla_bbox_to_bbox(carla_bbox_trans(actor.bounding_box, actor.get_transform()))
                # transform local coords to global
                # trans = actor.get_transform()
                # bbox = actor.bounding_box
                # world_vertices = bbox.get_world_vertices(trans)
                # output._bbox = carla_bbox_to_bbox(vertices_to_carla_bbox(world_vertices))
            elif actor.type_id.startswith("walker"):
                output._label = "walker"
                # print("WALKER:" + actor.type_id)
                output._bbox = carla_bbox_to_bbox(carla_bbox_trans(actor.bounding_box, actor.get_transform()))
                # trans = actor.get_transform()
                # bbox = actor.bounding_box
                # world_vertices = bbox.get_world_vertices(trans)
                # output._bbox = carla_bbox_to_bbox(vertices_to_carla_bbox(world_vertices))
            #BUG: cannot invoke get_light_boxes() for traffic light
            # elif actor.type_id.startswith("traffic.traffic_light"):
            #     print("LIGHT:" + actor.type_id)
            #     output._bbox = actor.get_light_boxes()
            else:
                #TODO: add more types
                continue
            output._loc = carla_location_to_location(actor.get_location())
            output._trans = carla_transform_to_transform(actor.get_transform())
            
            res.append(output)
        return res    
    
    def build_detector(self, obstacles_list):
        ## Project the points on 2D surface
        all_points = []
        for item in obstacles_list:
            o3d_bbox = item._bbox.to_open3d()
            bbox_vertices = np.asarray(o3d_bbox.get_box_points())            
            ## Open3D Bottom Order: 0 - 1 - 7 - 2
            ## To-DO: Consider 3D Rotation and Project the points...            
            ind_list = [0, 1, 7, 2, 0]
            bound_points = []
            for i in range(4):
                cur_ind = ind_list[i]
                next_ind = ind_list[i+1]
                points = np.c_[np.linspace(bbox_vertices[cur_ind][0], bbox_vertices[next_ind][0], num=10), np.linspace(-bbox_vertices[cur_ind][1], -bbox_vertices[next_ind][1], num=10)]
                bound_points.append(points)
            bound_points = np.vstack(bound_points)
            all_points.append(bound_points)
        ## Build the detector        
        if len(all_points) > 0:
            all_points = np.vstack(all_points)
            # obstacle_detector = CollisionChecker(all_points, 4.4, 2.2)                
            obstacle_detector = CollisionChecker(all_points, 6.4, 3.2)                
            return obstacle_detector
        return None
    
    def handle(self, ip, port, vehicle_id, terminating_value, obstacle_detector_queue):
        client = carla.Client(ip, port)
        world = client.get_world()
        ## Refresh the vehicle objects..        
        while terminating_value.value:
            obstacles_list = self.detect(world, vehicle_id)
            obstacle_detector = self.build_detector(obstacles_list)
            obstacle_detector_queue.append(obstacle_detector)                    

    def run_proxies(self, data_proxies, ip, port, vehicle_id):
        ## Spawn Process Here and Return its process object..
        terminating_value = data_proxies['terminating_value']
        obstacle_detector_queue = data_proxies['obstacle_detector_queue']        
        process = Process(target=self.handle, args=[ip, port, vehicle_id, terminating_value, obstacle_detector_queue])
        process.start()
        return process