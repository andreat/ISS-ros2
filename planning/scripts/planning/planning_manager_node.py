#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from ament_index_python.packages import get_package_share_directory

import os
import numpy as np
import time

import lanelet2
from lanelet2.projection import UtmProjector
from planning_utils.lanelet2_utils import get_solid_checker

from global_planner.lanelet2_planner import Lanelet2Planner
from motion_predictor.constant_velocity_predictor import ConstVelPredictor
from local_planner.lattice_planner import LatticePlanner

from iss_msgs.msg import State, StateArray, ObjectDetection3DArray
from iss_msgs.srv import SetGoal

class PlanningManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("planning_manager_node")
        service_cb_group = MutuallyExclusiveCallbackGroup()
        subscription_cb_group = MutuallyExclusiveCallbackGroup()
        self._ego_state_sub = self.create_subscription(State, "carla_bridge/gt_state", self._ego_state_callback, 100, callback_group=subscription_cb_group)
        self._obstacle_sub = self.create_subscription(ObjectDetection3DArray, "carla_bridge/gt_object_detection", self._obstacle_callback, 100, callback_group=subscription_cb_group)
        self._ego_state = None
        
        # Global planner 
        lanelet2_town06 = os.path.join(get_package_share_directory('planning'), "maps", "Town06_hy.osm")
        projector = UtmProjector(lanelet2.io.Origin(0., 0.))
        loadedMap, load_errors = lanelet2.io.loadRobust(lanelet2_town06, projector)
        traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                  lanelet2.traffic_rules.Participants.Vehicle)
        solid_checker = get_solid_checker(loadedMap)
        lanelet2_settings = dict()
        lanelet2_settings['TURNING_RADIUS'] = 5
        lanelet2_settings["GOAL_TORELANCE"] = 2
        self._global_planner = Lanelet2Planner(loadedMap, traffic_rules, solid_checker, lanelet2_settings)
        
        # Motion predictor
        predictor_settings = dict()
        predictor_settings['dt'] = 0.5
        predictor_settings['MAX_T'] = 6.0
        predictor_settings['ego_veh_info'] = { # Tesla Model 3
            'length': 4.69,
            'width': 2.0,
            'wheelbase': 2.8,
            'overhang_rear': 0.978,
            'overhang_front': 0.874
        }
        self._motion_predictor = ConstVelPredictor(predictor_settings)
        
        # Motion Planner
        lattice_settings = dict()
        lattice_settings['MAX_SPEED'] = 50.0 / 3.6     # maximum speed [m/s]
        lattice_settings['MAX_ACCEL'] = 5            # maximum acceleration [m/ss], tesla model3: 6.88
        lattice_settings['MAX_CURVATURE'] = 1.0      # maximum curvature [1/m], tesla model3's turning radius: 5.8    
        lattice_settings['D_S'] = 1                  # sample Frenet d
        lattice_settings['D_ROAD_W'] = 1.0             # road width sampling length [m]
        lattice_settings['DT'] = 1                   # prediction timestep length (s)
        lattice_settings['dt'] = 0.25                   # sample time
        lattice_settings['MAX_T'] = 6.0                # max prediction time [s]
        lattice_settings['MIN_T'] = 4.0                # min prediction time [s]
        lattice_settings['TARGET_SPEED'] = 30.0 / 3.6  # target speed [m/s]
        lattice_settings['D_T_S'] = 5 / 3.6          # target speed sampling length [m/s]
        lattice_settings['N_S_SAMPLE'] = 4             # sampling number of target speed    
        lattice_settings['K_J'] = 0.1
        lattice_settings['K_T'] = 0.1
        lattice_settings['K_D'] = 2.0
        lattice_settings['K_LAT'] = 1.0
        lattice_settings['K_LON'] = 0.8
        self._lattice_planner = LatticePlanner(loadedMap, traffic_rules, lattice_settings, solid_checker)
        
        self._global_planner_pub = self.create_publisher(StateArray, "planning/lanelet2_planner/trajectory", 1, callback_group=subscription_cb_group)
        self._local_planner_pub = self.create_publisher(StateArray, "planning/lattice_planner/trajectory", 1, callback_group=subscription_cb_group)
        self._set_goal_srv = self.create_service(SetGoal, "planning/set_goal", self._set_goal_srv_callback, callback_group=service_cb_group)
    
    def _set_goal_srv_callback(self, request, response):
        while self._ego_state == None:
            time.sleep(0.1)
        start_point = (self._ego_state.x, self._ego_state.y, self._ego_state.heading_angle)
        end_point = (request.x, request.y, request.yaw)
        global_traj = self._global_planner.run_step(start_point, end_point)
        if global_traj == None:
            self.get_logger().error("Global planning: Failed")
            response.success = False
        else:
            self.get_logger().info("Global planning: Success")
            self._global_planner_pub.publish(global_traj.to_ros_msg())
            self._lattice_planner.update(global_traj.get_waypoints())
            local_planning_frequency = self.declare_parameter("local_planning_frequency", 1).value
            self._lattice_planner_timer = self.create_timer(1.0/local_planning_frequency, self._local_planning_timer_callback)
            response.success = True
        return response
    
    def _ego_state_callback(self, state_msg):
        self._ego_state = state_msg
    
    def _obstacle_callback(self, obstacle_msg):
        self._motion_predictor.update(obstacle_msg)    

    def _local_planning_timer_callback(self):
        local_traj = self._lattice_planner.run_step(self._ego_state, self._motion_predictor)
        if local_traj.is_empty():
            self.get_logger().warning("Local planning: Failed")
            return
        self._local_planner_pub.publish(local_traj.to_ros_msg())
        if self._global_planner.is_goal_reached(self._ego_state):
            # self._lattice_planner_timer.shutdown()
            self.get_logger().info("Goal reached!")

def main():
    rclpy.init()
    planning_manager_node = PlanningManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(planning_manager_node)
    executor.spin()

if __name__ == "__main__":
    main()
