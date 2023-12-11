#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from wpt_tracker.pid_wpt_tracker import VehiclePIDController
from wpt_tracker.linear_mpc_tracker import VehicleLinearMPCController
from iss_msgs.msg import ControlCommand, StateArray, State
from planning_utils.trajectory import Trajectory
import numpy as np

class WPTTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("wpt_tracker_node")
        ctrl_freq = self.declare_parameter("~control_frequency", 10).value
        self._timer = self.create_timer(1 / ctrl_freq, self._timer_callback)
        self._ctrl_pub = self.create_publisher(ControlCommand, "control/wpt_tracker/control_command", 1)
        self._ego_state_sub = self.create_subscription(State, "carla_bridge/gt_state", self._state_callback, 100)
        self._trajectory_sub = self.create_subscription(StateArray, "planning/local_planner/trajectory", self._trajectory_callback, 100)
        self._ego_state = None
        
        # self._pid_tracker = VehiclePIDController()
        linear_mpc_settings = {
            "acc_table": {0: 0, 0.2: 0.5, 0.4: 0.8, 0.6: 0.9, 0.8: 0.95, 1: 1},
            "nx": 4,
            "nu": 2,
            "N": 10,
            "dt": 0.05,
            "steer_rate_max": np.deg2rad(15.0),
            "speed_max": 50 / 3.6,
            "ego_veh_info": {
                "wheelbase": 2.8,
                "steer_max": np.deg2rad(70.0),
                "acc_max": 8
            },
            "Q": np.diag([1.0, 1.0, 1.0, 4.0]),
            "Qf": np.diag([2.0, 2.0, 2.0, 4.0]),
            "R": np.diag([0.2, 1]),
            "Rd": np.diag([0.1, 1])
        }
        self._pid_tracker = VehiclePIDController()
        self._mpc_tracker = VehicleLinearMPCController(linear_mpc_settings, self.get_logger())
        self._trajectory = Trajectory()
        
    def _timer_callback(self):
        if self._ego_state is None:
            return
        throttle, steering = self._pid_tracker.run_step(self._ego_state)
        # throttle, steering = self._mpc_tracker.run_step(self._ego_state)
        self._ctrl_pub.publish(ControlCommand(steering=steering, throttle=throttle))
    
    def _state_callback(self, msg):
        self._ego_state = msg
    
    def _trajectory_callback(self, msg):
        self._trajectory.from_ros_msg(msg)
        # self._mpc_tracker.set_traj(self._trajectory)
        self._pid_tracker.set_traj(self._trajectory.get_states())
        

def main():
    rclpy.init()
    wpt_tracker_node = WPTTrackerNode()
    rclpy.spin(wpt_tracker_node)

if __name__ == "__main__":
    main()
