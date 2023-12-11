import rclpy

import numpy as np
import carla
from carla_agent.behavior_agent import BehaviorAgent
from geometry_msgs.msg import Twist
from iss_msgs.msg import ControlCommand
from iss_msgs.srv import SetGoal

class ControllerBridge:
    def __init__(self, node, vehicle) -> None:
        self._node = node
        self._vehicle = vehicle
        self._tele_op_sub = self._node.create_subscription(Twist, "/carla_bridge/cmd_vel", self._teleop_callback, 100)
        self._control = carla.VehicleControl()
        self._agent_sub = None
    
    def start_iss_agent(self, destination):
        self._agent_sub = self._node.create_subscription(ControlCommand, "control/control_command", self._agent_sub_callback, 100)
        self._call_set_goal_srv(destination)
    
    def _call_set_goal_srv(self, goal):
        client = self._node.create_client(SetGoal, 'planning/set_goal')
        client.wait_for_service()
        req = SetGoal.Request()
        req.x = goal.location.x
        req.y = -goal.location.y
        req.yaw = -np.deg2rad(goal.rotation.yaw)
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)
        return  future.result()

    def _agent_sub_callback(self, msg):
        self._set_control(msg.throttle, msg.steering)
        # rospy.loginfo("[simulator] throttle: %.2f,  steering: %.2f" % (msg.throttle, msg.steering))

    def start_simple_agent(self, destination, agent_control_frequency):
        self._simple_agent = BehaviorAgent(self._vehicle, behavior='normal')
        self._simple_agent.set_destination(destination)
        self._simple_agent_timer = self._node.create_timer(1 / agent_control_frequency, self._simple_agent_tick)
    
    def _simple_agent_tick(self, event):
        self._control = self._simple_agent.run_step()
    
    def _teleop_callback(self, msg):
        throttle = msg.linear.x
        steering = msg.angular.z
        scale_linear = self._node.declare_paramater("~scale_linear", 0.5).value
        scale_angular = self._node.declare_parameter("~scale_angular", 0.5).value
        self._set_control(throttle * scale_linear, steering * scale_angular)
    
    def apply_control(self):
        self._vehicle.apply_control(self._control)   
        
    def _set_control(self, throttle, steering):
        self._control.steer = min(max(-steering, -1.0), 1.0)
        if throttle < 0:
            self._control.throttle = 0
            self._control.brake = min(-throttle, 1.0)
        else:
            self._control.throttle = min(throttle, 1.0)
            self._control.brake = 0

