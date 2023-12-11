from iss_msgs.msg import State
import numpy as np

from planning_utils.angle import zero_2_2pi


class GTStateEstimator:
    def __init__(self, node, vehicle) -> None:
        self._node = node
        self._vehicle = vehicle
        self._state_estimation_pub = self._node.create_publisher(State, "carla_bridge/gt_state", 1)
        gt_state_estimation_frequency = self._node.declare_parameter('~gt_state_estimation_frequency', 10).value
        self._timer = self._node.create_timer(1 / gt_state_estimation_frequency, self._timer_callback)
    
    def _timer_callback(self, event):
        state = State()
        state.header.stamp = self._node.get_clock().now()
        state.header.frame_id = "map"
        state.name = "ego_vehicle"
        carla_transform = self._vehicle.get_transform()
        vehicle_location = carla_transform.location
        vehicle_rotation = carla_transform.rotation
        vehicle_velocity = self._vehicle.get_velocity()
        vehicle_acceleration = self._vehicle.get_acceleration()
        state.x = vehicle_location.x
        state.y = -vehicle_location.y
        state.heading_angle = zero_2_2pi(-np.deg2rad(vehicle_rotation.yaw))
        state.velocity = np.hypot(vehicle_velocity.x, vehicle_velocity.y)
        state.acceleration = np.hypot(vehicle_acceleration.x, vehicle_acceleration.y)
        self._state_estimation_pub.publish(state)
    
    def shutdown(self):
        self._timer.shutdown()
        self._state_estimation_pub.unregister()
