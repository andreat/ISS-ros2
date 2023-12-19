import carla

from iss_msgs.msg import StateArray, State

color_map = { 
    'red': carla.Color(255, 0, 0),
    'green': carla.Color(0, 255, 0),
    'blue': carla.Color(0, 0, 255),
    'yellow': carla.Color(255, 255, 0),
}

class CARLAVisualizer:
    def __init__(self, node) -> None:
        self._node = node
        self._global_planner_sub = self._node.create_subscription(StateArray, "planning/global_planner/trajectory", self._global_planner_callback, 100)
        self._local_planner_sub = self._node.create_subscription(StateArray, "planning/local_planner/trajectory", self._local_planner_callback, 100)
    
    def _global_planner_callback(self, msg):
        life_time = self._node.get_parameter('~simulation_duration').get_value()
        self._draw_trajectory_carla(msg, life_time=life_time, z=0.5, color=color_map['green'])
    
    def _local_planner_callback(self, msg):
        self._draw_trajectory_carla(msg, life_time=0.1, z=0.5, color=color_map['blue'])
    
    def _draw_trajectory_carla(self, msg, life_time, z=0.5, color=carla.Color(255, 0, 0)):
        # trajectory: StateArray
        for i, state in enumerate(msg.states):
            loc = carla.Location(x=state.x, y=-state.y, z=z)  # note: carla y is opposite to rviz y
            self._node._world.debug.draw_string(loc, str(i), life_time=life_time, color=color)
