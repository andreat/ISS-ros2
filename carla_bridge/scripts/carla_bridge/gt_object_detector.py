import numpy as np
import math

from iss_msgs.msg import ObjectDetection3DArray, ObjectDetection3D

from planning_utils.angle import zero_2_2pi

def quaternion_from_euler(self, ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk
    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss
    return q

class GTObjectDetector:
    def __init__(self, node, vehicle_id, world) -> None:
        self._node = node
        self._vehicle_id = vehicle_id
        self._world = world
        gt_object_detection_frequency = self._node.declare_parameter('~gt_object_detection_frequency', 10).value
        self._timer = self._node.create_timer(1 / gt_object_detection_frequency, self._timer_callback)
        self._object_detection_pub = self._node.create_publisher(ObjectDetection3DArray, "carla_bridge/gt_object_detection", 1)
        self._MAX_DISTANCE = 20
        
    def _timer_callback(self):
        ego_location = self._world.get_actor(self._vehicle_id).get_location()
        all_detections = ObjectDetection3DArray()
        for actor in self._world.get_actors().filter('vehicle.*'):
            if self._vehicle_id != None and self._vehicle_id == actor.id:
                continue
            if actor.get_location().distance(ego_location) > self._MAX_DISTANCE:
                continue
            detection = ObjectDetection3D()
            detection.header.stamp = self._node.get_clock().now().to_msg()
            detection.header.frame_id = "map"
            detection.id = actor.id
            detection.score = 1.0
            detection.state.x = actor.get_location().x
            detection.state.y = -actor.get_location().y
            detection.state.heading_angle = zero_2_2pi(-np.deg2rad(actor.get_transform().rotation.yaw))
            detection.state.velocity = np.hypot(actor.get_velocity().x, actor.get_velocity().y)
            detection.state.acceleration = np.hypot(actor.get_acceleration().x, actor.get_acceleration().y)
            detection.bbox.size.x = actor.bounding_box.extent.x * 2
            detection.bbox.size.y = actor.bounding_box.extent.y * 2
            detection.bbox.size.z = actor.bounding_box.extent.z * 2
            detection.bbox.center.position.x = actor.bounding_box.location.x
            detection.bbox.center.position.y = -actor.bounding_box.location.y
            detection.bbox.center.position.z = actor.bounding_box.location.z
            roll = np.deg2rad(actor.get_transform().rotation.roll)
            pitch = np.deg2rad(actor.get_transform().rotation.pitch)
            yaw = -np.deg2rad(actor.get_transform().rotation.yaw)
            quaternion = quaternion_from_euler(roll, pitch, yaw)
            detection.bbox.center.orientation.x = quaternion[0]
            detection.bbox.center.orientation.y = quaternion[1]
            detection.bbox.center.orientation.z = quaternion[2]
            detection.bbox.center.orientation.w = quaternion[3]
            all_detections.detections.append(detection)
        self._object_detection_pub.publish(all_detections)
            
    def shutdown(self):
        self._timer.shutdown()
        self._object_detection_pub.unregister()
            
