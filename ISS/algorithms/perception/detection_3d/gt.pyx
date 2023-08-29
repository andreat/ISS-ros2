from ISS.algorithms.perception.detection_3d.base import Detection3DBase
from ISS.algorithms.utils.dataexchange.perception.object_detection import ObjectDetectionOutput

import carla
import os
import numpy as np

class Detection3Dgt(Detection3DBase):
    def _preprocess(self, detection_3d_input):
        pass

    def detect(self, detection_3d_input, carla_world) -> np.array[ObjectDetectionOutput]:
        # directly return all actors in the world
        res = []
        for actor in carla_world.get_actors():
            output = ObjectDetectionOutput()
            output._label = actor.semantic_tags
            output._score = 1
            if actor.type_id.startswith("Vehicle") or actor.type_id.startswith("Walker"):
                output._bbox = actor.bounding_box
            elif actor.type_id.startswith("Infrastructure"):
                output._bbox = actor.get_light_boxes()
            output._loc = actor.get_location()
            output._trans = actor.get_transform()
            res.append(output)
        return res