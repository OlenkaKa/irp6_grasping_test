#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time

from irp6_grasping import GraspingIRPOS
from irpos import *

if __name__ == '__main__':

    front_desired_joints = [0.0, 7.692487217655884e-06, -1.666887782823125, -0.12526463087695183, 0.22135429144361307, 4.712395088435141, -1.5707949064846223]

    irpos = GraspingIRPOS('irp6ot_grasping', 'Irp6ot', 7, 'irp6ot_manager')
    irpos.set_tool_geometry_params(Pose(Point(0.0, 0.0, 0.225), Quaternion(0.0, 0.0, 0.0, 1.0)))
    irpos.tfg_to_joint_position(0.06, 3.0)
    irpos.move_to_joint_position(front_desired_joints, 10.00)

    irpos.start_scene_observation(1)
    irpos.stop_scene_observation()

    objects = irpos.get_objects_to_grasp(0.6)
    print objects

    if objects:
        irpos.grasp_object(objects[0].id)
        irpos.move_rel_to_cartesian_pose(3.0, Pose(Point(0, 0, -0.08), Quaternion(0, 0, 0, 1)))
        irpos.tfg_to_joint_position(0.07, 4.0)
    else:
        print 'No objects'
