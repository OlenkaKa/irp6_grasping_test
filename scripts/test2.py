#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import tf_conversions.posemath as posemath
from irpos import *
from irp6_grasping import GraspingIRPOS

object_models = [
    ['herbapol_mieta', PyKDL.Vector(0.140, -0.075, 0.064)],
    ['mnich', PyKDL.Vector(0.146, -0.077, 0.058)],
    ['sen', PyKDL.Vector(0.114, -0.061, 0.073)]
]

def print_objects(objects, view_id):
    print 'View'
    print view_id
    print objects


def move_up_object(object_info):
    obj_pose = posemath.fromMsg(object_info.pose.pose.pose)
    model_translation = None
    for model in object_models:
        if model[0] == 'mnich':
            model_translation = PyKDL.Vector(model[1].x() / 2, 0, model[1].z() / 2)
            break

    global_translation = obj_pose * model_translation + PyKDL.Vector(-0.05, 0, 0.19)
    return Point(global_translation.x(), global_translation.y(), global_translation.z())


def observe_scene(irpos, duration, view_id):
    irpos.start_scene_observation(view_id)
    time.sleep(duration)
    irpos.stop_scene_observation()


if __name__ == '__main__':

    right_quat = Quaternion(-0.3420201433256687, 0.0, 0.0, 0.9396926207859083)
    left_quat = Quaternion(-0.3420201433256687, 0.0, 0.0, -0.9396926207859083)

    front_desired_joints = [0.0, 7.692487217655884e-06, -1.666887782823125, -0.12526463087695183, 0.22135429144361307,
                            4.712395088435141, -1.5707949064846223]

    irpos = GraspingIRPOS('irp6ot_grasping', 'Irp6ot', 7, 'irp6ot_manager')

    irpos.tfg_to_joint_position(0.06, 3.0)
    irpos.move_to_joint_position(front_desired_joints, 10.00)

    observe_scene(irpos, 10, 1)

    objects = irpos.get_scene_objects()
    print_objects(objects, 1)

    if objects:
        object_info = irpos.get_scene_object_info(objects[0].id)
        next_pose = move_up_object(object_info)

        irpos.move_to_cartesian_pose(10.0, Pose(next_pose, Quaternion(0, 1, 0, 0)))
        observe_scene(irpos, 10, 2)

        objects = irpos.get_scene_objects()
        print_objects(objects, 2)

    else:
        print 'No objects'

    observe_quat = right_quat if irpos.get_cartesian_pose().position.y > 0 else left_quat

    observe_pose = Pose(Point(0.0, 0.0, 0.0), observe_quat)
    irpos.set_tool_geometry_params(Pose(Point(0.0, 0.0, 0.5), Quaternion(0.0, 0.0, 0.0, 1.0)))
    irpos.move_rel_to_cartesian_pose(10.0, observe_pose)
    irpos.set_tool_geometry_params(Pose(Point(0.0, 0.0, 0.25), Quaternion(0.0, 0.0, 0.0, 1.0)))
    observe_scene(irpos, 10, 3)

    objects = irpos.get_scene_objects()
    print_objects(objects, 3)

    if objects:
        irpos.grasp_object(objects[0].id)
    else:
        print 'No objects'
