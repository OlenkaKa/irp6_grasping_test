#!/usr/bin/env python

import time

from irp6_grasping import GraspingIRPOS

if __name__ == '__main__':

    front_desired_joints = [0.0, 7.692487217655884e-06, -1.666887782823125, -0.12526463087695183, 0.22135429144361307, 4.712395088435141, -1.5707949064846223]

    irpos = GraspingIRPOS('irp6ot_grasping', 'Irp6ot', 7, 'irp6ot_manager')

    irpos.move_to_joint_position(front_desired_joints, 10.00)

    irpos.start_scene_observation(1)
    time.sleep(10)
    irpos.stop_scene_observation()

    # objects = irpos.get_objects_to_grasp()
    objects = irpos.get_scene_objects()
    print objects

    if objects:
        irpos.grasp_object(objects[0].id)
    else:
        print 'No objects'
