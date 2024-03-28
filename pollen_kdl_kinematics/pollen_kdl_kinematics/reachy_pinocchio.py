#!/usr/bin/env python

import numpy as np
import pinocchio as pin
import tempfile

head_joints = [
    'neck_roll',
    'neck_pitch',
    'neck_yaw',
]

def arm_joint_list(arm):
    assert arm == 'l' or arm == 'r'
    l_arm_joints_tokeep = [
        '_shoulder_pitch',
        '_shoulder_roll',
        '_elbow_yaw',
        '_elbow_pitch',
        '_wrist_roll',
        '_wrist_pitch',
        '_wrist_yaw',
        # '_hand_finger',
        # '_hand_finger_mimic',
    ]

    return [arm + x for x in l_arm_joints_tokeep]


def add_framenames(model):
    model.frame_names = [x.name for x in model.frames.tolist()]


def model_data_from_joints(model, joints_to_keep):
    all_joints = model.names.tolist()
    tolock = []
    for joint in all_joints:
        if joint not in joints_to_keep and joint != 'universe':
            tolock.append(joint)

    # print('to_keep:', joints_to_keep)
    # print('to_lock:', tolock)
    # Get the ID of all existing joints
    jointsToLockIDs = []
    for jn in tolock:
        # print('jn:', jn)
        if model.existJointName(jn):
            jointsToLockIDs.append(model.getJointId(jn))

    model_reduced = pin.buildReducedModel(model, jointsToLockIDs,
                                          np.zeros(model.nq))
    add_framenames(model)

    return model_reduced, model_reduced.createData()


def arm_from_urdfstr(urdfstr, arm):
    model = pin.buildModelFromXML(urdfstr)
    arm_joints = arm_joint_list(arm)
    model, _ = model_data_from_joints(model, arm_joints)
    return model

def head_from_urdfstr(urdfstr):
    model = pin.buildModelFromXML(urdfstr)
    model, _ = model_data_from_joints(model, head_joints)
    return model
