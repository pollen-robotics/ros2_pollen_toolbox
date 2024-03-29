#!/usr/bin/env python

import tempfile

import numpy as np
import osqp
import pinocchio as pin
import PyKDL as kdl
import scipy.sparse as spa

head_joints = [
    'neck_roll',
    'neck_pitch',
    'neck_yaw',
]

def qd_from_Jpen(Jac, v, alpha=1e-3):
    return alpha*np.linalg.pinv(Jac)@(v)

def kdldiff_np(Mcur, Mdes):
    Mcur_kdl = Mcur
    Mdes_kdl = Mdes
    if isinstance(Mcur, pin.SE3):
        Mcur=pinSE3_to_np(Mcur)
    if isinstance(Mdes, pin.SE3):
        Mdes=pinSE3_to_np(Mdes)

    if isinstance(Mcur, np.ndarray):
        Mcur_kdl = np_to_kdlFrame(Mcur)
    if isinstance(Mdes, np.ndarray):
        Mdes_kdl = np_to_kdlFrame(Mdes)
    return np.fromiter(kdl.diff(Mcur_kdl, Mdes_kdl), float)

def pinSE3_to_np(M):
    temp = np.zeros([4,4])
    temp[:3,:3] = M.rotation
    temp[:3,3] = M.translation
    return temp

def velqp(Jac, log, q, q_reg, T, linear_factor=1, w_reg=1e-5):
    #  Quadratic Problem
    #  q => current arm joint angles
    # qd => joint angle velocities
    #  min   qd.T P qd + g.T qd
    #   qd
    #        qd_min <= qd <= qd_max
    #  (qd_max-q)/T <= qd <= (qd_max-q)/T

    g_q0 = -w_reg*2*(q_reg-q)
    g = -2*Jac.T@log / T + g_q0

    k = np.eye(6)
    k[:3,:] *= linear_factor
    P = spa.csc_matrix(2*Jac.T@k@k@Jac) # + w_reg*spa.csc_matrix(np.eye(len(q)))
    g = -2*Jac.T@k@log / T + g_q0

    eye = np.eye(len(q))
    # qp_A = spa.csc_matrix(np.vstack([eye, eye]))
    qp_A = spa.csc_matrix(np.vstack([eye]))

    # QDMAX = 200
    QDMAX = 5
    # QDMAX = 2
    # QDMAX = .5
    qd_max =  QDMAX*np.ones_like(q)
    qd_min = -QDMAX*np.ones_like(q)
    q_max =   np.pi*np.ones_like(q)
    q_min =  -np.pi*np.ones_like(q)
    # q_max =  np.inf*np.ones_like(q)
    # q_min = -np.inf*np.ones_like(q)

    # enables joint angle limits
    # qp_A = spa.csc_matrix(np.vstack([eye, eye]))
    # qd_max = np.hstack([qd_max, (q_max-q)/T])
    # qd_min = np.hstack([qd_min, (q_min-q)/T])

    qp_l, qp_u = qd_min, qd_max

    qp = osqp.OSQP()
    qp.setup(P=P, q=g, A=qp_A, l=qp_l, u=qp_u, verbose=False)
    results = qp.solve()
    print('status: {}'.format(results.info.status))
    return results.x


def pin_FK(model, data, q, frame_id):
    pin.computeJointJacobians(model, data, q)
    pin.framesForwardKinematics(model, data, q)
    return data.oMf[frame_id]

def kdl_FK(fk_solver, q):
    qq = kdl.JntArray(len(q))
    for i, j in enumerate(q):
        qq[i] = j

    pose = kdl.Frame()
    res = fk_solver.JntToCart(qq, pose)
    return pose


def np_to_kdlFrame(npmat):
    RR = npmat[:3, :3]
    pp = npmat[:3, 3]
    return kdl.Frame(kdl.Rotation(*RR.ravel().tolist()), kdl.Vector(*pp))

def kdlFrame_to_np(kdlframe):
    mat =  np.zeros((4,4))
    for i in range(3):
        mat [i,3] = kdlframe.p[i]
    for i in range(3):
        for j in range(3):
            mat[i,j] = kdlframe[i,j]
    return mat

def kdl_jacobian(jac_solver, myotherq):
    # compute jac
    jkdl = kdl.Jacobian(len(myotherq))
    qkdl = kdl.JntArray(len(myotherq))
    for i, j in enumerate(myotherq):
        qkdl[i] = j
    jac_solver.JntToJac(qkdl, jkdl)

    # kdl.Jacobian -> np.array
    myjac =  np.zeros((jkdl.rows(), jkdl.columns()))
    for i in range(jkdl.rows()):
        for j in range(jkdl.columns()):
            myjac[i,j] = jkdl[i,j]
    return myjac



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
