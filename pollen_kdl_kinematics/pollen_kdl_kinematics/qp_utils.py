
#!/usr/bin/env python

import tempfile

import numpy as np

OSQP_FOUND=True
try:
    import osqp
except ImportError as e:
    OSQP_FOUND=False
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

class LinConstraints:
    def __init__(self, A, Amin, Amax):
        self.A = A
        self.min = Amin
        self.max = Amax

def velqp(Jac, log, q, q_reg, T, linear_factor=1, k_qreg=10, w_reg=1e-5, q_old=None, linear_constraints=None):
    #  Quadratic Problem
    #  q => current arm joint angles
    # qd => joint angle velocities
    #  min   qd.T P qd + g.T qd
    #   qd
    #        qd_min <= qd <= qd_max
    #  (qd_max-q)/T <= qd <= (qd_max-q)/T
    if q_old is None:
        q_old = np.zeros_like(q)

    g_q0 = -w_reg*2*(k_qreg*(q_reg-q))# -w_reg*2*(q-q_old)/T
    g = -2*Jac.T@log / T + g_q0

    k = np.eye(6)
    k[:3,:] *= linear_factor
    P = spa.csc_matrix(2*Jac.T@k@k@Jac) + w_reg*spa.csc_matrix(np.eye(len(q)))
    g = -2*Jac.T@k@log / T + g_q0

    eye = np.eye(len(q))
    qp_A = spa.csc_matrix(np.vstack([eye]))

    # QDMAX = 200
    QDMAX = 10
    # QDMAX = 2
    # QDMAX = .5
    qd_max =  QDMAX*np.ones_like(q)
    qd_min = -QDMAX*np.ones_like(q)


    # joint limits
    q_max =  np.inf*np.ones_like(q)
    q_min = -np.inf*np.ones_like(q)
    # q_max =   np.pi*np.ones_like(q)
    # q_min =  -np.pi*np.ones_like(q)
    # factor = 0.3
    # q_max =factor* np.pi*np.ones_like(q)
    # q_min =factor*-np.pi*np.ones_like(q)
    # factor = 0.6
    # q_max[:-3] = factor * q_max[:-3]
    # q_min[:-3] = factor * q_min[:-3]
    q_max = np.array([1.57079633, 1.57079633, 1.57079633, 0.1, 0.35, 0.35, 1.57])
    q_min = np.array([-1.57079633, -1.57079633, -1.57079633, -2.25, -0.35, -0.35, -1.57])


    # enables joint angle limits
    TT = 2*T
    qp_A = np.vstack([eye, eye])
    qd_max = np.hstack([qd_max, (q_max-q)/(TT)])
    qd_min = np.hstack([qd_min, (q_min-q)/(TT)])

    if linear_constraints is not None:
        qp_A = np.vstack([qp_A, linear_constraints.A])
        qd_max = np.hstack([qd_max, linear_constraints.max])
        qd_min = np.hstack([qd_min, linear_constraints.min])

    qp_l, qp_u = qd_min, qd_max

    qp = osqp.OSQP()
    qp.setup(P=P, q=g, A=spa.csc_matrix(qp_A), l=qp_l, u=qp_u, verbose=False)
    results = qp.solve()
    print('status: {}'.format(results.info.status))
    return results.x

def pin_FK(model, data, q, frame_id, world_frame=False):
    pin.forwardKinematics(model, data, q)
    pin.framesForwardKinematics(model, data, q)

    X = data.oMf[frame_id]

    # if not world, then it's torso
    if not world_frame:
        X = data.oMf[model.getFrameId('torso')].actInv(X)
    return X


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

def pose_err(M, Mdes):
    Mdes = pin.SE3(M)
    return np.hstack([*(Mdes.translation-M.translation),
                        *pin.log(M.rotation.T@M.rotation)])

def pose_err_norms(M, Mdes):
    err = pose_err(M, Mdes)
    return np.linalg.norm(err[:3]), np.linalg.norm(err[3:])

class JointAngleQpController:

    def __init__(self, urdfstr, prefix):
        self.pinmodel = arm_from_urdfstr(urdfstr=urdfstr, arm=prefix)
        self.q_old = None
        self.prefix = prefix

    def fk(self, q):
        model = self.pinmodel
        data = model.createData()
        tip = self.prefix + '_arm_tip'
        frame_id = model.getFrameId(tip)
        return pin_FK(model, data, q, frame_id)

    def solve(self, q, M):

        #####################################################
        #####################################################
        #####################################################
        # TODO experimental QP SOLVER FOR ARMS

        Mdes = pin.SE3(M)


        ####################################################
        # pinocchio
        model = self.pinmodel
        data = model.createData()


        tip = self.prefix + '_arm_tip'
        frame_id = model.getFrameId(tip)
        # joint_id = model.getJointId(tip)
        # print('tip:{} frame_id:{}'.format(tip, frame_id))
        Mcur = pin_FK(model, data, q, frame_id)
        # Mcur = pin.SE3(rh.kdlFrame_to_np(rh.kdl_FK(self.fk_solver[name], q)))
        iMd = Mcur.actInv(Mdes)
        log = pin.log(iMd).vector
        # log[:3] = Mdes.translation - Mcur.translation
        # X0_pin, X1_pin = Mcur, Mdes
        # log[3:] = X0_pin.rotation @ pin.log(X0_pin.rotation.T @ X1_pin.rotation)

        # Jac = pin.computeJointJacobians(model, data, q)
        # Jac = pin.computeJointJacobian(model, data, q, joint_id)
        # Jac = pin.getJointJacobian(model, data, joint_id,
        #                            reference_frame=pin.LOCAL)
        # Jac = pin.computeJointJacobian(model, data, q, joint_id)
        Jac = pin.computeFrameJacobian(model, data, q, frame_id,
                                        reference_frame=pin.LOCAL)
        # Jac = pin.getFrameJacobian(model, data, frame_id,
        #                            reference_frame=pin.LOCAL)
        # Jac = pin.getFrameJacobian(model, data, frame_id,
        #                            reference_frame=pin.LOCAL_WORLD_ALIGNED)
        ####################################################
        # Jac = pin.computeFrameJacobian(model, data, q, frame_id,
        #                                reference_frame=pin.LOCAL_WORLD_ALIGNED)
        ####################################################
        # kdl
        # Jac = rh.kdl_jacobian(self.jac_solver[name], q)
        # Mcur = rh.kdl_FK(self.fk_solver[name], q)
        # log = rh.kdldiff_np(Mcur=Mcur, Mdes=M)
        # log2 = rh.kdldiff_np(Mcur=Mcur, Mdes=M)
        # log[3:] = log2[3:]
        # log = log2
        ####################################################
        # warn('msg.pose:{}'.format(msg.pose))


        # Mcur_pin = rh.pin_FK(model, data, q, frame_id)
        # Mcur_kdl = pin.SE3(rh.kdlFrame_to_np(rh.kdl_FK(self.fk_solver[name], q)))
        # err = np.linalg.norm(pin.log(Mcur_pin.actInv(Mcur_kdl)).vector)
        # if err > 1e-3:
        #     warn('WATTA FU GOIN ON')
        #     warn('Mcur_pin:{} \nMcur_kdl:{}'.format(Mcur_pin, Mcur_kdl))
        #     warn('err:{}'.format(err))

        T = 1/120 # [Hz]

        ####################################################
        # NOTE LEGACY J.T method IK in case no qp solver found
        if not OSQP_FOUND:
            transsol = q + qd_from_Jpen(Jac, v=log/T,
                                        alpha=1e-3)
            sol = transsol
            return sol
        # print('transsol: {}'.format(transsol))
        ####################################################


        ####################################################
        # regularization pose
        q_reg = np.zeros_like(q)
        # q_reg= np.array([0, 0, -np.pi / 2, 0, 0, 0])
        val = np.pi/3
        q_reg[0] = -val
        q_reg[1] = -np.pi/2 if self.prefix=='r' else np.pi/2
        # q_reg = np.ones_like(q)
        ####################################################



        ####################################################
        # extra constraints for anti self collision
        # tip = self.prefix + '_elbow_arm_joint'
        tip = self.prefix + '_elbow_yaw'
        frame_id = model.getFrameId(tip)
        yJacElbow = pin.computeFrameJacobian(model, data, q, frame_id,
                                             reference_frame=pin.LOCAL_WORLD_ALIGNED)[1, :]
        val = -0.2
        yplane = val if self.prefix=='r' else -val
        dydt = yplane - M[1,3]
        Alin = T*yJacElbow

        # print(f"Alin: {Alin}")
        # print(f"dydt:{dydt:.2f}, yplane:{yplane:.2f},  {self.prefix}_.y:{M[1,3]:.2f}")
        ymax = dydt
        ymin = -np.inf if dydt <0 else np.inf
        Amin = np.minimum(ymin, ymax)
        Amax = np.maximum(ymin, ymax)
        linear_constraints = LinConstraints(A=Alin, Amin=Amin, Amax=Amax)
        ####################################################

        # Jac = np.dot(pin.Jlog6(iMd.inverse()), Jac)
        qd = velqp(Jac, log, q, q_reg, T,
                    # linear_factor=5,
                    linear_factor=10,
                    # linear_factor=20,
                    # linear_factor=50,
                    k_qreg=10,
                    w_reg=1e-1,
                    # w_reg=1,
                    # w_reg=2,
                    # w_reg=100000000,
                    q_old=self.q_old,
                    # linear_constraints=linear_constraints,
                    )
        qpsol = q + qd*T
        sol = qpsol
        self.q_old = qpsol

        return sol
