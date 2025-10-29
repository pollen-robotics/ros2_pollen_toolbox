import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import numpy as np
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper

from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType

# name:
# - l_elbow_yaw
# - r_elbow_yaw
# - r_hand_finger
# - r_elbow_pitch
# - r_shoulder_roll
# - antenna_left
# - neck_pitch
# - l_wrist_roll
# - neck_yaw
# - r_shoulder_pitch
# - drivewhl1_joint
# - drivewhl3_joint
# - l_wrist_yaw
# - tripod_joint
# - drivewhl2_joint
# - antenna_right
# - r_wrist_yaw
# - r_wrist_pitch
# - r_wrist_roll
# - l_elbow_pitch
# - l_shoulder_pitch
# - l_wrist_pitch
# - neck_roll
# - l_hand_finger
# - l_shoulder_roll

class GravityCompensator(Node):
    def __init__(self):
        super().__init__('gravity_compensator')

        # payload masses (kg)
        self.l_arm_object_mass = 0
        self.r_arm_object_mass = 0
        # Robot model pinocchio
        self.robot = None

        # Subs and pubs
        self.sub = self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.sub_torque_on = self.create_subscription(Float64MultiArray, '/forward_torque_controller/commands', self.torque_on_cb, 10)
        self.sub_object_mass = self.create_subscription(Float64MultiArray, '/set_payload_mass', self.object_mass_cb, 10)
        
        self.publ = self.create_publisher(Float64MultiArray, '/l_arm_forward_effort_controller/commands', 10)
        self.pubr = self.create_publisher(Float64MultiArray, '/r_arm_forward_effort_controller/commands', 10)
        self.pub_object_mass = self.create_publisher(Float64MultiArray, '/current_payload_mass', 10)
        
        
        # Get URDF from parameter server
        self.cli = self.create_client(GetParameters, '/robot_state_publisher/get_parameters')
        if not self.cli.wait_for_service(timeout_sec=60.0):
            self.get_logger().error(f'Service not available: /robot_state_publisher/get_parameters')
            rclpy.shutdown(); return
        req = GetParameters.Request()
        req.names = ['robot_description']
        future = self.cli.call_async(req)
        future.add_done_callback(self._on_robot_description)
        
        self.create_timer(0.005, self.update)  # 200Hz update rate
        
        self.r_arm_on = False
        self.l_arm_on = False


    def torque_on_cb(self, msg: Float64MultiArray):
        # if torque is off, reset object masses to zero
        try:
            self.l_arm_on = True
            self.r_arm_on = True
            for i, val in enumerate(msg.data):
                if i > 1 and i < 4:
                    self.l_arm_on = self.l_arm_on and bool(val)
                elif i > 5 and i < 8:
                    self.r_arm_on = self.r_arm_on and bool(val)
        except Exception as e:
            self.get_logger().error(f"Error in torque_on_cb: {e}")
                    


    def _on_robot_description(self, future):
        try:
            response = future.result()
            if response:
                urdf_xml = response.values[0].string_value
                self.get_logger().info("URDF loaded from parameter server")
                
                # Robot model from parameter
                self.robot = self.load_robot_from_param(urdf_xml)
                self.q = pin.neutral(self.robot.model)
                
                self.get_logger().info("Robot model is ready - starting gravity compensation")
                
        except Exception as e:
            self.get_logger().error(f"Error getting parameters: {e}")


    # callback to update object masses carried by the arms
    def object_mass_cb(self, msg: Float64MultiArray):
    
        try:
            if len(msg.data) >= 2:
                self.l_arm_object_mass = msg.data[0]
                self.r_arm_object_mass = msg.data[1]
                self.get_logger().info(f"Updated object masses: Left Arm = {self.l_arm_object_mass}, Right Arm = {self.r_arm_object_mass}")
        except Exception as e:
            self.get_logger().error(f"Error in object_mass_cb: {e}")
        

    # load robot model from URDF string and reduce it by locking unneeded joints
    def load_robot_from_param(self, urdf_xml: str) -> RobotWrapper:
        
        print("Loading robot model from String URDF")
        model = pin.buildModelFromXML(urdf_xml)
        robot = pin.RobotWrapper(model)
            
        print("reducing model")
        # Optional: reduce model
        tolock = [
        'tripod_joint',
        #'l_shoulder_pitch',
        #'l_shoulder_roll',
        #'l_elbow_yaw',
        #'l_elbow_pitch',
        
        #'l_wrist_roll',
        #'l_wrist_pitch',
        #'l_wrist_yaw',
        
        'l_hand_finger',
        'l_hand_finger_proximal',
        'l_hand_finger_distal',
        'l_hand_finger_proximal_mimic',
        'l_hand_finger_distal_mimic',
        'left_bar_joint_mimic',
        'left_bar_prism_joint_mimic',
        'neck_roll',
        'neck_pitch',
        'neck_yaw',
        'antenna_left',
        'antenna_right',
        
        # 'r_shoulder_pitch',
        # 'r_shoulder_roll',
        # 'r_elbow_yaw',
        # 'r_elbow_pitch',
        #'r_wrist_roll',
        #'r_wrist_pitch',
        #'r_wrist_yaw',
        
        'r_hand_finger',
        'r_hand_finger_proximal',
        'r_hand_finger_distal',
        'r_hand_finger_proximal_mimic',
        'r_hand_finger_distal_mimic',
        'right_bar_joint_mimic',
        'right_bar_prism_joint_mimic'
        ]
        
        # Get the ID of all existing joints
        lvals = np.zeros(robot.nq)
        jointsToLockIDs = []
        for i,jn in enumerate(tolock):
            if robot.model.existJointName(jn):
                jointsToLockIDs.append(robot.model.getJointId(jn))
        model = pin.buildReducedModel(robot.model, jointsToLockIDs, lvals)
        data = model.createData()
        
        robot.model = model
        robot.data = data
        print("Robot model loaded and reduced")
        return robot

    def joint_state_cb(self, msg: JointState):

        if self.robot is None:
            return
        
        try:
            q = np.zeros(self.robot.nq)
            tau_k = np.zeros(self.robot.nv)
            names = [""]* self.robot.nq
            for i, name in enumerate(msg.name):
                if name in self.robot.model.names:
                    idx = self.robot.model.getJointId(name)
                    q[idx - 1] = msg.position[i]  # -1 since 0 is universe
                    names[idx - 1] = name
                    tau_k[idx - 1] = msg.effort[i]
                    
            self.q = q
            self.tau_k = tau_k

        except Exception as e:
            self.get_logger().error(f"Error in joint_state_cb: {e}")

    # function that will run continously
    def update(self):
        if self.robot is None:
            return
    
        # Compute gravity torques
        tau = pin.computeGeneralizedGravity(self.robot.model, self.robot.data, self.q)
        
        tau_l = np.zeros(7)
        l_m = 0
        tau_r = np.zeros(7)
        r_m = 0
        
        # extract left and right arm torques
        if self.l_arm_on:
            tau_l = np.array(tau[:7].copy())
            # get the jacobian at the left arm tip
            joint_id = self.robot.model.getFrameId("l_arm_tip")
            J = pin.computeFrameJacobian(self.robot.model,
                                                self.robot.data,
                                                self.q,
                                                joint_id,
                                                reference_frame=pin.LOCAL_WORLD_ALIGNED)[:3,:]
                
            # left payload mass estimation
            l_m = (np.linalg.pinv(J[:,:7].T)@(self.tau_k[:7] - tau_l))[2]/9.81
            # add payload gravity compensation - if payload mass is not zero
            if self.l_arm_object_mass != 0 :
                tau_l = tau_l + J[:,:7].T @ np.array([0,0, self.l_arm_object_mass*9.81])
                
        if self.r_arm_on:
            tau_r = np.array(tau[7:14].copy())
            
            # get the jacobian at the right arm tip
            joint_id = self.robot.model.getFrameId("r_arm_tip")
            J = pin.computeFrameJacobian(self.robot.model,
                                            self.robot.data,
                                            self.q,
                                            joint_id,
                                            reference_frame=pin.LOCAL_WORLD_ALIGNED)[:3,:]

            # right payload mass estimation
            r_m = (np.linalg.pinv(J[:,7:].T)@(self.tau_k[7:] - tau_r))[2]/9.81
            # add payload gravity compensation - if payload mass is not zero
            if self.r_arm_object_mass != 0:
                tau_r = tau_r + J[:,7:].T @ np.array([0,0, self.r_arm_object_mass*9.81])
        
            
        effort_msg = Float64MultiArray()
        effort_msg.data = tau_l.tolist()
        self.publ.publish(effort_msg)
        effort_msg.data = tau_r.tolist()
        self.pubr.publish(effort_msg)
        
        object_mass_msg = Float64MultiArray()
        object_mass_msg.data = [l_m, r_m]
        self.pub_object_mass.publish(object_mass_msg)
        
        
    
def main(args=None):
    rclpy.init(args=args)
    node = GravityCompensator()
    rclpy.spin(node)
    
if __name__ == '__main__':
    main()
