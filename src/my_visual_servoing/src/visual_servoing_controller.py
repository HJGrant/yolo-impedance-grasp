#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from vision_msgs.msg import Detection2DArray, CameraInfo
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

class JointIBVS:
    def __init__(self, model):
        rospy.init_node('joint_ibvs_node')
        
        self.model = model

        # --- Control Constants ---
        self.lmbda = 0.6
        self.z_dist = 1.0
        self.focal_length_x = 500.0 # Pixels
        self.focal_length_y = 500.0 # Pixels
        self.image_width = 0
        self.image_height = 0

        # Target: Center of image in normalized coordinates (0,0)
        self.s_star = np.array([0, 0])

        self.sub_depth = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)
        self.vel_pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=1)
        self.sub_det = rospy.Subscriber('/detections_output', Detection2DArray, self.detection_callback)
        self.sub_info = rospy.Subscriber('/camera/camera_info', CameraInfo, self.info_cb)
        
        self.current_u = None
        self.current_v = None

        self.theta = self.model.theta
        self.theta_num = np.asarray(self.theta.subs(self.model.params_list))

        #rospy.Timer(rospy.Duration(0.05), self.control_loop)

    def info_cb(self, msg):
        self.image_height = msg.height 
        self.image_width = msg.width

        # extract camera intrinsics
        intrinsics = msg.K

        # get the focal length
        self.focal_length_x = intrinsics[0]
        self.focal_length_x = intrinsics[4]

        # get camera center
        self.cx = intrinsics[2]
        self.cy = intrinsics[5]

    def depth_callback(self, msg):
        # 1. Convert ROS image to NumPy
        # '16UC1' is common for depth (mm), '32FC1' is for meters
        raw_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # 2. Handle NaNs/Infs if using 32FC1
        if raw_depth.dtype == np.float32:
            raw_depth = np.nan_to_num(raw_depth, nan=0.0, posinf=0.0, neginf=0.0)

        # 3. Apply Median Filter
        # kernel size 5 is a good balance. It must be an odd integer.
        # This fills in small holes and removes outliers.
        filtered_depth = cv2.medianBlur(raw_depth, 5)

        # 4. (Optional) Hole Filling / Dilation
        # If you still have black holes (0s), a small dilation can help
        mask = (filtered_depth == 0).astype(np.uint8)
        filtered_depth = cv2.inpaint(filtered_depth, mask, 3, cv2.INPAINT_TELEA)

        self.latest_depth_map = filtered_depth

    def detection_callback(self, msg):
        if msg.detections and self.latest_depth_map is not None:
            det = msg.detections[0]
            cx = det.bbox.center.x
            cy = det.bbox.center.y
            w  = det.bbox.size_x
            h  = det.bbox.size_y

            # 1. Calculate 4 corners in pixel coordinates
            corners_px = [
                (cx - w/2, cy - h/2), # Top-Left
                (cx + w/2, cy - h/2), # Top-Right
                (cx + w/2, cy + h/2), # Bottom-Right
                (cx - w/2, cy + h/2)  # Bottom-Left
            ]

            self.current_features = []
            self.current_depths = []

            for (u, v) in corners_px:
                # Ensure pixel coordinates are within image bounds to avoid crashes
                u_int = int(max(0, min(u, self.latest_depth_map.shape[1] - 1)))
                v_int = int(max(0, min(v, self.latest_depth_map.shape[0] - 1)))

                # 2. Extract Depth (Z)
                # Note: If depth is in mm (16UC1), divide by 1000.0 for meters
                z = self.latest_depth_map[v_int, u_int] / 1000.0 
                
                # Handle invalid depth (0.0 or NaN)
                if z <= 0 or np.isnan(z):
                    z = 1.0 # Fallback estimate or use previous valid depth

                # 3. Normalize u,v for the Jacobian
                norm_u = (u - self.cx) / self.focal_length_x
                norm_v = (v - self.cy) / self.focal_length_y

                self.current_features.extend([norm_u, norm_v])
                self.current_depths.append(z)

            self.current_features = np.array(self.current_features)

    def get_stacked_jacobian(self, features, depths):
        L_stacked = []
        for i in range(4):
            u, v = features[2*i], features[2*i+1]
            z = depths[i]
            
            # Form the 2x6 matrix for this specific corner
            L_i = np.array([
                [-1/z,  0,    u/z,  u*v,      -(1+u**2),  v],
                [ 0,   -1/z,  v/z,  (1+v**2),  -u*v,     -u]
            ])
            L_stacked.append(L_i)
        return np.vstack(L_stacked)
    
    def update(self, joint_states: np.ndarray, t: rospy.Time, dt: rospy.Duration) -> np.ndarray:
        """Update the controller

        Args:
            joint_states (np.ndarray): joint states from robot feedback [q, qdot, qddot] (3 x n)
            t (rospy.Time): current ros time
            dt (rospy.Duration): time interval

        Returns:
            np.ndarray: torque commands (n x 1)
        """
        # Calculate elapsed time
        #t_elapsed = t.to_sec() - self.t_start.to_sec()

        # TODO: update trajectory
        joint_states_des = self.trajectory(self.state, t)

        # TODO: update controller and return torque commands
        tau = np.zeros((self.model.nq, 1))
        tau = self.error_space_controller(joint_states, joint_states_des, dt=dt)

        return tau

    def visual_servoing_controller(self, joint_states: np.ndarray, joint_states_des: np.ndarray, dt: rospy.Duration) -> np.ndarray:

        if self.current_features is None or self.current_depths is None:
            return
        
        q = joint_states[0, :].reshape(-1, 1)
        qdot = joint_states[1, :].reshape(-1, 1)
        qddot = joint_states[2, :].reshape(-1, 1)

        tau = np.zeros((self.model.nq, 1))

        # 1. Image Error (8x1)
        error = self.current_features - self.s_star 

        # 2. Get Stacked Jacobian with individual Z values
        # self.current_depths is a list: [z1, z2, z3, z4]
        L = self.get_stacked_jacobian(self.current_features, self.current_depths)
        
        # 3. Robot Jacobian and Control Law
        # TODO: Get Robot Jacobain
        J_robot = self.model.J_ee_w_fn(*q)
        J_total = np.matmul(L, J_robot)
        
        # Compute joint velocities
        qdot_r = -self.lmbda * np.matmul(np.linalg.pinv(J_total), error)

        q_r = qdot * dt

        self.Sr = qdot - qdot_r

        Yr_evaluated = self.model.Yr_fn(q.flatten(), 
                                        qdot.flatten(), 
                                        self.qdot_r.flatten(), 
                                        self.qddot_r.flatten())
        
        Yr_evaluated_np = np.asarray(Yr_evaluated)

        tau = -self.Kd @ self.Sr + (Yr_evaluated_np @ self.theta_num)
        tau = np.array(tau).astype(np.float64).reshape(3, 1)

        # TODO: Calculate Lyapunov function and publish
        M_mat = self.model.M_fn(*q.flatten())
        V = 0.5 * self.Sr.T @ M_mat @ self.Sr # + 0.5 * delta_q.T @ V_Kp @ delta_q
        V_msg = Float32()
        V_msg.data = float(V.item())
        self.lyapunov_pub.publish(V_msg)

        # TODO: Calculate steady state error and publish
        delta_q = q - q_r
        err = delta_q
        steady_state_err_msg = Float32MultiArray()
        steady_state_err_msg.data = err.flatten().tolist()
        self.steady_state_error_pub.publish(steady_state_err_msg)



def main():

    model = UR10Model3DOF()
    robot = UR10Robot3DOF(model, q_init=np.array([0, 0, 0.01]))
    #controller = UR10Controller(model, q_init=np.array([0, 0, 0.01]))
    visual_controller = JointIBVS(model)

    rate = rospy.Rate(100)
    t = rospy.Time.now()
    t_prev = t

    while not rospy.is_shutdown():
        t = rospy.Time.now()
        dt = t - t_prev
        t_prev = t
        
        # TODO: Get current joint states from robot
        joint_states = robot.get_joint_states()

        # TODO: Update controller with joint states feedback
        tau = visual_controller.update(joint_states, t, dt)

        # TODO: Update robot dynamics with computed torque commands
        #tau = np.zeros((model.nq, 1))
        robot.update_dynamics(tau, dt)

        # TODO: publich joint states and TF frames from robot class
        robot.update_kinematics()
        robot.publish_joint_states()

        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('ur10_visual_control_node')
        main()
    except rospy.ROSInterruptException:
        pass