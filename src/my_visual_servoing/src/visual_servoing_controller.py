#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class JointIBVS:
    def __init__(self):
        rospy.init_node('joint_ibvs_node')

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

        rospy.Timer(rospy.Duration(0.05), self.control_loop)

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

    def get_robot_jacobian(self):
        """
        Placeholder for the Robot's Geometric Jacobian.
        In a real app, you'd use URDF/KDL to compute this based on current joint angles.
        Maps Joint Vel (q_dot) -> Camera Vel (v, w)
        """
        # Example: 6-DOF Robot Jacobian (6 rows, N joints)
        return np.eye(6) 

    def control_loop(self, event):
        def control_loop(self, event):
        if self.current_features is None or self.current_depths is None:
            return

        # 1. Image Error (8x1)
        error = self.current_features - self.s_star 

        # 2. Get Stacked Jacobian with individual Z values
        # self.current_depths is a list: [z1, z2, z3, z4]
        L = self.get_stacked_jacobian(self.current_features, self.current_depths)
        
        # 3. Robot Jacobian and Control Law
        # TODO: Get Robot Jacobain
        J_robot = self.get_robot_jacobian()
        J_total = np.matmul(L, J_robot)
        
        # Compute joint velocities
        q_dot = -self.lmbda * np.matmul(np.linalg.pinv(J_total), error)

        # 4. Publish


if __name__ == '__main__':
    try:
        JointIBVS()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass