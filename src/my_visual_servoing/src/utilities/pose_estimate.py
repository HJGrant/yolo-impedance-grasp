import numpy as np
import scipy.spatial.transform as st
import open3d as o3d 
from visualization_msgs.msg import Marker
import rospy
import pyransac3d as pyrsc

class BottlePoseEstimator:
    def __init__(self, alpha_pos=0.2, alpha_ori=0.1):
        # Filtering states
        self.prev_c = None
        self.prev_quat = None
        self.alpha_pos = alpha_pos
        self.alpha_ori = alpha_ori

        # Parameters for the Marker
        self.current_radius = 0.04 # Default fallback
        self.BOTTLE_HEIGHT = 0.26    # Constant for visualization

    def estimate_pose(self, points, principal_axis='z'):
        """
        Fits a cylinder using RANSAC and filters the resulting pose.
        """
        if points is None or points.shape[0] < 50:
            return None, None

        # 1. RANSAC CYLINDER FITTING
        cylinder = pyrsc.Cylinder()
        try:
            center, axis, radius, inliers_idx = cylinder.fit(points, thresh=0.01, maxIteration=100)
            # Store the fitted radius for the marker
            self.current_radius = radius 
        except Exception as e:
            rospy.logwarn(f"RANSAC Fitting failed: {e}")
            return None, None

        # 2. EXTRACT GEOMETRY
        v_main = np.array(axis)
        c_refined = np.array(center)

        # 3. DIRECTION CONSISTENCY
        if np.dot(v_main, np.array([0, 0, 1])) < 0:
            v_main = -v_main

        # 4. ORTHONORMAL BASIS CONSTRUCTION
        up_ref = np.array([0.0, -1.0, 0.0]) 
        if abs(np.dot(v_main, up_ref)) > 0.9:
            up_ref = np.array([1.0, 0.0, 0.0])

        if principal_axis.lower() == 'z':
            z_axis = v_main
            x_axis = np.cross(up_ref, z_axis)
            x_axis /= (np.linalg.norm(x_axis) + 1e-12)
            y_axis = np.cross(z_axis, x_axis)
            y_axis /= (np.linalg.norm(y_axis) + 1e-12)
        else:
            x_axis = v_main
            z_axis = np.cross(x_axis, up_ref)
            z_axis /= (np.linalg.norm(z_axis) + 1e-12)
            y_axis = np.cross(z_axis, x_axis)
            y_axis /= (np.linalg.norm(y_axis) + 1e-12)

        Rm = np.column_stack((x_axis, y_axis, z_axis))
        q_meas = st.Rotation.from_matrix(Rm).as_quat()

        # 5. FILTERING (EMA and SLERP)
        if self.prev_c is None:
            self.prev_c, self.prev_quat = c_refined, q_meas
        else:
            self.prev_c = self.alpha_pos * c_refined + (1 - self.alpha_pos) * self.prev_c
            if np.dot(self.prev_quat, q_meas) < 0:
                q_meas = -q_meas
            slerp = st.Slerp([0, 1], st.Rotation.from_quat([self.prev_quat, q_meas]))
            self.prev_quat = slerp([self.alpha_ori]).as_quat()[0]

        return self.prev_c, self.prev_quat

    def get_rviz_marker(self, frame_id="camera_rgb_optical_frame"):
        """
        Uses the stored filtered position, filtered orientation, 
        and the last fitted radius to build the marker.
        """
        if self.prev_c is None: return None
        
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "fitted_bottle"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # Position (Filtered)
        marker.pose.position.x = self.prev_c[0]
        marker.pose.position.y = self.prev_c[1]
        marker.pose.position.z = self.prev_c[2]
        
        # Orientation (Filtered)
        marker.pose.orientation.x = self.prev_quat[0]
        marker.pose.orientation.y = self.prev_quat[1]
        marker.pose.orientation.z = self.prev_quat[2]
        marker.pose.orientation.w = self.prev_quat[3]
        
        # Scale (using the radius returned by RANSAC)
        # Note: scale.x and y are diameter, scale.z is height
        marker.scale.x = self.current_radius * 2
        marker.scale.y = self.current_radius * 2
        marker.scale.z = self.BOTTLE_HEIGHT
        
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.0, 1.0, 0.0, 0.6)
        return marker