#!/usr/bin/env python
import rospy
import numpy as np
import tf2_ros
import json
import os
import tf.transformations as tr
from geometry_msgs.msg import PoseStamped, TransformStamped

class AutoCalibrator:
    def __init__(self):
        rospy.init_node('auto_camera_calibrator')
        
        # Parameters
        self.target_samples = rospy.get_param("~samples", 100)
        self.base_frame = rospy.get_param("~base_frame", "ursa_base_link")
        self.camera_frame = rospy.get_param("~camera_frame", "camera_link")
        self.optical_frame = rospy.get_param("~color_optical_frame", "camera_color_optical_frame")
        self.fixed_marker_frame = rospy.get_param("~fixed_marker_frame", "physical_marker_frame")
        
        self.use_saved = rospy.get_param("~use_saved", False)
        self.save_path = os.path.expanduser("~/.ros/camera_to_marker_calibration.json")
        
        self.is_calibrated = False
        self.camera_link_in_base_tf = None
        self.br = tf2_ros.StaticTransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        if self.use_saved and os.path.exists(self.save_path):
            self.load_calibration()
        else:
            rospy.loginfo("Auto-Calibration: Collecting samples from Optical Frame...")
            self.samples_matrices = []
            # Subscriber to ArUco pose (which is Marker relative to Optical Frame)
            self.sub = rospy.Subscriber("/aruco_single/pose", PoseStamped, self.pose_callback)

    def pose_callback(self, msg):
        if self.is_calibrated: return
        
        # 1. Marker relative to Optical Frame
        p, q = msg.pose.position, msg.pose.orientation
        marker_in_optical_m = tr.concatenate_matrices(
            tr.translation_matrix([p.x, p.y, p.z]),
            tr.quaternion_matrix([q.x, q.y, q.z, q.w])
        )

        # 2. Invert to get Optical relative to Marker
        optical_in_marker_m = tr.inverse_matrix(marker_in_optical_m)
        self.samples_matrices.append(optical_in_marker_m)
        
        if len(self.samples_matrices) % 10 == 0:
            rospy.loginfo("Progress: %d/%d", len(self.samples_matrices), self.target_samples)
        if len(self.samples_matrices) >= self.target_samples:
            self.finalize_calibration()

    def finalize_calibration(self):
        avg_matrix = np.mean(self.samples_matrices, axis=0)
        # Re-orthogonalize rotation
        u, _, vh = np.linalg.svd(avg_matrix[:3, :3])
        avg_matrix[:3, :3] = np.dot(u, vh)

        pos = tr.translation_from_matrix(avg_matrix).tolist()
        quat = tr.quaternion_from_matrix(avg_matrix).tolist()
        
        with open(self.save_path, 'w') as f:
            json.dump({'opt_in_mark_pos': pos, 'opt_in_mark_quat': quat}, f)
        
        self.compute_final_camera_pose(avg_matrix)
        self.is_calibrated = True
        self.sub.unregister()
        rospy.loginfo("SUCCESS: Calibration complete.")

    def load_calibration(self):
        try:
            with open(self.save_path, 'r') as f:
                data = json.load(f)
            m = tr.concatenate_matrices(
                tr.translation_matrix(data['opt_in_mark_pos']),
                tr.quaternion_matrix(data['opt_in_mark_quat'])
            )
            self.compute_final_camera_pose(m)
            self.is_calibrated = True
            rospy.loginfo("Loaded saved calibration.")
        except Exception as e:
            rospy.logerr("Load failed: %s", e)

    def compute_final_camera_pose(self, optical_in_marker_m):
        try:
            # A. Get Static Transform: Link -> Optical
            # We need this to find where the Link is if we know where the Optical is
            l_to_o_tf = self.tf_buffer.lookup_transform(
                self.camera_frame, self.optical_frame, rospy.Time(0), rospy.Duration(2.0))
            
            link_to_optical_m = tr.concatenate_matrices(
                tr.translation_matrix([l_to_o_tf.transform.translation.x, 
                                       l_to_o_tf.transform.translation.y, 
                                       l_to_o_tf.transform.translation.z]),
                tr.quaternion_matrix([l_to_o_tf.transform.rotation.x, 
                                      l_to_o_tf.transform.rotation.y, 
                                      l_to_o_tf.transform.rotation.z, 
                                      l_to_o_tf.transform.rotation.w])
            )

            # B. Calculate Link relative to Marker
            # Link_in_Marker = Optical_in_Marker * inv(Link_to_Optical)
            optical_to_link_m = tr.inverse_matrix(link_to_optical_m)
            link_in_marker_m = np.dot(optical_in_marker_m, optical_to_link_m)

            # C. Get World position of the Physical Marker
            world_to_mark_tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.fixed_marker_frame, rospy.Time(0), rospy.Duration(2.0))
            
            base_to_marker_m = tr.concatenate_matrices(
                tr.translation_matrix([world_to_mark_tf.transform.translation.x, 
                                       world_to_mark_tf.transform.translation.y, 
                                       world_to_mark_tf.transform.translation.z]),
                tr.quaternion_matrix([world_to_mark_tf.transform.rotation.x, 
                                      world_to_mark_tf.transform.rotation.y, 
                                      world_to_mark_tf.transform.rotation.z, 
                                      world_to_mark_tf.transform.rotation.w])
            )

            # D. Final Transform: Base -> Link
            final_m = np.dot(base_to_marker_m, link_in_marker_m)
            
            t = TransformStamped()
            t.header.frame_id = self.base_frame
            t.child_frame_id = self.camera_frame
            
            trans = tr.translation_from_matrix(final_m)
            rot = tr.quaternion_from_matrix(final_m)
            
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = trans
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = rot
            self.camera_link_in_base_tf = t
            
        except Exception as e:
            rospy.logerr("TF Chain Computation failed: %s", e)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.is_calibrated and self.camera_link_in_base_tf:
                self.camera_link_in_base_tf.header.stamp = rospy.Time.now()
                self.br.sendTransform(self.camera_link_in_base_tf)
            rate.sleep()

if __name__ == '__main__':
    AutoCalibrator().run()