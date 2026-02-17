#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point, PoseStamped
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header 
from visualization_msgs.msg import Marker

import torch
from ultralytics import YOLO


class YOLOv8:
    def __init__(self, mode):
        rospy.init_node('yolo_node')
        self.bridge = CvBridge()

        if mode == 'seg':
            self.model = YOLO('yolov8n-seg.pt').to('cuda')
        else:
            self.model = YOLO('yolov8n.pt').to('cuda')

        self.sub_rgb = rospy.Subscriber('/camera/rgb/image_rect_color', Image, self.image_data_callback)
        self.sub_depth = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_map_callback)
        self.cam_info_sub = rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.cam_info_cb)

        self.bbox_pub = rospy.Publisher('/object_centroid', Detection2DArray, queue_size=10)
        self.debug_pub = rospy.Publisher('/rgb_debug', Image, queue_size=10)
        self.pc_pub = rospy.Publisher('/target_object_cloud', PointCloud2, queue_size=1)

        self.coord_pub = rospy.Publisher('/target_point', Point, queue_size=1)
        self.pose_pub = rospy.Publisher('/target_pose', PoseStamped, queue_size=1)

        self.marker_pub = rospy.Publisher('/centroid_marker', Marker, queue_size=1)

        self.K = None
        self.current_depth_map = None

        self.v1_prev = None

    def depth_map_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.current_depth_map = cv_image

    def cam_info_cb(self, msg):
        self.K = msg.K
        self.cam_info_sub.unregister()

    def rotmat_to_quat_xyzw(self, R):
        t = np.trace(R)
        if t > 0.0:
            S = np.sqrt(t + 1.0) * 2.0
            qw = 0.25 * S
            qx = (R[2,1] - R[1,2]) / S
            qy = (R[0,2] - R[2,0]) / S
            qz = (R[1,0] - R[0,1]) / S
        else:
            if (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
                S = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2.0
                qw = (R[2,1] - R[1,2]) / S
                qx = 0.25 * S
                qy = (R[0,1] + R[1,0]) / S
                qz = (R[0,2] + R[2,0]) / S
            elif R[1,1] > R[2,2]:
                S = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2.0
                qw = (R[0,2] - R[2,0]) / S
                qx = (R[0,1] + R[1,0]) / S
                qy = 0.25 * S
                qz = (R[1,2] + R[2,1]) / S
            else:
                S = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2.0
                qw = (R[1,0] - R[0,1]) / S
                qx = (R[0,2] + R[2,0]) / S
                qy = (R[1,2] + R[2,1]) / S
                qz = 0.25 * S

        q = np.array([qx, qy, qz, qw], dtype=np.float64)
        q /= np.linalg.norm(q) + 1e-12
        return q
    
    #  if using x_axis up
    def pca_pose_from_points(self, points, v_prev=None, principal_axis='x'):
        """
        Computes pose from PCA.
        principal_axis: 'x' or 'z' - which axis the main eigenvector aligns with.
        """
        if points is None or points.shape[0] < 50:
            return None, None, None

        c = points.mean(axis=0)
        Q = points - c
        C = (Q.T @ Q) / max(points.shape[0], 1)

        # Solve for eigenvectors
        w, V = np.linalg.eigh(C)
        v1 = V[:, np.argmax(w)]  # Main principal component

        # Continuity with previous frame
        if v_prev is not None and np.dot(v1, v_prev) < 0:
            v1 = -v1

        # Global orientation consistency (keep it pointing generally "up" or "forward")
        if np.dot(v1, np.array([0.0, 0.0, 1.0])) < 0:
            v1 = -v1

        # Normalize the principal vector
        v_main = v1 / (np.linalg.norm(v1) + 1e-12)

        # Reference vector to define the "up" direction for the remaining axes
        # Typically, we want the y-axis or z-axis to stay as close to global up/down as possible
        up_ref = np.array([0.0, -1.0, 0.0]) 

        if principal_axis.lower() == 'x':
            x_axis = v_main
            # Project up_ref onto the plane orthogonal to x_axis to find z
            z_axis = up_ref - np.dot(up_ref, x_axis) * x_axis
            if np.linalg.norm(z_axis) < 1e-6:
                z_axis = np.array([0.0, 0.0, 1.0]) - np.dot(np.array([0.0, 0.0, 1.0]), x_axis) * x_axis
            
            z_axis /= (np.linalg.norm(z_axis) + 1e-12)
            y_axis = np.cross(z_axis, x_axis)
            y_axis /= (np.linalg.norm(y_axis) + 1e-12)

        elif principal_axis.lower() == 'z':
            z_axis = v_main
            # Project up_ref (or a forward ref) onto the plane orthogonal to z_axis to find x
            x_ref = np.array([1.0, 0.0, 0.0])
            x_axis = x_ref - np.dot(x_ref, z_axis) * z_axis
            if np.linalg.norm(x_axis) < 1e-6:
                x_axis = np.array([0.0, 1.0, 0.0]) - np.dot(np.array([0.0, 1.0, 0.0]), z_axis) * z_axis
                
            x_axis /= (np.linalg.norm(x_axis) + 1e-12)
            y_axis = np.cross(z_axis, x_axis)
            y_axis /= (np.linalg.norm(y_axis) + 1e-12)
        else:
            raise ValueError("principal_axis must be 'x' or 'z'")

        # Build Rotation Matrix: columns are x, y, z
        Rm = np.column_stack((x_axis, y_axis, z_axis))
        
        quat = self.rotmat_to_quat_xyzw(Rm)
        return c, quat, v1


    def image_data_callback(self, msg):
        if self.K is None:
            rospy.logwarn_throttle(5, "Waiting for camera_info to get intrinsic K...")
            return
        # 1. Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 2. Run Inference
        results = self.model.predict(source=cv_image, device='cuda', verbose=False)

        annotated_frame = results[0].plot()

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.debug_pub.publish(debug_msg)
        except Exception as e:
            rospy.logerr(f"Failed to publish debug image: {e}")

        res = results[0]

        target_class_id = 39 # Example: 'cup'. Change as needed. 39
        best_mask = None

        if res.masks is not None:
            # 1. Get classes and confidence scores as numpy arrays
            clss = res.boxes.cls.cpu().numpy()
            confs = res.boxes.conf.cpu().numpy()

            # 2. Find indices that match our target class
            target_indices = np.where(clss == target_class_id)[0]

            if len(target_indices) > 0:
                # 3. From those target indices, find the one with the highest confidence
                best_idx = target_indices[np.argmax(confs[target_indices])]

                # 4. Extract the mask for that specific index
                best_mask = res.masks.data[best_idx].cpu().numpy().astype(bool)

                rospy.loginfo(f"Target found! Conf: {confs[best_idx]:.2f}")


        # In your callback, after getting best_mask:
        if best_mask is not None and self.current_depth_map is not None:
            cloud_msg, centroid, points = self.get_masked_cloud(best_mask, self.current_depth_map)

            # If filtering in get_masked_cloud returns None, skip publishing
            if cloud_msg is None:
                return

            # Create and publish messages
            point_msg = Point()
            point_msg.x = centroid[0]
            point_msg.y = centroid[1]
            point_msg.z = centroid[2]
            self.coord_pub.publish(point_msg)
            self.pc_pub.publish(cloud_msg)
            self.publish_marker(centroid)

            centroid_np, quat_xyzw, v1 = self.pca_pose_from_points(points, v_prev=self.v1_prev)
            if centroid_np is not None:
                self.v1_prev = v1

                pose_msg = PoseStamped()
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.header.frame_id = "camera_rgb_optical_frame"

                pose_msg.pose.position.x = float(centroid_np[0])
                pose_msg.pose.position.y = float(centroid_np[1])
                pose_msg.pose.position.z = float(centroid_np[2])

                pose_msg.pose.orientation.x = float(quat_xyzw[0])
                pose_msg.pose.orientation.y = float(quat_xyzw[1])
                pose_msg.pose.orientation.z = float(quat_xyzw[2])
                pose_msg.pose.orientation.w = float(quat_xyzw[3])

                self.pose_pub.publish(pose_msg)

        else:
            # Specific logging to see WHICH one is missing
            if self.current_depth_map is None:
                rospy.logwarn_throttle(5, "Waiting for depth map on /camera/depth/image_rect_raw...")
            if best_mask is None:
                rospy.loginfo_throttle(5, f"Target class {target_class_id} not detected in RGB frame.")


    def get_masked_cloud(self, mask, depth_map):
        # 1. Get indices where mask is True
        v_indices, u_indices = np.where(mask)
        z_values = depth_map[v_indices, u_indices] / 1000.0 

        # 2. Filter out invalid depth (0)
        z = z_values[z_values > 0]
        u = u_indices[z_values > 0]
        v = v_indices[z_values > 0]

        # --- ADVANCED FOREGROUND FILTER ---
        # 1. Find the median depth (most likely the bottle surface)
        z_median = np.median(z)

        # 2. Calculate how far every point is from that median
        abs_deviation = np.abs(z - z_median)
        m_dev = np.median(abs_deviation)

        # 3. Filter: Only keep points within a tight window of the median
        # (0.1 means +/- 10cm. Adjust based on your bottle's thickness)
        tight_threshold = 0.2
        inlier_mask = abs_deviation < tight_threshold

        z = z[inlier_mask]
        u = u[inlier_mask]
        v = v[inlier_mask]

        if len(z) == 0:
            rospy.logwarn("Points Outside of Threshold!")
            return None, None, None

        workspace_mask = (z > 0.1) & (z < 1.5)
        z = z[workspace_mask]
        u = u[workspace_mask]
        v = v[workspace_mask]

        if len(z) == 0:
            rospy.logwarn("Points Outside of Workspace!")
            return None, None, None

        # 3. Project to 3D using pinhole camera equations
        fx, cx = self.K[0], self.K[2]
        fy, cy = self.K[4], self.K[5]

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        # 4. Calculate 3D Centroid
        centroid = [np.mean(x), np.mean(y), np.mean(z)]

        # 5. Create PointCloud2 data
        points = np.vstack((x, y, z)).T
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "camera_rgb_optical_frame" # Adjust to your frame

        cloud_msg = pc2.create_cloud_xyz32(header, points)

        return cloud_msg, centroid, points

    def publish_marker(self, centroid):
        marker = Marker()
        marker.header.frame_id = "camera_rgb_optical_frame" # Must match your cloud frame
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Set the 3D position
        marker.pose.position.x = centroid[0]
        marker.pose.position.y = centroid[1]
        marker.pose.position.z = centroid[2]

        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)


if __name__ == '__main__':
    try:
        yolo_node = YOLOv8(mode='seg')
        rospy.loginfo("YOLOv8 Segmentation Node Started")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
