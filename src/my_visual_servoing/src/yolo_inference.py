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
from geometry_msgs.msg import Point
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
        self.marker_pub = rospy.Publisher('/centroid_marker', Marker, queue_size=1)
        
        self.K = None

        self.current_depth_map = None

    def depth_map_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.current_depth_map = cv_image

    def cam_info_cb(self, msg):
        # K is [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.K = msg.K
        self.cam_info_sub.unregister() # Only need this once

    def image_data_callback(self, msg):
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

        target_class_id = 39 # Example: 'cup'. Change as needed.
        best_mask = None

        if res.masks is not None:
            # 1. Get classes and confidence scores as numpy arrays
            clss = res.boxes.cls.cpu().numpy()
            confs = res.boxes.conf.cpu().numpy()

            # 2. Find indices that match our target class
            target_indices = np.where(clss == target_class_id)[0]

            if len(target_indices) > 0:
                # 3. From those target indices, find the one with the highest confidence
                # target_indices[np.argmax(...)] gives us the absolute index in the original list
                best_idx = target_indices[np.argmax(confs[target_indices])]
                
                # 4. Extract the mask for that specific index
                # .data[best_idx] is the binary mask on the GPU; move to CPU and convert to bool
                best_mask = res.masks.data[best_idx].cpu().numpy().astype(bool)
                
                rospy.loginfo(f"Target found! Conf: {confs[best_idx]:.2f}")


        # In your callback, after getting best_mask:
        if best_mask is not None and self.current_depth_map is not None:
            
            cloud_msg, centroid = self.get_masked_cloud(best_mask, self.current_depth_map)

            # Create and publish message
            point_msg = Point()
            point_msg.x = centroid[0] # Horizontal pixel
            point_msg.y = centroid[1] # Vertical pixel
            point_msg.z = centroid[2] # Distance in meters
            self.coord_pub.publish(point_msg) 

            self.pc_pub.publish(cloud_msg)

            self.publish_marker(centroid)
        elif not self.current_depth_map:
            rospy.loginfo(f"No depth map available. Not publishing target..")
        elif not best_mask:
            rospy.loginfo(f"No best_mask. Not publishing target..")
    
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
        inlier_mask = m_dev < tight_threshold
        
        z = z[inlier_mask]
        u = u[inlier_mask]
        v = v[inlier_mask]

        if len(z) == 0:
            rospy.logwarn("Points Outside of Threshold!")
            return None, None

        workspace_mask = (z > 0.1) & (z < 1.5)
        z = z[workspace_mask]
        u = u[workspace_mask]
        v = v[workspace_mask]

        if len(z) == 0:
            rospy.logwarn("Points Outside of Workspace!")
            return None, None

        # 3. Project to 3D using pinhole camera equations
        # X = (u - cx) * Z / fx
        # Y = (v - cy) * Z / fy
        fx, cx = self.K[0], self.K[2]
        fy, cy = self.K[4], self.K[5]

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        # 4. Calculate 3D Centroid
        centroid = [np.mean(x), np.mean(y), np.mean(z)]

        # 5. Create PointCloud2 data (list of [x, y, z] tuples)
        points = np.vstack((x, y, z)).T
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "camera_rgb_optical_frame" # Adjust to your frame
        
        cloud_msg = pc2.create_cloud_xyz32(header, points)
        
        return cloud_msg, centroid

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
        
        # Scale: 5cm sphere
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        
        # Color: Bright Green (don't forget A=1.0 for visibility!)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0 
        
        self.marker_pub.publish(marker)

if __name__ == '__main__':
    try:
        # 1. Initialize the node (best to do this here rather than inside __init__)
        # If you keep it in __init__, just call the class.
        yolo_node = YOLOv8(mode='seg') 
        
        rospy.loginfo("YOLOv8 Segmentation Node Started")
        
        # 2. Spin to keep the script alive and processing callbacks
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass