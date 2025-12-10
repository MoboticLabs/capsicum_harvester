#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import os

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.publisher_ = self.create_publisher(PoseStamped, '/target_pose', 10)
        
        # Realsense Setup
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        
        self.pipeline = rs.pipeline()
        self.profile = self.pipeline.start(self.config)
        
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        
        self.intr = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        
        # YOLO Setup
        model_path = '/root/ros2_ws/src/capsicum_harvester/detection/last_capsv8s_seg.pt'
        self.get_logger().info(f'Loading model from: {model_path}')
        self.model = YOLO(model_path)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Detection Node Started')

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        
        if not color_frame or not depth_frame:
            return
            
        img = np.asanyarray(color_frame.get_data())
        
        # Inference
        results = self.model(img, verbose=False)
        
        for result in results:
            boxes = result.boxes
            for box in boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                name = result.names[cls_id]
                
                if name == 'capsicum' and conf >= 0.89:
                    # Bounding box coordinates
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    
                    # Center point
                    x_center = int((x1 + x2) / 2)
                    y_center = int((y1 + y2) / 2)
                    
                    # Depth calculation logic from test_realsense_v8.py
                    # get_distance returns meters
                    dist_m = depth_frame.get_distance(x_center + 4, y_center + 8)
                    dist_mm = dist_m * 1000
                    
                    if dist_mm <= 0:
                        continue

                    # Coordinate transformation from original script (results in mm)
                    # Xtarget = (dist*(x+4 - intr.ppx)/intr.fx - 35) 
                    # Ytarget = (dist*(y+8 - intr.ppy)/intr.fy)
                    # Ztarget = dist
                    
                    Xtarget_mm = (dist_mm * (x_center + 4 - self.intr.ppx) / self.intr.fx) - 35
                    Ytarget_mm = (dist_mm * (y_center + 8 - self.intr.ppy) / self.intr.fy)
                    Ztarget_mm = dist_mm
                    
                    # Convert back to meters for ROS
                    x_ros = -1*(Xtarget_mm / 1000.0)
                    z_ros = -1*(Ytarget_mm / 1000.0)
                    y_ros = -1*(Ztarget_mm / 1000.0)
                    
                    msg = PoseStamped()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'link1' 
                    
                    msg.pose.position.x = x_ros
                    msg.pose.position.y = y_ros
                    msg.pose.position.z = z_ros
                    msg.pose.orientation.w = 1.0
                    
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Published target: {name} at ({x_ros:.3f}, {y_ros:.3f}, {z_ros:.3f})')

    def __del__(self):
        try:
            self.pipeline.stop()
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
