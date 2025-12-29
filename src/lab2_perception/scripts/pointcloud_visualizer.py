#!/usr/bin/env python3
import rospy
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import open3d as o3d
import cv2

class PointCloudVisualizer:
    def __init__(self):
        rospy.init_node('pointcloud_visualizer', anonymous=True)
        
        self.bridge = CvBridge()
        
        # Open3D Visualizer
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="3D Point Cloud", width=800, height=600)
        
        self.pcd = o3d.geometry.PointCloud()
        self.is_first_frame = True
        
        # Camera Intrinsics (TurtleBot3 Waffle Pi / RealSense R200)
        self.width = 640
        self.height = 480
        self.fx = 554.25
        self.fy = 554.25
        self.cx = 320.5
        self.cy = 240.5
        
        # Open3D Intrinsic Object
        self.o3d_intrinsic = o3d.camera.PinholeCameraIntrinsic(
            self.width, self.height, self.fx, self.fy, self.cx, self.cy
        )
        
        # Subscribers with Synchronization
        rgb_sub = message_filters.Subscriber('/camera/rgb/image_raw', Image)
        depth_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
        
        self.ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 10, 0.1)
        self.ts.registerCallback(self.callback)
        
        self.latest_rgb = None
        self.latest_depth = None
        self.has_new_data = False
        
        rospy.loginfo("PointCloud Visualizer Initialized")

    def callback(self, rgb_msg, depth_msg):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            self.latest_depth = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
            self.has_new_data = True
        except Exception as e:
            rospy.logerr(f"Error processing images: {e}")

    def update(self):
        if self.has_new_data and self.latest_rgb is not None and self.latest_depth is not None:
            # Convert OpenCV images to Open3D images
            # Open3D expects RGB, OpenCV is BGR
            rgb = cv2.cvtColor(self.latest_rgb, cv2.COLOR_BGR2RGB)
            
            # Create Open3D Image objects
            o3d_color = o3d.geometry.Image(rgb)
            o3d_depth = o3d.geometry.Image(self.latest_depth)
            
            # Create RGBD Image
            # convert_rgb_to_intensity=False ensures color is preserved
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                o3d_color, o3d_depth, 
                depth_scale=1.0, # Data is already in meters if using 32FC1 usually, but check. 
                               # If standard uint16 mm, scale is 1000. 
                               # ROS depth 32FC1 is usually meters. Open3D default depth_trunc is 3.0.
                depth_trunc=3.0, 
                convert_rgb_to_intensity=False
            )
            
            # Generate Point Cloud
            pcd_new = o3d.geometry.PointCloud.create_from_rgbd_image(
                rgbd, self.o3d_intrinsic
            )
            
            # Coordinate frame transformation (ROS to Open3D/Computer Vision)
            # ROS Camera: X Right, Y Down, Z Forward
            # Open3D: Same usually.
            # However, usually we want to flip to see it upright if needed. 
            # For now, let's keep it as is.
            # pcd_new.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
            
            self.pcd.points = pcd_new.points
            self.pcd.colors = pcd_new.colors
            
            if self.is_first_frame:
                self.vis.add_geometry(self.pcd)
                self.is_first_frame = False
            else:
                self.vis.update_geometry(self.pcd)
            
            self.has_new_data = False

        self.vis.poll_events()
        self.vis.update_renderer()

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()
        self.vis.destroy_window()

if __name__ == '__main__':
    node = PointCloudVisualizer()
    node.run()
