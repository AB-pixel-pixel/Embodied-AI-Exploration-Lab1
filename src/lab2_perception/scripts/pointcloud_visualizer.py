#!/usr/bin/env python3
import rospy
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import open3d as o3d
import threading

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
        
        # 线程锁和控制
        self.update_lock = threading.Lock()
        self.need_geometry_update = False
        
        # 性能优化参数
        self.voxel_size = 0.03  # 体素大小（米），可调整：0.02-0.05
        self.frame_skip = 3  # 跳帧数，处理每第N帧
        self.frame_count = 0
        self.max_points = 50000  # 最大点数限制（可选）
        
        rospy.loginfo("PointCloud Visualizer Initialized")
        rospy.loginfo(f"Voxel size: {self.voxel_size}m, Frame skip: {self.frame_skip}")

    def callback(self, rgb_msg, depth_msg):
        """接收图像数据的回调函数"""
        self.frame_count += 1
        
        # 跳帧处理
        if self.frame_count % self.frame_skip != 0:
            return
        
        # 如果正在处理，跳过本帧（非阻塞）
        if self.update_lock.locked():
            return
            
        try:
            # 直接转换为RGB格式，避免后续颜色转换
            self.latest_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "rgb8")
            self.latest_depth = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
            self.has_new_data = True
        except Exception as e:
            rospy.logerr(f"Error processing images: {e}")

    def process_pointcloud(self):
        """在后台线程处理点云生成（耗时操作）"""
        with self.update_lock:
            if not self.has_new_data:
                return
            
            try:
                # 创建Open3D图像对象
                o3d_color = o3d.geometry.Image(self.latest_rgb)
                o3d_depth = o3d.geometry.Image(self.latest_depth)
                
                # 创建RGBD图像
                rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                    o3d_color, o3d_depth,
                    depth_scale=1.0,  # 深度已经是米单位
                    depth_trunc=2.5,  # 只显示2.5米内的点（可调整：2.0-4.0）
                    convert_rgb_to_intensity=False
                )
                
                # 从RGBD图像生成点云
                pcd_new = o3d.geometry.PointCloud.create_from_rgbd_image(
                    rgbd, self.o3d_intrinsic
                )
                
                # 体素降采样（关键优化）
                pcd_new = pcd_new.voxel_down_sample(self.voxel_size)
                
                # 可选：限制最大点数（如果点云仍然太大）
                if len(pcd_new.points) > self.max_points:
                    indices = np.random.choice(
                        len(pcd_new.points), 
                        self.max_points, 
                        replace=False
                    )
                    pcd_new = pcd_new.select_by_index(indices)
                
                # 可选：移除离群点（会稍微变慢，但点云更干净）
                # pcd_new, _ = pcd_new.remove_statistical_outlier(
                #     nb_neighbors=20, 
                #     std_ratio=2.0
                # )
                
                # 更新点云数据
                self.pcd.points = pcd_new.points
                self.pcd.colors = pcd_new.colors
                
                self.has_new_data = False
                self.need_geometry_update = True
                
            except Exception as e:
                rospy.logerr(f"Error creating point cloud: {e}")

    def update(self):
        """主线程渲染循环（轻量级操作）"""
        # 只在有新数据时更新几何体
        if self.need_geometry_update:
            with self.update_lock:
                if self.is_first_frame:
                    self.vis.add_geometry(self.pcd)
                    self.is_first_frame = False
                else:
                    self.vis.update_geometry(self.pcd)
                self.need_geometry_update = False
        
        # 处理交互事件和渲染（必须在主线程）
        self.vis.poll_events()
        self.vis.update_renderer()

    def run(self):
        """启动可视化器"""
        # 启动后台处理线程
        def processing_loop():
            rate = rospy.Rate(5)  # 点云更新频率5Hz（可调整：3-10Hz）
            while not rospy.is_shutdown():
                self.process_pointcloud()
                try:
                    rate.sleep()
                except rospy.ROSInterruptException:
                    break
        
        process_thread = threading.Thread(target=processing_loop, daemon=True)
        process_thread.start()
        
        rospy.loginfo("Starting visualization loop...")
        
        # 主线程只负责渲染，保持高帧率以确保交互流畅
        rate = rospy.Rate(60)  # 渲染帧率60Hz
        while not rospy.is_shutdown():
            self.update()
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break
        
        # 清理
        self.vis.destroy_window()
        rospy.loginfo("Visualizer closed")

if __name__ == '__main__':
    try:
        node = PointCloudVisualizer()
        node.run()
    except rospy.ROSInterruptException:
        pass
