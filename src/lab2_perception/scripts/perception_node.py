#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PointStamped
from cv_bridge import CvBridge
from lab2_perception.msg import ObjectCoordinates
import tf2_ros
import tf2_geometry_msgs
from lab2_perception.cfg import PerceptionHSVConfig
from dynamic_reconfigure.server import Server

from sensor_msgs.msg import Image, CameraInfo # 添加 CameraInfo

from enum import Enum

class RobotState(Enum):
    SEARCHING = 0      # 搜索目标
    ALIGNING = 1       # 对准目标(旋转调整)
    APPROACHING = 2    # 接近目标(前进)
    REACHED = 3        # 到达目标


class PerceptionNode:
    def __init__(self):
        rospy.init_node('perception_node')
        
        self.bridge = CvBridge()
        self.latest_depth = None
        self.depth_colormap = None
               
        # # 获取HSV范围参数
        # self.lower_green = rospy.get_param('~lower_green', [0, 120, 70])  # 默认值 [0, 120, 70]
        # self.upper_green = rospy.get_param('~upper_green', [10, 255, 255])  # 默认值 [10, 255, 255]
        # 使用动态重新配置
        self.hsv = dict()
        self.dr_srv = Server(PerceptionHSVConfig, self.reconfig_cb)
        self.display_scale = rospy.get_param("~display_scale", 0.5)

        # self.client = dynamic_reconfigure.client.Client("/perception_node", timeout=30)
        # self.client.update_configuration({'lower_h': 0, 'lower_s': 120, 'lower_v': 70, 'upper_h': 10, 'upper_s': 255, 'upper_v': 255})

        
        # TF Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscribers
        self.rgb_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
        
        # Publisher
        self.coord_pub = rospy.Publisher('detected_object', ObjectCoordinates, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # 状态机相关
        self.current_state = RobotState.SEARCHING
        self.target_lost_count = 0
        self.max_lost_frames = 10  # 连续丢失多少帧后重新搜索
        
        # 控制参数
        self.align_threshold = 0.05  # 对准阈值(归一化坐标)
        self.distance_threshold = 0.1  # 距离到达阈值(米)
        self.desired_distance = 0.5  # 期望距离(米)
        
        # PID控制参数
        self.angular_kp = 2.0
        self.linear_kp = 0.5
        self.max_angular_speed = 0.5
        self.max_linear_speed = 0.2


        # 初始化内参变量
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        
        # 添加 CameraInfo 订阅者
        self.info_sub = rospy.Subscriber("/camera/rgb/camera_info", CameraInfo, self.camera_info_callback)


        
        rospy.loginfo("Perception Node Started")


    def camera_info_callback(self, msg):
        # K 矩阵是一个 9 个元素的数组
        # [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.fx = msg.K[0]
        self.cx = msg.K[2]
        self.fy = msg.K[4]
        self.cy = msg.K[5]
        
        # 获取一次后取消订阅，避免不必要的开销（假设内参不变）
        self.info_sub.unregister()
        rospy.loginfo(f"Camera Info received: fx={self.fx}, cx={self.cx}")

    def reconfig_cb(self, config, level):
        self.hsv["lower"] = np.array([
            config.lower_h,
            config.lower_s,
            config.lower_v
        ])
        self.hsv["upper"] = np.array([
            config.upper_h,
            config.upper_s,
            config.upper_v
        ])
        self.display_scale = config.display_scale

        rospy.loginfo_throttle(2.0,
            f"HSV updated: {self.hsv['lower']} - {self.hsv['upper']}, "
            f"scale={self.display_scale}"
        )
        return config




    def rgb_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_image(cv_image)
        except Exception as e:
            rospy.logerr(f"RGB error: {e}")

    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            
            # 新增：生成彩色深度图用于可视化
            if self.latest_depth is not None:
                # 归一化深度值到 0-255
                depth_normalized = cv2.normalize(
                    self.latest_depth, 
                    None, 
                    0, 255, 
                    cv2.NORM_MINMAX, 
                    dtype=cv2.CV_8U
                )
                
                # 应用彩色映射：COLORMAP_JET (蓝色=近，红色=远)
                # 其他选项: COLORMAP_TURBO, COLORMAP_RAINBOW, COLORMAP_HOT
                self.depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
                
        except Exception as e:
            rospy.logerr(f"Depth error: {e}")


    def process_image(self, rgb_image):
        # --- 2.1 HSV Color Space Conversion & Detection ---
        hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_image, self.hsv["lower"], self.hsv["upper"])        
        hsv_result = cv2.bitwise_and(rgb_image, rgb_image, mask=mask)
        
        # --- 2.2 Contour Detection ---
        gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray_image, 100, 200)
        contours_canny, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        canny_display = rgb_image.copy()
        cv2.drawContours(canny_display, contours_canny, -1, (0, 255, 0), 3)
        
        # --- Object Detection & State Machine ---
        contours_mask, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        cmd = Twist()  # 默认停止命令
        target_found = False
        
        if contours_mask:
            c = max(contours_mask, key=cv2.contourArea)
            if cv2.contourArea(c) > 100:
                target_found = True
                self.target_lost_count = 0
                
                # Draw contour
                cv2.drawContours(rgb_image, [c], -1, (0, 0, 255), 3)
                
                # Calculate center
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    
                    cv2.circle(rgb_image, (cX, cY), 7, (255, 255, 255), -1)
                    
                    # --- Calculate 3D Coordinates ---
                    if self.latest_depth is not None:
                        h, w = self.latest_depth.shape
                        if 0 <= cX < w and 0 <= cY < h:
                            X, Y, Z = self.calculate_3d_coordinates(cX, cY, self.latest_depth)
                            
                            # --- Publish Coordinates ---
                            msg = ObjectCoordinates()
                            msg.x = X
                            msg.y = Y
                            msg.z = Z
                            self.coord_pub.publish(msg)
                            
                            # --- State Machine Control ---
                            # 计算归一化的水平偏移 (图像中心为0)
                            image_center_x = w / 2.0
                            horizontal_error = (cX - image_center_x) / image_center_x
                            
                            # 当前距离
                            current_distance = Z
                            distance_error = current_distance - self.desired_distance
                            
                            # 状态转换和控制逻辑
                            if self.current_state == RobotState.SEARCHING:
                                if target_found:
                                    self.current_state = RobotState.ALIGNING
                                    rospy.loginfo("State: SEARCHING -> ALIGNING")
                            
                            elif self.current_state == RobotState.ALIGNING:
                                # 只旋转,不前进
                                if abs(horizontal_error) > self.align_threshold:
                                    # 需要继续对准
                                    cmd.angular.z = -self.angular_kp * horizontal_error
                                    cmd.angular.z = max(-self.max_angular_speed, 
                                                       min(self.max_angular_speed, cmd.angular.z))
                                    cmd.linear.x = 0.0
                                else:
                                    # 已对准,切换到接近状态
                                    self.current_state = RobotState.APPROACHING
                                    rospy.loginfo("State: ALIGNING -> APPROACHING")
                            
                            elif self.current_state == RobotState.APPROACHING:
                                # 同时保持对准和前进
                                # 如果偏移过大,回到对准状态
                                if abs(horizontal_error) > self.align_threshold * 2:
                                    self.current_state = RobotState.ALIGNING
                                    rospy.loginfo("State: APPROACHING -> ALIGNING (lost alignment)")
                                elif abs(distance_error) < self.distance_threshold:
                                    # 到达目标距离
                                    self.current_state = RobotState.REACHED
                                    rospy.loginfo("State: APPROACHING -> REACHED")
                                else:
                                    # 继续接近,同时微调角度
                                    cmd.angular.z = -self.angular_kp * 0.5 * horizontal_error  # 降低增益
                                    cmd.angular.z = max(-self.max_angular_speed * 0.5, 
                                                       min(self.max_angular_speed * 0.5, cmd.angular.z))
                                    
                                    if distance_error > 0:  # 距离大于期望距离,前进
                                        cmd.linear.x = self.linear_kp * distance_error
                                        cmd.linear.x = max(0, min(self.max_linear_speed, cmd.linear.x))
                                    else:  # 距离小于期望距离,后退
                                        cmd.linear.x = self.linear_kp * distance_error
                                        cmd.linear.x = max(-self.max_linear_speed * 0.5, 
                                                          min(0, cmd.linear.x))
                            
                            elif self.current_state == RobotState.REACHED:
                                # 保持位置,微调
                                if abs(horizontal_error) > self.align_threshold:
                                    cmd.angular.z = -self.angular_kp * 0.3 * horizontal_error
                                if abs(distance_error) > self.distance_threshold:
                                    self.current_state = RobotState.APPROACHING
                                    rospy.loginfo("State: REACHED -> APPROACHING (distance changed)")
                                cmd.linear.x = 0.0
                            
                            # --- TF Transformation ---
                            try:
                                point_stamped = PointStamped()
                                point_stamped.header.frame_id = "camera_rgb_optical_frame"
                                point_stamped.header.stamp = rospy.Time(0)
                                point_stamped.point.x = X
                                point_stamped.point.y = Y
                                point_stamped.point.z = Z
                                
                                if self.tf_buffer.can_transform("base_footprint", 
                                                                point_stamped.header.frame_id, 
                                                                rospy.Time(0), 
                                                                rospy.Duration(1.0)):
                                    point_base = self.tf_buffer.transform(point_stamped, "base_footprint")
                            except Exception as e:
                                rospy.logwarn(f"TF Error: {e}")
                            
                            # Display info
                            state_text = f"State: {self.current_state.name}"
                            coord_text = f"X:{X:.2f} Y:{Y:.2f} Z:{Z:.2f}"
                            error_text = f"H_Err:{horizontal_error:.3f} D_Err:{distance_error:.2f}"
                            
                            cv2.putText(rgb_image, state_text, (10, 30),
                                      cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 0), 2)
                            cv2.putText(rgb_image, coord_text, (cX - 50, cY - 30),
                                      cv2.FONT_HERSHEY_SIMPLEX, 2.0, (255, 255, 255), 2)
                            cv2.putText(rgb_image, error_text, (10, 60),
                                      cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 0), 2)
                            
                            # 绘制中心十字线
                            cv2.line(rgb_image, (int(image_center_x) - 20, int(h/2)), 
                                   (int(image_center_x) + 20, int(h/2)), (0, 255, 255), 2)
                            cv2.line(rgb_image, (int(image_center_x), int(h/2) - 20), 
                                   (int(image_center_x), int(h/2) + 20), (0, 255, 255), 2)
        
        # 目标丢失处理
        if not target_found:
            self.target_lost_count += 1
            if self.target_lost_count > self.max_lost_frames:
                if self.current_state != RobotState.SEARCHING:
                    rospy.loginfo(f"State: {self.current_state.name} -> SEARCHING (target lost)")
                    self.current_state = RobotState.SEARCHING
                # 搜索模式:缓慢旋转
                cmd.angular.z = 0.3
                cmd.linear.x = 0.0
        
        # 发布控制命令
        self.cmd_vel_pub.publish(cmd)
        
        # Show windows
        cv2.imshow("Original RGB", self.resize(rgb_image))
        cv2.imshow("HSV Result", self.resize(hsv_result))
        cv2.imshow("Canny Edges", self.resize(edges))
        if self.depth_colormap is not None:
            cv2.imshow("Depth Colormap", self.resize(self.depth_colormap))
        
        cv2.waitKey(1)

    def resize(self,image):
        # scale = 0.2   # 0.5 = 缩小到原来的一半，0.25 = 四分之一
        self.display_scale = rospy.get_param("~display_scale", 0.5)
        return cv2.resize(
            image,
            None,              # 不指定目标尺寸
            fx=self.display_scale,
            fy=self.display_scale,
            interpolation=cv2.INTER_AREA
        )



    def calculate_3d_coordinates(self, u, v, depth_image):
        # Camera Intrinsic Parameters (TurtleBot3 Waffle Pi / RealSense R200)
        # Resolution 640x480, FOV ~60 deg
        if self.fx is None:
            fx = 554.25
            fy = 554.25
            cx = 320.5
            cy = 240.5

            Z = depth_image[v, u]  # Depth in meters
            
            # Handle invalid depth
            if np.isnan(Z) or Z <= 0:
                return 0.0, 0.0, 0.0

            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy

            return float(X), float(Y), float(Z)
        else:
            Z = depth_image[v, u] 
    
            if np.isnan(Z) or Z <= 0:
                return 0.0, 0.0, 0.0

            X = (u - self.cx) * Z / self.fx
            Y = (v - self.cy) * Z / self.fy

            return float(X), float(Y), float(Z)

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    node = PerceptionNode()
    node.run()
