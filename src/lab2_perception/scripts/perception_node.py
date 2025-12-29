#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from lab2_perception.msg import ObjectCoordinates

class PerceptionNode:
    def __init__(self):
        rospy.init_node('perception_node')
        
        self.bridge = CvBridge()
        self.latest_depth = None
        
        # Subscribers
        self.rgb_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
        
        # Publisher
        self.coord_pub = rospy.Publisher('detected_object', ObjectCoordinates, queue_size=10)
        
        rospy.loginfo("Perception Node Started")

    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except Exception as e:
            rospy.logerr(f"Depth error: {e}")

    def rgb_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_image(cv_image)
        except Exception as e:
            rospy.logerr(f"RGB error: {e}")

    def process_image(self, rgb_image):
        # --- 2.1 HSV Color Space Conversion & Detection ---
        hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
        
        # Red color range (as per manual)
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        
        mask = cv2.inRange(hsv_image, lower_red, upper_red)
        hsv_result = cv2.bitwise_and(rgb_image, rgb_image, mask=mask)
        
        # --- 2.2 Contour Detection (Demonstration with Canny) ---
        gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray_image, 100, 200)
        # Find contours on edges (just for visualization as per manual example)
        # Note: Usually we find contours on the mask for object detection, 
        # but here we follow the manual's structure to show we can do it.
        contours_canny, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        canny_display = rgb_image.copy()
        cv2.drawContours(canny_display, contours_canny, -1, (0, 255, 0), 3)
        
        # --- Integrated Object Detection (HSV -> Contour -> 3D) ---
        # Find contours on the HSV mask to target the red object
        contours_mask, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours_mask:
            # Find largest contour
            c = max(contours_mask, key=cv2.contourArea)
            if cv2.contourArea(c) > 100:
                # Draw contour on original image
                cv2.drawContours(rgb_image, [c], -1, (0, 0, 255), 3)
                
                # Calculate center
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    
                    cv2.circle(rgb_image, (cX, cY), 7, (255, 255, 255), -1)
                    
                    # --- 3.1 Calculate 3D Coordinates ---
                    if self.latest_depth is not None:
                        # Ensure cX, cY are within bounds
                        h, w = self.latest_depth.shape
                        if 0 <= cX < w and 0 <= cY < h:
                            X, Y, Z = self.calculate_3d_coordinates(cX, cY, self.latest_depth)
                            
                            # --- 4. Data Publishing ---
                            msg = ObjectCoordinates()
                            msg.x = X
                            msg.y = Y
                            msg.z = Z
                            self.coord_pub.publish(msg)
                            
                            # Display text
                            text = f"X:{X:.2f} Y:{Y:.2f} Z:{Z:.2f}"
                            cv2.putText(rgb_image, text, (cX - 20, cY - 20),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Show windows
        cv2.imshow("Original RGB", rgb_image)
        cv2.imshow("HSV Result", hsv_result)
        cv2.imshow("Canny Edges", edges)
        cv2.waitKey(1)

    def calculate_3d_coordinates(self, u, v, depth_image):
        # Camera Intrinsic Parameters
        fx = 525.0
        fy = 525.0
        cx = 319.5
        cy = 239.5

        Z = depth_image[v, u]  # Depth in meters
        
        # Handle invalid depth
        if np.isnan(Z) or Z <= 0:
            return 0.0, 0.0, 0.0

        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy

        return float(X), float(Y), float(Z)

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    node = PerceptionNode()
    node.run()
