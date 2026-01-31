

import rospy
import cv2
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class YOLODetector:
    def __init__(self):
        rospy.init_node('yolo_detector', anonymous=True)
        
        # 参数
        self.model_path = "~/catkin_ws/models/yolo26s.pt" # rospy.get_param('~model_path', '')
        self.confidence_threshold = rospy.get_param('~confidence', 0.5)
        self.image_topic = rospy.get_param('~image_topic', '/camera/rgb/image_raw')
        
        # 加载模型
        rospy.loginfo(f"正在加载模型: {self.model_path}")
        self.model = YOLO(self.model_path)
        rospy.loginfo("模型加载完成!")
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 发布器
        self.image_pub = rospy.Publisher('/detection_image', Image, queue_size=10)
        
        # 订阅器
        self.image_sub = rospy.Subscriber(
            self.image_topic, 
            Image, 
            self.image_callback, 
            queue_size=1,
            buff_size=2**24
        )
        
        rospy.loginfo(f"YOLO 检测器已启动，订阅话题: {self.image_topic}")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge 错误: {e}")
            return
        
        # YOLO 推理
        results = self.model(cv_image, conf=self.confidence_threshold)
        print(results)
        # 获取带标注的图像
        for result in results:
            # 绘制检测框
            annotated_frame = result.plot()
            
            # 打印检测结果（可选）
            if result.boxes is not None and len(result.boxes) > 0:

                for box in result.boxes:
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    cls_name = self.model.names[cls_id]
                    rospy.loginfo_throttle(1, f"检测到: {cls_name}, 置信度: {conf:.2f}")
        
            # 发布图像
            try:
                image_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
                image_msg.header = msg.header
                self.image_pub.publish(image_msg)
            except CvBridgeError as e:
                rospy.logerr(f"CvBridge 错误: {e}")


def main():
    try:
        detector = YOLODetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
