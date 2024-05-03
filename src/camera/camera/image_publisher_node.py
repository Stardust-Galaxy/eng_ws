
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image', 10)
        self.timer = self.create_timer(1.0 / 30, self.timer_callback)
        self.cv_bridge = CvBridge()

        # Open the video capture device
        self.cap = cv2.VideoCapture('/dev/video0')
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open video capture device')
            return
        """
        设置摄像头参数
        """
        # 设置光圈参数（示例）
        self.cap.set(cv2.CAP_PROP_EXPOSURE, 0)  # 将曝光度设置为最小值
        # 关闭自动白平衡
        self.cap.set(cv2.CAP_PROP_AUTO_WB, 1)
        # 设置手动白平衡的色温
        self.cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 1000)  # 例：设置色温为 2000

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.cap.release()
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
