from .preProcession import ImagePreprocessor
from .detector import TriangleDetector
from .AngleSolver import AngleSolver
from .trans import Quadrilateral

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from second_interface.msg import Second
from msg_interfaces.msg import Angle

import cv2
import numpy as np
from cv_bridge import CvBridge


class second_node(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("start %s"% name)
        qos = QoSProfile(depth=5)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            10
        )
        self.pub_second = self.create_publisher(Angle,"second",qos)

        K = np.array([[623.6946919854665, 0.0,325.16044119395104], 
                      [0.0, 623.0212233788975, 243.62731542546734], 
                      [0.0, 0.0, 1.0]])
        D = np.array([0.04744837973812748, 0.11038950430220529, -0.0051339047333748295, -0.0006166345162193224, 0.0])
        self.preProcession = ImagePreprocessor(True)
        self.detector = TriangleDetector()
        self.p3p = AngleSolver(K,D)
        self.trans = Quadrilateral()

        cv2.namedWindow("cv_image", cv2.WINDOW_NORMAL)
        self.bridge = CvBridge()
    
    def __del__(self):
        self.img_processor.release_camera()
    
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if cv_image is None:
            self.get_logger().error('Failed to capture image, skipping this frame.')
            return
        #cv2.imshow("cv_image", cv_image)
        cp_frame = self.preProcession.set_image(cv_image)
        binary_image = self.preProcession.preProcession_main()
        sorted_points = self.detector.detector_main(binary_image,cp_frame)
        cv2.imshow("cv_image", binary_image)
        cv2.waitKey(10)
        if len(sorted_points)==0:
            print("len(sorted_points)==0!")
            rvecs = np.zeros((3, 1))
            tvecs = np.zeros((3, 1))
            mode = 1
        else:
            rvecs, tvecs ,mode = self.p3p.angle_solver_main(sorted_points,cv_image)
        self.trans.init(tvecs[0],tvecs[1],tvecs[2],225,rvecs[0],rvecs[1],rvecs[2])
        if mode:
            if tvecs[0] != 0 or tvecs[1] != 0 or tvecs[2] != 0:
                center_point = self.trans.calculate_center_point_left()
                print("left!")
            else:
                center_point = np.zeros((3,1))
                print("no1!")
        else:
            if tvecs[0] != 0 or tvecs[1] != 0 or tvecs[2] != 0:
                center_point = self.trans.calculate_center_point_right()
                print("right!")
            else:
                center_point = np.zeros((3,1))
                print("no2!")

        print("旋转向量1:", rvecs)
        print("平移向量1:", center_point)
        msg = Angle()
        msg.x = float(center_point[0])
        msg.y = float(center_point[1])
        msg.z = float(center_point[2])
        msg.roll = float(rvecs[0])
        msg.pitch = float(rvecs[1])
        msg.yaw = float(rvecs[2])
        msg.mode = 1
        msg.found = 1
        self.pub_second.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = second_node("second_node")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()