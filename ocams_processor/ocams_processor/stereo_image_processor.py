import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ContinuousNode(Node):
    def __init__(self):
        super().__init__('stereo_image_processor')  # 노드 이름 설정
        self.timer = self.create_timer(0.05, self.timer_callback)  # 1초 주기로 타이머 생성
        self.device_id = '/dev/video0'
        self.cap = cv2.VideoCapture(self.device_id)

        self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        self.bridge = CvBridge()

        self.publisher_left = self.create_publisher(Image, '/camera/image_left', 10)
        self.publisher_right = self.create_publisher(Image, '/camera/image_right', 10)


    def split_images(self, input_frame):
        right, left = cv2.split(input_frame)

        left = cv2.cvtColor(left, cv2.COLOR_BayerGR2RGB)
        right = cv2.cvtColor(right, cv2.COLOR_BayerGR2RGB)

        return left, right

    def timer_callback(self):
        ret, frame = self.cap.read()
        left_rgb, right_rgb = self.split_images(frame)

        left_rgb_msg = self.bridge.cv2_to_imgmsg(left_rgb, encoding="bgr8")
        right_rgb_msg = self.bridge.cv2_to_imgmsg(right_rgb, encoding="bgr8")

        current_time = self.get_clock().now()
        left_rgb_msg.header.stamp = current_time.to_msg()
        right_rgb_msg.header.stamp = current_time.to_msg()
        self.publisher_left.publish(left_rgb_msg)
        self.publisher_right.publish(right_rgb_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ContinuousNode()
    try:
        rclpy.spin(node)  # 노드를 실행 상태로 유지
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped manually.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
