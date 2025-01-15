import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class StereoImageProcessor(Node):
    def __init__(self):
        super().__init__('stereo_image_processor')
        
        # 이미지 토픽 구독
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',  # 카메라 이미지 토픽 이름
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.frame_count = 0  # 프레임 번호 추적
        
        self.left_image = None
        self.right_image = None

    def image_callback(self, msg):
        # ROS 이미지 메시지를 OpenCV 이미지로 변환
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 프레임 번호로 Left/Right 결정
        if self.frame_count % 2 == 0:
            self.left_image = frame
            self.get_logger().info('Left image received.')
            cv2.imshow('Left Image', self.left_image)
        else:
            self.right_image = frame
            self.get_logger().info('Right image received.')
            cv2.imshow('Right Image', self.right_image)

        self.frame_count += 1

        # OpenCV 윈도우 업데이트
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    processor = StereoImageProcessor()

    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        processor.get_logger().info('Shutting down...')
    finally:
        processor.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
