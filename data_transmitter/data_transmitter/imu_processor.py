import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import socket
import json

class ImuSender(Node):
    def __init__(self):
        super().__init__('imu_sender')

        # 송신 대상 IP와 포트 설정
        self.target_ip = '147.46.245.118'  # 외부 PC의 IP
        self.target_port = 5005           # 외부 PC의 포트

        # UDP 소켓 초기화
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # IMU 메시지 구독 설정
        self.subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10
        )

        self.get_logger().info("IMU Sender Node Initialized")

    def imu_callback(self, msg: Imu):
        # IMU 데이터를 JSON으로 변환
        imu_data = {
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z,
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z,
            },
            'orientation': {
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z,
                'w': msg.orientation.w,
            }
        }

        # JSON 직렬화
        imu_json = json.dumps(imu_data)

        # UDP 송신
        self.sock.sendto(imu_json.encode('utf-8'), (self.target_ip, self.target_port))

        self.get_logger().info(f"IMU data sent to {self.target_ip}:{self.target_port}")

def main(args=None):
    rclpy.init(args=args)
    node = ImuSender()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # 종료 시 노드 정리
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

