import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import socket
import base64
import json

class ImageSenderTCP(Node):
    def __init__(self):
        super().__init__('image_sender_tcp')

        # ROS 2 Subscription
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',  # Subscribe to camera topic
            self.image_callback,
            10
        )
        print("setting skrr")
        # TCP Socket Configuration
        self.target_ip = '147.46.245.118'  # 외부 PC의 IP 주소
        self.target_port = 5005          # 외부 PC의 수신 포트
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print("connection 1")
        self.tcp_socket.connect((self.target_ip, self.target_port))
        print("connection 2")
        self.get_logger().info(f"Connected to {self.target_ip}:{self.target_port}")
        print("connection 3")

    def image_callback(self, msg):
        try:
            # Serialize the image data as JSON
            encoded_data = base64.b64encode(msg.data).decode('utf-8')
            image_message = {
                "width": msg.width,
                "height": msg.height,
                "encoding": msg.encoding,
                "data": encoded_data
            }
            json_message = json.dumps(image_message)

            # Send the JSON message over TCP
            self.tcp_socket.sendall(json_message.encode('utf-8'))
            self.tcp_socket.sendall(b'\n')  # Add a delimiter for message separation
            self.get_logger().info("Sent image data")
        except Exception as e:
            self.get_logger().error(f"Error sending image: {str(e)}")

    def destroy_node(self):
        # Close the TCP connection
        self.tcp_socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImageSenderTCP()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
