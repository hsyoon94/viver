#!/usr/bin/env python3

import sys
import socket
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, PointCloud2
import threading
import queue

class SingleTopicSender(Node):
    def __init__(self, server_ip, port, topic_name):
        super().__init__('single_topic_sender')
        self.server_ip = server_ip
        self.port = port
        self.topic_name = topic_name

        # 소켓 설정
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1048576 * 100)  # 2MB
        try:
            self.client_socket.connect((self.server_ip, self.port))
            self.get_logger().info(f"Connected to server at {self.server_ip}:{self.port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to server: {e}")
            sys.exit(1)

        # QoS 설정
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # 토픽 구독
        self.subscription = self.create_subscription(
            # CompressedImage,  # ROS2 message type
            PointCloud2,
            self.topic_name,
            self.image_callback,
            qos_profile
        )
        self.get_logger().info(f"Subscribed to topic: {self.topic_name}")

        # 메시지 전송을 위한 큐와 스레드 설정
        self.send_queue = queue.Queue()
        self.send_thread = threading.Thread(target=self.send_loop, daemon=True)
        self.send_thread.start()

    def image_callback(self, msg):
        try:
            serialized_msg = serialize_message(msg)
            self.send_queue.put(serialized_msg)
            # 디버그 로그 레벨을 INFO에서 DEBUG로 변경하여 로그 양을 줄임
            self.get_logger().debug(f"Image queued from topic '{self.topic_name}'")
        except Exception as e:
            self.get_logger().error(f"Error queueing image from topic '{self.topic_name}': {e}")

    def send_loop(self):
        while rclpy.ok():
            try:
                serialized_msg = self.send_queue.get(timeout=1)
                msg_size = len(serialized_msg)
                # 메시지 크기 전송ls
                self.client_socket.sendall(msg_size.to_bytes(4, byteorder='big'))
                # 메시지 데이터 전송
                self.client_socket.sendall(serialized_msg)
                self.get_logger().debug(f"Image sent from topic '{self.topic_name}'")
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Error sending image from topic '{self.topic_name}': {e}")
                break

    def destroy_node(self):
        self.client_socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 4:
        print("Usage: ros2 run image_publisher_pkg image_sender_single_node <server_ip> <port> <topic_name>")
        sys.exit(1)

    server_ip = sys.argv[1]
    port = int(sys.argv[2])
    topic_name = sys.argv[3]

    sender_node = SingleTopicSender(server_ip, port, topic_name)
    try:
        rclpy.spin(sender_node)
    except KeyboardInterrupt:
        sender_node.get_logger().info("Shutting down node.")
    finally:
        sender_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
