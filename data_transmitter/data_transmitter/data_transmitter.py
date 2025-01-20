#!/usr/bin/env python3

import sys
import socket
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, Imu
import threading
import queue
import time

class TripleTopicSender(Node):
    def __init__(self, server_ip, port, topic_name1, topic_name2, topic_name3):
        super().__init__('triple_topic_sender')
        self.server_ip = server_ip
        self.port = port
        self.topic_name1 = topic_name1
        self.topic_name2 = topic_name2
        self.topic_name3 = topic_name3

        # 소켓 설정
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1048576 * 100)  # 10MB
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

        # 토픽 구독 설정
        self.subscription1 = self.create_subscription(
            CompressedImage,
            self.topic_name1,
            self.callback1,
            qos_profile
        )
        self.subscription2 = self.create_subscription(
            CompressedImage,
            self.topic_name2,
            self.callback2,
            qos_profile
        )
        self.subscription3 = self.create_subscription(
            Imu,
            self.topic_name3,
            self.callback3,
            qos_profile
        )

        self.get_logger().info(f"Subscribed to topics: {self.topic_name1}, {self.topic_name2}, {self.topic_name3}")

        # 메시지 전송을 위한 큐와 스레드 설정
        self.send_queue = queue.Queue()
        self.send_thread = threading.Thread(target=self.send_loop, daemon=True)
        self.send_thread.start()

    def callback1(self, msg):
        self.enqueue_message(msg, topic_id=1)

    def callback2(self, msg):
        self.enqueue_message(msg, topic_id=2)

    def callback3(self, msg):
        self.enqueue_message(msg, topic_id=3)

    def enqueue_message(self, msg, topic_id):
        try:
            serialized_msg = serialize_message(msg)
            self.send_queue.put((topic_id, serialized_msg))
            self.get_logger().debug(f"Message queued for topic ID {topic_id}")
        except Exception as e:
            self.get_logger().error(f"Error queueing message for topic ID {topic_id}: {e}")

    def send_loop(self):
        while rclpy.ok():
            try:
                topic_id, serialized_msg = self.send_queue.get(timeout=1)
                msg_size = len(serialized_msg)
                # 메시지 크기와 토픽 ID 전송
                self.client_socket.sendall(msg_size.to_bytes(4, byteorder='big') + bytes([topic_id]))
                # 메시지 데이터 전송
                self.client_socket.sendall(serialized_msg)
                self.get_logger().debug(f"Message sent for topic ID {topic_id}")
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Error sending message: {e}")
                break

    def destroy_node(self):
        self.client_socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    server_ip = '147.47.239.116'
    port = 4000
    topic_name1 = '/viver1/camera/image/left'
    topic_name2 = '/viver1/camera/image/right'
    topic_name3 = '/viver1/imu/ocams/data'

    sender_node = TripleTopicSender(server_ip, port, topic_name1, topic_name2, topic_name3)
    time.sleep(10)
    try:
        rclpy.spin(sender_node)
    except KeyboardInterrupt:
        sender_node.get_logger().info("Shutting down node.")
    finally:
        sender_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
