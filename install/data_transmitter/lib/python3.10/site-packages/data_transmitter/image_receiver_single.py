#!/usr/bin/env python3

import sys
import socket
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import threading
import queue

class SingleTopicReceiver(Node):
    def __init__(self, host_ip, port, topic_name):
        super().__init__('single_topic_receiver')
        self.host_ip = host_ip
        self.port = port
        self.topic_name = topic_name

        # 소켓 설정
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1048576 * 10)  # 2MB
        try:
            self.server_socket.bind((self.host_ip, self.port))
            self.server_socket.listen(1)
            self.get_logger().info(f"Socket bound to {self.host_ip}:{self.port}")
            self.get_logger().info("Server is listening for a connection")
        except Exception as e:
            self.get_logger().error(f"Failed to bind or listen on socket: {e}")
            sys.exit(1)

        # 클라이언트 연결 수락
        try:
            self.connection, self.addr = self.server_socket.accept()
            self.get_logger().info(f"Connection established with {self.addr}")
        except Exception as e:
            self.get_logger().error(f"Error accepting connection: {e}")
            sys.exit(1)

        # QoS 설정
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # 토픽 발행자 생성
        self.publisher = self.create_publisher(
            CompressedImage,
            self.topic_name,
            qos_profile
        )
        self.get_logger().info(f"Publisher initialized on topic: {self.topic_name}")

        # 메시지 수신을 위한 큐와 스레드 설정
        self.receive_queue = queue.Queue()
        self.receive_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.receive_thread.start()

        # 발행을 위한 타이머 설정
        self.publish_timer = self.create_timer(1.0 / 30.0, self.publish_callback)  # 30Hz로 발행 시도

    def publish_callback(self):
        try:
            serialized_data = self.receive_queue.get_nowait()
            image_msg = deserialize_message(serialized_data, CompressedImage)
            self.publisher.publish(image_msg)
            self.get_logger().debug(f"Published image to topic '{self.topic_name}'")
        except queue.Empty:
            # 큐에 메시지가 없을 경우
            pass
        except Exception as e:
            self.get_logger().error(f"Error publishing image: {e}")

    def recvall(self, n):
        """n 바이트를 모두 수신할 때까지 대기"""
        data = b''
        while len(data) < n:
            packet = self.connection.recv(n - len(data))
            if not packet:
                return None
            data += packet
        return data

    def receive_loop(self):
        try:
            while rclpy.ok():
                # 메시지 크기 수신
                msg_size_bytes = self.recvall(4)
                if not msg_size_bytes:
                    self.get_logger().info("Connection closed by sender.")
                    break
                msg_size = int.from_bytes(msg_size_bytes, byteorder='big')

                # 메시지 데이터 수신
                serialized_data = self.recvall(msg_size)
                if not serialized_data:
                    self.get_logger().info("Connection closed by sender during data reception.")
                    break

                # 큐에 메시지 추가
                self.receive_queue.put(serialized_data)
        except Exception as e:
            self.get_logger().error(f"Error receiving image: {e}")
        finally:
            self.connection.close()
            self.server_socket.close()
            self.get_logger().info("Socket closed.")

    def destroy_node(self):
        self.connection.close()
        self.server_socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 4:
        print("Usage: ros2 run image_receiver image_receiver_single <host_ip> <port> <topic_name>")
        sys.exit(1)

    host_ip = sys.argv[1]
    port = int(sys.argv[2])
    topic_name = sys.argv[3]

    receiver_node = SingleTopicReceiver(host_ip, port, topic_name)
    try:
        rclpy.spin(receiver_node)
    except KeyboardInterrupt:
        receiver_node.get_logger().info("Shutting down node.")
    finally:
        receiver_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
