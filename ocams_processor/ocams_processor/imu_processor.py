import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import math

class OCamIMUNode(Node):
    def __init__(self):
        super().__init__('ocams_imu_node')

        # Parameters for serial communication
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        # Get parameters
        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        # Initialize serial communication
        self.serial_port = serial.Serial(port, baudrate, timeout=1)

        # Publisher for /imu/data
        self.imu_publisher = self.create_publisher(Imu, '/imu/ocams/data', 10)

        # Timer to read and publish data periodically
        self.timer = self.create_timer(0.01, self.publish_imu_data)  # 100 Hz

        self.get_logger().info(f'Initialized oCamS IMU Node on port {port} with baudrate {baudrate}')

    def publish_imu_data(self):
        try:
            # Read a line of data from the IMU
            line = self.serial_port.readline().decode('utf-8').strip()
            
            # Check if the line starts with the expected header
            if not line.startswith('$LMGQUA'):
                self.get_logger().warn(f"Ignoring unexpected data: {line}")
                return

            # Split the line into components
            parts = line.split(',')
            if len(parts) != 15:
                self.get_logger().warn(f"Unexpected data length: {len(parts)}. Raw data: {line}")
                return

            # Parse the data fields
            timestamp = int(parts[1])
            ax, ay, az = map(lambda v: float(v) * 9.81 / 1000, parts[2:5])  # Convert accel to m/s²
            gx, gy, gz = map(lambda v: float(v) * math.pi / 180 / 1000, parts[5:8])  # Convert gyro to rad/s
            mx, my, mz = map(float, parts[8:11])  # Magnetometer values
            quat_w, quat_x, quat_y, quat_z = map(lambda v: float(v) / 4096, parts[11:15])  # Quaternion

            # Populate Imu message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'

            # Linear acceleration (m/s^2)
            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az

            # Angular velocity (rad/s)
            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz

            # Orientation (quaternion)
            imu_msg.orientation.w = quat_w
            imu_msg.orientation.x = quat_x
            imu_msg.orientation.y = quat_y
            imu_msg.orientation.z = quat_z

            # Publish the IMU data
            current_time = self.get_clock().now()
            imu_msg.header.stamp = current_time.to_msg()
            self.imu_publisher.publish(imu_msg)
        except Exception as e:
            self.get_logger().error(f'Error processing IMU data: {e}')


    def destroy_node(self):
        self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OCamIMUNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down oCamS IMU Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
