import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import serial
import threading

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.port = '/dev/ttyUSB0'
        self.baud = 9600

        try:
            self.serial = serial.Serial(self.port, self.baud, timeout=0.5)
            self.get_logger().info("Serial Port Opened: {}".format(self.serial.is_open))
        except serial.SerialException as e:
            self.get_logger().error("Failed to open serial port: {}".format(e))
            raise

        self.buf_length = 11
        self.RxBuff = [0] * self.buf_length
        self.ACCData = [0.0] * 8
        self.GYROData = [0.0] * 8
        self.AngleData = [0.0] * 8
        self.CheckSum = 0
        self.start = 0
        self.data_length = 0

        self.acc = [0.0] * 3
        self.gyro = [0.0] * 3
        self.Angle = [0.0] * 3

        self.lock = threading.Lock()  # Mutex lock for thread safety

        # Timer for reading data
        self.timer = self.create_timer(0.01, self.read_data)

    def read_data(self):
        with self.lock:
            if self.serial.in_waiting > 0:
                RXdata = self.serial.read(1)
                RXdata = int(RXdata.hex(), 16)

                if RXdata == 0x55 and self.start == 0:
                    self.start = 1
                    self.data_length = self.buf_length
                    self.CheckSum = 0
                    for i in range(self.buf_length):
                        self.RxBuff[i] = 0

                if self.start == 1:
                    self.CheckSum += RXdata
                    self.RxBuff[self.buf_length - self.data_length] = RXdata
                    self.data_length -= 1

                    if self.data_length == 0:
                        self.CheckSum = (self.CheckSum - RXdata) & 0xff
                        self.start = 0

                        if self.RxBuff[self.buf_length - 1] == self.CheckSum:
                            self.process_data(self.RxBuff)

    def process_data(self, list_buf):
        with self.lock:  # Ensure thread-safe access
            imu_msg = Imu()
            imu_msg.header = Header()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu'

            if list_buf[1] == 0x51:
                for i in range(6):
                    self.ACCData[i] = list_buf[2 + i]
                self.acc = self.get_acc(self.ACCData)
                imu_msg.linear_acceleration.x = self.acc[0]
                imu_msg.linear_acceleration.y = self.acc[1]
                imu_msg.linear_acceleration.z = self.acc[2]

            elif list_buf[1] == 0x52:
                for i in range(6):
                    self.GYROData[i] = list_buf[2 + i]
                self.gyro = self.get_gyro(self.GYROData)
                imu_msg.angular_velocity.x = self.gyro[0]
                imu_msg.angular_velocity.y = self.gyro[1]
                imu_msg.angular_velocity.z = self.gyro[2]

            elif list_buf[1] == 0x53:
                for i in range(6):
                    self.AngleData[i] = list_buf[2 + i]
                self.Angle = self.get_angle(self.AngleData)

            self.publisher_.publish(imu_msg)
            self.get_logger().info("Published IMU data")

    def get_acc(self, datahex):
        axl = datahex[0]
        axh = datahex[1]
        ayl = datahex[2]
        ayh = datahex[3]
        azl = datahex[4]
        azh = datahex[5]
        k_acc = 16.0
        acc_x = (axh << 8 | axl) / 32768.0 * k_acc
        acc_y = (ayh << 8 | ayl) / 32768.0 * k_acc
        acc_z = (azh << 8 | azl) / 32768.0 * k_acc
        if acc_x >= k_acc:
            acc_x -= 2 * k_acc
        if acc_y >= k_acc:
            acc_y -= 2 * k_acc
        if acc_z >= k_acc:
            acc_z -= 2 * k_acc
        return acc_x, acc_y, acc_z

    def get_gyro(self, datahex):
        wxl = datahex[0]
        wxh = datahex[1]
        wyl = datahex[2]
        wyh = datahex[3]
        wzl = datahex[4]
        wzh = datahex[5]
        k_gyro = 2000.0
        gyro_x = (wxh << 8 | wxl) / 32768.0 * k_gyro
        gyro_y = (wyh << 8 | wyl) / 32768.0 * k_gyro
        gyro_z = (wzh << 8 | wzl) / 32768.0 * k_gyro
        if gyro_x >= k_gyro:
            gyro_x -= 2 * k_gyro
        if gyro_y >= k_gyro:
            gyro_y -= 2 * k_gyro
        if gyro_z >= k_gyro:
            gyro_z -= 2 * k_gyro
        return gyro_x, gyro_y, gyro_z

    def get_angle(self, datahex):
        rxl = datahex[0]
        rxh = datahex[1]
        ryl = datahex[2]
        ryh = datahex[3]
        rzl = datahex[4]
        rzh = datahex[5]
        k_angle = 180.0
        angle_x = (rxh << 8 | rxl) / 32768.0 * k_angle
        angle_y = (ryh << 8 | ryl) / 32768.0 * k_angle
        angle_z = (rzh << 8 | rzl) / 32768.0 * k_angle
        if angle_x >= k_angle:
            angle_x -= 2 * k_angle
        if angle_y >= k_angle:
            angle_y -= 2 * k_angle
        if angle_z >= k_angle:
            angle_z -= 2 * k_angle
        return angle_x, angle_y, angle_z

def main(args=None):
    rclpy.init(args=args)
    imu_node = ImuNode()

    try:
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        imu_node.get_logger().info('Shutting down IMU node.')
    finally:
        imu_node.serial.close()
        imu_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
