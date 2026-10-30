import math
import statistics
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from smbus2 import SMBus

# LSM6DSOX registers
WHO_AM_I = 0x0F
CTRL1_XL = 0x10
CTRL2_G = 0x11
CTRL3_C = 0x12
OUTX_L_G = 0x22  # gyro X,Y,Z then accel X,Y,Z (12 bytes, little endian)

WHO_AM_I_VALUE = 0x6C

# CTRL1_XL = 0x48 -> 104 Hz, +-4 g   (0.122 mg/LSB)
# CTRL2_G  = 0x44 -> 104 Hz, +-500 dps (17.5 mdps/LSB)
ACC_SENS_G = 0.122e-3
GYRO_SENS_DPS = 17.5e-3

G = 9.80665
DEG2RAD = math.pi / 180.0


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        self.declare_parameter('bus', 1)
        self.declare_parameter('address', 0x6A)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('rate', 50.0)
        self.declare_parameter('calib_samples', 300)

        bus_num = self.get_parameter('bus').value
        self.addr = self.get_parameter('address').value
        self.frame_id = self.get_parameter('frame_id').value
        rate = self.get_parameter('rate').value
        n_calib = self.get_parameter('calib_samples').value

        self.bus = SMBus(bus_num)

        who = self.bus.read_byte_data(self.addr, WHO_AM_I)
        if who != WHO_AM_I_VALUE:
            self.get_logger().warn(
                f'Unexpected WHO_AM_I 0x{who:02X} (expected 0x{WHO_AM_I_VALUE:02X})')

        # software reset, then configure
        self.bus.write_byte_data(self.addr, CTRL3_C, 0x01)
        time.sleep(0.1)
        self.bus.write_byte_data(self.addr, CTRL3_C, 0x44)  # BDU + auto increment
        self.bus.write_byte_data(self.addr, CTRL1_XL, 0x48)
        self.bus.write_byte_data(self.addr, CTRL2_G, 0x44)
        time.sleep(0.1)

        self.gyro_bias = [0.0, 0.0, 0.0]
        self.calibrate_gyro(n_calib)

        self.pub = self.create_publisher(Imu, 'imu/data', 10)
        self.create_timer(1.0 / rate, self.publish)
        self.get_logger().info(f'IMU ready on bus {bus_num}, address 0x{self.addr:02X}')

    def read_raw(self):
        data = self.bus.read_i2c_block_data(self.addr, OUTX_L_G, 12)
        return struct.unpack('<hhhhhh', bytes(data))

    def calibrate_gyro(self, n):
        self.get_logger().info(f'Calibrating gyro bias ({n} samples), keep the robot still')
        samples = []
        while len(samples) < n:
            try:
                raw = self.read_raw()
            except OSError:
                time.sleep(0.01)
                continue
            samples.append([raw[0] * GYRO_SENS_DPS,
                            raw[1] * GYRO_SENS_DPS,
                            raw[2] * GYRO_SENS_DPS])
            time.sleep(0.01)

        for i in range(3):
            self.gyro_bias[i] = statistics.fmean(s[i] for s in samples)

        spread = statistics.pstdev(s[2] for s in samples)
        if spread > 0.5:
            self.get_logger().warn(
                f'Gyro Z spread is {spread:.2f} dps, the robot may have moved during calibration')
        self.get_logger().info(
            'Gyro bias (dps): x=%.3f y=%.3f z=%.3f' % tuple(self.gyro_bias))

    def publish(self):
        try:
            raw = self.read_raw()
        except OSError as exc:
            self.get_logger().warn(f'I2C read failed: {exc}')
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.orientation.w = 1.0
        msg.orientation_covariance[0] = -1.0  # no orientation estimate

        msg.angular_velocity.x = (raw[0] * GYRO_SENS_DPS - self.gyro_bias[0]) * DEG2RAD
        msg.angular_velocity.y = (raw[1] * GYRO_SENS_DPS - self.gyro_bias[1]) * DEG2RAD
        msg.angular_velocity.z = (raw[2] * GYRO_SENS_DPS - self.gyro_bias[2]) * DEG2RAD

        msg.linear_acceleration.x = raw[3] * ACC_SENS_G * G
        msg.linear_acceleration.y = raw[4] * ACC_SENS_G * G
        msg.linear_acceleration.z = raw[5] * ACC_SENS_G * G

        gyro_var = 1.0e-4
        acc_var = 0.05
        msg.angular_velocity_covariance = [
            gyro_var, 0.0, 0.0,
            0.0, gyro_var, 0.0,
            0.0, 0.0, gyro_var,
        ]
        msg.linear_acceleration_covariance = [
            acc_var, 0.0, 0.0,
            0.0, acc_var, 0.0,
            0.0, 0.0, acc_var,
        ]
        self.pub.publish(msg)

    def destroy_node(self):
        try:
            self.bus.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()