from MangDang.mini_pupper.ESP32Interface import ESP32Interface

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu


class IMU(Node):

    def __init__(self):
        super().__init__('mini_pupper_imu')
        self.publisher = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.esp32 = ESP32Interface()
        self.gyro_x_offset = 0.0  # [rad/s]
        self.gyro_y_offset = 0.0  # [rad/s]
        self.gyro_z_offset = 0.0  # [rad/s]
        self.accel_x_offset = 0.0  # [m/s²]
        self.accel_y_offset = 0.0  # [m/s²]
        self.accel_z_offset = 0.0  # [m/s²]
        self.frequency = 0.01  # [100Hz]
        self.calibrate_imu()
        self.timer = self.create_timer(self.frequency, self.publish_imu)

    def read_imu(self):
        raw = self.esp32.imu_get_data()
        imu_readings = []
        # fix orientation and units
        imu_readings.append(raw['ay'] * 9.81 - self.accel_x_offset)
        imu_readings.append(raw['ax'] * 9.81 - self.accel_y_offset)
        imu_readings.append(raw['az'] * -9.81 - self.accel_z_offset)
        imu_readings.append(raw['gy'] * 0.017453 - self.gyro_x_offset)
        imu_readings.append(raw['gx'] * 0.017453 - self.gyro_y_offset)
        imu_readings.append(raw['gz'] * -0.017453 - self.gyro_z_offset)
        return imu_readings

    def calibrate_imu(self):
        calibration_count = 200
        gyro_x_offset = 0.0  # [rad/s]
        gyro_y_offset = 0.0  # [rad/s]
        gyro_z_offset = 0.0  # [rad/s]
        accel_x_offset = 0.0  # [m/s²]
        accel_y_offset = 0.0  # [m/s²]
        accel_z_offset = 0.0  # [m/s²]
        for i in range(calibration_count):
            ax, ay, az, gx, gy, gz = self.read_imu()
            accel_x_offset += ax
            accel_y_offset += ay
            accel_z_offset += az
            gyro_x_offset += gx
            gyro_y_offset += gy
            gyro_z_offset += gz

        self.accel_x_offset = accel_x_offset / calibration_count
        self.accel_y_offset = accel_y_offset / calibration_count
        self.accel_z_offset = accel_z_offset / calibration_count
        self.accel_z_offset -= 9.81
        self.gyro_x_offset = gyro_x_offset / calibration_count
        self.gyro_y_offset = gyro_y_offset / calibration_count
        self.gyro_z_offset = gyro_z_offset / calibration_count

        self.get_logger().info("accel_x_offset: %s" % self.accel_x_offset)
        self.get_logger().info("accel_y_offset: %s" % self.accel_y_offset)
        self.get_logger().info("accel_z_offset: %s" % self.accel_z_offset)
        self.get_logger().info("gyro_x_offset:%s" % self.gyro_x_offset)
        self.get_logger().info("gyro_y_offset:%s" % self.gyro_y_offset)
        self.get_logger().info("gyro_z_offset:%s" % self.gyro_z_offset)

    def publish_imu(self):
        ax, ay, az, gx, gy, gz = self.read_imu()
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu"
        msg.linear_acceleration_covariance[0] = 0.005 * 9.81
        msg.linear_acceleration_covariance[4] = 0.005 * 9.81
        msg.linear_acceleration_covariance[8] = 0.005 * 9.81
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.angular_velocity_covariance[0] = 0.05 * 0.017453
        msg.angular_velocity_covariance[4] = 0.05 * 0.017453
        msg.angular_velocity_covariance[8] = 0.05 * 0.017453
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        # Invalidate quaternion
        #TODO calculate quaternion
        msg.orientation_covariance[0] = -1
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    imu = IMU()

    rclpy.spin(imu)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
