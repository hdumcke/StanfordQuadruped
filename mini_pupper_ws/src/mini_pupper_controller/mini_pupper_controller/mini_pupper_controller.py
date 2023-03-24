import numpy as np
import sys
import os
sys.path.append(os.path.join(os.path.expanduser('~'), 'StanfordQuadruped'))
from src.Controller import Controller
from src.Command import Command
from src.State import BehaviorState, State
from MangDang.mini_pupper.HardwareInterface import HardwareInterface
from MangDang.mini_pupper.Config import Configuration
from pupper.Kinematics import four_legs_inverse_kinematics
from MangDang.mini_pupper.display import Display

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class MiniPupper(Node):

    def __init__(self):
        super().__init__('mini_pupper_controller')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.time_last = None
        self.time_now = None
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.v_x = 0.
        self.v_z = 0.
        self.active_timeout = 0.5

        # Create config
        self.config = Configuration()
        self.hardware_interface = HardwareInterface()
        self.disp = Display()
        self.disp.show_ip()

        self.controller = Controller(
            self.config,
            four_legs_inverse_kinematics,
        )
        self.state = State()
        self.state.behavior_state = BehaviorState.REST
        self.set_rest_state = True

        self.last_loop = int(self.get_clock().now().nanoseconds / 1e9)

        self.get_logger().info("Summary of gait parameters:")
        self.get_logger().info("overlap time: %s" % self.config.overlap_time)
        self.get_logger().info("swing time: %s" % self.config.swing_time)
        self.get_logger().info("z clearance: %s" % self.config.z_clearance)
        self.get_logger().info("x shift: %s" % self.config.x_shift)

    def get_command(self):

        command = Command()

        x_vel = self.v_x * self.config.max_x_velocity
        y_vel = self.v_z * -self.config.max_y_velocity
        command.horizontal_velocity = np.array([x_vel, y_vel])

        return command

    def read_orientation(self):
        #TODO
        return np.array([1, 0, 0, 0])

    def timer_callback(self):
        if self.time_now is None:
            return
        # self.time_now set by cmd_vel subscriber
        if self.get_clock().now().nanoseconds - self.time_now > self.active_timeout * 2e+9:
            if not self.set_rest_state:
                self.disp.show_state(BehaviorState.DEACTIVATED)
                return
        self.set_rest_state = False    
        # Standford time check now = int(self.get_clock().now().nanoseconds / 1e9)
        now = int(self.get_clock().now().nanoseconds / 1e9)
        if now - self.last_loop < self.config.dt:
            return
        self.last_loop = int(self.get_clock().now().nanoseconds / 1e9)

        command = self.get_command()

        # Read imu data. Orientation will be None if no data was available
        quat_orientation = (
            self.read_orientation()
        )
        self.state.quat_orientation = quat_orientation

        # Step the controller forward by dt
        self.controller.run(self.state, command, self.disp)

        # Update the pwm widths going to the servos
        self.hardware_interface.set_actuator_postions(self.state.joint_angles)

        # switch to TROT
        self.state.behavior_state = BehaviorState.TROT

    def listener_callback(self, msg):
        if self.time_last is None:
            self.time_last = self.get_clock().now().nanoseconds
            return
        self.time_now = self.get_clock().now().nanoseconds
        self.v_x = msg.linear.x
        self.v_z = msg.angular.z


def main(args=None):
    rclpy.init(args=args)

    mp = MiniPupper()

    rclpy.spin(mp)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mp.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
