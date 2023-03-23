import numpy as np
import time
from src.IMU import IMU
from src.Controller import Controller
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
        super().__init__('mini_pupper')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription # prevent unused variable warning
        self.time_last = None
        self.time_now = None
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.v_x = 0.
        self.v_z = 0.
        self.active_timeout = 0.5

    # Create config
    config = Configuration()
    hardware_interface = HardwareInterface()
    disp = Display()
    disp.show_ip()


    # Create controller and user input handles
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )
    state = State()

    last_loop = time.time()

    print("Summary of gait parameters:")
    print("overlap time: ", config.overlap_time)
    print("swing time: ", config.swing_time)
    print("z clearance: ", config.z_clearance)
    print("x shift: ", config.x_shift)

    def timer_callback(self):
        if self.time_now is None:
            return
        # self.time_now set by cmd_vel subscriber
        if self.get_clock().now().nanoseconds - self.time_now > self.active_timeout * 2e+9:
            return
        msg= "%0.2f %0.2f" % (self.v_x, self.v_z)
        print(msg)

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
