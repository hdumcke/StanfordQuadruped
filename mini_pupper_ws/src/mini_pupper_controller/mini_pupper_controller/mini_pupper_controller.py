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
        self.v_x = 0.
        self.v_z = 0.
        self.active_timeout = 0.5

        # Create config
        self.declare_params()
        self.config = Configuration()
        self.set_params()
        self.hardware_interface = HardwareInterface()
        self.disp = Display()
        self.disp.show_ip()
        self.timer = self.create_timer(self.config.dt, self.timer_callback)

        self.controller = Controller(
            self.config,
            four_legs_inverse_kinematics,
        )
        self.state = State()
        self.state.behavior_state = BehaviorState.REST
        self.set_rest_state = True

        self.get_logger().info("Summary of gait parameters:")
        self.get_logger().info("dt: %s" % self.config.dt)
        self.get_logger().info("overlap time: %s" % self.config.overlap_time)
        self.get_logger().info("swing time: %s" % self.config.swing_time)
        self.get_logger().info("z clearance: %s" % self.config.z_clearance)
        self.get_logger().info("x shift: %s" % self.config.x_shift)

    def declare_params(self):
        # ################### COMMANDS ####################
        self.declare_parameter('max_x_velocity', 0.20, '')
        self.declare_parameter('max_y_velocity', 0.20, '')
        self.declare_parameter('max_yaw_rate', 2, '')
        self.declare_parameter('max_pitch', 20.0 * np.pi / 180.0, '')

        # ################### MOVEMENT PARAMS ####################
        self.declare_parameter('z_time_constant', 0.02, '')
        self.declare_parameter('z_speed', 0.01, 'maximum speed [m/s]')
        self.declare_parameter('pitch_deadband', 0.02, '')
        self.declare_parameter('pitch_time_constant', 0.25, '')
        self.declare_parameter('max_pitch_rate', 0.15, '')
        self.declare_parameter('roll_speed', 0.16, 'maximum roll rate [rad/s]')
        self.declare_parameter('yaw_time_constant', 0.3, '')
        self.declare_parameter('max_stance_yaw', 1.2, '')
        self.declare_parameter('max_stance_yaw_rate', 1.5, '')

        # ################### STANCE ####################
        self.declare_parameter('delta_x', 0.059, '')
        self.declare_parameter('delta_y', 0.050, '')
        self.declare_parameter('x_shift', 0.0, '')
        self.declare_parameter('z_shift', 0.0, '')
        self.declare_parameter('default_z_ref', -0.08, '')

        # ################### SWING ######################
        self.declare_parameter('z_clearance', 0.03, '')
        self.declare_parameter('alpha', 0.5, 'Ratio between touchdown distance and total horizontal stance movement')
        self.declare_parameter('beta', 0.5, 'Ratio between touchdown distance and total horizontal stance movement')

        # ################### GAIT #######################
        self.declare_parameter('dt', 0.015, '')
        self.declare_parameter('overlap_time', 0.09, 'duration of the phase where all four feet are on the ground')
        self.declare_parameter('swing_time', 0.1, 'duration of the phase when only two feet are on the ground')

    def set_params(self):
        # ################### COMMANDS ####################
        self.config.max_x_velocity = self.get_parameter('max_x_velocity')
        self.config.max_y_velocity = self.get_parameter('max_y_velocity')
        self.config.max_yaw_rate = self.get_parameter('max_yaw_rate')
        self.config.max_pitch = self.get_parameter('max_pitch')

        # ################### MOVEMENT PARAMS ####################
        self.config.z_time_constant = self.get_parameter('z_time_constant')
        self.config.z_speed = self.get_parameter('z_speed')
        self.config.pitch_deadband = self.get_parameter('pitch_deadband')
        self.config.pitch_time_constant = self.get_parameter('pitch_time_constant')
        self.config.max_pitch_rate = self.get_parameter('max_pitch_rate')
        self.config.roll_speed = self.get_parameter('roll_speed')
        self.config.yaw_time_constant = self.get_parameter('yaw_time_constant')
        self.config.max_stance_yaw = self.get_parameter('max_stance_yaw')
        self.config.max_stance_yaw_rate = self.get_parameter('max_stance_yaw_rate')

        # ################### STANCE ####################
        self.config.delta_x = self.get_parameter('delta_x')
        self.config.delta_y = self.get_parameter('delta_y')
        self.config.x_shift = self.get_parameter('x_shift')
        self.config.z_shift = self.get_parameter('z_shift')
        self.config.default_z_ref = self.get_parameter('default_z_ref')

        # ################### SWING ######################
        self.config.z_clearance = self.get_parameter('z_clearance')
        self.config.alpha = self.get_parameter('alpha')
        self.config.beta = self.get_parameter('beta')

        # ################### GAIT #######################
        self.config.dt = self.get_parameter('dt')
        self.config.overlap_time = self.get_parameter('overlap_time')
        self.config.swing_time = self.get_parameter('swing_time')

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
        if self.get_clock().now().nanoseconds - self.time_now > self.active_timeout * 1e+9:
            if not self.set_rest_state:
                self.state.behavior_state = BehaviorState.REST
                self.disp.show_state(BehaviorState.REST)
                return
        self.set_rest_state = False

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
        # we have to cycle through REST at least once to set initial state
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
