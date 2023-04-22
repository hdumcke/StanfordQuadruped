import numpy as np
import sys
import os
from copy import deepcopy
sys.path.append(os.path.join(os.path.expanduser('~'), 'StanfordQuadruped'))
from src.Controller import Controller
from src.Command import Command
from src.State import BehaviorState, State
from src.Utilities import deadband, clipped_first_order_filter
from MangDang.mini_pupper.HardwareInterface import HardwareInterface
from MangDang.mini_pupper.Config import Configuration
from pupper.Kinematics import four_legs_inverse_kinematics
from MangDang.mini_pupper.display import Display
from MangDang.mini_pupper.shutdown import ShutDown

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy
from rcl_interfaces.msg import ParameterDescriptor
from nav_msgs.msg import Odometry
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from transforms3d.euler import quat2euler


class MiniPupper(Node):

    def __init__(self):
        super().__init__('mini_pupper_controller')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10)
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(Odometry, 'odom_legs', 10)
        self.broadcaster = TransformBroadcaster(self, 10)  # odom frame broadcaste
        self.time_last = None
        self.time_now = None
        self.orientation = np.array([1.0, 0.0, 0.0, 0.0])
        self.v_x = 0.
        self.v_y = 0.
        self.a_z = 0.
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
        self.joy_command = Command()
        self.activated = False
        self.activated_by = None
        self.previous_gait_toggle = 0
        self.previous_state = BehaviorState.REST
        self.previous_hop_toggle = 0
        self.previous_activate_toggle = 0
        self.sutdown_time = ShutDown()
        self.message_dt = 0.001 # publishing freqyency of /joy topic

        self.get_logger().info("Summary of gait parameters:")
        self.get_logger().info("dt: %s" % self.config.dt)
        self.get_logger().info("overlap time: %s" % self.config.overlap_time)
        self.get_logger().info("swing time: %s" % self.config.swing_time)
        self.get_logger().info("z clearance: %s" % self.config.z_clearance)
        self.get_logger().info("x shift: %s" % self.config.x_shift)

    def declare_params(self):
        # ################### COMMANDS ####################
        self.declare_parameter('max_x_velocity', 0.20, ParameterDescriptor(description=''))
        self.declare_parameter('max_y_velocity', 0.20, ParameterDescriptor(description=''))
        self.declare_parameter('max_yaw_rate', 2.0, ParameterDescriptor(description=''))
        self.declare_parameter('max_pitch', 20.0 * np.pi / 180.0, ParameterDescriptor(description=''))

        # ################### MOVEMENT PARAMS ####################
        self.declare_parameter('z_time_constant', 0.02, ParameterDescriptor(description=''))
        self.declare_parameter('z_speed', 0.01, ParameterDescriptor(description='maximum speed [m/s]'))
        self.declare_parameter('pitch_deadband', 0.02, ParameterDescriptor(description=''))
        self.declare_parameter('pitch_time_constant', 0.25, ParameterDescriptor(description=''))
        self.declare_parameter('max_pitch_rate', 0.15, ParameterDescriptor(description=''))
        self.declare_parameter('roll_speed', 0.16, ParameterDescriptor(description='maximum roll rate [rad/s]'))
        self.declare_parameter('yaw_time_constant', 0.3, ParameterDescriptor(description=''))
        self.declare_parameter('max_stance_yaw', 1.2, ParameterDescriptor(description=''))
        self.declare_parameter('max_stance_yaw_rate', 1.5, ParameterDescriptor(description=''))

        # ################### STANCE ####################
        self.declare_parameter('delta_x', 0.059, ParameterDescriptor(description=''))
        self.declare_parameter('delta_y', 0.050, ParameterDescriptor(description=''))
        self.declare_parameter('x_shift', 0.0, ParameterDescriptor(description=''))
        self.declare_parameter('z_shift', 0.0, ParameterDescriptor(description=''))
        self.declare_parameter('default_z_ref', -0.08, ParameterDescriptor(description=''))

        # ################### SWING ######################
        self.declare_parameter('z_clearance', 0.03, ParameterDescriptor(description=''))
        self.declare_parameter('alpha', 0.5, ParameterDescriptor(description='Ratio between touchdown distance and total horizontal stance movement'))
        self.declare_parameter('beta', 0.5, ParameterDescriptor(description='Ratio between touchdown distance and total horizontal stance movement'))

        # ################### GAIT #######################
        self.declare_parameter('dt', 0.015, ParameterDescriptor(description=''))
        self.declare_parameter('overlap_time', 0.09, ParameterDescriptor(description='duration of the phase where all four feet are on the ground'))
        self.declare_parameter('swing_time', 0.1, ParameterDescriptor(description='duration of the phase when only two feet are on the ground'))

    def set_params(self):
        # ################### COMMANDS ####################
        self.config.max_x_velocity = self.get_parameter('max_x_velocity').get_parameter_value().double_value
        self.config.max_y_velocity = self.get_parameter('max_y_velocity').get_parameter_value().double_value
        self.config.max_yaw_rate = self.get_parameter('max_yaw_rate').get_parameter_value().double_value
        self.config.max_pitch = self.get_parameter('max_pitch').get_parameter_value().double_value

        # ################### MOVEMENT PARAMS ####################
        self.config.z_time_constant = self.get_parameter('z_time_constant').get_parameter_value().double_value
        self.config.z_speed = self.get_parameter('z_speed').get_parameter_value().double_value
        self.config.pitch_deadband = self.get_parameter('pitch_deadband').get_parameter_value().double_value
        self.config.pitch_time_constant = self.get_parameter('pitch_time_constant').get_parameter_value().double_value
        self.config.max_pitch_rate = self.get_parameter('max_pitch_rate').get_parameter_value().double_value
        self.config.roll_speed = self.get_parameter('roll_speed').get_parameter_value().double_value
        self.config.yaw_time_constant = self.get_parameter('yaw_time_constant').get_parameter_value().double_value
        self.config.max_stance_yaw = self.get_parameter('max_stance_yaw').get_parameter_value().double_value
        self.config.max_stance_yaw_rate = self.get_parameter('max_stance_yaw_rate').get_parameter_value().double_value

        # ################### STANCE ####################
        self.config.delta_x = self.get_parameter('delta_x').get_parameter_value().double_value
        self.config.delta_y = self.get_parameter('delta_y').get_parameter_value().double_value
        self.config.x_shift = self.get_parameter('x_shift').get_parameter_value().double_value
        self.config.z_shift = self.get_parameter('z_shift').get_parameter_value().double_value
        self.config.default_z_ref = self.get_parameter('default_z_ref').get_parameter_value().double_value

        # ################### SWING ######################
        self.config.z_clearance = self.get_parameter('z_clearance').get_parameter_value().double_value
        self.config.alpha = self.get_parameter('alpha').get_parameter_value().double_value
        self.config.beta = self.get_parameter('beta').get_parameter_value().double_value

        # ################### GAIT #######################
        self.config.dt = self.get_parameter('dt').get_parameter_value().double_value
        self.config.overlap_time = self.get_parameter('overlap_time').get_parameter_value().double_value
        self.config.swing_time = self.get_parameter('swing_time').get_parameter_value().double_value

    def get_command_from_cmd_vel(self):
        command = Command()
        command.horizontal_velocity = np.array([self.v_x, self.v_y])
        command.yaw_rate = self.a_z
        return command

    def get_command_from_joy(self):
        return self.joy_command

    def get_command(self):

        if self.activated_by == 'cmd_vel':
            return self.get_command_from_cmd_vel()
        else:
            return self.get_command_from_joy()

    def read_orientation(self):
        return self.orientation

    def timer_callback(self):
        if not self.activated:
            return
        if self.time_now is None:
            return
        if self.state.behavior_state == BehaviorState.REST:
            self.set_params()
        # Deactivate if no messages are received
        if self.get_clock().now().nanoseconds - self.time_now > self.active_timeout * 1e+9:
            self.v_x = 0.
            self.v_y = 0.
            self.a_z = 0.
            self.activated = False
            self.disp.show_state(BehaviorState.DEACTIVATED)
            self.state.behavior_state == BehaviorState.REST

        command = self.get_command()

        # Read imu data. Orientation will be None if no data was available
        quat_orientation = (
            self.read_orientation()
        )
        self.state.quat_orientation = quat_orientation

        #if self.state.behavior_state == BehaviorState.REST:
        #    (roll, pitch, yaw) = quat2euler(quat_orientation)
        #    command.roll = roll
        #    command.pitch = pitch

        # Step the controller forward by dt
        self.controller.run(self.state, command, self.disp)

        # Update the pwm widths going to the servos
        self.hardware_interface.set_actuator_postions(self.state.joint_angles)

        self.publish_odometry()

        # activate TROT for cmd_vel
        if self.activated_by == 'cmd_vel' and self.previous_state == BehaviorState.REST:
            self.state.behavior_state == BehaviorState.TROT


    def listener_callback(self, msg):
        if self.time_last is None:
            self.time_last = self.get_clock().now().nanoseconds
            return
        self.activated_by = 'cmd_vel'
        self.activated = True
        self.time_now = self.get_clock().now().nanoseconds
        self.v_x = msg.linear.x
        self.v_y = msg.linear.y
        self.a_z = msg.angular.z

    def publish_odometry(self):
        x = 0.0
        y = 0.0
        z = 0.0
        quat_x = 0.0
        quat_y = 0.0
        quat_z = 0.0
        quat_w = 1.0
        msg = Odometry()
        current_time = self.get_clock().now().to_msg()
        msg.header.stamp = current_time
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.x = quat_x
        msg.pose.pose.orientation.y = quat_y
        msg.pose.pose.orientation.z = quat_z
        msg.pose.pose.orientation.w = quat_w
        msg.twist.twist.linear.x = self.v_x
        msg.twist.twist.linear.y = self.v_y
        msg.twist.twist.linear.z = 0.
        msg.twist.twist.angular.x = 0.
        msg.twist.twist.angular.y = 0.
        msg.twist.twist.angular.z = self.a_z
        msg.pose.covariance[0] = -1.0  # Note : we dont need a covariance matrix because EKF is configured not to use pose (x,y,z,roll,pitch,yaw) from odometry
        msg.twist.covariance[0] = 0.01  # vx variance = 0.01m/s
        msg.twist.covariance[35] = 0.05  # wz variance = 0.05rad/s ~3deg/s (must be higher thant IMU so EKF uses IMU)
        self.publisher.publish(msg)

        tfs = TransformStamped()
        tfs.header.stamp = current_time
        tfs.header.frame_id = 'world'
        tfs.child_frame_id = 'base_link'
        tfs.transform.translation.x = msg.pose.pose.position.x
        tfs.transform.translation.y = msg.pose.pose.position.y
        tfs.transform.translation.z = msg.pose.pose.position.z
        tfs.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(tfs)

    def imu_callback(self, msg):
        self.orientation[0] = msg.orientation.w
        self.orientation[1] = msg.orientation.x
        self.orientation[2] = msg.orientation.y
        self.orientation[3] = msg.orientation.z

    def joy_callback(self, msg):
        self.activated_by = 'joy'
        command = Command()
        ####### Handle discrete commands ########

        # Check for shotdown requests
        if msg.buttons[3]:
            self.disp.show_state(BehaviorState.SHUTDOWN)
            self.sutdown_time.request_shutdown()
        else:
            self.sutdown_time.cancel_shutdown()

        # Check if requesting a state transition to trotting, or from trotting to resting
        gait_toggle = msg.buttons[5]
        command.trot_event = (gait_toggle == 1 and self.previous_gait_toggle == 0)

        # Check if requesting a state transition to hopping, from trotting or resting
        hop_toggle = msg.buttons[1]
        command.hop_event = (hop_toggle == 1 and self.previous_hop_toggle == 0)

        activate_toggle = msg.buttons[4]
        command.activate_event = (activate_toggle == 1 and self.previous_activate_toggle == 0)

        # Update previous values for toggles and state
        self.previous_gait_toggle = gait_toggle
        self.previous_hop_toggle = hop_toggle
        self.previous_activate_toggle = activate_toggle

        ####### Handle continuous commands ########
        x_vel = msg.axes[1] * self.config.max_x_velocity
        y_vel = msg.axes[0] * -self.config.max_y_velocity
        command.horizontal_velocity = np.array([x_vel, y_vel])
        command.yaw_rate = msg.axes[2] * -self.config.max_yaw_rate

        pitch = msg.axes[5] * self.config.max_pitch
        deadbanded_pitch = deadband(
            pitch, self.config.pitch_deadband
        )
        pitch_rate = clipped_first_order_filter(
            self.state.pitch,
            deadbanded_pitch,
            self.config.max_pitch_rate,
            self.config.pitch_time_constant,
        )
        command.pitch = self.state.pitch + self.message_dt * pitch_rate

        height_movement = msg.axes[13]
        command.height = self.state.height - self.message_dt * self.config.z_speed * height_movement

        roll_movement = - msg.axes[12]
        command.roll = self.state.roll + self.message_dt * self.config.roll_speed * roll_movement

        # Hande activation state
        if command.activate_event:
            self.activated = not self.activated
            if not self.activated:
                self.disp.show_state(BehaviorState.DEACTIVATED)

        if self.activated:
            self.time_now = self.get_clock().now().nanoseconds
        self.joy_command = deepcopy(command)


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
