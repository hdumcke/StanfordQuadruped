import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy

from PS4Joystick import Joystick

PUPPER_COLOR = {"red":0, "blue":0, "green":255}

class MiniPupperJoy(Node):

    def __init__(self):
        super().__init__('mini_pupper_joy')
        self.publisher = self.create_publisher(Joy, 'joy', 10)
        self.freq = 20.
        self.joystick = Joystick()
        self.joystick.led_color(**PUPPER_COLOR)
        values = self.joystick.get_input()
        self.timer = self.create_timer(1./self.freq, self.timer_callback)

    def timer_callback(self):
        values = self.joystick.get_input()
    
        left_y = -values["left_analog_y"]
        right_y = -values["right_analog_y"]
        right_x = values["right_analog_x"]
        left_x = values["left_analog_x"]
    
        L2 = values["l2_analog"]
        R2 = values["r2_analog"]
    
        R1 = values["button_r1"]
        L1 = values["button_l1"]
    
        square = values["button_square"]
        x = values["button_cross"]
        circle = values["button_circle"]
        triangle = values["button_triangle"]
    
        dpadx = values["dpad_right"] - values["dpad_left"]
        dpady = values["dpad_up"] - values["dpad_down"]
    
        msg = Joy()
        msg.buttons.append(square)
        msg.buttons.append(x)
        msg.buttons.append(circle)
        msg.buttons.append(triangle)
        msg.buttons.append(L1)
        msg.buttons.append(R1)
        msg.axes.append(left_x)
        msg.axes.append(left_y)
        msg.axes.append(right_x)
        msg.axes.append(right_y)
        msg.axes.append(dpadx)
        msg.axes.append(dpady)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    joy = MiniPupperJoy()

    rclpy.spin(joy)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    jop.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
