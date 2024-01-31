#
# IceCube Haptic Tesla Glove Calibration System
# IceCube Solutions
# 31/1-24 - UiA Grimstad
# Maintainer: Martin Økter
#

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist


class TeslaGloveServo:
    def __init__(self):
        self.pos_min = 0
        self.pos_max = 0
        self.speed_max = 0
        self.current_pos = 0

    def __set__(self, instance, value):
        self.instance = value

    def __get__(self, instance, owner):
        return self.instance


class IceTeslaCalibration:

    def __init__(self):
        self.thumb_servo = TeslaGloveServo()
        self.index_servo = TeslaGloveServo()
        self.middle_servo = TeslaGloveServo()
        self.ring_servo = TeslaGloveServo()
        # self.little_servo = TeslaGloveServo()
        # self.palm_servo = TeslaGloveServo()

    def set_init_state(self):
        self.thumb_servo.current_pos = 90
        self.index_servo.current_pos = 90
        self.middle_servo.current_pos = 90
        self.ring_servo.current_pos = 90

    def set_thumb(self):
        self.thumb_servo.pos_min = 2
        print(self.thumb_servo.pos_min)

    def test(self):
        print(self.thumb_servo.current_pos)


class IceTeslaCalNode(Node):
    def __init__(self):
        super().__init__('ice_tesla_calibration')

        self.publisher_ = self.create_publisher(Twist, '/ice_glove_id1', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.tesla_cal_ = IceTeslaCalibration()
        self.tesla_cal_.set_init_state()

    def timer_callback(self):
        self.tesla_cal_.set_init_state()
        self.tesla_cal_.test()
        # self.get_logger().info('Ran the Hand Recognition')

        msg = Twist()
        # msg.data = control_string
        msg.linear.x = float(self.tesla_cal_.thumb_servo.current_pos)
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    tesla_cal = IceTeslaCalNode()

    rclpy.spin(tesla_cal)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # hand_gesture_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
