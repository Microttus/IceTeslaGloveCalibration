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

from enum import Enum


class Fingers(Enum):
    thumb = 1
    index = 2
    middle = 3
    ring = 4
    little = 5
    palm = 6

class Fingers_servo(Enum):
    thumb_servo = 1
    index_servo = 2
    middle_servo = 3
    ring_servo = 4
    little_servo = 5
    palm_servo = 6


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

    def test(self):
        #self.set_thumb_servo(0, 180)
        print('Test activated')


class IceTeslaCalNode(Node):
    def __init__(self):
        super().__init__('ice_tesla_calibration')

        self.publisher_ = self.create_publisher(Twist, '/ice_glove_id1', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.tesla_cal_ = IceTeslaCalibration()
        self.tesla_cal_.set_init_state()

        print('Welcome to the IceCube Calibration Application for the Haptic Tesla GLove')

        self.current_finger_index = 1
        self.current_finger = Fingers(self.current_finger_index)
        self.current_servo = Fingers_servo(self.current_finger_index)

    def user_input(self):
        while True:
            user_input = input('New servo pos or next for next finger: ')
            if user_input != 'next' and user_input != 'max' and user_input != 'min':
                try:
                    user_input = int(user_input)
                    break
                except ValueError:
                    print('Not a valid input')
            else:
                break

        return user_input

    def main_cal_loop(self):
        self.current_finger = Fingers(self.current_finger_index)
        self.current_servo = Fingers_servo(self.current_finger_index)

        print('We will now adjust servo connected to', self.current_finger.name)

        user_input = self.user_input()

        if user_input == 'next':
            self.current_finger_index += 1
        elif user_input == 'min':
            #self.thumb_calibration(int(user_input))
            setattr(self.current_servo, 'pos_min', 80)
        elif user_input == 'max':
            #self.thumb_calibration(int(user_input))
            print('Heio')
        else:
            #self.thumb_calibration(int(user_input))
            setattr(self.current_servo, 'current_pos', int(user_input))



    def timer_callback(self):
        self.tesla_cal_.test()
        # self.get_logger().info('Ran the Hand Recognition')

        # Main switch for line of action
        self.main_cal_loop()

        msg = Twist()
        # msg.data = control_string
        msg.linear.x = float(self.tesla_cal_.thumb_servo.current_pos)
        msg.linear.y = float(self.tesla_cal_.index_servo.current_pos)
        msg.linear.z = float(self.tesla_cal_.middle_servo.current_pos)
        msg.angular.x = float(self.tesla_cal_.ring_servo.current_pos)
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)

    def thumb_calibration(self, instance, value):

        current_servo = str(self.current_finger.name + '_servo')

        print(current_servo)


        if self.tesla_cal_.thumb_servo.current_pos > 180:
            self.tesla_cal_.thumb_servo.current_pos = 0


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
