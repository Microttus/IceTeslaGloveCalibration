#
# IceCube Haptic Tesla Glove Calibration System
# IceCube Solutions
# 31/1-24 - UiA Grimstad
# Maintainer: Martin Økter
#
import sys

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

from enum import Enum

import csv


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

    def set_min_states(self):
        self.thumb_servo.current_pos = self.thumb_servo.pos_min
        self.index_servo.current_pos = self.index_servo.pos_min
        self.middle_servo.current_pos = self.middle_servo.pos_min
        self.ring_servo.current_pos = self.ring_servo.pos_min


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

        self.input_key_words = ['next', 'max', 'min', 'end']

    def run(self):

        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            print('Shutdown due to keyboard interrupt ...')
        except Exception:
            print('--> Thank you for using the IceCube Haptic Glove Calibration')
        finally:
            self.stop()

    def stop(self):
        print('Shutting down ... \nNo cake for you I guess ...')
        sys.exit()
        # print('Ups, cant kill GLadOS')

    def user_input(self):
        while True:
            user_input = input('New servo pos or "next" for next finger: ')
            if user_input not in self.input_key_words:
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

        print('\n')
        print('We will now adjust servo connected to', self.current_finger.name, 'finger')

        user_input = self.user_input()
        current_servo_obj = getattr(self.tesla_cal_, str(self.current_servo.name))

        if user_input == 'next':
            self.current_finger_index += 1
        elif user_input == 'min':
            min_pos = getattr(current_servo_obj, 'current_pos')
            setattr(current_servo_obj, 'pos_min', min_pos)
            print('--> Min value of', self.current_finger.name, 'is set to:', min_pos)
        elif user_input == 'max':
            max_pos = getattr(current_servo_obj, 'current_pos')
            setattr(current_servo_obj, 'pos_max', max_pos)
            print('--> Max value of', self.current_finger.name, 'is set to:', max_pos)
        elif user_input == 'end':
            self.end_of_loop()
        else:
            setattr(current_servo_obj, 'current_pos', int(user_input))
            print('--> Position of', self.current_finger.name, 'is set to:', user_input)

    def end_of_loop(self):
        profile_name = input('New profile name: ')

        profile_data = [str(profile_name),
                        self.tesla_cal_.thumb_servo.pos_min,
                        self.tesla_cal_.thumb_servo.pos_max,
                        self.tesla_cal_.index_servo.pos_min,
                        self.tesla_cal_.index_servo.pos_max,
                        self.tesla_cal_.middle_servo.pos_min,
                        self.tesla_cal_.middle_servo.pos_max,
                        self.tesla_cal_.ring_servo.pos_min,
                        self.tesla_cal_.ring_servo.pos_max]

        with open('icecube_tesla_glove_profiles.csv', 'a', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(profile_data)

        csv_file.close()

        print('Profile data has been added!')
        
        raise Exception


    def timer_callback(self):
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


def main(args=None):
    rclpy.init(args=args)

    tesla_cal_node = IceTeslaCalNode()

    tesla_cal_node.run()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # hand_gesture_publisher.destroy_node()

    # rclpy.shutdown()


if __name__ == '__main__':
    main()
