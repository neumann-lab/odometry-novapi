"""
gemini - assumption college sriracha
{
    "test": 05
    "name": odometry-test
}

Piyaphat Liamwilai, 2024-05-28
"""

import novapi
import time
import math
from mbuild.encoder_motor import encoder_motor_class
from mbuild import power_expand_board
from mbuild import gamepad
from mbuild.led_matrix import led_matrix_class
from mbuild.smart_camera import smart_camera_class
from mbuild.ranging_sensor import ranging_sensor_class
from mbuild.smartservo import smartservo_class
from mbuild import power_manage_module

left_forward_wheel = encoder_motor_class("M3", "INDEX1")
right_forward_wheel = encoder_motor_class("M5", "INDEX1")
left_back_wheel = encoder_motor_class("M1", "INDEX1")
right_back_wheel = encoder_motor_class("M6", "INDEX1")


"""
odometry class
used for doing the odometry calculations
"""
class odometry:
    """
    Initialize the odometry class with the encoders of the four wheels
    left_front_encoder: encoder of the left front wheel
    left_rear_encoder: encoder of the left rear wheel (left back wheel)
    right_front_encoder: encoder of the right front wheel
    right_rear_encoder: encoder of the right rear wheel (right back wheel)
    """
    def __init__(self, left_front_encoder: encoder_motor_class, left_rear_encoder: encoder_motor_class, right_front_encoder: encoder_motor_class, right_rear_encoder: encoder_motor_class):
        self.left_front_encoder = left_front_encoder
        self.left_rear_encoder = left_rear_encoder
        self.right_front_encoder = right_front_encoder
        self.right_rear_encoder = right_rear_encoder
        self.wheel_radius = 0.04 # wheel radius (meter)
        self.wheel_distance = 0.35 # wheel distance from center of robot (meter)
        self.dt = 0.1 # time interval (second)
        self.x = 0.0 # x position (meter)
        self.y = 0.0 # y position (meter)
        self.theta = 0.0 # heading angle (radian)

    def get_gyro_angle(self):
        return novapi.get_roll()
    
    def update_position(self):
        # get the rpm of each wheel
        left_front_rpm = self.left_front_encoder.get_value("speed")
        left_rear_rpm = self.left_rear_encoder.get_value("speed")
        right_front_rpm = self.right_front_encoder.get_value("speed")
        right_rear_rpm = self.right_rear_encoder.get_value("speed")
        
        # get the gyro angle
        theta = self.get_gyro_angle()

        # calculate the wheel speeds using the circumference of the wheel
        left_front_speed = (left_front_rpm * 2 * math.pi * self.wheel_radius) / 60
        left_rear_speed = (left_rear_rpm * 2 * math.pi * self.wheel_radius) / 60
        right_front_speed = (right_front_rpm * 2 * math.pi * self.wheel_radius) / 60
        right_rear_speed = (right_rear_rpm * 2 * math.pi * self.wheel_radius) / 60

        velocity_x = (left_front_speed + left_rear_speed + right_front_speed + right_rear_speed) / 4
        velocity_y = (left_front_speed + left_rear_speed + right_front_speed + right_rear_speed) / 4

        # update the position
        self.x += velocity_x * math.cos(self.theta) * self.dt
        self.y += velocity_y * math.sin(self.theta) * self.dt
        self.theta = theta
        
        return self.x, self.y, self.theta

    def get_position(self):
        return self.x, self.y, self.theta
    
robot_odometry = odometry(left_forward_wheel, left_back_wheel, right_forward_wheel, right_back_wheel)

while True:
    robot_odometry.update_position()
    print(robot_odometry.get_position())
    time.sleep(robot_odometry.dt)