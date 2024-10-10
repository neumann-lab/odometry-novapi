"""
gemini - assumption college sriracha
{
    "test": 05
    "name": odometry-test
}

Piyaphat Liamwilai, 2024-10-10
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


# Hello ChatGPT
class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.setpoint = setpoint  # Desired value (target)
        self.integral = 0  # Sum of errors over time
        self.previous_error = 0  # Previous error (used for derivative)

    def update(self, current_value):
        # Calculate the error (setpoint - current value)
        error = self.setpoint - current_value
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term
        self.integral += error
        I = self.Ki * self.integral

        # Derivative term
        derivative = error - self.previous_error
        D = self.Kd * derivative

        # Calculate the output
        output = P + I + D

        # Save the current error for the next update
        self.previous_error = error

        return output

    def set_setpoint(self, setpoint):
        """ Update the target setpoint for the PID controller """
        self.setpoint = setpoint
        self.integral = 0  # Reset the integral to avoid wind-up
        self.previous_error = 0  # Reset previous error to avoid a large derivative spike

class motors:
    
    def drive(lf: int, lb: int, rf: int, rb: int):
        left_back_wheel.set_speed(lb) # left back :DDDDD
        right_back_wheel.set_speed(-rb)  # RIGHT BACK  
        right_forward_wheel.set_speed(-(rf))      # RIGHT FORWARD
        left_forward_wheel.set_speed(lf)             # LEFT BACK
    
    def stop():
        motors.drive(0, 0, 0, 0)
        
class util:

    def restrict(val, minimum, maximum):
        return max(min(val, maximum), minimum)
    
    def in_range(val, val2, range):
        return abs(val - val2) <= range

"""
For more information and explanation, please visit:
https://github.com/neumann-lab/holonomic-mecanum/
"""
class holonomic:        

    """
    Holonomic driving system for mecanum.
    vx, the desired x velocity
    vy, the desired y velocity
    wL, the desired angular velocity
    deadzone, the deadzone where the value lower than this value will be set to 0
    pid, enable pid control
    """
    def drive(vx, vy, wL, deadzone=5):
        global SPEED_MULTIPLIER
        # Create a deadzone so that if the joystick isn't moved perfectly,
        # the controller can still make the robot move perfectly.
        if math.fabs(vx) < math.fabs(deadzone):
            vx = 0
        if math.fabs(vy) < math.fabs(deadzone):
            vy = 0
        if math.fabs(wL) < math.fabs(deadzone):
            wL = 0
            
        # Calculation for the wheel speed
        # Visit https://github.com/neumann-lab/holonomic-mecanum/blob/main/th.md for the formula
        vFL = (vx + vy + wL) * SPEED_MULTIPLIER
        vFR = (-(vx) + vy - wL) * SPEED_MULTIPLIER
        vBL = (-(vx) + vy + wL) * SPEED_MULTIPLIER
        vBR = (vx + vy - wL) * SPEED_MULTIPLIER

        # Velocity
        vFL = util.restrict(vFL, -255, 255)
        vFR = util.restrict(vFR, -255, 255)
        vBL = util.restrict(vBL, -255, 255)
        vBR = util.restrict(vBR, -255, 255)
        # Drive motor
        motors.drive(vFL, vBL, vFR, vBR)

class robot:

    def __init__(self, odometry, drive):
        self.odometry = odometry
        self.drive = drive
        self.x_pid = PID(1, 0, 0, 0)
        self.y_pid = PID(1, 0, 0, 0)
        self.theta_pid = PID(1, 0, 0, 0)   

    def update(self):
        self.x, self.y, self.theta = self.odometry.update_position()

    def move_to(self, x, y, wL, range):

        self.x_pid.set_setpoint(x)
        self.y_pid.set_setpoint(y)
        self.theta_pid.set_setpoint(wL)

        while True:
            self.update()
            if util.in_range(self.x, x, range) and util.in_range(self.y, y, range) and util.in_range(self.theta, wL, range):
                break
            vx = self.x_pid.update(self.x)
            vy = self.y_pid.update(self.y)
            wL_ = self.theta_pid.update(self.theta)
            self.drive.drive(vx, vy, wL_)
            time.sleep(self.odometry.dt)

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
    
gemini = robot(odometry(left_forward_wheel, left_back_wheel, right_forward_wheel, right_back_wheel), holonomic)

while True:
    gemini.move_to(1, 1, 0, 0.1)
