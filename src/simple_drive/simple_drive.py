#!/usr/bin/python

import rospy
import serial
import struct
import RPi.GPIO as io

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32



def main():
    rospy.init_node("simple_drive")
    
    left_dir_pin = 22
    right_dir_pin = 23
    left_PWM_pin = 17
    right_PWM_pin = 18

    io.setmode(io.BCM)
    io.setup(left_dir_pin, io.OUT)
    io.setup(right_dir_pin, io.OUT)
    io.setup(left_PWM_pin, io.OUT)
    io.setup(right_PWM_pin, io.OUT)

    left_motor = io.PWM(left_PWM_pin,19000)
    right_motor = io.PWM(right_PWM_pin,19000)

    left_motor.start(0)
    right_motor.start(0)


    def set_motors(leftSpeed, rightSpeed):
        io.output(left_dir_pin, True if leftSpeed > 0 else False)
        left_motor.ChangeDutyCycle(abs(leftSpeed))

        io.output(right_dir_pin, False if rightSpeed > 0 else True)
        right_motor.ChangeDutyCycle(abs(rightSpeed))
        print(leftSpeed)
        print(rightSpeed)

    def on_new_twist(data):
        #print((data.linear.x + data.angular.z) * 100, (data.linear.x - data.angular.z) * 100)
        set_motors((data.linear.x + data.angular.z) * 100, (data.linear.x - data.angular.z) * 100)



    subscriber_twist = rospy.Subscriber("cmd_vel", Twist, on_new_twist, queue_size=10)

    rospy.spin()
    io.cleanup()
