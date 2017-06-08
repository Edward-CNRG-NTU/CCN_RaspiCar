#!/usr/bin/python
# motor_control.py
# Control car with PWM
#
# Author : DWARD CHEN
# Date   : 2017/05/08

import RPi.GPIO as GPIO
import time


class MotorControl(object):
    def __init__(self, control_pin=[16, 18, 11, 13], t=0.1, dc_level=80, balance=1.0, pwm_freq=500):
        self.control_pin = control_pin
        self.t = t
        self.balance = balance
        self.l_level = dc_level * 2 / (balance + 1)
        self.r_level = self.l_level * balance

        GPIO.setmode(GPIO.BOARD)
        [GPIO.setup(control_pin[pin], GPIO.OUT, initial=GPIO.LOW) for pin in range(4)]

        self.pwm_r1 = GPIO.PWM(control_pin[0], pwm_freq)
        self.pwm_r2 = GPIO.PWM(control_pin[1], pwm_freq)
        self.pwm_l1 = GPIO.PWM(control_pin[2], pwm_freq)
        self.pwm_l2 = GPIO.PWM(control_pin[3], pwm_freq)
        self.pwm_r1.start(0)
        self.pwm_r2.start(0)
        self.pwm_l1.start(0)
        self.pwm_l2.start(0)

    def stop(self):
        self.pwm_r1.ChangeDutyCycle(0)
        self.pwm_r2.ChangeDutyCycle(0)
        self.pwm_l1.ChangeDutyCycle(0)
        self.pwm_l2.ChangeDutyCycle(0)

    def forward(self, speed=1.0, t=None):
        self.pwm_r1.ChangeDutyCycle(self.r_level*speed)
        self.pwm_r2.ChangeDutyCycle(0)
        self.pwm_l1.ChangeDutyCycle(self.l_level*speed)
        self.pwm_l2.ChangeDutyCycle(0)
        if t is None:
            time.sleep(self.t)
        else:
            time.sleep(t)
        self.stop()

    def backward(self, speed=0.8, t=None):
        self.pwm_r1.ChangeDutyCycle(0)
        self.pwm_r2.ChangeDutyCycle(self.r_level*speed)
        self.pwm_l1.ChangeDutyCycle(0)
        self.pwm_l2.ChangeDutyCycle(self.l_level*speed)
        if t is None:
            time.sleep(self.t)
        else:
            time.sleep(t)
        self.stop()

    def turn_left(self, speed=0.6, t=None):
        self.pwm_r1.ChangeDutyCycle(self.r_level*speed)
        self.pwm_r2.ChangeDutyCycle(0)
        self.pwm_l1.ChangeDutyCycle(0)
        self.pwm_l2.ChangeDutyCycle(0)
        if t is None:
            time.sleep(self.t)
        else:
            time.sleep(t)
        self.stop()

    def turn_right(self, speed=0.6, t=None):
        self.pwm_r1.ChangeDutyCycle(0)
        self.pwm_r2.ChangeDutyCycle(0)
        self.pwm_l1.ChangeDutyCycle(self.l_level*speed)
        self.pwm_l2.ChangeDutyCycle(0)
        if t is None:
            time.sleep(self.t)
        else:
            time.sleep(t)
        self.stop()

    def arbitrary_speed(self, speed=[1.0, 1.0], t=None):
        if 0 < speed[0]:
            self.pwm_r1.ChangeDutyCycle(self.r_level * speed[0])
            self.pwm_r2.ChangeDutyCycle(0)
        elif speed[0] < 0:
            self.pwm_r1.ChangeDutyCycle(0)
            self.pwm_r2.ChangeDutyCycle(self.r_level * speed[0])
        if 0 < speed[1]:
            self.pwm_l1.ChangeDutyCycle(self.l_level * speed[1])
            self.pwm_l2.ChangeDutyCycle(0)
        elif speed[1] < 0:
            self.pwm_l1.ChangeDutyCycle(0)
            self.pwm_l2.ChangeDutyCycle(self.l_level * speed[1])
        if t is None:
            return
        else:
            time.sleep(t)
            self.stop()

    def cleanup(self):
        self.stop()
        self.pwm_r1.stop()
        self.pwm_r2.stop()
        self.pwm_l1.stop()
        self.pwm_l2.stop()
        GPIO.cleanup()
