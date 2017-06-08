#!/usr/bin/python
# motor_control.py
# Control car with PWM
#
# Author : DWARD CHEN
# Date   : 2017/05/08

import RPi.GPIO as GPIO
import threading
import time
import motor_control


class MotorControl(motor_control.MotorControl):
    def __init__(self, control_pin=[16, 18, 11, 13], t=0.1, dc_level=80, balance=1.0, pwm_freq=500):
        super(MotorControl, self).__init__(control_pin, t, dc_level, balance, pwm_freq)

        self.thread_stopper = threading.Event()
        self.pending_operation = None

        def motor_control_routine():
            while not self.thread_stopper.is_set():
                if self.pending_operation:
                    self.pending_operation[0](*self.pending_operation[1])
                    self.pending_operation = None
                else:
                    time.sleep(0.1)

        t = threading.Thread(target=motor_control_routine)
        t.setDaemon(True)
        t.start()

    def forward(self, speed=1.0, t=None):
        while self.pending_operation:
            time.sleep(0.01)
        self.pending_operation = (super(MotorControl, self).forward, (speed, t))
        print('F')

    def backward(self, speed=0.8, t=None):
        while self.pending_operation:
            time.sleep(0.01)
        self.pending_operation = (super(MotorControl, self).backward, (speed, t))
        print('B')

    def turn_left(self, speed=0.6, t=None):
        while self.pending_operation:
            time.sleep(0.01)
        self.pending_operation = (super(MotorControl, self).turn_left, (speed, t))
        print('L')

    def turn_right(self, speed=0.6, t=None):
        while self.pending_operation:
            time.sleep(0.01)
        self.pending_operation = (super(MotorControl, self).turn_right, (speed, t))
        print('R')

    def arbitrary_speed(self, speed=[1.0, 1.0], t=None):
        while self.pending_operation:
            time.sleep(0.01)
        self.pending_operation = (super(MotorControl, self).arbitrary_speed, (speed, t))
        print('A')

    def cleanup(self):
        try:
            self.thread_stopper.set()
        except AttributeError:
            pass
        super(MotorControl, self).cleanup()
