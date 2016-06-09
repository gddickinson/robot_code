# -*- coding: utf-8 -*-
"""
Created on Sat Apr 30 20:25:37 2016

@author: pi
"""

import RPi.GPIO as GPIO
import time

pin = 21

GPIO.setmode(GPIO.BCM)

GPIO.setup(pin, GPIO.OUT)

GPIO.output(pin, GPIO.HIGH)

time.sleep(1)

GPIO.output(pin, GPIO.LOW)

