#!/bin/python

import RPi.GPIO as GPIO

gpio_number = 26

GPIO.setmode(GPIO.BCM)

GPIO.setup(gpio_number, GPIO.IN, pull_up_down=GPIO.PUD_UP)

try:
    GPIO.wait_for_edge(gpio_number, GPIO.FALLING)
    os.system("cd /home/pi/Avalanche-Drone/build/bin && ./Avalanche-drone UserConfig.txt UserConfig.txt")

except:
    pass

GPIO.cleanup()
