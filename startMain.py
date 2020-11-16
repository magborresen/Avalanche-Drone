#!/bin/python

from gpiozero import Button
import time
import os

startButton = Button(26)

while True:
    if startButton.is_pressed:
        os.system("cd /home/pi/Avalanche-Drone/build/bin && sudo ./Avalanche-drone UserConfig.txt UserConfig.txt")
        break
    time.sleep(1)
