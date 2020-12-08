#!/bin/python
from gpiozero import Button #import button from the Pi GPIO library
import time # import time functions
import os #imports OS library for Shutdown control

startButton = Button(26) # defines the button as an object and chooses GPIO 26

while True: #infinite loop
    if startButton.is_pressed: #Check to see if button is pressed
		os.system("cd /home/pi/Avalanche-Drone/build/bin && sudo ./Avalanche-drone UserConfig.txt UserConfig.txt")
		break
    time.sleep(1) # wait to loop again so we dont use the processor too much.

