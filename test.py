#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import cv2
import numpy as np
import time
from lib.Robot import Robot

def main():
    try:

        robot = Robot() 
        robot.startOdometry()

        while True:
            print(robot.read_ultrasonic())
            time.sleep(0.5)

        robot.stopOdometry()


    except KeyboardInterrupt: 
    # except the program gets interrupted by Ctrl+C on the keyboard.
    # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()

if __name__ == "__main__":

    main()


