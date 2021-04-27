#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import sys
import cv2
import numpy as np
import time
from lib.Robot import Robot

def main():
    try:

        robot = Robot(init_position=[0.0,0.0,0.0]) 
        robot.startOdometry()
        
        robot.setSpeed(0.0, -0.3)
        min_s = robot.read_ultrasonic()
        t = 0.0
        c = 0
        while t < 20:
            s = robot.read_ultrasonic()
            print(s)
            if s <= min_s:
                c = 0
                min_s = s
            else:
                c += 1

            if c > 1:
                break;
            # t += 0.1
            time.sleep(0.03)
        robot.stopOdometry()


    except KeyboardInterrupt: 
    # except the program gets interrupted by Ctrl+C on the keyboard.
    # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()

if __name__ == "__main__":

    main()


