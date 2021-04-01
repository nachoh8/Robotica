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

        robot.go(0, -1)

        robot.stopOdometry()


    except KeyboardInterrupt: 
    # except the program gets interrupted by Ctrl+C on the keyboard.
    # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()

if __name__ == "__main__":

    main()


