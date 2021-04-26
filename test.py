#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import sys
import cv2
import numpy as np
import time
from lib.Robot import Robot

def acc(v_f, robot):
    v_s = 0.01
    v = v_s
    while v < v_f:
        time.sleep(0.1)
        v += v_s
        robot.setSpeed(v, 0.0)
        

def main():
    try:

        robot = Robot(init_position=[0.0,0.0,1.579]) 
        robot.startOdometry()
        
        robot.go_to(0,-0.3, 0,0,0.015000001, 0.015)
        robot.stopOdometry()


    except KeyboardInterrupt: 
    # except the program gets interrupted by Ctrl+C on the keyboard.
    # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()

if __name__ == "__main__":

    main()


