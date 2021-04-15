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

        robot = Robot() 
        robot.startOdometry()
        v = float(sys.argv[1])
        
        x = 0.0
        num_b = int(sys.argv[2])
        for i in range(1,num_b):
            x_f = 0.4*i
            
            acc(v, robot)
            while x_f-x > 0:
                time.sleep(robot.P_CHECK_POS)
                x,y,th = robot.readOdometry()
            
            robot.setSpeed(0,0)
            print("odom")
            print(x,y,th)
            
            time.sleep(0.3)
        robot.stopOdometry()


    except KeyboardInterrupt: 
    # except the program gets interrupted by Ctrl+C on the keyboard.
    # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()

if __name__ == "__main__":

    main()


