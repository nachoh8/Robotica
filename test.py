#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import sys
import cv2
import numpy as np
import time
from lib.Robot import Robot
from multiprocessing import Process, Value, Array, Lock


MAGIC_NUMBER_x = (0.3 * 180 / np.pi) / (1.983 * (1 / 0.03))

offset = 2357.4
def print_gyro(robot, x):
    acum = 0
    acum_2 = 0
    t = 0
    n = 0
    n_2 = 0
    while True:
        g = (robot.read_gyro() - offset) * MAGIC_NUMBER_x
        acum += g
        n += 1
        #print(acum / n)
        
        t += 0.03
        if t >= 0.99:
            aux = acum / n
            #print("mean " + str(aux))
            n_2 += 1
            acum_2 += aux
            print("mean " + str(acum_2 / n_2))
            acum = 0
            n = 0
            t = 0
        
        time.sleep(0.03)

def main():
    
    try:

        robot = Robot(init_position=[0.0,0.0,0.0]) 
        #p_gyro = Process(target=print_gyro, args=(robot,1))
        #p_gyro.start()
        
        robot.startOdometry()
        error = 00.5
        
        robot.go_to(0, 0.3, 0, 0, np.pi/2, error)
        robot.setSpeed(0.0,0.0)
        print(robot.readOdometry())
        time.sleep(2)
        robot.go_to(0.0, -0.3, 0, 0, 0.0, error)
        print(robot.readOdometry())
        robot.setSpeed(0.0,0.0)
        
        robot.stopOdometry()


    except KeyboardInterrupt: 
    # except the program gets interrupted by Ctrl+C on the keyboard.
    # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()

if __name__ == "__main__":

    main()


