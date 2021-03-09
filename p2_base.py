#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import numpy as np
import time
from Robot import *
import math
 

def wait_pos():
        x, y, th = robot.readOdometry()
        time.sleep(0.1)
        
def main(args):
    try:
        if args.radioD < 0:
            print('d must be a positive value')
            exit(1)

        # Instantiate Odometry. Default value will be 0,0,0
        # robot = Robot(init_position=args.pos_ini)
        robot = Robot()

        print("X value at the beginning from main X= %.2f" %(robot.x.value))

        # 1. launch updateOdometry Process()
        robot.startOdometry()

        # 2. perform trajectory


        # DUMMY CODE! delete when you have your own

        # PART 1:
        dl = 0.1
        # 90º dch
        robot.setSpeed(0,-0.25)
        th = 0
        while th > -math.pi / 2:
                robot.lock_odometry.acquire()
                th = robot.th.value
                print(th)
                robot.lock_odometry.release()
                time.sleep(dl)
        robot.setSpeed(0,0)
        #wait_th(robot, -math.pi / 4)
               
        # arco izq
        time.sleep(2)
        robot.setSpeed(0.1,0.25)
        while th < math.pi / 2:
                robot.lock_odometry.acquire()
                th = robot.th.value
                print(th)
                robot.lock_odometry.release()
                time.sleep(dl)
        robot.setSpeed(0,0)
        
        # arco dch
        time.sleep(2)
        robot.setSpeed(0.1,-0.25)
        while th > -math.pi / 2:
                robot.lock_odometry.acquire()
                th = robot.th.value
                print(th)
                robot.lock_odometry.release()
                time.sleep(dl)
        robot.setSpeed(0,0)
        
        # arco dch
        time.sleep(2)
        robot.setSpeed(0.1,-0.25)
        while th > math.pi / 2 or th < 0:
                robot.lock_odometry.acquire()
                th = robot.th.value
                print(th)
                robot.lock_odometry.release()
                time.sleep(dl)
        robot.setSpeed(0,0)
        
        # arco izq
        time.sleep(2)
        robot.setSpeed(0.1,0.25)
        while th < -math.pi / 2 or th > 0:
                robot.lock_odometry.acquire()
                th = robot.th.value
                print(th)
                robot.lock_odometry.release()
                time.sleep(dl)
        robot.setSpeed(0,0)


        # PART 2:
        # robot.setSpeed()
        # until ...

        # ...



        # 3. wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors,
        # and restore the LED to the control of the BrickPi3 firmware.
        robot.stopOdometry()


    except KeyboardInterrupt:
    # except the program gets interrupted by Ctrl+C on the keyboard.
    # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()

if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--radioD", help="Radio to perform the 8-trajectory (mm)",
                        type=float, default=40.0)
    args = parser.parse_args()

    main(args)



