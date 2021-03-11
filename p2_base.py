#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import numpy as np
import time
from lib.Robot import Robot
import math

def trayectoria1(robot: Robot, error: float): # trayectoria en 8, con radio = 0.4
        robot.go_to(0, -0.25, 0, 0, -np.pi/2, error)
        robot.go_to(0.1, 0.25, 0, 0, np.pi/2, error)
        robot.go_to(0.1, -0.25, 0, 0, -np.pi/2, error)
        robot.go_to(0.1, -0.25, 0, 0, np.pi/2, error)
        robot.go_to(0.1, 0.25, 0, 0, -np.pi/2, error)

def trayectoria2(robot: Robot, error: float): # trayectoria circuito, radio pequeño = 0.2, radio grande = 0.4
        robot.go_to(0, 0.25, 0, 0, np.pi/2, error)
        robot.go_to(0.1, -0.5, 0, 0, np.pi/12 , error)
        robot.setSpeed(0.25,0)
        time.sleep(4.4) # 1,1 m
        robot.setSpeed(0,0)
        robot.go_to(0.1, -0.25, 0, 0, 15*np.pi/16, 0.02)
        robot.setSpeed(0.25,0)
        time.sleep(4.4) # 1,1 m
        robot.setSpeed(0,0)
        robot.go_to(0.1, -0.5, 0, 0, np.pi/2 , error)
        

def main(args):
    try:
        init_pos = [0.0, 0.0, 0.0]
        robot = Robot(init_position=init_pos, log_file_name=args.fLog)
        
        print("Inital position " + str(robot.readOdometry()))

        robot.startOdometry()        
        
        error = 0.015 # rad
        if args.trayectoria == 1:
                trayectoria1(robot, error)
        elif args.trayectoria == 2:
                trayectoria2(robot, error)
        else:
                print("Trayectoria no reconocida")
        
        robot.stopOdometry()


    except KeyboardInterrupt:
        # Ctrl+C
        robot.stopOdometry()

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--trayectoria", help="trayectoria(1|2)",
                        type=int, default=1)
    parser.add_argument("-log", "--fLog", help="log file",
                        type=str, default="log_odom.csv")
    args = parser.parse_args()

    main(args)



