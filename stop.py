#!/usr/bin/python
# -*- coding: UTF-8 -*-
from lib.Robot import Robot

def main():
    
    try:

        robot = Robot()
        robot.startOdometry()
        robot.stopOdometry()


    except KeyboardInterrupt: 
    # except the program gets interrupted by Ctrl+C on the keyboard.
    # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()

if __name__ == "__main__":

    main()


