#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import os
import cv2
import numpy as np
import time
from lib.Robot import Robot
from lib.MapLib import Map2D
from lib.utils import read_light

def main(args):

    try:
        #robot = Robot(init_position = [init_x, init_y, 0.0])
        if read_light() > 1550:
            # negro
            pass
        else:
            # blanco
            pass

    except KeyboardInterrupt:
    # except the program gets interrupted by Ctrl+C on the keyboard.
    # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()
        print('do something to stop Robot when we Ctrl+C ...')


if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--mapfile", help="path to find map file",
                        default="P4/mapa0.txt")
    parser.add_argument("-ix", "--initx", help="initial position x",
                        default=0, type = int)
    parser.add_argument("-iy", "--inity", help="initial position y",
                        default=0, type = int)                    
    parser.add_argument("-fx", "--finalx", help="final position x",
                        default=0, type = int)
    parser.add_argument("-fy", "--finaly", help="final position y",
                        default=2, type = int)                    
    args = parser.parse_args()
    main(args)



