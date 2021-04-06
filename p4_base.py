#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import os
import cv2
import numpy as np
import time
from lib.Robot import Robot
from lib.MapLib import Map2D

def main(args):
    """
    Example to load "mapa1.txt"
    """

    try:
        if not os.path.isfile(args.mapfile):
            print('Map file %s does not exist' % args.mapfile)
            exit(1)

        map_file = args.mapfile;
        init_pos = (args.initx, args.inity);
        final_pos = (args.finalx, args.finaly);
        # Instantiate Odometry with your own files from P2/P3
        # inicializar el robot en la mitad de la baldosa (0.4x0.4)
        init_x = init_pos[0]* 0.4 + 0.2
        init_y = init_pos[1] * 0.4 + 0.2
        robot = Robot(init_position = [init_x, init_y, 0.0])
        # ...

        # 1. load map and compute costs and path
        myMap = Map2D(map_file)
        myMap.planPath(init_pos[0], init_pos[1], final_pos[0], final_pos[1])
        path = myMap.currentPath
        if path is None: exit(1)
        print(path)
       
        # 2. launch updateOdometry thread()
        robot.startOdometry()
        # ...


        # 3. perform trajectory
        # robot.setSpeed(1,1) ...
        while len(path) > 0:
            next_pos = path.pop(0)
            dir_pos = ((next_pos[0] - init_pos[0]) * 0.4, (next_pos[1] - init_pos[1]) * 0.4)
            print(dir_pos)
            x, y, th = robot.readOdometry()
            print(x,y,th)
            if not robot.go(x + dir_pos[0], y + dir_pos[1]):
                print("Recalculando ruta...")
				# Añadir pared detectada
                neigh = myMap.get_numneigh(dir_pos[0], dir_pos[1])
                myMap.deleteConnection(init_pos[0], init_pos[1], neigh)
                
				# Volver a planear la ruta
                myMap.planPath(init_pos[0], init_pos[1], final_pos[0], final_pos[1])
                path = myMap.currentPath
                if path is None: break
                print(path)
            else:
                init_pos = next_pos
				
				
				
            # check if there are close obstacles
            # deal with them...
            # Avoid_obstacle(...) OR RePlanPath(...)
        
        # 4. wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors,
        # and restore the LED to the control of the BrickPi3 firmware.
        robot.stopOdometry()

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



