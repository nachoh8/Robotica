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

        map_file = args.mapfile
        init_pos = (args.initx, args.inity
        final_pos = (args.finalx, args.finaly)
        # Instantiate Odometry with your own files from P2/P3
        # inicializar el robot en la mitad de la baldosa (0.4x0.4)
        init_x = init_pos[0]* 0.4 + 0.2
        init_y = init_pos[1] * 0.4 + 0.2
        robot = Robot(init_position = [init_x, init_y, 0.0])


        # 1. load map and compute costs and path
        myMap = Map2D(map_file)
        path = myMap.planPath(init_pos[0], init_pos[1], final_pos[0], final_pos[1])
        if path is None:
			print("No he encontrado el camino")
			exit(1)
        print(path)
       
        # 2. launch updateOdometry thread()
        robot.startOdometry()

        # 3. perform trajectory
        while len(path) > 0:
            next_pos = path.pop(0)
            dir_pos = ((next_pos[0] - init_pos[0]) * 0.4, (next_pos[1] - init_pos[1]) * 0.4)
            x, y, th = robot.readOdometry()
            if not robot.go(x + dir_pos[0], y + dir_pos[1], error=0.005):
                print("Recalculando ruta...")
				# Añadir pared detectada
                neigh = myMap.get_numneigh(dir_pos[0], dir_pos[1])
                for n in neigh:
                    myMap.deleteConnection(init_pos[0], init_pos[1], n)
                
				# Volver a planear la ruta
                path = myMap.planPath(init_pos[0], init_pos[1], final_pos[0], final_pos[1])
                if path is None:
					print("Me he perdido")
					break
                print(path)
            else:
                init_pos = next_pos
                
        print("Ha llegado a su destino")
				        
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



