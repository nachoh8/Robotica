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

    path = []
    init_pos = None
    fin_pos = None
    map_file = ""

    try:
        #robot = Robot(init_position = [init_x, init_y, 0.0])
        #if read_light() > 1550:
        if False:
            # negro
            print("negro")
            init_pos = (0,1)
            fin_pos = (4,3)
            path = [(0,0), (1,0), (2,0), (2,1), (2,2), (3,2), (4,2), (4,1)]
            map_file = "P5/mapaB_CARRERA.txt"
            
        else:
            # blanco
            print("blanco")
            init_pos = (4,1)
            fin_pos = (4,6)
            path = [(0,0), (1,0), (2,0), (2,1), (2,2), (3,2), (4,2), (4,1)]
            map_file = "P5/mapaA_CARRERA.txt"
            
        init_x = init_pos[0] * 0.4 + 0.2
        init_y = init_pos[1] * 0.4 + 0.2
        
        robot = Robot(init_position=[init_x, init_y, 0.0])
        myMap = Map2D(map_file)
                
        robot.startOdometry()

        # Perform trajectory
        while len(path) > 0:
            next_pos = path.pop(0)
            print(next_pos)
            dir_pos = ((next_pos[0] - init_pos[0]) * 0.4, (next_pos[1] - init_pos[1]) * 0.4)
            x, y, th = robot.readOdometry()
            print("Odom: " + str(x) + " " + str(y) + " "+ str(th))
            goal = (next_pos[0]* 0.4 + 0.2, next_pos[1]* 0.4 + 0.2)
            print("Destino: " + str(goal))
            if not robot.go(goal[0], goal[1], error=0.005):
                print("Recalculando ruta...")
                exit(0)
            else:
                init_pos = next_pos
        
        print("Fin S alcanzado")
        
        path = myMap.planPath(init_pos[0], init_pos[1], fin_pos[0], fin_pos[1])
        print(path)
        # Perform trajectory
        while len(path) > 0:
            next_pos = path.pop(0)
            print(next_pos)
            dir_pos = ((next_pos[0] - init_pos[0]) * 0.4, (next_pos[1] - init_pos[1]) * 0.4)
            x, y, th = robot.readOdometry()
            print("Odom: " + str(x) + " " + str(y) + " "+ str(th))
            goal = (next_pos[0]* 0.4 + 0.2, next_pos[1]* 0.4 + 0.2)
            print("Destino: " + str(goal))
            if not robot.go(goal[0], goal[1], error=0.005):
                print("Recalculando ruta...")
				# Añadir pared detectada
                neigh = myMap.get_numneigh(dir_pos[0], dir_pos[1])
                for n in neigh:
                    myMap.deleteConnection(init_pos[0], init_pos[1], n)
                
				# Volver a planear la ruta
                path = myMap.planPath(init_pos[0], init_pos[1], fin_pos[0], fin_pos[1])
                if path is None:
                    print("Me he perdido")
                    break
                print(path)
            else:
                init_pos = next_pos
        
        print("Fin PLAN")
                
        
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



