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
from lib.rec_logo import RecLogo

def _8_A(robot):
    robot.go_to(0, -0.25, 0, 0, -np.pi/2, error)
    robot.go_to(0.1, 0.25, 0, 0, np.pi/2, error)
    robot.go_to(0.1, -0.25, 0, 0, -np.pi/2, error)

def _8_B(robot):
    robot.go_to(0, 0.25, 0, 0, np.pi/2, error)
    robot.go_to(0.1, -0.25, 0, 0, -np.pi/2, error)
    robot.go_to(0.1, 0.25, 0, 0, np.pi/2, error)

def main(args):

    path = []
    init_pos = None
    fin_pos = None
    logo_pos = None
    fin_dch = (0,6)
    fin_izq = (0,3)
    map_file = ""

    try:
        #robot = Robot(init_position = [init_x, init_y, 0.0])
        #if read_light() > 1550:
        traj_black = read_light() > 1550
        if False:
            # negro
            print("negro")
            init_pos = (0,1)
            fin_pos = (4,6)
            path = [(0,0), (1,0), (2,0), (2,1), (2,2), (3,2), (4,2), (4,1)] # TODO
            map_file = "P5/mapaB_CARRERA.txt"
            logo_pos = (0,5)
            
        else:
            # blanco
            print("blanco")
            init_pos = (4,1)
            fin_pos = (4,3)
            path = [(0,0), (1,0), (2,0), (2,1), (2,2), (3,2), (4,2), (4,1)]
            map_file = "P5/mapaA_CARRERA.txt"
            logo_pos = (0,4)
            
        init_x = init_pos[0] * 0.4 + 0.2
        init_y = init_pos[1] * 0.4 + 0.2
        
        robot = Robot(init_position=[init_x, init_y, 0.0])
        myMap = Map2D(map_file)
                
        robot.startOdometry()

        # Perform trajectory
        # Slalom
        """while len(path) > 0:
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
                init_pos = next_pos"""
        
        if traj_black:
            _8_B(robot)
        else:
            _8_A(robot)
            
        robot.rotate(0.0, 0.25)
        
        print("Fin S alcanzado")
        
        # Obstacles
        path = myMap.planPath(init_pos[0], init_pos[1], fin_pos[0], fin_pos[1])   
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
        
        # Red ball
        if traj_black:
            robot.go(3, 6, error=0.005)
        else:
            robot.go(3, 3, error=0.005)
            
        robot.trackObject((0,200,20),(5,255,200) , (170,200,20),(180,255,200))
        
        # Logo
        robot.go(logo_pos[0], logo_pos[1], error=0.005)
        robot.rotate(np.pi, 0.25)
        rec = RecLogo(debug=0)
        
        r2d2 = False
        bb8 = False
        while not r2d2 and not bb8:
            r2d2 = rec.find_logo("P5/R2_D2.png")
            bb8 = rec.find_logo("P5/BB8_s.png")
            time.sleep(0.1)
        
        if r2d2:
            robot.go(fin_dch[0], fin_dch[1], error=0.005)
            # Salir por la puerta
            robot.rotate(np.pi, 0.25)
            robot.go(fin_dch[0] - 1, fin_dch[1], error=0.005)
        else:
            robot.go(fin_izq[0], fin_izq[1], error=0.005)
            # Salir por la puerta
            robot.rotate(np.pi, 0.25)
            robot.go(fin_izq[0] - 1, fin_izq[1], error=0.005)
        
        
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



