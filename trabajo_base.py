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

error = 0.015



def ceil_to_odom(ceil):
    return (ceil[0] * 0.4 + 0.2, ceil[1] * 0.4 + 0.2)

def odom_to_cell(odom):
    return (int(odom[0] / 0.4 - 0.2), int(odom[1] / 0.4 - 0.2))

def _8_A(robot):
    robot.go_to(0, -0.3, 0, 0, -np.pi/2, error, manual=True)
    print(robot.readOdometry())
    robot.go_to(0.12, 0.3, 0, 0, np.pi/2, error, manual=True)
    print(robot.readOdometry())
    robot.setSpeed(0.12,0.3)
    time.sleep(0.7)
    print(robot.readOdometry())
    robot.go_to(0.12, -0.3, 0, 0, -np.pi/2, error, manual=True)

def _8_B(robot):
    robot.go_to(0, 0.25, 0, 0, np.pi/2, error)
    robot.go_to(0.1, -0.25, 0, 0, -np.pi/2, error)
    robot.go_to(0.1, 0.25, 0, 0, np.pi/2, error)

def execute_plan(robot, myMap, init_pos, fin_pos, path=None):
    if path is None:
        path = myMap.planPath(init_pos[0], init_pos[1], fin_pos[0], fin_pos[1])
    print(path)
    
    while len(path) > 0:
        next_pos = path.pop(0)
        print(next_pos)
        dir_pos = ((next_pos[0] - init_pos[0]) * 0.4, (next_pos[1] - init_pos[1]) * 0.4)
        x, y, th = robot.readOdometry()
        print("Odom: " + str(x) + " " + str(y) + " "+ str(th))
        goal = ceil_to_odom(next_pos) #(next_pos[0]* 0.4 + 0.2, next_pos[1]* 0.4 + 0.2)
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
                return False
            print(path)
        else:
            init_pos = next_pos
    
    robot.setSpeed(0.0, 0.0)
    return True

def main(args):

    path = []
    init_pos = None
    fin_pos = None
    see_ball_pos = None
    fin_dch = (0,6)
    fin_izq = (0,3)
    fin_recorrido_1 = None
    fin_recorrido_2 = None
    map_file = ""
    
    logo_pos_izq = (1,4)
    logo_pos_drch = (1,5)
    logo_pos_1 = None
    logo_pos_2 = None
    logo_rec = ""
    
    try:
        
        traj_black = read_light() > 1550
        traj_black = False
        if traj_black:
            # negro
            print("negro")
            init_pos = (0,1)
            fin_pos = (4,6)
            path = [(0,0), (1,0), (2,0), (2,1), (2,2), (3,2), (4,2), (4,1)] # TODO
            map_file = "P5/mapaB_CARRERA.txt"
            see_ball_pos = (3,6)
            logo_rec = "/home/pi/Robotica/P5/BB8_s.png"
            fin_recorrido_1 = fin_dch
            fin_recorrido_2 = fin_izq
            logo_pos_1 = logo_pos_drch
            logo_pos_2 = logo_pos_izq
        else:
            # blanco
            print("blanco")
            init_pos = (0,1)
            fin_pos = (4,3)
            path = [(0,0), (1,0), (2,0), (2,1), (2,2), (3,2), (4,2), (4,1)]
            map_file = "P5/mapaA_CARRERA.txt"
            see_ball_pos = (3,4)
            logo_rec = "/home/pi/Robotica/P5/R2-D2_s.png"
            fin_recorrido_2 = fin_dch
            fin_recorrido_1 = fin_izq
            logo_pos_2 = logo_pos_drch
            logo_pos_1 = logo_pos_izq
            
        init_x = init_pos[0] * 0.4 + 0.2
        init_y = init_pos[1] * 0.4 + 0.2
        
        robot = Robot(init_position=[init_x, init_y, 0.0]) # -np.pi
        myMap = Map2D(map_file)
        
        robot.startOdometry()

        # Perform trajectory
        # Slalom
        """
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
        """
        
        if traj_black:
            _8_B(robot)
            #robot.changeOdometry(1.8,0.6,np.pi/2)
        else:
            _8_A(robot)
            x,y,th = robot.readOdometry()
            robot.changeOdometry(x,0.6,-1.7)
        
        # robot.go_to(0, 0.25, 0, 0, 0, error)
        
        print(robot.readOdometry())
        print("Fin S alcanzado")
        
        # Obstacles
        init_pos = (4,1)
        # path = myMap.planPath(init_pos[0], init_pos[1], fin_pos[0], fin_pos[1])
        execute_plan(robot, myMap, init_pos, fin_pos)
        
        print(robot.readOdometry())
        print("Fin PLAN")
        
        
        # Red ball
        pos_f = ceil_to_odom(see_ball_pos)
        robot.go(pos_f[0], pos_f[1], error=0.005)
            
        robot.trackObject((0,200,20),(5,255,200) , (170,200,20),(180,255,200), not traj_black)
        
        print(robot.readOdometry())
        print("Fin BALL")
        
        # Logo
        odom = robot.readOdometry()
        odom_cell = odom_to_cell(odom)
        print(odom, odom_cell)
        
        execute_plan(robot, myMap, odom_cell, logo_pos_1)
        robot.go_to(0.0, -0.3, 0.0, 0.0, -np.pi, 0.005)
        robot.setSpeed(0.0, 0.0)
        
        print(robot.readOdometry())
        print("A reconocer")
        
        rec = RecLogo(logo_rec)
        
        found = rec.find_logo()

        odom = robot.readOdometry()
        odom_cell = odom_to_cell(odom)
        print(odom, odom_cell)
        
        if found:
            pos_f = fin_recorrido_1
        else:
            print("A reconocer en la otra posicion")
            execute_plan(robot, myMap, odom_cell, logo_pos_2)
            robot.go_to(0.0, -0.3, 0.0, 0.0, -np.pi, 0.005)
            robot.setSpeed(0.0, 0.0)
            
            found = rec.find_logo()
            odom = robot.readOdometry()
            odom_cell = odom_to_cell(odom)
            print(odom, odom_cell)
            
            pos_f = fin_recorrido_2
        

        print("Fin reconocer")
        
        execute_plan(robot, myMap, odom_cell, pos_f)
        robot.setSpeed(0.0, 0.0)
        
        # Salir por la puerta
        print(robot.readOdometry())
        
        robot.go_to(0.0, -0.3, 0.0, 0.0, np.pi, 0.005)
        print(robot.readOdometry())
        
        pos_f = ceil_to_odom(pos_f)
        robot.go(pos_f[0] - 0.4, pos_f[1], error=0.005)
        robot.setSpeed(0.0, 0.0)
        
        print("Meta alcanzada")
        
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



