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

W_ROBOT = 0.3
V_ROBOT = 0.12
    

def ceil_to_odom(ceil):
    return (ceil[0] * 0.4 + 0.2, ceil[1] * 0.4 + 0.2)

def odom_to_cell(odom):
    return (int(odom[0] / 0.4 - 0.2), int(odom[1] / 0.4 - 0.2))

def _8_A(robot):
    robot.rotate(-W_ROBOT, -np.pi/2, dir_w=False)
    robot.rotate(W_ROBOT, np.pi/2, v=V_ROBOT, dir_w=False)
    robot.setSpeed(V_ROBOT,W_ROBOT)
    time.sleep(0.5)
    robot.rotate(-W_ROBOT, -np.pi/2, v=V_ROBOT, dir_w=False)

def _8_B(robot):
    robot.rotate(W_ROBOT, np.pi/2, dir_w=False)
    robot.rotate(-W_ROBOT, -np.pi/2, v=V_ROBOT, dir_w=False)
    robot.setSpeed(V_ROBOT,-W_ROBOT)
    robot.rotate(W_ROBOT, np.pi/2, v=V_ROBOT, dir_w=False)

def execute_plan(robot, myMap, init_pos, fin_pos, path=None):
    if path is None:
        path = myMap.planPath(init_pos[0], init_pos[1], fin_pos[0], fin_pos[1])
    
    print("Ejecutando plan")
    print(path)
    
    while len(path) > 0:
        next_pos = path.pop(0)
        print("Objetivo: " + str(next_pos))
        dir_pos = ((next_pos[0] - init_pos[0]) * 0.4, (next_pos[1] - init_pos[1]) * 0.4)
        x, y, th = robot.readOdometry()
        print("Odom: " + str(x) + " " + str(y) + " "+ str(th))
        goal = ceil_to_odom(next_pos) #(next_pos[0]* 0.4 + 0.2, next_pos[1]* 0.4 + 0.2)
        print("Destino: " + str(goal))
        
        if not robot.go(goal[0], goal[1], error=0.015):
            print("Recalculando ruta...")
            # Añadir pared detectada
            neigh = myMap.get_numneigh(dir_pos[0], dir_pos[1])
            for n in neigh:
                myMap.deleteConnection(init_pos[0], init_pos[1], n)
            
            # recalcular posicion
            robot.check_th(0.05, 0.2)
            
            time.sleep(0.5)
            odom = robot.readOdometry()
            print("Despues del barrido: ", odom)
            aux = ceil_to_odom(init_pos)
            print(aux)
            if dir_pos[0] != 0:
                ref = next_pos[0]* 0.4
                print(ref)
                if odom[2] > np.pi / 2 or odom[2] < -np.pi / 2:
                    ref += 0.4
                print("2", ref)
                robot.check_x(ref, aux[0])
                time.sleep(0.5)
                print("Despues del corregir x: ", robot.readOdometry())
            else:
                ref = next_pos[1]* 0.4
                if odom[2] < 0:
                    ref += 0.4
                robot.check_y(ref, aux[1])
                time.sleep(0.5)
                print("Despues del corregir y: ", robot.readOdometry())
            
            # Volver a planear la ruta
            path = myMap.planPath(init_pos[0], init_pos[1], fin_pos[0], fin_pos[1])
            if path is None:
                print("No he encontrado un camino")
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
    map_file = ""
    
    logo_pos_1 = None
    logo_pos_2 = None
    logo_rec = ""
    
    try:
        
        traj_black = read_light() > 1550
        #traj_black = True
        if traj_black:
            # negro
            print("negro")
            init_pos = (0,5)
            fin_pos = (4,3)
            map_file = "P5/mapaB_CARRERA.txt"
            see_ball_pos = (2,2)
            logo_rec = "/home/pi/Robotica/P5/BB8_s.png"
            logo_pos_1 = (1,2)
            logo_pos_2 = (1,1)
        else:
            # blanco
            print("blanco")
            init_pos = (0,1)
            fin_pos = (4,3)
            map_file = "P5/mapaA_CARRERA.txt"
            see_ball_pos = (2,4)
            logo_rec = "/home/pi/Robotica/P5/R2-D2_s.png"
            logo_pos_2 = (1,5)
            logo_pos_1 = (1,4)
            
        init_x = init_pos[0] * 0.4 + 0.2
        init_y = init_pos[1] * 0.4 + 0.2
        
        robot = Robot(init_position=[init_x, init_y, 0.0])
        myMap = Map2D(map_file)
        
        robot.startOdometry()

        # Perform trajectory
        # Slalom
        if traj_black:
            _8_B(robot)
        else:
            _8_A(robot)
        
        robot.setSpeed(0.0, 0.0)
        print("Fin S alcanzado | position: ", robot.readOdometry())
        
        robot.check_th(0.05, 0.2)
        time.sleep(0.5)
        print("Despues del th: ", robot.readOdometry())
        
        if traj_black:
            robot.check_y(2.8, 2.2)
        else:
            robot.check_y(0, 0.6)
        time.sleep(0.5)
        
        odom = robot.readOdometry()
        print("Despues del corregir y: ", odom)
        
        # Obstacles
        init_pos = odom_to_cell((odom[0], odom[1]))
        execute_plan(robot, myMap, init_pos, fin_pos)
        
        print("Fin PLAN | position ", robot.readOdometry())
        
        # Red ball
        # avanzar una baldosa en X
        f_pos = ceil_to_odom(fin_pos)
        robot.go_direct(f_pos[0]-0.5, f_pos[1])
        # avanzar en diagonal
        f_pos = ceil_to_odom(see_ball_pos)
        robot.go_direct(f_pos[0], f_pos[1])
        print("Buscar BALL | position ", odom)
        
        robot.trackObject((0,200,20),(5,255,200) , (170,200,20),(180,255,200), traj_black)
        
        odom = robot.readOdometry() 
        print("Fin BALL | position ", odom)
        
        # Logo
        
        # relocalizar
        go_logo_1 = True # ir al logo marcado al inicio (logo_pos_1)
        logo_pos = logo_pos_1
        if traj_black:
            if odom[1] > 0.8:
                th_f = np.pi / 2
                y_ref = 1.6
            else:
                go_logo_1 = False
                logo_pos = logo_pos_2
                th_f = -np.pi / 2
                y_ref = 0.0
        else:
            if odom[1] > 2.0:
                go_logo_1 = False
                logo_pos = logo_pos_2
                th_f = np.pi / 2
                y_ref = 2.8
            else:
                th_f = -np.pi / 2
                y_ref = 1.2
            
        robot.rotate(W_ROBOT, th_f)
        robot.check_th(0.1, 0.2)
        print("Odom barrido th ", robot.readOdometry())
        
        pos_f = ceil_to_odom(logo_pos)
        robot.check_y(y_ref, pos_f[1])
        print("Odom barrido y ",robot.readOdometry())
        
        robot.rotate(W_ROBOT, -np.pi)
        robot.check_x(0, 0.4)
        print("Odom barrido x ", robot.readOdometry())
        
        robot.rotate(W_ROBOT, -np.pi)
        print("A reconocer | position ", robot.readOdometry())
        
        rec = RecLogo(logo_rec)
        
        found = rec.find_logo()
        
        print("Fin reconocer: ", ("" if found else "no "), "encontrado")
        
        if (found and go_logo_1) or (not found and not go_logo_1): # ir a salida mas cercana respecto al inicio del robot
            if traj_black:
                th_f = np.pi / 2
                y_f = 1.4
                y_ref = 1.6
            else:
                th_f = -np.pi / 2
                y_f = 1.4
                y_ref = 1.2
        else: #((not found) and go_logo_1) or (found and (not go_logo_1)): # ir a la otra salida
            if traj_black:
                th_f = -np.pi / 2
                y_f = 0.2
                y_ref = 0.0
            else:
                th_f = np.pi / 2
                y_f = 2.6
                y_ref = 2.8
        
        robot.rotate(W_ROBOT, th_f)
        
        robot.check_th(0.1, 0.2)
        print("Odom barrido th ",robot.readOdometry())
        
        robot.check_y(y_ref, y_f)
        print("Odom barrido y ",robot.readOdometry())
        
        robot.rotate(W_ROBOT, -np.pi)
        print("Odom rotate ",robot.readOdometry())
        
        robot.setSpeed(0.1, 0.0)
        time.sleep(7)
        
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



