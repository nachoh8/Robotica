#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import time     # import the time library for the sleep function
import sys
import math

import numpy as np


debug = False

if debug:
    from DebugBlockPi import DebugBlockPi
    BP = DebugBlockPi()
else:
    import brickpi3
    BP = brickpi3.BrickPi3()


# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, Array, Lock


def norm_pi(rad):
        if rad >= -np.pi:
                return -np.pi + (rad + np.pi) % (2 * np.pi)
        else:
                return np.pi - (-np.pi - rad) % (2 * np.pi)

class Robot:
    def __init__(self, init_position=[0.0, 0.0, 0.0], log_file_name="log_odom.txt"):
        """
        Initialize basic robot params. \

        Initialize Motors and Sensors according to the set up in your robot
        """
######## UNCOMMENT and FILL UP all you think is necessary (following the suggested scheme) ########

        # Robot construction parameters
        self.radio_rueda = 0.0275
        self.long_ruedas = 0.115
        self.BP = BP
        self.PORT_LEFT_WHEEL = BP.PORT_C
        self.PORT_RIGHT_WHEEL = BP.PORT_B

        ##################################################
        # Motors and sensors setup

        # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
        #self.BP = brickpi3.BrickPi3()

        # Configure sensors, for example a touch sensor.
        #self.BP.set_sensor_type(self.BP.PORT_1, self.BP.SENSOR_TYPE.TOUCH)

        # reset encoder B and C (or all the motors you are using)
        #self.BP.offset_motor_encoder(self.BP.PORT_B,
        #    self.BP.get_motor_encoder(self.BP.PORT_B))
        #self.BP.offset_motor_encoder(self.BP.PORT_C,
        #    self.BP.get_motor_encoder(self.BP.PORT_C))

        ##################################################
        # odometry shared memory values
        self.x = Value('d',0.0)
        self.y = Value('d',0.0)
        self.th = Value('d',0.0)
        self.finished = Value('b',1) # boolean to show if odometry updates are finished
        
        self.v = Value('d',0.0) # esto solo sirve para el modo debug
        self.w = Value('d',0.0)
        
        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = Lock()

        # odometry update period
        self.P = 0.03
        
        # lectura previa en radianes
        self.rdMotorR_prev = 0
        self.rdMotorL_prev = 0
        
        # 
        self.encoder_timer = 0
        
        self.log_file = open(log_file_name, "w+")

    def reset():
            self.BP.set_motor_dps(self.PORT_RIGHT_WHEEL, 0)
            self.BP.set_motor_dps(self.PORT_LEFT_WHEEL, 0)

    def setSpeed(self, v, w):
        """
        * Pre:
        - v: linear velocity m/s
        - w: angular velocity rad/s
        * Post:
        - Le da caña a los motores
        """

        # print("setting speed to %.2f %.2f" % (v, w))

        # compute the speed that should be set in each motor ...
        wMotorR = np.array([1/self.radio_rueda, self.long_ruedas/(2 * self.radio_rueda)]).dot(np.array([v, w]))
        wMotorL = np.array([1/self.radio_rueda, -self.long_ruedas/(2 * self.radio_rueda)]).dot(np.array([v, w]))

        print(str(wMotorR) + " " + str(wMotorL))

        self.BP.set_motor_dps(self.PORT_RIGHT_WHEEL, math.degrees(wMotorR))
        self.BP.set_motor_dps(self.PORT_LEFT_WHEEL, math.degrees(wMotorL) * 0.99)
        
        # update debug velocity
        if debug:
                self.lock_odometry.acquire()
                self.v = v
                self.w = w
                self.lock_odometry.release()


    def readSpeed(self):
        self.lock_odometry.acquire()	
        if debug:
                v = self.v.value
                w = self.w.value
        else:
                # Lectura de los grados de giro de la rueda en un instante
                grMotorR = self.BP.get_motor_encoder(self.BP.PORT_B)
                grMotorL = self.BP.get_motor_encoder(self.BP.PORT_C)
                
                rdMotorR = math.radians(grMotorR)
                rdMotorL = math.radians(grMotorL)
        
                # Tiempo transcurrido entre lecturas
                self.t_prev = self.encoder_timer
                self.encoder_timer = time.time()
                
                t = self.encoder_timer - self.t_prev
                
                # Calculo velocidades angulares en cada motor
                wMotorR = (rdMotorR - self.rdMotorR_prev) / t
                wMotorL = (rdMotorL - self.rdMotorL_prev) / t
                  
                # Calculo velocidades actuales
                v_w = np.array([[self.radio_rueda / 2, self.radio_rueda / 2],
                                [self.radio_rueda / self.long_ruedas, -self.radio_rueda / self.long_ruedas]
                               ]).dot(np.array([wMotorR, wMotorL]))

                # Actualizacion de variables
                self.rdMotorR_prev = rdMotorR
                self.rdMotorL_prev = rdMotorL
                        
                v = v_w[0]
                w = v_w[1]

        self.lock_odometry.release()

        return v, w

    def readOdometry(self):
        """ Returns current value of odometry estimation """
        self.lock_odometry.acquire()
        x = self.x.value
        y = self.y.value
        th = self.th.value
        self.lock_odometry.release()
        
        return x, y, th

    def startOdometry(self):
        """ This starts a new process/thread that will be updating the odometry periodically """
        self.finished.value = False
        self.p = Process(target=self.updateOdometry, args=()) #additional_params?))
        self.p.start()
        print("PID: ", self.p.pid)

    # You may want to pass additional shared variables besides the odometry values and stop flag
    def updateOdometry(self): #, additional_params?):
        """ To be filled ...  """
        t_count = 0
        while not self.finished.value:
            # current processor time in a floating point value, in seconds
            tIni = time.clock()

            # compute updates
            
            x = self.x.value
            y = self.y.value
            th = self.th.value
            
            v,w = self.readSpeed()
            th_f = 0
            if w == 0: # moviemiento recto
                    x += self.P * v * math.cos(th)
                    y += self.P * v * math.sin(th)
                    th_f = th
            else: # movimiento circular
                    div = v/w
                    th_f = norm_pi(th + w * self.P)
                    x += div * (math.sin(th_f) - math.sin(th))
                    y -= div * (math.cos(th_f) - math.cos(th))

            # update odometry values
            self.lock_odometry.acquire()
            self.x.value = x
            self.y.value = y
            self.th.value = th_f
            #print(str(self.x.value) + " " + str(self.y.value) + " " + str(self.th.value) + " " + str(v) + " " + str(w))
            self.lock_odometry.release()
            
            t_count += 1
            try:
                # Each of the following BP.get_motor_encoder functions returns the encoder value
                # (what we want to store).
                if t_count == 10:
                        t_count = 0
                        self.log_file.write("x: " + str(x) + " y: " + str(y) + " th: " + str(th_f))
                #sys.stdout.write("Reading encoder values .... \n")
                #[encoder1, encoder2] = [self.BP.get_motor_encoder(self.BP.PORT_B),
                #    self.BP.get_motor_encoder(self.BP.PORT_C)]
            except IOError as error:
                #print(error)
                sys.stdout.write(error)

            #sys.stdout.write("Encoder (%s) increased (in degrees) B: %6d  C: %6d " %
            #        (type(encoder1), encoder1, encoder2))


            # save LOG
            # Need to decide when to store a log with the updated odometry ...

            ######## UPDATE UNTIL HERE with your code ########


            tEnd = time.clock()
            time.sleep(self.P - (tEnd-tIni))

        #print("Stopping odometry ... X= %d" %(self.x.value))
        sys.stdout.write("Stopping odometry ... X=  %.2f, \
                Y=  %.2f, th=  %.2f \n" %(self.x.value, self.y.value, self.th.value))
                

    # Stop the odometry thread.
    def stopOdometry(self):
        self.finished.value = True
        #self.BP.reset_all()
        self.setSpeed(0,0)
        self.log_file.close()

