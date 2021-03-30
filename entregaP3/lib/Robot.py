#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import time     # import the time library for the sleep function
import sys
import math
import cv2
import picamera
from picamera.array import PiRGBArray

import numpy as np

from lib.utils import norm_rad


debug = False

if debug:
    from lib.DebugBlockPi import DebugBlockPi
    BP = DebugBlockPi()
else:
    import brickpi3
    BP = brickpi3.BrickPi3()

from multiprocessing import Process, Value, Array, Lock


class Robot:
    def __init__(self, init_position=[0.0, 0.0, 0.0], log_file_name="log_odom.csv"):
        """
        Initialize basic robot params. \

        Initialize Motors and Sensors according to the set up in your robot
        """

        # Robot construction parameters
        self.radio_rueda = 0.028 # m
        self.long_ruedas = 0.115 # m
        self.BP = BP
        self.PORT_LEFT_WHEEL = BP.PORT_C
        self.PORT_RIGHT_WHEEL = BP.PORT_B
        self.PORT_GRIPPER = BP.PORT_D
        
        # inicializar camara // TODO

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

        self.resetMotors()
        ##################################################
        # odometry shared memory values
        self.x = Value('d',init_position[0])
        self.y = Value('d',init_position[1])
        self.th = Value('d',init_position[2])
        self.finished = Value('b',1) # boolean to show if odometry updates are finished
        
        self.v = Value('d',0.0) # esto solo sirve para el modo debug
        self.w = Value('d',0.0)
        
        # blob shared memory values
        self.x_blob = Value('d',-1)
        self.y_blob = Value('d',-1)
        self.size_blob = Value('d',-1)
        
        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = Lock()
        self.lock_camera = Lock()

        self.P = 0.03 # odometry update period
        self.P_CHECK_POS = 0.03 # chech position period
        
        # lectura previa en radianes
        self.rdMotorR_prev = 0
        self.rdMotorL_prev = 0
        
        # 
        # self.encoder_timer = 0
        
        self.log_file_name = log_file_name

    def resetMotors(self):
        """Reset encoders PORT_RIGHT_WHEEL, PORT_LEFT_WHEEL and PORT_GRIPPER"""

        self.BP.offset_motor_encoder(self.PORT_RIGHT_WHEEL, self.BP.get_motor_encoder(self.PORT_RIGHT_WHEEL))
        self.BP.offset_motor_encoder(self.PORT_LEFT_WHEEL, self.BP.get_motor_encoder(self.PORT_LEFT_WHEEL))
        self.BP.offset_motor_encoder(self.PORT_GRIPPER, self.BP.get_motor_encoder(self.PORT_GRIPPER))

    def setSpeed(self, v: float, w: float):
        """
        - v: linear velocity m/s
        - w: angular velocity rad/s
        """

        # compute the speed that should be set in each motor ...
        wMotorR = np.array([1/self.radio_rueda, self.long_ruedas/(2 * self.radio_rueda)]).dot(np.array([v, w]))
        wMotorL = np.array([1/self.radio_rueda, -self.long_ruedas/(2 * self.radio_rueda)]).dot(np.array([v, w]))

        #print(str(wMotorR) + " " + str(wMotorL))

        self.BP.set_motor_dps(self.PORT_RIGHT_WHEEL, math.degrees(wMotorR))
        self.BP.set_motor_dps(self.PORT_LEFT_WHEEL, math.degrees(wMotorL))
        
        # update debug velocity
        if debug:
            self.lock_odometry.acquire()
            self.v = v
            self.w = w
            self.lock_odometry.release()

    def readSpeed(self) -> list:
        """
        Returns current speed: [v(m/s), w(rad/s)]
        Auxialiar function of updateOdometry()
        """

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
            #self.t_prev = self.encoder_timer
            #self.encoder_timer = time.time()
            
            #t = self.encoder_timer - self.t_prev
            
            # Calculo velocidades angulares en cada motor
            wMotorR = (rdMotorR - self.rdMotorR_prev) / self.P
            wMotorL = (rdMotorL - self.rdMotorL_prev) / self.P
              
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

    def readOdometry(self) -> list:
        """ Returns current value of odometry estimation [x(m), y(m), th(rad)]"""

        self.lock_odometry.acquire()
        x = self.x.value
        y = self.y.value
        th = self.th.value
        self.lock_odometry.release()
        
        return x, y, th

    def startOdometry(self):
        """ This starts a new process/thread that will be updating the odometry periodically """
        self.finished.value = False
        self.p = Process(target=self.updateOdometry, args=())
        self.p.start()
        print("PID: ", self.p.pid)

    def updateOdometry(self):
        """ Updae the odomery every self.P seconds  """
        log_file = open(self.log_file_name, "w+")
        log_file.write("x,y,th\n")
        t_count = 0
        while not self.finished.value:
            # current processor time in a floating point value, in seconds
            tIni = time.clock()

            # compute updates
            x,y,th = self.readOdometry()
            
            v,w = self.readSpeed()

            th_f = 0
            if w == 0: # moviemiento recto
                x += self.P * v * math.cos(th)
                y += self.P * v * math.sin(th)
                th_f = norm_rad(th)
            else: # movimiento circular
                div = v/w
                th_f = norm_rad(th + w * self.P)
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
                if t_count == 10: # writes every 0,3 seconds
                    t_count = 0
                    log_file.write(str(x) + "," + str(y) + "," + str(th_f) + "\n")
            except IOError as error:
                #print(error)
                sys.stdout.write(error)


            tEnd = time.clock()
            time.sleep(self.P - (tEnd-tIni))
        
        log_file.close()
        sys.stdout.write("Stopping odometry ... X=  %.2f, \
                Y=  %.2f, th=  %.2f \n" %(self.x.value, self.y.value, self.th.value))
                

    # Stop the odometry thread.
    def stopOdometry(self):
        """ Stop updating odometry """

        self.finished.value = True
        self.setSpeed(0,0)
        self.BP.reset_all()
    
    def go_to(self, v:float, w:float, x_f: float, y_f:float, th_f: float, error_margin: float):
        """
        Move to [x_f,y_f,th_f] position with speed [v,w]
        - v: Linear Speed m/s
        - w: Angular Speed rad/s
        - x_f: X Position in m
        - y_f: Y Position in m
        - th_f: Angle in rad
        """
        x, y, th = self.readOdometry()
        self.setSpeed(v,w)
        th_f_down = norm_rad(th_f - error_margin)
        th_f_up = norm_rad(th_f + error_margin)
        
        if th_f_down > 0 and th_f_up < 0 or th_f_down < 0 and th_f_up > 0:
            while not (th > th_f_down or th < th_f_up):
                time.sleep(self.P_CHECK_POS)
                x,y,th = self.readOdometry()
                print(str(x) + " | " + str(y) + " | " + str(th))
        else:
            while not (th > th_f_down and th < th_f_up):
                time.sleep(self.P_CHECK_POS)
                x,y,th = self.readOdometry()
                print(str(x) + " | " + str(y) + " | " + str(th))
                
    def catch (self, up:bool):
        """
        Si up, entonces la cesta sube
        Sino baja 
        """
        v = 95
        if up:
            self.BP.set_motor_dps(self.PORT_GRIPPER, -v)
            time.sleep(1)
            self.BP.set_motor_dps(self.PORT_GRIPPER, 0)
        else:
            self.BP.set_motor_dps(self.PORT_GRIPPER, v)
            time.sleep(1)
            self.BP.set_motor_dps(self.PORT_GRIPPER, 0)
        
    def camera(self, colorRangeMin0:tuple, colorRangeMax0:tuple, colorRangeMin1:tuple, colorRangeMax1:tuple):
        """
        Proceso independiente que ejecuta la cámara y procesa blobs
        """
        cam = picamera.PiCamera()

        cam.resolution = (320, 240)
        cam.framerate = 32
        rawCapture = PiRGBArray(cam, size=(320, 240))
         
        # allow the camera to warmup
        time.sleep(0.5)

        for img in cam.capture_continuous(rawCapture, format="bgr", use_video_port=True):

            frame = img.array
            # cv2.imshow('Captura', frame)
            x, y, size = self.get_blobs(frame, colorRangeMin0, colorRangeMax0, colorRangeMin1, colorRangeMax1)
            self.lock_camera.acquire()
            self.x_blob.value, self.y_blob.value, self.size_blob.value = x, y, size
            self.lock_camera.release()
            
            cv2.imshow("Camera", frame)
            # clear the stream in preparation for the next frame
            rawCapture.truncate(0)
             
            k = cv2.waitKey(1) & 0xff
            if k == 27: # ESC
                cam.close()
                break

        cv2.destroyAllWindows()
        
    def get_blobs (self, frame, hsvMin0, hsvMax0, hsvMin1, hsvMax1):
        """
        Devuelve la posición y el tamaño del blob más grande encontrado
        en frame a partir de los rangos HSV
        """
        # Font to write text overlay
        font = cv2.FONT_HERSHEY_SIMPLEX
        b = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(b, cv2.COLOR_BGR2HSV)

        # Apply HSV thresholds 
        mask0 = cv2.inRange(hsv, hsvMin0, hsvMax0)

        mask1 = cv2.inRange(hsv, hsvMin1, hsvMax1)

        mask = cv2.bitwise_or(mask1, mask0)
        #mask = cv2.inRange(hsv, hsvMin, hsvMax)

        # Erode and dilate
        mask = cv2.erode(mask, None, iterations=3)
        mask = cv2.dilate(mask, None, iterations=3)

        # Adjust detection parameters
        params = cv2.SimpleBlobDetector_Params()
         
        # Change thresholds
        params.minThreshold = 0;
        params.maxThreshold = 100;
         
        # Filter by Area.
        params.filterByArea = False
        params.minArea = 50
        params.maxArea = 50000
         
        # Filter by Circularity
        params.filterByCircularity = False
        params.minCircularity = 0.01
         
        # Filter by Convexity
        params.filterByConvexity = False
        params.minConvexity = 0.5
         
        # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.5

        # Detect blobs
        detector = cv2.SimpleBlobDetector_create(params)

        # Invert the mask
        reversemask = 255-mask

        # Run blob detection
        keypoints = detector.detect(reversemask)

        # Get the number of blobs found
        blobCount = len(keypoints)

        # Write the number of blobs found
        text = "Count=" + str(blobCount) 
        cv2.putText(frame, text, (5,25), font, 1, (0, 255, 0), 2)
        
        kx, ky, ksize = -1, -1, -1
        for k in keypoints:
            if ksize < k.size:
                kx = k.pt[0]
                ky = k.pt[1]
                ksize = k.size
        
        # ponerlo opcional // TODO
        text2 = "X=" + "{:.2f}".format(kx )
        cv2.putText(frame, text2, (5,50), font, 1, (0, 255, 0), 2)
        text3 = "Y=" + "{:.2f}".format(ky)
        cv2.putText(frame, text3, (5,75), font, 1, (0, 255, 0), 2)
        text4 = "S=" + "{:.2f}".format(ksize)
        cv2.putText(frame, text4, (5,100), font, 1, (0, 255, 0), 2)
        
        cv2.circle(frame, (int(kx),int(ky)), int(ksize / 2), (0, 255, 0), 2)
        
        return kx, ky, ksize

    
    def getBlob(self):
        """
        Devuelve la posición y tamaño del último blob
        """
        self.lock_camera.acquire()
        x, y, s = self.x_blob.value, self.y_blob.value, self.size_blob.value
        self.lock_camera.release()
        print (x, y, s)
        
        return x, y, s
        
    def orientate(self, x):
        """
        Ajusta la velocidad angular a partir de la posición x del blob
        """
        d = abs(160 - x)
        if x < 160: # image_x_size = 320
            w = 1
        else:
            w = -1
        
        if d < 20: 
            w = 0
        elif d < 80:
            w = w * 0.2
        else:
            w = w * 0.5
            
        return w
        
    def calculate_speed(self, x, size):
        """
        Función escalón para ajustar la velocidad lineal y angular 
        a partir del tamaño y posición del blob
        """
        a = size - 90
        if a > 0 or size == -1: v = 0
        elif a > -30: v = 0.05
        else: v = 0.15
        
        w = self.orientate(x) # corrección error
        return v, w
        
        
    def trackObject(self, colorRangeMin0:tuple, colorRangeMax0:tuple, colorRangeMin1:tuple, colorRangeMax1:tuple):
        """
        Busca y captura el objetivo (pelota roja) en tres pasos:
        1. Gira sobre sí mismo hasta encontrar el primer blob
        2. Avanza hacia el objetivo ajustando las velocidades lineal y angular con una función escalón
        3. Captura el objetivo
        """
        self.p_camera = Process(target=self.camera, args=(colorRangeMin0, colorRangeMax0, colorRangeMin1, colorRangeMax1))
        self.p_camera.start()
        
        while not self.finished.value:
            x, y, size = self.getBlob()
            
            # 1. buscar y orientar a blob             
            while size == -1:
                x, y, size = self.getBlob()
                self.setSpeed(0, 0.5)  
                time.sleep(0.05)
            
            # 3. avanzar hacia objetivo
            v = 1
            w = 1
            n = 0
            while (v != 0 or w != 0) and n < 4:
                x, y, size = self.getBlob()
                if size == -1: n += 1
                else: n = 0
                v, w = self.calculate_speed(x, size)
                self.setSpeed(v, w)
                time.sleep(0.05)
            
            self.setSpeed(0,0)
            # 3. coge el objetivo
            if size >= 90:
                self.setSpeed(0.05,0)
                time.sleep(0.2)
                self.catch(False)
                break  
        
        self.p_camera.terminate()
        
