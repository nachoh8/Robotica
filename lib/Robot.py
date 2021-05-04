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

from lib.utils import norm_rad, norm_pi, norm_rad_to_degrees


debug = False

if debug:
    from lib.DebugBlockPi import DebugBlockPi
    BP = DebugBlockPi()
else:
    import brickpi3
    BP = brickpi3.BrickPi3()

from multiprocessing import Process, Value, Array, Lock

MAGIC_NUMBER = (0.3 * 180.0 / np.pi) / (1.983 * (1 / 0.03))
GYRO_OFFSET = 2357.4

class Robot:
    def __init__(self, init_position=[0.0, 0.0, 0.0], log_file_name="log_odom.csv", verbose=False):
        """
        Initialize basic robot params. \

        Initialize Motors and Sensors according to the set up in your robot
        """

        self.verbose = verbose

        # Robot construction parameters
        self.radio_rueda = 0.028 # m
        self.long_ruedas = 0.11 # m
        self.BP = BP
        self.PORT_LEFT_WHEEL = BP.PORT_C
        self.PORT_RIGHT_WHEEL = BP.PORT_B
        self.PORT_GRIPPER = BP.PORT_D
        self.PORT_ULTRASONIC_SENSOR = BP.PORT_1
        self.PORT_GYRO_SENSOR = BP.PORT_3
        
        self.BP.set_sensor_type(self.PORT_ULTRASONIC_SENSOR, self.BP.SENSOR_TYPE.EV3_ULTRASONIC_CM)
        self.BP.set_sensor_type(self.PORT_GYRO_SENSOR, self.BP.SENSOR_TYPE.CUSTOM, [(self.BP.SENSOR_CUSTOM.PIN1_ADC)])
        #self.BP.set_sensor_type(self.PORT_GYRO_SENSOR, self.BP.TYPE_SENSOR_EV3_GYRO_M0)
        time.sleep(5)
        
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
        self.change_odom = Value('b', False)
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
        self.P_CHECK_POS = 0.08 # check position period
        
        # lectura previa en radianes
        self.rdMotorR_prev = 0
        self.rdMotorL_prev = 0
        
        self.area_blob_catch = 85
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

        #self.lock_odometry.acquire()	
        if debug:
            v = self.v.value
            w = self.w.value
        else:
            # Lectura de los grados de giro de la rueda en un instante
            grMotorR = self.BP.get_motor_encoder(self.BP.PORT_B)
            grMotorL = self.BP.get_motor_encoder(self.BP.PORT_C)
            
            rdMotorR = math.radians(grMotorR)
            rdMotorL = math.radians(grMotorL)
            
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

        #self.lock_odometry.release()

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
        """ Update the odomery every self.P seconds  """
        log_file = open(self.log_file_name, "w+")
        log_file.write("x,y,th\n")
        t_count = 0

        while not self.finished.value:
            # current processor time in a floating point value, in seconds
            tIni = time.clock()

            # compute updates
            self.lock_odometry.acquire()
            x = self.x.value
            y = self.y.value
            th = self.th.value
            
            v,w = self.readSpeed()

            th_f = 0
            gyro = norm_pi(-(self.read_gyro() - GYRO_OFFSET) * MAGIC_NUMBER)
            #print("odom ", w, gyro)
            w = (w + gyro) / 2.0
            
            #w = norm_pi(gyro)
            #print(w)
            if w == 0: # moviemiento recto
                x += self.P * v * math.cos(th)
                y += self.P * v * math.sin(th)
                th_f = th
                
            else: # movimiento circular
                div = v/w
                th_aux = norm_rad(th + w * self.P)
                x += div * (math.sin(th_aux) - math.sin(th))
                y -= div * (math.cos(th_aux) - math.cos(th))
                th_f = th_aux                

            # update odometry values
            # self.lock_odometry.acquire()
            if self.change_odom.value:
                self.change_odom.value = False
            else:
                self.x.value = x
                self.y.value = y
                self.th.value = th_f
            # print(str(x) + "," + str(y) + "," + str(th_f) + "\n")
            self.lock_odometry.release()
            
            t_count += 1
            try:
                if t_count == 10: # writes every 0,3 seconds
                    t_count = 0
                    log_file.write(str(x) + "," + str(y) + "," + str(th_f) + "\n")
            except IOError as error:
                #print(error)
                sys.stdout.write(error)

            time.sleep(self.P)
        
        log_file.close()
        sys.stdout.write("Stopping odometry ... X=  %.2f, \
                Y=  %.2f, th=  %.2f \n" %(self.x.value, self.y.value, self.th.value))
    
    def changeOdometry(self, x, y, th):
        self.lock_odometry.acquire()
        self.change_odom.value = True
        self.x.value = x
        self.y.value = y
        self.th.value = th
        self.lock_odometry.release()
        

    # Stop the odometry thread.
    def stopOdometry(self):
        """ Stop updating odometry """

        self.finished.value = True
        time.sleep(0.5)
        self.setSpeed(0,0)
        self.BP.reset_all()
    
    def rotate_dir(self, th, th_f):
        if abs(th-th_f) <= 1: return 0
        
        if th_f > th:
            d_izq = th_f - th
        else:
            d_izq = 360 - th + th_f
        
        return 1 if d_izq <= 180 else -1
    
    def get_th_grados(self):
        _,_,th = self.readOdometry()
        return norm_rad_to_degrees(th)
        
    def rotate(self, w, th_f, v = 0.0, dir_w=True):
        th_f = norm_rad_to_degrees(th_f)
        
        th = self.get_th_grados()

        if dir_w: w = w * self.rotate_dir(th, th_f)
        #print("rotate ", th, th_f,w)
        if w == 0: return
        
        self.setSpeed(v, w)
        time.sleep(0.05)
        th = self.get_th_grados()
    
        while abs(th_f - th) >= 1.5:
            time.sleep(self.P_CHECK_POS)
            th = self.get_th_grados()
            
        self.setSpeed(0,0)
    
    def read_ultrasonic(self):
        while True:
            try:
                value = self.BP.get_sensor(self.BP.PORT_1)
                while value == 0.0:
                    value = self.BP.get_sensor(self.BP.PORT_1)
                return value
            except: 
                pass
            time.sleep(0.1)
    
    def read_gyro(self):
        while True:
            try:
                return self.BP.get_sensor(self.PORT_GYRO_SENSOR)[0]
            except: 
                pass
            time.sleep(0.02)
    
    def go(self, x_goal, y_goal, error=0.015):
           x,y,th = self.readOdometry()
           
           # Calcular th final para rotar
           v = (x_goal - x, y_goal - y)
           if abs(v[0]) > abs(v[1]):
               if v[0] >= 0.0:
                   th_goal = 0.0 #error+0.0001
               else:
                   th_goal = -np.pi
           else:
               if v[1] >= 0.0:
                   th_goal = np.pi/2
               else:
                   th_goal = -np.pi/2
                   
           #print("th_g: " + str(th_goal))
           
           # Rotar
           x,y,th = self.readOdometry()
           
           # corregimos th porque sabemos que se desvia ligeramente
           # self.changeOdometry(x,y,th-0.1)
           
           self.rotate(0.3, th_goal)
           
           
           # Avanzar
           count = 0
           times = 3
           for _ in range(times):
               read_ultra = self.read_ultrasonic()
               if read_ultra < 35:
                   count += 1
            
           if count == times:
               return False
           x,y,th = self.readOdometry()
           self.setSpeed(0.1,0)
           
           if abs(v[0]) > abs(v[1]): #abs(x_goal - x) > 0.2: # en X
               #print("cmp x")
               while abs(x_goal - x) > error:
                   time.sleep(self.P_CHECK_POS)
                   x,y,th = self.readOdometry()
                   #print("go ",x,y,th)
           else: # en Y
               while abs(y_goal - y) > error:
                   time.sleep(self.P_CHECK_POS)
                   x,y,th = self.readOdometry()
                   #print("go ",x,y,th)
               
           self.setSpeed(0,0)
           return True
     
    def go_direct(self, x_goal, y_goal, error=0.015):
       x,y,th = self.readOdometry()
       
       # Calcular th final para rotar
       v = (x_goal - x, y_goal - y)
       th_goal = norm_rad(np.arctan2(v[1], v[0]))
       
       # Rotar
       x,y,th = self.readOdometry()
       
       self.rotate(0.3, th_goal)
       
       x,y,th = self.readOdometry()
       self.setSpeed(0.1,0)
       
       if abs(v[0]) > abs(v[1]):# en X
           while abs(x_goal - x) > error:
               time.sleep(self.P_CHECK_POS)
               x,y,th = self.readOdometry()
       else: # en Y
           while abs(y_goal - y) > error:
               time.sleep(self.P_CHECK_POS)
               x,y,th = self.readOdometry()
               #print("go ",x,y,th)
           
       self.setSpeed(0,0)
       return True
                                 
    def catch (self, up:bool):
        """
        Si up, entonces la cesta sube
        Sino baja 
        """
        v = 90
        t = 1
        if up:
            self.BP.set_motor_dps(self.PORT_GRIPPER, -v)
            time.sleep(t)
            self.BP.set_motor_dps(self.PORT_GRIPPER, 0)
        else:
            self.BP.set_motor_dps(self.PORT_GRIPPER, v)
            time.sleep(t)
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
        size_l = 3
        px = 320-size_l
        py = 240-size_l
        for img in cam.capture_continuous(rawCapture, format="bgr", use_video_port=True):

            frame = img.array
            frame = cv2.line(frame,(0,0),(px,0),(0,0,0),size_l)
            frame = cv2.line(frame,(0,0),(0,py),(0,0,0),size_l)
            frame = cv2.line(frame,(px,py),(px,0),(0,0,0),size_l)
            frame = cv2.line(frame,(px,py),(0,py),(0,0,0),size_l)
            
            x, y, size = self.get_blobs(frame, colorRangeMin0, colorRangeMax0, colorRangeMin1, colorRangeMax1)
            self.lock_camera.acquire()
            self.x_blob.value, self.y_blob.value, self.size_blob.value = x, y, size
            self.lock_camera.release()
            
            if self.verbose: cv2.imshow("Camera", frame)
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
        params.minThreshold = 0
        params.maxThreshold = 100
         
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
        
        kx, ky, ksize = -1, -1, -1
        for k in keypoints:
            if ksize < k.size:
                kx = k.pt[0]
                ky = k.pt[1]
                ksize = k.size
        
        if self.verbose:
            text = "Count=" + str(blobCount) 
            cv2.putText(frame, text, (5,25), font, 1, (0, 255, 0), 2)
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
        
        if self.verbose: print (x, y, s)
        
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
        a = size - self.area_blob_catch
        if a > 0 or size == -1: v = 0
        elif a > -30: v = 0.05
        else: v = 0.1
        
        w = self.orientate(x) # corrección error
        return v, w
        
        
    def trackObject(self, colorRangeMin0:tuple, colorRangeMax0:tuple, colorRangeMin1:tuple, colorRangeMax1:tuple, turn_right:bool):
        """
        Busca y captura el objetivo (pelota roja) en tres pasos:
        1. Gira sobre sí mismo hasta encontrar el primer blob
        2. Avanza hacia el objetivo ajustando las velocidades lineal y angular con una función escalón
        3. Captura el objetivo
        """
        self.p_camera = Process(target=self.camera, args=(colorRangeMin0, colorRangeMax0, colorRangeMin1, colorRangeMax1))
        self.p_camera.start()
        
        w_dir = -1 if turn_right else 1
        
        while not self.finished.value:
            x, y, size = self.getBlob()
            
            # 1. buscar y orientar a blob             
            while size == -1:
                x, y, size = self.getBlob()
                self.setSpeed(0, 0.3*w_dir)  
                time.sleep(0.05)
            
            # 3. avanzar hacia objetivo
            v = 1
            w = 1
            n = 0
            while (v != 0 or w != 0) and n < 4:
                time.sleep(0.05)
                x, y, size = self.getBlob()
                if size == -1: n += 1
                else: n = 0
                v, w = self.calculate_speed(x, size)
                self.setSpeed(v, w)
            
            self.setSpeed(0,0)
            # 3. coge el objetivo
            if size >= self.area_blob_catch:
                self.setSpeed(0.05,0)
                time.sleep(0.7)
                self.catch(False)
                break  
        
        print("Blob final: ", x,y,size)
        self.p_camera.terminate()
        
    def take_dis(self, error, reads):
        r = self.read_ultrasonic()
        times = 1
        while times < reads:
            r_aux = self.read_ultrasonic()
            if r < r_aux + error and r > r_aux - error:
                times += 1
            else:
                times = 0
                r = r_aux
            time.sleep(0.05)
        return r

    def barrido (self, rotate_dch, margin, error):
        centrando = True
        reads = 4
        min_r = self.take_dis(error, reads) # medicion inicial
        ant = min_r
        descentrado = 0
        equal_dist = 0
        
        while centrando:
            x, y, th = self.readOdometry()
            # Rotar un poco
            if rotate_dch:
                self.rotate(-0.3, th - margin, dir_w=False)
            else:
                self.rotate(0.3, th + margin, dir_w=False)
        
            aux = self.take_dis(error, reads) # lectura distancia
            # print(min_r, ant, aux)
            if aux <= min_r: # se va centrando
                equal_dist += 1
                min_r = aux
                ant = aux
                descentrado = 0
            else: # se descentra
                #centrando = False
                descentrado += 1
                #print(descentrado)
                if descentrado >= 2:
                    centrando = False
                    x, y, th = self.readOdometry()
                    # deshace lo rotado
                    if rotate_dch:
                        self.rotate(0.3, th + (descentrado * margin), dir_w=False)
                    else:
                        self.rotate(-0.3, th - (descentrado * margin), dir_w=False)
                        
            time.sleep(0.3)
        return equal_dist

    def check_th (self, margin, error):
        x,y,th = self.readOdometry() # guardamos hacia donde mira
        
        # Conocer eje más cercano
        objs = [(45,315), (45,135), (135,235), (225,315)]
        th_obj = None
        th_d = norm_rad_to_degrees(th)
        for i in range(len(objs)):
            if i == 0:
                if th_d < objs[i][0] or th_d > objs[i][1]:
                    th_obj = objs[i][0] - 45
            else:
                if th_d > objs[i][0] and th_d < objs[i][1]:
                    th_obj = objs[i][0] + 45
                    
                
        #print("obj")
        #print(th_obj, th_d)
        # Hacia donde empezar a girar
        start_dch = None
        if th_obj == 0:
            if th_d < 90:
                start_dch = True
            else:
                start_dch = False
        else:
            if th_d > th_obj:
                start_dch = True
            else:
                start_dch = False
            
        # Barridos

        """
        if start_dch:
            w = w*-1
            _ = self.barrido(True, margin, error) # barrido derecha rotando "margin" rad y margen de error en lecturas de 0.3 cm
            equal_dist = self.barrido(False, margin, error)
        else:
        """
        _ = self.barrido(start_dch, margin, error)
        equal_dist = self.barrido(not start_dch, margin, error)
        #print("equal_dist " + str(equal_dist))#
        
        # Correccion
        th_f = (equal_dist/2.0 * margin) * (-1.0 if start_dch else 1.0)
        
        w = 0.3 * (-1.0 if start_dch else 1.0)
        x,y,th = self.readOdometry()
        #print(th_f)
        #print(th+th_f)
        self.rotate(w, th + th_f, dir_w=False)
        #print(th_obj)
        self.changeOdometry(x,y, norm_pi(th_obj))
        
    def check_y(self, y_ref, y_f):
        n_reads = 5
        v_t = 0.0
        
        for _ in range(n_reads):
            v_t += self.read_ultrasonic()
        v_t = (v_t / n_reads) / 100
        #print("dist ", v_t, " ref ", y_ref)
        
        x,y,th = self.readOdometry()
        
        if y >= y_ref: # mirando hacia el 0
            y = y_ref + v_t
        else: # mirando hacia el infinito
            y = y_ref - v_t
        
        self.changeOdometry(x,y,th)
        # print(self.readOdometry())
        
        v = 0.08
        sum_ref = 0.025
        if th > 0:
            y_f += sum_ref
            v = (-v if y_f < y else v)
        else:
            y_f -= sum_ref
            v = (v if y_f < y else -v)
        
        # print(v, x, y, th, y_f)
        self.setSpeed(v, 0.0)
        
        while abs(y_f - y) > 0.015:
            time.sleep(self.P_CHECK_POS)
            x,y,th = self.readOdometry()
        self.setSpeed(0,0)
        
        self.changeOdometry(x,y_f,th)
    
    def check_x(self, x_ref, x_f):
        n_reads = 5
        v_t = 0.0
        
        for _ in range(n_reads):
            v_t += self.read_ultrasonic()
        v_t = (v_t / n_reads) / 100
        # print(v_t)
        
        x,y,th = self.readOdometry()
        if x >= x_ref: # mirando hacia el 0
            x = x_ref + v_t
        else: # mirando hacia el infinito
            x = x_ref - v_t
        
        self.changeOdometry(x,y,th)
        # print(self.readOdometry())
        
        v = 0.08
        sum_ref = 0.025
        if th < np.pi / 2 and th > -np.pi / 2:
            x_f += sum_ref
            v = (-v if x_f < x else v)
        else:
            x_f -= sum_ref
            v = (v if x_f < x else -v)
        
        # print(v, x, y, th, x_f)
        self.setSpeed(v, 0.0)
        
        while abs(x_f - x) > 0.015:
            time.sleep(self.P_CHECK_POS)
            x,y,th = self.readOdometry()
        self.setSpeed(0,0)
        
        self.changeOdometry(x_f,y,th)
            
            
        
        
        
        
        
            
        

        
