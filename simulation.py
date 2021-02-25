# Ignacio Herrera Seara 756920

import numpy as np
import matplotlib.pyplot as plt
import math

# Dibuja robot en location_eje con color (c) y tamano (p/g)
def dibrobot(loc_eje,c,tamano):
  if tamano=='p':
    largo=0.1
    corto=0.05
    descentre=0.01
  else:
    largo=0.5
    corto=0.25
    descentre=0.05

  trasera_dcha=np.array([-largo,-corto,1])
  trasera_izda=np.array([-largo,corto,1])
  delantera_dcha=np.array([largo,-corto,1])
  delantera_izda=np.array([largo,corto,1])
  frontal_robot=np.array([largo,0,1])
  tita=loc_eje[2]
  Hwe=np.array([[np.cos(tita), -np.sin(tita), loc_eje[0]],
             [np.sin(tita), np.cos(tita), loc_eje[1]],
              [0,        0 ,        1]])
  Hec=np.array([[1,0,descentre],
              [0,1,0],
              [0,0,1]])
  extremos=np.array([trasera_izda, delantera_izda, delantera_dcha, trasera_dcha, trasera_izda, frontal_robot, trasera_dcha])
  robot=np.dot(Hwe,np.dot(Hec,np.transpose(extremos)))
  plt.plot(robot[0,:], robot[1,:], c)

# Simula movimiento del robot con vc=[v,w] en T seg. desde xWR
def simubot(vc,xWR,T):
  if vc[1]==0:   # w=0
      xRk=np.array([vc[0]*T, 0, 0])
  else:
      R=vc[0]/vc[1]
      dtitak=vc[1]*T
      titak=norm_pi(dtitak)
      xRk=np.array([R*np.sin(titak), R*(1-np.cos(titak)), titak])  

  xWRp=loc(np.dot(hom(xWR),hom(xRk)))   # nueva localizacion xWR
  return xWRp

# Pre: x angulo en grados
# Post: rad(if x > 180º: x -> [0, -180º] else: x)
def norm_pi(x):
  return math.radians(x if x <= 180 else x - 360)

def hom(xWR: np.array) -> np.array:
  c = np.cos(xWR[2])
  s = np.sin(xWR[2])
  return np.array([[c, -1*s if s != 0 else 0, xWR[0]],
                    [s, c, xWR[1]],
                    [0,0,1]])

def loc(tWR: np.array) -> np.array:
  return np.array([tWR[0][2], tWR[1][2], np.arctan2(tWR[1][0], tWR[0][0])])