import numpy as np
import math

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

def norm_pi(angle: float) -> float:
  """ Normalize angle in degrees to radians """
  return norm_rad(math.radians(angle))

def norm_rad(rad: float) -> float:
  """ Normalize angle in radians """
  if rad >= -np.pi:
      return -np.pi + (rad + np.pi) % (2 * np.pi)
  else:
      return np.pi - (-np.pi - rad) % (2 * np.pi)

def hom(xWR: np.array) -> np.array:
  c = np.cos(xWR[2])
  s = np.sin(xWR[2])
  return np.array([[c, -1*s if s != 0 else 0, xWR[0]],
                    [s, c, xWR[1]],
                    [0,0,1]])

def loc(tWR: np.array) -> np.array:
  return np.array([tWR[0][2], tWR[1][2], np.arctan2(tWR[1][0], tWR[0][0])])