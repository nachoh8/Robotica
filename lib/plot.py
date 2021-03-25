import numpy as np
import matplotlib.pyplot as plt
import csv
import sys

COLOR_RED = 'r'
SMALL_SIZE = 'p'
BIG_SIZE = 'b'

# Dibuja robot en location_eje con color (c) y tamano (p/g)
def dibrobot(loc_eje: list, c: str, tamano: str):
  """
  Plot robot on location loc_eje
  - loc_eje: Location [x (m), y (m), th (rads)]
  - c: Color of plot ('r': red, 'b': blue, ...)
  - tamano: Size of plot ('p': small, othet: big)
  """

  if tamano == SMALL_SIZE:
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

def plot_log_file(file_name: str, color: str, size: str, show=False):
  """
  Plot points of file_name and show plot at the end
  - file_name: csv file
  - color: Color of plot
  - size = Size of plot
  """

  with open(file_name) as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    n_line = 0
    for line in csv_reader:
      if len(line) != 3:
        print("Error in line " + str(n_line))
        return
      
      if show: print(line[0] + " | " + line[1] + " | "+  line[2])
      
      if n_line > 0:
        dibrobot([float(line[0]), float(line[1]), float(line[2])], color, size)  
      n_line += 1

  plot_show()

def plot_show():
  plt.gca().set_aspect('equal', adjustable='box')
  plt.show()

def main():
  if len(sys.argv) != 2:
    print("Error")
    return
  plot_log_file(sys.argv[1], COLOR_RED, SMALL_SIZE, True)

if __name__ == "__main__":
    main()
  
