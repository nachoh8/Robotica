from lib.Robot import Robot
import time
import numpy as np
import math
import lib.plot as plot
from lib.utils import simubot

def main():
    """
    robot = Robot()
    robot.setSpeed(0.1, 0)
    time.sleep(2)
    robot.setSpeed(0.2, 0)
    time.sleep(2)
    robot.setSpeed(0.3, 0)
    time.sleep(16)
    robot.setSpeed(0, 0)
    """

    # plot.plot_log_file("test_plot.csv", plot.COLOR_RED, plot.SMALL_SIZE)
    robot = Robot()
    robot.startOdometry()
    robot.go_to(0,-0.25,0,0,-np.pi/2, 0.015)
    robot.stopOdometry()
    return
    xWR = np.array([0,0,0])
    T = 0.03

    """
    t = 0
    while t < 10:
        #print(xWR)
        plot.dibrobot(xWR, plot.COLOR_RED, plot.BIG_SIZE)
        t += T
        xWR = simubot(vc, xWR, T)
    """

    # 90º dch
    """
    vc = [0, np.rad2deg(-0.25)]
    #plot.dibrobot(xWR, plot.COLOR_RED, plot.BIG_SIZE)
    T = 0.03
    t = 0.00
    while xWR[2] > -np.pi / 2:
        xWR = simubot(vc, xWR, T)
        t += T
        if t > 0.3:
            t = 0.00
            #plot.dibrobot(xWR, plot.COLOR_RED, plot.BIG_SIZE)
            print(str(xWR))
    """
    # arco izq
    print("arco izq")
    vc = [0.1, 0]
    plot.dibrobot(xWR, plot.COLOR_RED, plot.BIG_SIZE)
    t = 0.00
    while xWR[0] < 2:
        xWR = simubot(vc, xWR, T)
        t += T
        if t > 0.5:
            t = 0.00
            plot.dibrobot(xWR, plot.COLOR_RED, plot.BIG_SIZE)
            print(str(xWR))
        
    plot.plot_show()
        


if __name__ == "__main__":
    main()
