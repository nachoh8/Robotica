from lib.Robot import Robot
import time
import lib.plot as plot

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

    plot.plot_log_file("test_plot.csv", plot.COLOR_RED, plot.SMALL_SIZE)

if __name__ == "__main__":
    main()
