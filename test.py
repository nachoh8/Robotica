from Robot import Robot
import time

def main():
    robot = Robot()
    robot.setSpeed(0, 1)
    time.sleep(2)
    robot.setSpeed(0, 0)

if __name__ == "__main__":
    main()
