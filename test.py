from Robot import Robot
import time

def main():
    robot = Robot()
    robot.setSpeed(0.1, 0)
    time.sleep(2)
    robot.setSpeed(0.2, 0)
    time.sleep(2)
    robot.setSpeed(0.3, 0)
    time.sleep(16)
    robot.setSpeed(0, 0)

if __name__ == "__main__":
    main()
