import cozmo
import threading
import json
import math
import sys
import time
import numpy as np


async def CozmoPID(robot: cozmo.robot.Robot):
    global stopevent
    with open("./config.json") as file:
        config = json.load(file)
    kp = config["kp"]
    ki = config["ki"]
    kd = config["kd"]
    ###############################
    # PLEASE ENTER YOUR CODE BELOW
    ###############################


class RobotThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self, daemon=False)

    def run(self):
        cozmo.run_program(CozmoPID)
        stopevent.set()


if __name__ == "__main__":
    global stopevent
    stopevent = threading.Event()
    robot_thread = RobotThread()
    robot_thread.start()
    stopevent.set()
