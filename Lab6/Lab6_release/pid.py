import cozmo
import threading
import json
import math
import sys
import time
import numpy as np
import asyncio


async def CozmoPID(robot: cozmo.robot.Robot):
    global stopevent
    with open("./config.json") as file:
        config = json.load(file)
    kp = config["kp"]
    ki = config["ki"]
    kd = config["kd"]
    await robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()
    if robot.world.visible_object_count() > 0:
        for obj in robot.world.visible_objects:
            cube = obj
            break
    else:
        print("Cube not found")
        return
    robot.pose.position._x = 0
    robot.pose.position._y = 0
    cube_pos = cube.pose.position.x
    while True:
        # print(robot.pose.position.x)
        dist = cube_pos - 100 - robot.pose.position.x
        velocity = kp*dist
        await robot.drive_wheels(velocity, velocity)
        await asyncio.sleep(1)


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
