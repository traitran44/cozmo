import cozmo
import threading
import json
import math
import sys
import time
import numpy as np
import asyncio
import time


async def CozmoPID(robot: cozmo.robot.Robot):
    global stopevent
    with open("./config.json") as file:
        config = json.load(file)
    kp = config["kp"]
    ki = config["ki"]
    kd = config["kd"]
    await robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()
    await robot.set_lift_height(1).wait_for_completed()
    robot.move_lift(1.5)
    if robot.world.visible_object_count() > 0:
        for obj in robot.world.visible_objects:
            cube = obj
            break
    else:
        print("Cube not found")
        return
    robot.pose.position._x = 0
    robot.pose.position._y = 0
    robot.pose._rotation = cozmo.util.Rotation(0, 0, 0, 0)
    cube_pos = cube.pose.position.x
    # total_dist = 0
    dist = cube_pos - 140 - robot.pose.position.x
    while True:
        # print(robot.pose.position.x)
        prev_dist = dist
        dist = cube_pos - 140 - robot.pose.position.x
        # total_dist = ki * (total_dist + dist)
        dist_change = dist - prev_dist
        # if dist_change == 0:
        #     dist_change = 1
        velocity = kp*dist + kd*dist_change
        print("Velocity: ", velocity)
        if abs(velocity) < 20:
            print("Arrived at goal")
            await robot.drive_wheel_motors(0, 0)
            break
        await robot.drive_wheel_motors(velocity, velocity)
        await asyncio.sleep(0.01)


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
