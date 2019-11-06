import cozmo
import math
import sys
import time
import numpy as np
import heapq

from cmap import *
from gui import *
from utils import *

MAX_NODES = 20000


def step_from_to(node0, node1, limit=75):
    ########################################################################
    dist = get_dist(node0, node1)
    if dist < limit:
        return node1
    new_x = node0.x + (node1.x - node0.x) * limit / dist
    new_y = node0.y + (node1.y - node0.y) * limit / dist
    new_node = Node([new_x, new_y])
    return new_node
    ############################################################################


def node_generator(cmap):
    ############################################################################
    if np.random.rand() < 0.05:
        rand_node = Node(cmap.get_goals()[0])
        return rand_node
    valid_node = False
    while not valid_node:
        rand_node = Node([np.random.rand()*cmap.width, np.random.rand()*cmap.height])
        if (cmap.is_inbound(rand_node) and not cmap.is_inside_obstacles(rand_node)):
            valid_node = True
    ############################################################################
    return rand_node


def RRT(cmap, start):
    cmap.add_node(start)
    map_width, map_height = cmap.get_size()
    while (cmap.get_num_nodes() < MAX_NODES):
        ########################################################################
        rand_node = cmap.get_random_valid_node()
        nodes = cmap.get_nodes()
        node_heap = []
        for i in range(len(nodes)):
            heapq.heappush(node_heap, (get_dist(rand_node, nodes[i]), i))
        tmp_dist, tmp_nearest_index = heapq.heappop(node_heap)
        tmp_nearest = nodes[tmp_nearest_index]
        tmp_new = step_from_to(tmp_nearest, rand_node)
        no_nodes = False
        while cmap.is_collision_with_obstacles((tmp_nearest, tmp_new)):
            if len(node_heap) == 0:
                no_nodes = True
                break
            tmp_dist, tmp_nearest_index = heapq.heappop(node_heap)
            tmp_nearest = nodes[tmp_nearest_index]
            tmp_new = step_from_to(tmp_nearest, rand_node)
        if no_nodes:
            continue
        nearest_node = tmp_nearest
        rand_node = tmp_new
        ########################################################################
        time.sleep(0.01)
        cmap.add_path(nearest_node, rand_node)
        if cmap.is_solved():
            break

    path = cmap.get_path()
    smoothed_path = cmap.get_smooth_path()

    if cmap.is_solution_valid():
        print("A valid solution has been found :-) ")
        print("Nodes created: ", cmap.get_num_nodes())
        print("Path length: ", len(path))
        print("Smoothed path length: ", len(smoothed_path))
    else:
        print("Please try again :-(")

# async def object_handler(self, *args, **kwargs):
#     print(args)
#     print(kwargs)

async def CozmoPlanning(conn):
    # Allows access to map and stopevent, which can be used to see if the GUI
    # has been closed by checking stopevent.is_set()
    global cmap, stopevent
    robot = await conn.wait_for_robot()
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = False
    robot.camera.enable_auto_exposure()
    robot.set_head_angle(cozmo.util.degrees(0))
    # robot.add_event_handler(cozmo.objects.EvtObjectObserved, object_handler)
    print("head angle: ", robot.head_angle)
    ########################################################################
    while True:
        await detect_cube_and_update_cmap(robot, [], Node([0, 0]))

def get_global_node(local_angle, local_origin, node):
    """Helper function: Transform the node's position (x,y) from local coordinate frame specified by local_origin and local_angle to global coordinate frame.
                        This function is used in detect_cube_and_update_cmap()
        Arguments:
        local_angle, local_origin -- specify local coordinate frame's origin in global coordinate frame
        local_angle -- a single angle value
        local_origin -- a Node object

        Outputs:
        new_node -- a Node object that decribes the node's position in global coordinate frame
    """
    ########################################################################
    # TODO: please enter your code below.
    new_node = None
    return new_node


async def detect_cube_and_update_cmap(robot, marked, cozmo_pos):
    """Helper function used to detect obstacle cubes and the goal cube.
       1. When a valid goal cube is detected, old goals in cmap will be cleared and a new goal corresponding to the approach position of the cube will be added.
       2. Approach position is used because we don't want the robot to drive to the center position of the goal cube.
       3. The center position of the goal cube will be returned as goal_center.

        Arguments:
        robot -- provides the robot's pose in G_Robot
                 robot.pose is the robot's pose in the global coordinate frame that the robot initialized (G_Robot)
                 also provides light cubes
        cozmo_pose -- provides the robot's pose in G_Arena
                 cozmo_pose is the robot's pose in the global coordinate we created (G_Arena)
        marked -- a dictionary of detected and tracked cubes (goal cube not valid will not be added to this list)

        Outputs:
        update_cmap -- when a new obstacle or a new valid goal is detected, update_cmap will set to True
        goal_center -- when a new valid goal is added, the center of the goal cube will be returned
    """
    print("function")
    global cmap

    # Padding of objects and the robot for C-Space
    cube_padding = 40.
    cozmo_padding = 100.

    # Flags
    update_cmap = False
    goal_center = None

    # Time for the robot to detect visible cubes
    time.sleep(1)
    for obj in robot.world.visible_objects:
        print(robot.world.visible_objects)

    for obj in robot.world.visible_objects:

        if obj.object_id in marked:
            continue

        # Calculate the object pose in G_Arena
        # obj.pose is the object's pose in G_Robot
        # We need the object's pose in G_Arena (object_pos, object_angle)
        dx = obj.pose.position.x - robot.pose.position.x
        dy = obj.pose.position.y - robot.pose.position.y

        object_pos = Node((cozmo_pos.x+dx, cozmo_pos.y+dy))
        object_angle = obj.pose.rotation.angle_z.radians
        print("object_pos: ", object_pos.x, object_pos.y)
        print("object_angle: ", object_angle)

        # The goal cube is defined as robot.world.light_cubes[cozmo.objects.LightCube1Id].object_id
        if robot.world.light_cubes[cozmo.objects.LightCube1Id].object_id == obj.object_id:

            # Calculate the approach position of the object
            local_goal_pos = Node((0, -cozmo_padding))
            goal_pos = get_global_node(object_angle, object_pos, local_goal_pos)

            # Check whether this goal location is valid
            if cmap.is_inside_obstacles(goal_pos) or (not cmap.is_inbound(goal_pos)):
                print("The goal position is not valid. Please remove the goal cube and place in another position.")
            else:
                cmap.clear_goals()
                cmap.add_goal(goal_pos)
                goal_center = object_pos

        # Define an obstacle by its four corners in clockwise order
        obstacle_nodes = []
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((cube_padding, cube_padding))))
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((cube_padding, -cube_padding))))
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((-cube_padding, -cube_padding))))
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((-cube_padding, cube_padding))))
        cmap.add_obstacle(obstacle_nodes)
        marked[obj.object_id] = obj
        update_cmap = True

    return update_cmap, goal_center, marked


# class RobotThread(threading.Thread):
#     """Thread to run cozmo code separate from main thread
#     """
#
#     def __init__(self):
#         threading.Thread.__init__(self, daemon=True)
#
#     def run(self):
#         # Please refrain from enabling use_viewer since it uses tk, which must be in main thread
#         cozmo.setup_basic_logging()
#         cozmo.connect(CozmoPlanning)
#         # cozmo.run_program(CozmoPlanning,use_3d_viewer=False, use_viewer=False)
#         stopevent.set()


class RRTThread(threading.Thread):
    """Thread to run RRT separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        while not stopevent.is_set():
            RRT(cmap, cmap.get_start())
            time.sleep(100)
            cmap.reset()
        stopevent.set()


if __name__ == '__main__':
    global cmap, stopevent
    stopevent = threading.Event()
    robotFlag = False
    for i in range(0,len(sys.argv)):
        if (sys.argv[i] == "-robot"):
            robotFlag = True

    if (robotFlag):
        cmap = CozMap("maps/emptygrid.json", node_generator)
        # visualizer = Visualizer(cmap)
        # visualizer.start()
        cozmo.setup_basic_logging()
        cozmo.connect(CozmoPlanning)
        # stopevent.set()
    else:
        cmap = CozMap("maps/map2.json", node_generator)
        sim = RRTThread()
        sim.start()
    # visualizer = Visualizer(cmap)
    # visualizer.start()
    # stopevent.set()

    # cmap = CozMap("maps/map2.json", node_generator)
    # node0 = node_generator(cmap)
    # node1 = node_generator(cmap)
    # node2 = step_from_to(node0, node1)
    # print(node0.x, node0.y)
    # print(node1.x, node1.y)
    # print(node2.x, node2.y)
    # print((node1.x - node0.x)/(node1.y - node0.y))
    # print((node2.x - node0.x)/(node2.y - node0.y))
    # print(get_dist(node0, node2))