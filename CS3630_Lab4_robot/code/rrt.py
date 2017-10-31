import cozmo

from cmap import *
from gui import *
from utils import *
import math
import random
from time import sleep
from transitions import Machine
import numpy as np

MAX_NODES = 20000

################################################################################
# NOTE:
# Before you start, please familiarize yourself with class Node in utils.py
# In this project, all nodes are Node object, each of which has its own
# coordinate and parent if necessary. You could access its coordinate by node.x
# or node[0] for the x coordinate, and node.y or node[1] for the y coordinate
################################################################################

def step_from_to(node0, node1, limit=75):
    ############################################################################
    # TODO: please enter your code below.
    # 1. If distance between two nodes is less than limit, return node1
    # 2. Otherwise, return a node in the direction from node0 to node1 whose
    #    distance to node0 is limit. Recall that each iteration we can move
    #    limit units at most
    # 3. Hint: please consider using np.arctan2 function to get vector angle
    # 4. Note: remember always return a Node object
    distance = get_dist(node0, node1)
    if distance <= limit:
        newNode = Node([node1.x, node1.y], node0)
        return newNode
    else:
        angle = np.arctan2(node1.y - node0.y, node1.x - node0.x)
        newX = node0.x + (limit * math.cos(angle))
        newY = node0.y + (limit * math.sin(angle))
        newNode = Node([newX, newY], node0)
        return newNode
    ############################################################################


def node_generator(cmap):
    rand_node = None
    ############################################################################
    # TODO: please enter your code below.
    # 1. Use CozMap width and height to get a uniformly distributed random node
    # 2. Use CozMap.is_inbound and CozMap.is_inside_obstacles to determine the
    #    legitimacy of the random node.
    # 3. Note: remember always return a Node object
    w, h = cmap.get_size()
    newX = random.randint(0, w)
    newY = random.randint(0, h)

    rand_node = Node([newX, newY])

    while not cmap.is_inbound(rand_node) or cmap.is_inside_obstacles(rand_node):
        newX = random.randint(0, w)
        newY = random.randint(0, h)

        rand_node = Node([newX, newY])
    ############################################################################
    return rand_node


def RRT(cmap, start):
    cmap.set_start(start)

    map_width, map_height = cmap.get_size()

    while (cmap.get_num_nodes() < MAX_NODES):
        ########################################################################
        # TODO: please enter your code below.
        # 1. Use CozMap.get_random_valid_node() to get a random node. This
        #    function will internally call the node_generator above
        # 2. Get the nearest node to the random node from RRT
        # 3. Limit the distance RRT can move
        # 4. Add one path from nearest node to random node
        #
        rand_node = cmap.get_random_valid_node()
        nodes = cmap.get_nodes()
        distance = get_dist(rand_node, cmap.get_start())
        index = 0
        if len(nodes) > 0:
            for i in range(0,len(nodes)):
                if get_dist(rand_node, nodes[i]) < distance:
                    index = i
                    distance = get_dist(rand_node, nodes[i])
            nearest_node = nodes[index]
        else:
            nearest_node = cmap.get_start()
        next_node = step_from_to(nearest_node, rand_node)
        pass
        ########################################################################
        sleep(0.01)
        cmap.add_path(nearest_node, next_node)
        if cmap.is_solved():
            break

    if cmap.is_solution_valid():
        print("A valid solution has been found :-) ")
    else:
        print("Please try again :-(")


class CubeSearcher(object):
    global cmap, stopevent

    states = ["search_cube", "plan_path", "drive_path", "found_dest"]

    def __init__(self, robot):
        self.fsm = Machine(model=self, states=CubeSearcher.states, initial='search_cube', ignore_invalid_triggers=True)

        self.fsm.add_transition(trigger='found_cube', source='search_cube', dest='plan_path')
        self.fsm.add_transition(trigger='found_path', source='plan_path', dest='drive_path')
        self.fsm.add_transition(trigger='found_face', source='*', dest='found_dest')
        self.robot = robot

async def CozmoPlanning(robot: cozmo.robot.Robot):
    # Allows access to map and stopevent, which can be used to see if the GUI
    # has been closed by checking stopevent.is_set()
    global cmap, stopevent

    ########################################################################
    # TODO: please enter your code below.
    # Description of function provided in instructions

    start = Node([100, 75])
    cubeSearcher = CubeSearcher(robot)
    angled = False
    centered = False

    try:

        detect_cube = False
        cube_seen = None
        path = []

        while (True):

            state = cubeSearcher.state

            # FOR TESTING ###############
            # state = "hey"
            if state == "hey":
                try:
                    cube_seen = await robot.world.wait_for_observed_light_cube(timeout=2)
                    print ("x : {} , y : {} , angle : {} ".format(cube_seen.pose.position.x,
                                                                  cube_seen.pose.position.y,
                                                                  cozmo.util.Angle(cube_seen.pose.rotation.angle_z.radians).degrees))
                except:
                    cube_seen = None
            ###############################

            if state == "search_cube":
                try:
                    cube_seen = await robot.world.wait_for_observed_light_cube(timeout=2)
                    print ("Id : ", cube_seen.object_id)
                    print ("Real Id : ", robot.world.light_cubes[cozmo.objects.LightCube1Id].object_id)
                    if cube_seen.object_id == robot.world.light_cubes[cozmo.objects.LightCube1Id].object_id:
                        z_angle = cube_seen.pose.rotation.angle_z.radians
                        x = cube_seen.pose.position.x + 22 * np.cos(z_angle) + 55 * np.cos(z_angle)
                        y = cube_seen.pose.position.y + 22 * np.sin(z_angle) + 55 * np.sin(z_angle)
                        # angle_z =cozmo.util.Angle(z_angle + np.pi)

                        #x = cube_seen.pose.position.x
                        #y = cube_seen.pose.position.y

                        print (x)
                        print (y)
                        cmap.add_goal(Node([x, y]))
                        ObX, ObY = cube_seen.pose.position.x, cube_seen.pose.position.y
                        offset = 25
                        cmap.add_obstacle([Node([ObX - offset, ObY + offset]),
                                           Node([ObX + offset, ObY + offset]),
                                           Node([ObX + offset, ObY - offset]),
                                           Node([ObX - offset, ObY - offset])])
                        cubeSearcher.found_cube()
                        print ("THE REAL STATE : ", cubeSearcher.state)

                    else:
                        pass
                        # ObX, ObY = cube_seen.pose.position.x, cube_seen.pose.position.y
                        # offset = 32.5
                        # cmap.add_obstacle([Node([ObX - offset, ObY + offset]),
                        #                    Node([ObX + offset, ObY + offset]),
                        #                    Node([ObX + offset, ObY - offset]),
                        #                    Node([ObX - offset, ObY - offset])])
                except:
                    angle = 45
                    dist = 275
                    if not angled and not cube_seen:
                        await robot.turn_in_place(angle=cozmo.util.Angle(degrees=45)).wait_for_completed()
                        angled = True
                    elif not centered and not cube_seen:
                        xDist = dist * math.cos(math.pi / 4)
                        yDist = dist * math.sin(math.pi / 4)
                        robot.drive_straight(distance=cozmo.util.Distance(275.0), speed=cozmo.util.Speed(600.0))
                        centered = True
                        start = Node([start.x + xDist, start.y + yDist])
                    else:
                        await robot.drive_wheels(-60, 60, duration=1)


            if state == "plan_path":
                cubeSearcher.found_path()
                RRT(cmap, start)


            

            if state == "drive_path":
                print(cmap.is_solved())
                index = 0

                if cmap.is_solved():
                    goal = cmap.get_goals()[0]
                    print(goal.parent)
                    cur = goal
                    while cur.parent is not None:
                        path.append([cur.x,cur.y])
                        cur = cur.parent
                        index = index + 1
                    
                    print(path)

                    cubeSearcher.found_face()

            if state == "found_face":
                pass

            # if not detect_cube:
            #     try:
            #         cube_seen = await robot.world.wait_for_observed_light_cube(timeout=5)
            #
            #         if cube_seen.object_id == robot.world.light_cubes[cozmo.objects.LightCube2Id].object_id:
            #             print (cube_seen)
            #             detect_cube = True
            #             cmap.add_goal(Node([cube_seen.pose.Position.x, cube_seen.pose.Position.y]))
            #             bang = False
            #     except:
            #         if not cube_seen:
            #             detect_cube = False
            #             print ("hey")
            #
            #             await  robot.drive_wheels(-60, 60, duration=1)
            #             robot.drive_straight(distance=cozmo.util.Distance(275.0), speed=cozmo.util.Speed(600.0))




                            # oX1 = start.x + cube_seen.pose.position.x - 32
                            # oY1 = start.y + cube_seen.pose.position.y + 32
                            #
                            # oX2 = start.x + cube_seen.pose.position.x + 32
                            # oY2 = start.y + cube_seen.pose.position.y + 32
                            #
                            # oX3 = start.x + cube_seen.pose.position.x + 32
                            # oY3 = start.y + cube_seen.pose.position.y - 32
                            #
                            # oX4 = start.x + cube_seen.pose.position.x - 32
                            # oY4 = start.y + cube_seen.pose.position.y - 32
                            #
                            # cmap.add_obstacle([Node([oX1, oY1]), Node([oX2, oY2]), Node([oX3, oY3]), Node([oX4, oY4])])
                            # print ("hey")


    except KeyboardInterrupt:
        print("")
        print("Exit requested by user")
    except cozmo.RobotBusy as e:
        print(e)

    

################################################################################
#                     DO NOT MODIFY CODE BELOW                                 #
################################################################################

class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        # Please refrain from enabling use_viewer since it uses tk, which must be in main thread
        cozmo.run_program(CozmoPlanning,use_3d_viewer=False, use_viewer=False)
        stopevent.set()


class RRTThread(threading.Thread):
    """Thread to run RRT separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        while not stopevent.is_set():
            RRT(cmap, cmap.get_start())
            sleep(100)
            cmap.reset()
        stopevent.set()


if __name__ == '__main__':
    global cmap, stopevent
    stopevent = threading.Event()
    cmap = CozMap("maps/emptygrid.json", node_generator)
    robot_thread = RobotThread()
    robot_thread.start()
    visualizer = Visualizer(cmap)
    visualizer.start()
    stopevent.set()
