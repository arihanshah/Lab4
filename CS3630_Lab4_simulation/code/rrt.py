from time import sleep

from cmap import *
from gui import *
from utils import *
import random
import math

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
    print(get_dist(node0, node1))
    if distance <= limit:
        print(get_dist(node1, node0))
        return node1
    else:
        angle = np.arctan2(node1.y - node0.y, node1.x - node0.x)
        newX = node0.x + (limit * math.cos(angle))
        newY = node0.y + (limit * math.sin(angle))
        newNode = Node([newX, newY], node0)
        print(get_dist(newNode, node0))
        return newNode
    #didn't use arctan2 don't know what a vector angle is supposed to do.
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
    newX = random.randint(0,w)
    newY = random.randint(0,h)

    rand_node = Node([newX, newY])
    
    while not cmap.is_inbound(rand_node) or cmap.is_inside_obstacles(rand_node):
        newX = random.randint(0,w)
        newY = random.randint(0,h)

        rand_node = Node([newX, newY])
    ############################################################################
    return rand_node


def RRT(cmap, start):
    cmap.add_node(start)

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
        distance = get_dist(rand_node, nodes[0])
        index = 0 
        for i in range(1,len(nodes)):
            if get_dist(rand_node, nodes[i]) < distance:
                index = i
                distance = get_dist(rand_node, nodes[i])
        nearest_node = nodes[index]
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

################################################################################
#                     DO NOT MODIFY CODE BELOW                                 #
################################################################################

class RRTThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
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
    global grid, stopevent
    stopevent = threading.Event()
    cmap = CozMap("maps/map2.json", node_generator)
    visualizer = Visualizer(cmap)
    robot = RRTThread()
    robot.start()
    visualizer.start()
    stopevent.set()
