# The Node class is declared here
import numpy as np

class Node:
    def __init__(self, position, travelled = False, searched = False):
        self.position = position    # Position in the Image
        # Walls/Obstacles will have walked initalized to True
        self.walked = travelled     # Walked Nodes won't be put in the Priority Queue
        self.searched = searched    # Checks if the node has been searched once, i.e. it's present in the priority queue
        self.neighbours = []        # Neighbours of the node will be pre-processed (Hopefully, it reduces computation time)
        self.g_cost = np.inf        # Cost functions, initalized to a high value akin to infinity
        self.h_cost = np.inf
        self.f_cost = np.inf

    def __lt__(self, other):        # For heapq checks on objects, we need this function
        return self.f_cost < other.f_cost

    def set_neighbours(self, neighbour_node):           # Called to add walkable neighbours
        if not neighbour_node.walked:
            self.neighbours.append(neighbour_node)

    def has_neighbours(self):                           # Test function used in debugging, not needed in implementation
        return len(self.neighbours) != 0

    def check_costs_star(self, parent_node, end_node, pathcost, hueristic, scale1 = 1, scale2 = 1):   # Calculates the cost and updates it if it is better than current
        g = parent_node.g_cost + pathcost(self.position, parent_node.position, scale = scale1, ignore = False)
        h = hueristic(self.position, end_node.position, scale = scale2, ignore = True)  # ignore is kept for Dijkstra
        if self.f_cost > g + h:
            self.g_cost = g; self.h_cost = h; self.f_cost = g + h
            self.parent = parent_node
            exitVal = self.searched                                 # Check if obj already exists in the queue
            self.searched = True
            return exitVal                                          # Returns False if obj is not present in queue
        else:
            return -1
