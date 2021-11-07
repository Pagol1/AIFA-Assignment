# The Map class is declared here
# Map will be used to generate a grid of nodes from an image
# Runs over them to find neighbours. This results in a "string" of nodes.

from Node import *
import cv2
import math as m 
import numpy as np 
import heapq as hq 

class Map:
    def __init__(self, img_loc, path_val, obs_val, start_val, end_val):
        self.path_val = path_val                    # Colors corresponding to different objects
        self.obs_val = obs_val
        self.start_val = start_val
        self.end_val = end_val
        self.premap = cv2.imread(img_loc,cv2.IMREAD_GRAYSCALE)
        self.start_nodes = []                       # Start Nodes detected
        self.end_nodes = []                         # Goal nodes detected
        self.walked_nodes = []                      # Walked nodes, does not include obstacles as they are pruned
        self.frontier = []                          # List of nodes in the priority queue
        self.priority_queue = []                    # Priority Queue, is kept as a heap
        hq.heapify(self.priority_queue)
        self.count = 0

    def create_map(self, kind):
        """ Creates the "string" of Nodes with predefined move sets, change [kind] to change movement type """
        h, w = self.premap.shape
        grid = [[0 for x in range(w)] for y in range(h)] 
        for y in range(h):
            for x in range(w):
                grid[y][x] = Node([y,x], bool(self.premap[y,x] == self.obs_val))    # Defines a grid of Nodes, bool() is True for Walls
                if self.premap[y,x] == self.start_val:
                    self.start_nodes.append(grid[y][x])                             # Stores the start nodes
                elif self.premap[y,x] == self.end_val:
                    self.end_nodes.append(grid[y][x])                               # Stores the end nodes

        m_type = [[1,0], [0,1], [0,-1], [-1,0], [1,1], [1,-1], [-1,1], [-1,-1]]     # Stores values of permissible moves
        # k = 4 Gives Square Movement; while k = 8 Gives Octagonal
        for y in range(1,h-1):
            for x in range(1,w-1):
                for mov in m_type[:kind]:
                    grid[y][x].set_neighbours(grid[y+mov[0]][x+mov[1]])

        for y in [0,h-1]:
            for x in range(0,w):
                for mov in m_type[:kind]:
                    if self.isValid([y+mov[0],x+mov[1]]):
                        grid[y][x].set_neighbours(grid[y+mov[0]][x+mov[1]])

        for x in [0,w-1]:
            for y in range(1,h-1):
                for mov in m_type[:kind]:
                    if self.isValid([y+mov[0],x+mov[1]]):
                        grid[y][x].set_neighbours(grid[y+mov[0]][x+mov[1]])

    def isValid(self,pos):
        h, w = self.premap.shape
        if (pos[0]>=0 and pos[0]<h) and (pos[1]>=0 and pos[1]<w):
            return True
        else:
            return False

    def mark_walked(self, node):
        self.walked_nodes.append(node)
        node.walked = True

    def add_to_frontier(self, node):
        if node.walked == False:
            self.frontier.append(node)

    def remove_from_frontier(self, node):
        self.mark_walked(node)
        self.frontier.remove(node)

    def draw_path(self, img):
        node = self.end_node
        while node.parent is not self.start_node:
            img[node.position[0],node.position[1]] = [50,205,50]
            node = node.parent

    def draw_temp_path(self, img, node1):
        node = node1
        while node.parent is not self.start_node:
            img[node.position[0],node.position[1]] = [50,205,50]
            node = node.parent

    def draw_frontier(self, img):
        for node in self.frontier:
            img[node.position[0],node.position[1]] = [255,106,3]

    def draw_travesed(self,img):
        for node in self.walked_nodes:
            img[node.position[0],node.position[1]] = [3,125,255]