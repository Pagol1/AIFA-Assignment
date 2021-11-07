# Generates a random map in the form of an image.
# MUST be run atleast once by setting "generate_random_map" to 1 in config.json
# Saves image to ./process/random_map.png
import numpy as np
import cv2

class GenerateMap:
    def __init__(self, height, width, start, goal, path_val, obs_val, start_val, end_val):
        self.height = height
        self.width = width
        # Coloring scheme to identify nodes
        self.path_val = path_val
        self.obs_val = obs_val
        self.start_val = start_val
        self.end_val = end_val

        self.map =  np.ones((self.height,self.width), np.uint8) * self.path_val

        self.start = start
        self.goal = goal

        self.map[self.start[0],self.start[1]] = self.start_val
        self.map[self.goal[0],self.goal[1]] = self.end_val

    def add_obstacle(self,x,y,width,height,clearance):
        flag = 1
        for i in range(x - clearance , x + width + clearance):
            for j in range(y - clearance , y + height + clearance):
                if(i == self.start[0] and j == self.start[1]):
                    flag = 0
                    break
                if(i == self.goal[0] and j == self.goal[1]):
                    flag = 0
                    break
        
        if(flag):
            for i in range(x,x+width):
                for j in range(y,y+height):
                    self.map[i,j] = self.obs_val

    def save_map(self, img_loc):
        cv2.imwrite(img_loc, self.map)
