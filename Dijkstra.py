from Map import *
from Node import *
import math as m 
import heapq as hq 
import cv2
import time
def stopwatch(func):
    def timeWrapper(*args, **kwargs):
        begin = time.process_time()
        img = func(*args, **kwargs)
        end = time.process_time()
        print(end-begin, end = '\n')
        return img
    return timeWrapper

class Dijkstra(Map):
    """ Runs Dijkstra on the pre-processed map """
    def __init__(self, img_loc, path_val, obs_val, start_val, end_val, show_process):
        self.img_loc = img_loc
        self.show_process = show_process
        super().__init__(img_loc, path_val, obs_val, start_val, end_val)

    @stopwatch
    def run(self, hueristic = "Euclidean", scale1 = 1, scale2 = 1):
        h_f = eval("Dijkstra."+hueristic)
        #Initialise
        self.start_node = self.start_nodes[0]
        self.end_node = self.end_nodes[0]
        self.start_node.parent = self.start_node
        self.start_node.g_cost = 0; self.start_node.h_cost = h_f(self.start_node.position, self.end_node.position, scale = scale2, ignore = True)
        self.start_node.f_cost = self.start_node.g_cost + self.start_node.h_cost
        self.end_node.h_cost = 0
        hq.heappush(self.priority_queue, self.start_node)
        self.add_to_frontier(self.start_node)

        while (len(self.frontier)>0):
            #Select
            curr = hq.heappop(self.priority_queue)
            self.remove_from_frontier(curr)
            #Goal Test
            if curr == self.end_node:
                break
            else:
                pass
            #Expand
            for neigh in curr.neighbours:
                if neigh.walked == False:
                    val = neigh.check_costs_star(curr, self.end_node, h_f, h_f, scale1, scale2)

                    if val == False:
                        self.add_to_frontier(neigh)
                    if val != -1:
                        self.count += 1
            
            if (self.show_process):
                # Animate the progress
                timg = cv2.imread(self.img_loc)  
                self.draw_travesed(timg)
                self.draw_frontier(timg)
                self.draw_temp_path(timg, curr)
                # timg = upConstant(timg,int(1000/timg.shape[0])).astype(np.uint8) # Upscale image
                cv2.imshow("Dijkstra-"+hueristic, timg.astype(np.uint8))
                cv2.waitKey(1)

            self.priority_queue = self.frontier.copy()
            hq.heapify(self.priority_queue)
        
        cv2.destroyAllWindows()

    def Manhattan(pos1, pos2, scale, ignore=False):
        return (abs(pos1[0]-pos2[0]) + abs(pos1[1]-pos2[1]))*scale*(not ignore)

    def Euclidean(pos1, pos2, scale, ignore=False):
        return (m.sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2))*scale*(not ignore)