from Astar import *
from Dijkstra import *
from Upscale import *
from GenerateMap import *
import cv2
import os
import time
import json
import random

if __name__ == "__main__":

    s_time = time.time()

    f = open('config.json',)
    
    data = json.load(f)
    
    for param in data['parameters']:
        height = param['height']
        width = param['width']
        start_pose = param['start_pose']
        goal_pose = param['goal_pose']
        generate_random_map = param['generate_random_map']
        num_random_obstacles = param['num_random_obstacles']
        rand_obstacle_size = param['rand_obstacle_size']
        clearance = param['clearance']
        algo_huer = param['type']
        valid_moves = param['valid_moves']
        show_process = param['show_process']
    
    f.close()

    parent_dir_path = os.path.dirname(os.path.realpath(__file__))
    dir_path = os.path.join(parent_dir_path, "process")
    if not os.path.exists(dir_path):
        os.mkdir(dir_path)
    img_loc = os.path.join(dir_path, "random_map.png")

    path_val = 0; obs_val = 255; start_val = 100; end_val = 150

    if generate_random_map:
        _map = GenerateMap(height, width, start_pose, goal_pose, path_val, obs_val, start_val, end_val)
        for i in range(num_random_obstacles):
            _map.add_obstacle(random.randint(0,width - rand_obstacle_size), random.randint(0,height - rand_obstacle_size), random.randint(0,rand_obstacle_size), random.randint(0,rand_obstacle_size), clearance)
        _map.save_map(img_loc)

    if not os.path.exists(img_loc):
        print("ERROR: Please generate a map first by setting [generate_random_map] to 1 in config.json")
        exit()
    
    # Algorithm, hueristic and move set:
    # Valid options:
        # Algorithms:
        # + Astar:  [Path cost is euclidean and is precise as only adjacent nodes are neighbours]
        #     - Chessboard (Chebyshev Distance)
        #     - Manhattan
        #     - Euclidean
        #     - Diagonal
        # + Dijkstra: [Used to calculate path cost]
        #     - Manhattan   [valid_moves = 4]
        #     - Euclidean   [valid_moves = 8]
    algo, huer = algo_huer.split('-')
    algo = eval(algo)
    Solver = algo(img_loc, path_val, obs_val, start_val, end_val, show_process)
    Solver.create_map(kind = valid_moves)
    
    print(algo_huer+":-\n  Execution time: ",end='')
    Solver.run(huer,1,1)
    print("  Number of expanded nodes: "+str(Solver.count))
    result = cv2.imread(img_loc)
    Solver.draw_travesed(result)
    Solver.draw_path(result)
    Solver.draw_frontier(result)
    cv2.imshow(algo_huer+": Result",upConstant(result,int(1000/height)).astype(np.uint8))
    cv2.waitKey(0)
