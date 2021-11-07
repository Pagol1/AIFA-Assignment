# AIFA Assignment
Group project for Assignment #1 of the AIFA course \[AI61005\].

### Group Members
- Chirahg Gupta [20ME30016]
- Saksham Kumar Singh [20EE10065]@photon999
- Saumyadip Nandy [20EC10072] @Pagol1
- Vibhanshu Vaibhav [20EE30030]

## Project Description
Our project implements the A* and Dijkstra algorithms with different hueristics.\
The project uses concepts related to the AI problem of search to minimize the path cost.

### Language and Libraries Used
- Python 3
- opencv-python
- numpy
- heapq

### Usage
Clone the repository and run the command `python3 Run.py` to execute.

## Configuration
The parameters in the config.json file can be changed to used to switch between algorithms, heuristics etc. The parameters are:
+ `height`: Height of the map
+ `width`: Width of the map
+ `start_pose`: Start coordinates
+ `goal_pose`: Goal coordinates
+ `generate_random_map`:\
	@Values:
	- 0: Skip map generation and use pre-existing map
	- 1: Generate a new random map\
	@Note: Must be set to 1 in the initial run
+ `num_random_obstacles`: Maximum number of obstacles
+ `rand_obstacle_size`: Maximum ostacle dimension (length or breadth)
+ `clearance`: Minimum obstacle distance from start and end nodes
+ `type`: Specifies the algorithm and hueristic to be used. In the format `[Algorithm]-[Hueristic]`\
	@Values:
	- Algorithms:
		+ `Astar`
		@Hueristics: `Manhattan`, `Chessboard`, `Euclidean` and `Diagonal`
		+ `Dijkstra`
		@Hueristics: `Manhattan`, `Euclidean` (Used to calculate path cost)\
		@Example: `Astar-Euclidean`
+ `valid_moves`: \
	@Values:
	- 4: Square movement
	- 8: Octagonal movement\
	@Note: For Dijkstra use the Euclidean hueristic for octagonal movement
+ `show_process`: Set to 1 to see the current path, frontier and explored nodes; set to 0 otherwise.\
	@Note: Enabling this option heavily increases the execution time.
