# coding: utf-8

# # Rapidly-Exploring Random Tree (RRT)
# 
# Your task is to generate an RRT based on the following pseudocode:
# 
# ```
# def generate_RRT(x_init, num_vertices, dt):
#     rrt = RRT(x_init)
#     for k in range(num_vertices):
#         x_rand = sample_state()
#         x_near = nearest_neighbor(x_rand, rrt)
#         u = select_input(x_rand, x_near)
#         x_new = new_state(x_near, u, dt)
#         # directed edge
#         rrt.add_edge(x_near, x_new, u)
#     return rrt
# ```
#     
# The `RRT` class has already been implemented. Your task is to complete the implementation of the following functions:
# 
# * `sample_state`
# * `nearest_neighbor`
# * `select_input`
# * `new_state`
# 

import numpy as np 
import matplotlib
#matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from sklearn.neighbors import KDTree
import networkx as nx
from IPython import get_ipython
import time

from enum import Enum
from queue import PriorityQueue

get_ipython().run_line_magic('matplotlib', 'inline')
plt.switch_backend('Qt5agg')

plt.rcParams['figure.figsize'] = 12, 12


class RRT:
    def __init__(self, x_init):
        # A tree is a special case of a graph with
        # directed edges and only one path to any vertex.
        self.tree = nx.DiGraph()
        self.tree.add_node(x_init)
                
    def add_vertex(self, x_new):
        self.tree.add_node(tuple(x_init))
    
    def add_edge(self, x_near, x_new, u):
        self.tree.add_edge(tuple(x_near), tuple(x_new), orientation=u)
        
    @property
    def vertices(self):
        return self.tree.nodes()
    
    @property
    def edges(self):
        return self.tree.edges()
                                                  

'''def create_grid():
    grid = np.zeros((100, 100))
    # build some obstacles
    grid[10:20, 10:20] = 1
    grid[63:80, 10:20] = 1
    grid[43:60, 30:40] = 1
    grid[71:86, 38:50] = 1
    grid[10:20, 55:67] = 1
    grid[80:90, 80:90] = 1
    grid[75:90, 80:90] = 1
    grid[30:40, 60:82] = 1
    return grid'''


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """
       
    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1
    
    # ~print('INFO', grid, drone_altitude, safety_distance)
    # ~print(grid, int(north_min), int(east_min))        
 

    #print(grid, drone_altitude, safety_distance)
    #print(grid, int(north_min), int(east_min))
    return grid, int(north_min), int(east_min)



    
# environment encoded as a grid

TARGET_ALTITUDE = 5
SAFETY_DISTANCE = 5

#self.target_position[2] = TARGET_ALTITUDE

# TODO: read lat0, lon0 from colliders into floating point values

# TODO: set home position to (lon0, lat0, 0)

# TODO: retrieve current global position

# TODO: convert to current local position using global_to_local()

#print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                    #self.local_position))
# Read in obstacle map
data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

# Define a grid for a particular altitude and safety margin around obstacles
grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

# Let's take a look at the example environment we'll be using.

plt.imshow(grid, cmap='Greys', origin='upper')
    

# Next you'll implement the functions necessary to generate an RRT. Feel free to change the function signatures however you please, just remember to update `generate_RRT` accordingly.

# ### Sampling States
# 
# The first part of generating an RRT is sampling states based on the environment. The sampled state must be in free space. 


def sample_state(grid):
    x = np.random.uniform(0, grid.shape[0])
    y = np.random.uniform(0, grid.shape[1])
    return (x, y)


# ### Nearest Neighbors
# 
# A critical part of the RRT procedure is finding the closest vertex to the sampled random point. This the most computationally intensive part so be mindful of that. Depending on the number of vertices a naive implementation will run into trouble quickly.


def nearest_neighbor(x_rand, rrt):
    
    x_goal = (30, 750)
    print (x_goal) 
    
    wp_radius = np.linalg.norm(x_goal)
    print (wp_radius)
 
    closest_dist = 100000
    closest_vertex = None
    x_rand = np.array(x_rand)
    x_goal = ( 30,750)
    print ("Generating RRT")

    for v in rrt.vertices:
        d = np.linalg.norm(x_rand - np.array(v[:2]))
        if d < closest_dist:
            closest_dist = d
            closest_vertex = v
            
            beans = np.array(v[:2])
            spinach = x_goal - np.array(v[:2])
            
<<<<<<< HEAD
            print ("x_goal", x_goal)
            print ("np.array",beans)
            print ("matrix_norm", spinach)
            print ("np.array", beans) 
            print ("x_rand", x_rand)            
            
=======
            
            '''
            print ("matrix_norm", spinach)
            print ("np.array", beans) 
            print ("x_rand", x_rand)
            print ("np.array",)'''
>>>>>>> 9a425b1a87633760cd28a26a5f24d71ab92593e8
           
            # ~arrive at goal  
            # spinach = np.linalg.norm(v[:2] - x_goal)
            # beans = np.array[np.linalg.norm(v[:2])]
        #if np.linalg.norm(v[:2] - x_goal) < 1.0:
        if np.linalg.norm(spinach) < 1.0:
            print("Found Goal")    
            break
    'print (np.array(v[:2])'
    print(bool(np.linalg.norm(spinach) < 1.0))
    return closest_vertex


# ### Selecting Inputs
# 
# Select input which moves `x_near` closer to `x_rand`. This should return the angle or orientation of the vehicle.


def select_input(x_rand, x_near):
    return np.arctan2(x_rand[1] - x_near[1], x_rand[0] - x_near[0])


# ### New State
# 
# 

# The new vertex `x_new` is calculated by travelling from the current vertex `x_near` with a orientation `u` for time `dt`.


def new_state(x_near, u, dt):
    nx = x_near[0] + np.cos(u)*dt
    ny = x_near[1] + np.sin(u)*dt
    return [nx, ny]


# ### Putting It All Together
# 
# Awesome! Now we'll put everything together and generate an RRT.


def generate_RRT(grid, x_init, num_vertices, dt,):
    
    'print ("Generating RRT...")'
    rrt = RRT(x_init)
    
    for _ in range(num_vertices):
        
        x_rand = sample_state(grid)
        # sample states until a free state is found
        while grid[int(x_rand[0]), int(x_rand[1])] == 1:
            x_rand = sample_state(grid)
            
        x_near = nearest_neighbor(x_rand, rrt)
        u = select_input(x_rand, x_near)
        x_new = new_state(x_near, u, dt)
            
        if grid[int(x_new[0]), int(x_new[1])] == 0:
            # the orientation `u` will be added as metadata to
            # the edge
            rrt.add_edge(x_near, x_new, u)
    
    print ("RRT Path Mapped")
    return rrt            
'''       
        
    # ~ arrive at goal    
       
    def found_goal(grid, h, start, goal):    
        path = []
        path_cost = 0
        queue = PriorityQueue()
        queue.put((0, start))
        visited = set(start)

        branch = {}
        found = False
        
        while not queue.empty():
            item = queue.get()
            current_node = item[1]
            if current_node == start:
                current_cost = 0.0
            else:              
                current_cost = branch[current_node][0]
                
            if current_node == goal:        
                print('Found a path.')
                found = True
                break
            else:
                for action in valid_actions(grid, current_node):
                    # get the tuple representation
                    da = action.delta
                    next_node = (current_node[0] + da[0], current_node[1] + da[1])
                    branch_cost = current_cost + action.cost
                    queue_cost = branch_cost + h(next_node, goal)
                    
                    if next_node not in visited:                
                        visited.add(next_node)               
                        branch[next_node] = (branch_cost, current_node, action)
                        queue.put((queue_cost, next_node))
                
        if found:
            # retrace steps
            n = goal
            path_cost = branch[n][0]
            path.append(goal)
            while branch[n][1] != start:
                path.append(branch[n][1])
                n = branch[n][1]
            path.append(branch[n][1])
        else:
            print('**********************')
            print('Failed to find a path!')
            print('**********************') 
        print('~path from a_star method', path[::-1])
        return rrt, path[::-1], path_cost

         
    #return rrt

def heuristic(position, goal_position):
    
    #print('position =', position, 'goal position = ', goal_position)
    #print ('np.linag.norm = ',np.linalg.norm(np.array(position) - np.array(goal_position)))
    return np.linalg.norm(np.array(position) - np.array(goal_position))'''

# Feel free to change any of the values below.

x_goal = (30, 750)
num_vertices = 1600
dt = 18
x_init = (20, 150)

rrt = generate_RRT(grid, x_init, num_vertices, dt)

# Now let's plot the generated RRT.


plt.imshow(grid, cmap='Greys', origin='lower')
plt.plot(x_init[1], x_init[0], 'ro')
plt.plot(x_goal[1], x_goal[0], 'ro')

for (v1, v2) in rrt.edges:
    plt.plot([v1[1], v2[1]], [v1[0], v2[0]], 'y-')

plt.show(block=True)
#plt.show()
