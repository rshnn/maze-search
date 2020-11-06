# -*- coding: utf-8 -*-
import numpy as np 
from queue import PriorityQueue

import matplotlib.pyplot as plt
from matplotlib import colors
from matplotlib.path import Path
import matplotlib.patches as patches
import matplotlib.animation as animation
from matplotlib.animation import FFMpegWriter


from maze import Maze 
from position import Position 
from collections import deque
from search import (Metrics, quick_setup, 
                    setup_fringe_and_closedset, 
                    Astar_setup_fringe_and_closedset, 
                    manhattan_distance, euclidean_distance)  


class State():
    """State contains a path of positions and the current maze configuration.  
    """
    def __init__(self, position_path, maze):
        self.position_path = position_path 
        self.maze = maze


    def __lt__(self, other):
        return self.position_path[-1][0] < other.position_path[-1][0]


def fire_setup(dim=15, p=0.2, q=0.5):
    """Returns a maze, initial_state, goal_state.  
        Simiilar to quick_setup(), except q>0.  
    """
    
    maze = Maze(dim, p, q)

    initial_state = State([Position([0,0])], maze.copy()) 
    goal_state = State([Position([maze.dim-1, maze.dim-1])], maze.copy())

    return maze, initial_state, goal_state 


def state_setup(maze): 
    """Returns inital_state and goal_state using the State() object 
        geared towards solving the fire maze problem 
    """

    initial_state = State([Position([0,0])], maze.copy()) 
    goal_state = State([Position([maze.dim-1, maze.dim-1])], maze.copy())

    return initial_state, goal_state 



def simulate_path_and_fire(maze, position_path):
    """An agent runs the input path on the input maze.   
        The input position path is an output to one of the search algorithms
            from the Search module (e.g.  A*, dfs, bfs, ...)
        The agent will proceed one position per time step.
        The maze's fire will spread once per time step.

        Return:  Tuple.
            Boolean if agent survives, 
            List of tuples(path at time t, maze at time t)
    """

    if position_path is None or len(position_path) == 0: 
        raise ValueError("Invalid input: position_path")

    # Create copy of input maze as to not mutate the original 
    maze_ = maze.copy()
    state_path = []

    # Each iteration in this loop can be considered a time step
    # 
    # For each position in position state: 
    #    grow fire in maze   
    #    get subset of path at this time step 
    #    store new maze state and position path up to this time step in a new state
    #    check if fire expanded into current position.  
    #        If so, you dead.  Terminate loop and return state path.
    for idx, position in enumerate(position_path):
        maze_.fire_grows()
        new_s = State(position_path[:idx], maze_.copy())
        state_path.append(new_s)

        if maze_.grid[position[0], position[1]] == 2: 
            return False, state_path

    return True, state_path 





def shortest_distance_to_fire(position, maze, dist_metric=manhattan_distance):
    """Returns the shortest distance to fire stored in maze.cells_on_fire 
    """

    shortest = maze.dim+1
    
    # Loops through the stored list of cells on fire in the Maze object
    for fire_cell in maze.cells_on_fire:

        # Calculate distance from position to each fire cell 
        dist = dist_metric(position, fire_cell) 
        
        # Store the shortest distance 
        if dist < shortest: 
            shortest = dist 

    return shortest   


    
def Astar_fire_setup_fringe_and_closedset(initial_state):
    fringe=PriorityQueue()
    """Returns a fringe with initial_state and an emtpy closed_set
    """

    # Set of states yet to visit along with path leading up to the state
    #   Implemented using Priority queue.  
    # First item is the weight, second item is a tiebreakter, and third is the
    # path taken
    # First item in fringe is initial_state and empty path
    # fringe.put((0,0,[initial_state]))
    fringe.put((0, initial_state.position_path[0], [initial_state]))


    # Set of states already visited.  Set objects do not hold duplicates
    #   The set needs to hold hashable items (immutable).
    #   Thus, tuples representing coordinates will be put into the set.
    #      e.g.    closed_set.add(tuple(position)) 
    closed_set = set()

    return fringe, closed_set




def fire_heuristic_1(position, goal, maze, fire_coef=0):

    fire_origin = Position(0, maze.dim-1)
    fire_dist = manhattan_distance(position, fire_origin)

    goal_dist = manhattan_distance(position, goal)

    return goal_dist - fire_dist


def fire_heuristic_2(position, goal, maze, fire_coef=0):

    fire_origin = Position(0, maze.dim-1)
    fire_dist = manhattan_distance(position, fire_origin)
    goal_dist = manhattan_distance(position, goal)

    return goal_dist - fire_coef*fire_dist



def fire_heuristic_3(position, goal, maze, fire_coef=0.1):

    fire_dist = shortest_distance_to_fire(position, maze)
    goal_dist = manhattan_distance(position, goal)

    return goal_dist - fire_dist*fire_coef




def fire_heuristic_4(position, goal, maze, fire_coef=0.1):

    fire_dist = shortest_distance_to_fire(position, maze, euclidean_distance)
    goal_dist = euclidean_distance(position, goal)

    return goal_dist - fire_dist*fire_coef




def A_star_fire(maze, initial_state, goal_state, 
                heuristic_function=fire_heuristic_3, 
                heuristic_function_coef=0.01):
    """A star search adjusted to avoid fire.
        heuristic_function computes the approximate distance from a given cell 
            to the goal cell.  The function must take the form:
            func(position_1, goal_position, heuristic_coef)
    """
    
    # Initializing fringe and closed_set.  
    #  The fringe is implemented as a PriorirtQueue.  The closed_set is a set() 
    fringe, closed_set = Astar_fire_setup_fringe_and_closedset(initial_state)
        
    # Object that gathers metrics about the search algorithm on this maze
    metrics = Metrics()

    # dictionary for running cost of States
    cost_so_far = {}
    cost_so_far[tuple(initial_state.position_path[-1])] = 0  

    # Create copy of maze to not mutate original 
    maze_ = maze.copy()


    # Each iteration of this while loop is one time step.  
    #  The agent's position will advance once per time step. 
    #  The maze will grow its fire once per time step.  
    while not fringe.empty():
        
        # Updates max fringe size for metrics 
        if fringe.qsize() > metrics.max_fringe_size:
            metrics.max_fringe_size = fringe.qsize()



        # Get next item from the fringe and unpack it 
        #  The fringe contains: (priority, current_position, state_path)
        _, current_position, state_path = fringe.get()
        current_state = state_path[-1]


        # Create copy of maze to not mutate the maze of the previous state
        maze_ = current_state.maze.copy()
        # Time step fire!  burrrrn 
        maze_.fire_grows()


        # Agent's cell caught fire.  They dead
        if maze_.grid[current_position[0], current_position[1]] == 2: 
            return False, state_path, metrics 

        
        if current_position == goal_state.position_path[0]:
            metrics.path_length = len(current_state.position_path)
            return True, state_path, metrics 
        
        if tuple(current_position) not in closed_set:

            # Increment total nodes expanded for metrics 
            metrics.total_nodes_expanded += 1
            closed_set.add(tuple(current_position))


            
            i = current_position[0]
            j = current_position[1]

            cardinal_moves = [[i, j+1], [i, j-1], 
                             [i+1, j], [i-1, j]]

            
            
            for move in cardinal_moves:
                if tuple(move) not in closed_set: 
                    if maze.is_valid_cell(move) and maze.is_empty_cell(move):

                        # Copy current_state's position_path and append move to it 
                        p = list.copy(current_state.position_path)
                        p.append(move)

                        # Create new state to add to the state_path
                        new_s = State(p, maze_.copy())

                        # Create new state_path with new_s appended to it 
                        new_state_path = list.copy(state_path)
                        new_state_path.append(new_s)

                        # calculate the weight, which is equal to the sum of the
                        # number of cells travelled through to reach the move plus the 
                        # heurisitic as an estimate of the distance remaining to goal
                        gNode=len(p)

                        if tuple(move) not in cost_so_far or gNode < cost_so_far[tuple(move)]:
                            cost_so_far[tuple(move)] = gNode
                            hNode = heuristic_function(move, goal_state.position_path[0], 
                                                       maze_, heuristic_function_coef)
                            fNode=gNode+hNode
                            
                            fringe.put((fNode, move, new_state_path))


    return False, state_path, metrics 



def A_star_fire_old(maze, initial_state, goal_state, 
                heuristic_function=fire_heuristic_3, 
                heuristic_function_coef=0.01):
    """A star search adjusted to avoid fire.
        heuristic_function computes the approximate distance from a given cell 
            to the goal cell.  The function must take the form:
            func(position_1, goal_position, heuristic_coef)
    """
    
    # Initializing fringe and closed_set.  
    #  The fringe is implemented as a PriorirtQueue.  The closed_set is a set() 
    fringe, closed_set = Astar_fire_setup_fringe_and_closedset(initial_state)
        
    # Object that gathers metrics about the search algorithm on this maze
    metrics = Metrics()

    # dictionary for running cost of States
    cost_so_far = {}
    cost_so_far[tuple(initial_state.position_path[-1])] = 0  

    # Create copy of maze to not mutate original 
    maze_ = maze.copy()


    # Each iteration of this while loop is one time step.  
    #  The agent's position will advance once per time step. 
    #  The maze will grow its fire once per time step.  
    while not fringe.empty():
        
        # Updates max fringe size for metrics 
        if fringe.qsize() > metrics.max_fringe_size:
            metrics.max_fringe_size = fringe.qsize()

        # Increment total nodes expanded for metrics 
        metrics.total_nodes_expanded += 1


        # Get next item from the fringe and unpack it 
        #  The fringe contains: (priority, current_position, state_path)
        _, current_position, state_path = fringe.get()
        current_state = state_path[-1]
        closed_set.add(tuple(current_position))


        # Create copy of maze to not mutate the maze of the previous state
        maze_ = current_state.maze.copy()
        # Time step fire!  burrrrn 
        maze_.fire_grows()


        # # Get next item from the Priority Queue and unpack it
        # h,count,path_of_states = fringe.get()
        # current_state = path_of_states[-1]
        # s=current_state.path[-1]
        # maze_ = current_state.maze.copy() 

        # maze_.fire_grows()


        # Agent's cell caught fire.  They dead
        if maze_.grid[current_position[0], current_position[1]] == 2: 
            return False, state_path, metrics 

        
        if current_position == goal_state.position_path[0]:
            metrics.path_length = len(current_state.position_path)
            return True, state_path, metrics 
        
        else:
            
            i = current_position[0]
            j = current_position[1]

            cardinal_moves = [[i, j+1], [i, j-1], 
                             [i+1, j], [i-1, j]]

            
            
            for move in cardinal_moves:
                if tuple(move) not in closed_set: 
                    if maze.is_valid_cell(move) and maze.is_empty_cell(move):

                        # Copy current_state's position_path and append move to it 
                        p = list.copy(current_state.position_path)
                        p.append(move)

                        # Create new state to add to the state_path
                        new_s = State(p, maze_.copy())

                        # Create new state_path with new_s appended to it 
                        new_state_path = list.copy(state_path)
                        new_state_path.append(new_s)

                        # calculate the weight, which is equal to the sum of the
                        # number of cells travelled through to reach the move plus the 
                        # heurisitic as an estimate of the distance remaining to goal
                        gNode=len(p)

                        if tuple(move) not in cost_so_far or gNode < cost_so_far[tuple(move)]:
                            cost_so_far[tuple(move)] = gNode
                            hNode = heuristic_function(move, goal_state.position_path[0], 
                                                       maze_, heuristic_function_coef)
                            fNode=gNode+hNode
                            
                            fringe.put((fNode, move, new_state_path))


    return False, state_path, metrics 




def dfs_fire_trivial(maze, initial_state, goal_state, fringe, closed_set):
    """Runs a DFS on a maze with fire.
        At each timestep, the agent can move one position and the fire will grow 
            one position. The agent will prefer to move in this priority order: 
            South, East, West, North 

        Returns True, path, and metrics on success  
        Returns False, None, and metrics on failure   
    """ 

    # Checking the the fringe and closed_set are initialized 
    #   and not already populated from a previous run
    if (len(fringe) != 1) or (len(closed_set) != 0):
        fringe, closed_set = setup_fringe_and_closedset(initial_state)

    # Object that gathers metrics about the search algorithm on this maze
    metrics = Metrics()

    
    maze_ = maze.copy()
    
    while(len(fringe) > 0):

        # Updates max fringe size for metrics 
        if len(fringe) > metrics.max_fringe_size:
            metrics.max_fringe_size = len(fringe)

        # Increment total nodes expanded for metrics 
        metrics.total_nodes_expanded += 1

     
        curr_state, state_path = fringe.pop()
        curr_position = curr_state.path[-1]
        maze_ = curr_state.maze.copy() 
        closed_set.add(tuple(curr_position))

        
        maze_.fire_grows()

        # Agent's cell caught fire.  They dead. 
        if maze_.grid[curr_position[0], curr_position[1]] == 2: 
            return False, state_path, metrics 
        

        if curr_position == goal_state.path[0]: 
            print("Found path")
            metrics.path_length = len(curr_state.path)
            return True, state_path, metrics

        else: 
            # list of all cardinal moves 
       
            all_cardinal_moves = [curr_position.move_north(maze), 
                                  curr_position.move_west(maze), 
                                  curr_position.move_east(maze), 
                                  curr_position.move_south(maze)]        
        
            

            # add valid moves to fringe 
            #    - valid space on grid 
            #    - not in closed_set
            for move in all_cardinal_moves:
                if move is not None and \
                   maze_.is_empty_cell(move) and \
                   (tuple(move) not in closed_set):

                    # Copy position path 
                    pos_path = list.copy(curr_state.path)
                    pos_path.append(move)
                    
                    # Create new state 
                    new_state = State(pos_path, maze_.copy())
                    
                    new_state_path = list.copy(state_path)
                    new_state_path.append(new_state)
                        
                    fringe.append([new_state, new_state_path])


    # print('No path')
    return False, state_path, metrics  




def dfs_fire_avoid(maze, initial_state, goal_state, fringe, closed_set):
    """Runs a DFS on a maze with fire.
        At each timestep, the agent can move one position and the fire will grow 
            one position. The agent will prefer to move in this priority order: 
            South, East, West, North 

        Returns True, path, and metrics on success  
        Returns False, None, and metrics on failure   
    """ 

    # Checking the the fringe and closed_set are initialized 
    #   and not already populated from a previous run
    if (len(fringe) != 1) or (len(closed_set) != 0):
        fringe, closed_set = setup_fringe_and_closedset(initial_state)

    # Object that gathers metrics about the search algorithm on this maze
    metrics = Metrics()

    
    maze_ = maze.copy()
    
    while(len(fringe) > 0):

        # Updates max fringe size for metrics 
        if len(fringe) > metrics.max_fringe_size:
            metrics.max_fringe_size = len(fringe)

        # Increment total nodes expanded for metrics 
        metrics.total_nodes_expanded += 1

     
        curr_state, state_path = fringe.pop()
        curr_position = curr_state.path[-1]
        maze_ = curr_state.maze.copy()  
        closed_set.add(tuple(curr_position))
        
        maze_.fire_grows()

        # Agent's cell caught fire.  They dead. 
        if maze_.grid[curr_position[0], curr_position[1]] == 2: 
            return False, state_path, metrics 


        if curr_position == goal_state.path[0]: 
            print("Found path")
            metrics.path_length = len(curr_state.path)
            return True, state_path, metrics

        else: 
            # list of all neighbors ordered by preference 
            #   returns only valid, empty cell neighbors  
            ordered_list_of_neighbors = curr_position.neighbors_away_from_fire(maze_)           

            # add valid moves to fringe 
            #    - valid space on grid 
            #    - not in closed_set
            for move in ordered_list_of_neighbors:
                if (tuple(move) not in closed_set):

                    # Copy position path 
                    pos_path = list.copy(curr_state.path)
                    pos_path.append(move)
                    
                    # Create new state 
                    new_state = State(pos_path, maze_.copy())
                    
                    new_state_path = list.copy(state_path)
                    new_state_path.append(new_state)
                        
                    fringe.append([new_state, new_state_path])


    # print('No path')
    return False, state_path, metrics  








def animate(states, filename='mazerunner', path_shadow_color='green', 
            show_planned_path=False, path_color='blue'):
        """Outputs a .mp4 file at workng directory level.
            Input must be a list of class States
            Uses the (path, maze) states in input.  
        """

        if len(states) == 0:
            raise ValueError('Input list is empty')

        # Final state's path 
        final_path = states[-1].position_path

        fig = plt.figure()
        ims = []

        for i, state in enumerate(states):
            
            if i == 0:
                continue 

            p = state.position_path 
            maze_ = state.maze

            # Making a copy so that self.grid is not mutated 
            grid_copy = np.copy(maze_.grid)

            # Defining color map for grid.
            #    - [0] white = empty 
            #    - [1] black = occupied 
            #    - [2] red = fire 
            #    - [3] blueish = start 

            cmap = colors.ListedColormap(['white','black','red', path_shadow_color])
            
            
            # Setting the positions in the path to 3 
            for position in p: 
                grid_copy[position[0], position[1]] = 3

            
            if show_planned_path: 

                # Sigh.  Need to reverse the order of the coordinates because
                #   matplotlib.patches interprets it backwards (i.e. col, row)
                def reverse_coords_order(position):
                    return (position[1], position[0])

                # Builds mpl path object to add to the axis.  
                #  Connects all the positions in the path with a line.
                verts = [reverse_coords_order(position) for position in final_path]
                codes = [Path.MOVETO,]
                for j in range(len(final_path)-1):
                    codes.append(Path.LINETO)
                drawpath = Path(verts, codes)
                patch = patches.PathPatch(drawpath, facecolor='none', 
                                          lw=2, edgecolor='blue')

            
            # plots path 
            ax = plt.axes()
            if show_planned_path:
                ax.add_patch(patch)    
            # Shows maze.grid as a 2D image with colormap applied.  
            im = ax.imshow(grid_copy, interpolation='None',
                            cmap=cmap, aspect='equal', animated=True)

            ims.append([im])

            
            # # stop if fire reaches position 
            # curr = p[-1]
            # if maze_.grid[curr[0], curr[1]] == 2: 
            #     break
            


        ani = animation.ArtistAnimation(fig, ims, interval=200, blit=True,
                                        repeat_delay=1000)
        writer = FFMpegWriter(fps=10, metadata=dict(artist='Me'), bitrate=1500)
        ani.save(filename + ".mp4", writer=writer)



