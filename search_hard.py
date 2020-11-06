# -*- coding: utf-8 -*-

from maze import Maze 
from position import Position
import utilities as Util 

from collections import deque
import search as Search 



class HardState(): 

    def __init__(self, maze, metrics=None):
        self.maze = maze 
        if metrics is None: 
            metrics = Search.Metrics()
        self.metrics = metrics 

    def copy(self):

        maze_ = self.maze.copy()
        metrics_ = self.metrics.copy() 
        return HardState(maze_, metrics_)



def trivial(N, dim, p, search_algo=Search.A_star_man): 
    """ The trivial strategy to developing hard mazes. 
        Generates N number of valid mazes.  Solves each one to maximize metrics. 
        Returns the value of the largest metric and the associated maze.  

        Return: 
        (
            (longest_path, maze_metrics_dict[longest_path_idx]), 
            (largest_nodes_expanded, maze_metrics_dict[largest_nodes_expanded_idx]), 
            (largest_fringe_size, maze_metrics_dict[largest_fringe_size_idx]), 
            maze_dict
        ) 
    """


    # Generate N valid mazes 
    mazes = []
    for i in range(int(N)):
        found_valid = False 
        while(not found_valid):
            m = Maze(dim, p, q=0)
            valid, _, _ = Util.valid_maze(m)
            if valid: 
                mazes.append(m)
                found_valid = True 

    # Setup for A* 
    initial_state = Position([0, 0])
    goal_state = Position([dim-1, dim-1])

    # The output structure 
    maze_metrics_dict = {}

    # These are the metrics to be maximized 
    longest_path = 0
    largest_nodes_expanded = 0
    largest_fringe_size = 0

    # The key for the associated maximized metrics 
    longest_path_idx = 0
    largest_nodes_expanded_idx = 0
    largest_fringe_size_idx = 0

    # For each maze in N, run the search algorithm.  
    #   Compare the solution's metrics with the current best. Replace if applicable
    for idx, maze in enumerate(mazes): 
        _, _, metrics = search_algo(maze, initial_state, goal_state)
        maze_metrics_dict[idx] = (maze, metrics)
        
        if metrics.path_length >= longest_path:
            longest_path = metrics.path_length
            longest_path_idx = idx 
            
        if metrics.total_nodes_expanded >= largest_nodes_expanded:
            largest_nodes_expanded = metrics.total_nodes_expanded
            largest_nodes_expanded_idx = idx 
            
        if metrics.max_fringe_size >= largest_fringe_size: 
            largest_fringe_size = metrics.max_fringe_size
            largest_fringe_size_idx = idx 


    return (maze_metrics_dict[longest_path_idx], 
            maze_metrics_dict[largest_nodes_expanded_idx], 
            maze_metrics_dict[largest_fringe_size_idx], 
            maze_metrics_dict)  


def max_fringe_size(metric):
    return metric.max_fringe_size


def path_length(metric):
    return metric.path_length


def total_nodes_expanded(metric):
    return metric.total_nodes_expanded


def sum_of_metrics(metric):
    return metric.path_length + metric.total_nodes_expanded + metric.max_fringe_size 



def generate_neighbor_states(hard_state): 
    """Generates neighbors of the input maze
        Returns all the neighboring mazes in a list 
    """
    neighbors = []

    maze = hard_state.maze     
    dim = maze.dim 

    for i in range(dim): 
        for j in range(dim): 
            new_state = hard_state.copy()
            if new_state.maze.grid[i, j] == 1: 
                continue 
            new_state.maze.grid[i, j] = 1

            neighbors.append(new_state)

    return neighbors






def hill_climb(seed_state, iterations=20, search_algo=Search.A_star_man, 
                metric_function=max_fringe_size, reset_limit=20, log=False): 
    """An implementation of the hill climbing algorithm.
        The search will generate neighbor states of a seed state.  Over each neighbor state, 
        the input search_algo is conducted over it.  A running account of the state
        that maximizes the metric_function is kept. Each iteration, a new set of 
        neighbor states is generated.  
        The search halts after a certain number of iterations has been met or a 
        local maximia has been met. 
    """
    
    # Gather parameters from the seed state 
    dim = seed_state.maze.dim 
    p = seed_state.maze.p 
    
    # variable that returns the metric values 
    metric_list = []

    # initial and goal states for the search algorithm 
    initial_state = Position([0, 0])
    goal_state = Position([dim-1, dim-1])

    # Generates all neighboring states 
    neighbors = generate_neighbor_states(seed_state)
    current_best = seed_state.copy() 

    # This is the list of current_best states on which a local maximia was reached
    #  If the reset_limit condition is met, we will return the hardest maze 
    #  in this list.  
    local_maximas = [] 

    # Loops for number of iterations.  Iter must be an int value.  
    i = 0
    # Set the reset counter.  
    reset_count = 0 
    while(i < iterations):
        

        metric_list.append(metric_function(current_best.metrics))
        if log: print("iter {}..".format(i), end='')
        found_new_best = False
        neighbors = generate_neighbor_states(current_best)

        # For each state in the neighbors of current_best, 
        #  Check if the neighbor is the new currest_best 
        for state in neighbors: 
            valid, _, metrics = search_algo(state.maze, initial_state, goal_state)
            state.metrics = metrics 

            if valid: 
                metric = metric_function(metrics)
                if metric > metric_function(current_best.metrics):  
                    # Update current best 
                    # if log: print("\tFound new best: {}".format(metric))
                    current_best = state.copy() 
                    found_new_best = True 

        # If none of the neighbors can top the current best
        #   We are at a local maximia. 
        if not found_new_best: 
            if log: print("**At a local maxima...metric val:{}".format(metric_function(current_best.metrics)))
            
            # At this maze to the local maximia list.  
            local_maximas.append(current_best)


            # If we've reset beyond the limit, return the best of local maximia 
            if reset_count == reset_limit: 
                if log: print("**Max reset limit reached.  Returning best of local maximia.")

                # Finding best maximia 
                best = current_best 
                for maximia in local_maximas: 
                    score = metric_function(maximia.metrics) 
                    if score > metric_function(best.metrics): 
                        best = maximia 

                return best, metric_list         

            # Preparing to reset the serach at a new maze
            reset_count += 1
            valid = False 

            # Loop to ensure the new maze is solveable before restarting 
            while(not valid): 
                current_best = HardState(Maze(dim, p, 0))
                valid, _, metrics = search_algo(current_best.maze, initial_state, goal_state)
                current_best.metrics = metrics 


            # Reset iter index.  Restart the search.  
            i = 0
            if log: print("**Resetting...{} of {}.\n".format(reset_count, reset_limit))
            continue 

        i+=1 

    # Total iteration cap has been met.  Return current_best 
    if log: print("Max iterations met.")
    return current_best, metric_list 
