import numpy as np 
import seaborn as sns

from position import Position
from maze import Maze 
import search as Search
import search_fire as SearchFire
import utilities as Util 





def evaluate_trivial_Aman(maze_dict, dim, log=True):
    """Evaluates the trivial A* strategy using manhattan distance 
            over the input maze_dict. 
        Returns a summary of the performance of this algorithm as a dict. 
    """

    # default init and goal states for A* 
    initial_state = Position([0, 0])
    goal_state = Position([dim-1, dim-1])

    # init structure to return 
    triv_Aman_stats = {}

    i=1
    total = len(maze_dict)

    # For each maze in the list of mazes: run the search
    #  Store the values within the output metrics 
    #  Take the average of those values and add them to the dictionary for the 
    #   associated key.  

    for key, val in maze_dict.items():

        if log: print("Working q: {0:.4}\t\t {1} of {2}".format(key, i, total))
        success = [] 
        path_length = []
        max_fringe_size = []
        nodes_expanded = []
        
        for maze in val: 
            _, path, metrics = Search.A_star_man(maze, initial_state, goal_state)
            survived, _ = SearchFire.simulate_path_and_fire(maze, path)
            success.append(survived)
            path_length.append(metrics.path_length)
            max_fringe_size.append(metrics.max_fringe_size)
            nodes_expanded.append(metrics.total_nodes_expanded)
            
        success_ratio = sum(success)/len(success)
        
        triv_Aman_stats[key] = (success_ratio, np.mean(path_length), 
                                np.mean(nodes_expanded), np.mean(max_fringe_size))
        
        i+=1    


    return triv_Aman_stats




def evaluate_trivial_Aeuc(maze_dict, dim, log=True):
    """Evaluates the trivial A* using Euclidean distance metric.  
        Returns a summary of the performance of this algorithm as a dict.  
    """

    initial_state = Position([0, 0])
    goal_state = Position([dim-1, dim-1])

    triv_Aeuc_stats = {}

    # vars for printing logs 
    i=1
    total = len(maze_dict)

    # For each maze in the list of mazes: run the search
    #  Store the values within the output metrics 
    #  Take the average of those values and add them to the dictionary for the 
    #   associated key.  
    for key, val in maze_dict.items():

        if log: print("Working q: {0:.4}\t\t {1} of {2}".format(key, i, total))
        success = [] 
        path_length = []
        max_fringe_size = []
        nodes_expanded = []


        for maze in val: 
            _, path, metrics = Search.A_star_euc(maze, initial_state, goal_state)
            survived, _ = SearchFire.simulate_path_and_fire(maze, path)
            success.append(survived)
            path_length.append(metrics.path_length)
            max_fringe_size.append(metrics.max_fringe_size)
            nodes_expanded.append(metrics.total_nodes_expanded)
            
        success_ratio = sum(success)/len(success)
        
        triv_Aeuc_stats[key] = (success_ratio, np.mean(path_length), 
                                np.mean(nodes_expanded), np.mean(max_fringe_size))
            
        i+=1

    return triv_Aeuc_stats






def evaluate_fire_Astar(maze_dict, dim, 
                        heur_fn=SearchFire.fire_heuristic_4, heur_const=0.1, log=True):
    """Evaluates the modified A* using a fire heuristic.
        Returns a summary of results in a dictionary.  
    """

    Afire_stats = {}

    # variables for printing logs 
    i=1
    total = len(maze_dict)

    # For each maze in the list of mazes: run the search
    #  Store the values within the output metrics 
    #  Take the average of those values and add them to the dictionary for the 
    #   associated key.  
    for key, val in maze_dict.items():
        
        if log: print("Working q: {0:.4}\t\t {1} of {2}".format(key, i, total))
        
        success = []
        path_length = []
        max_fringe_size = []
        nodes_expanded = []


        for maze in val: 
            initial_state, goal_state = SearchFire.state_setup(maze)
            survived, _, metrics = SearchFire.A_star_fire(maze, initial_state, goal_state, 
                                                    heur_fn, heur_const)
            success.append(survived)
            path_length.append(metrics.path_length)
            max_fringe_size.append(metrics.max_fringe_size)
            nodes_expanded.append(metrics.total_nodes_expanded)

        success_ratio = sum(success)/len(success)
        
        Afire_stats[key] = (success_ratio, np.mean(path_length), 
                                np.mean(nodes_expanded), np.mean(max_fringe_size))

        i += 1


    return Afire_stats





def build_mazes_for_fire_const(dim, p, q, heur_const_list=np.linspace(0,1,11), 
                              heur_fn=SearchFire.fire_heuristic_4, 
                              number_of_valid_mazes_per_config=5, 
                              log=True):
    """Builds a dict of mazes for the purpose of evaluating various values of 
        the heuristic function,f 
    """


    # build mazes (key = f.  value = list of mazes) 
    maze_dict = {}

    if log: print("Generating {} mazes.".format(number_of_valid_mazes_per_config*len(heur_const_list)))

    for f in heur_const_list:
        
        maze_list_of_n = []

        while(len(maze_list_of_n) <  number_of_valid_mazes_per_config):
            maze = Maze(dim, p, q)
            valid_path_exists, _, _ = Util.valid_maze(maze)
            valid_fire_path_exists, _, _ = Util.valid_fire_maze(maze)
            if valid_path_exists and valid_fire_path_exists: 
                maze_list_of_n.append(maze.copy())

        maze_dict[f] = list.copy(maze_list_of_n)

    if log: print("Completed maze generation.")

    return maze_dict 




def evaluate_fire_Astar_const(maze_dict, dim, 
                              heur_fn=SearchFire.fire_heuristic_4, log=True):
    """Evalustes the modified A* algorithm for the purpose of evaluating 
        different values of the heuristic constant, f
    """

    Afire_stats = {}

    i=1
    total = len(maze_dict)

    for key, val in maze_dict.items():
        
        if log: print("Working f: {0:.4}\t\t {1} of {2}".format(key, i, total))
        
        success = []
        path_length = []
        max_fringe_size = []
        nodes_expanded = []


        for maze in val: 
            initial_state, goal_state = SearchFire.state_setup(maze)
            survived, _, metrics = SearchFire.A_star_fire(maze, initial_state, goal_state, 
                                                    heur_fn, key)
            success.append(survived)
            path_length.append(metrics.path_length)
            max_fringe_size.append(metrics.max_fringe_size)
            nodes_expanded.append(metrics.total_nodes_expanded)

        success_ratio = sum(success)/len(success)
        
        Afire_stats[key] = (success_ratio, np.mean(path_length), 
                                np.mean(nodes_expanded), np.mean(max_fringe_size))

        i += 1


    return Afire_stats






def barplot_from_stats(stats, var='success', title_prepend=None, axis=None, xlabel='q'):
    """Helper function to print a seaborn bar plot from the output summary dicts
    """

    if var == 'success': idx=0
    elif var == 'path_length': idx=1
    elif var == 'nodes_expanded': idx = 2
    elif var == 'fringe_size': idx = 3
    else: idx=0

    colors = "blue red green purple".split()
    c = colors[idx]

    q = []
    y = []

    for key, val in stats.items():

        q.append(np.round(key, 3))
        y.append(val[idx])



    f = sns.barplot(q, y, color=c)
    f.set_title('{}: {} vs {}'.format(title_prepend, xlabel, var))
    f.set_xlabel(xlabel)
    if var == "success":
        ylabel = "percent success"
    else: 
        ylabel = "avg " + var
    f.set_ylabel(ylabel);

    return f
