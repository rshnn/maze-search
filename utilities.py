import numpy as np 

from position import Position
from maze import Maze 
import search as Search


def valid_fire_maze(maze, algo='Astar'):
    """True if there is a valid path from fire source to South-west corner 
       Algo options are 'dfs' and 'Astar'
       Return: boolean_for_validity, path, metrics 
    """

    initial_state = Position([0, maze.dim - 1])
    goal_state = Position([maze.dim - 1, 0])

    if algo == 'Astar':
        fringe, closed_set = Search.Astar_setup_fringe_and_closedset(initial_state)
        valid, path, metrics = Search.A_star_man(maze, initial_state, goal_state)

    elif algo == 'dfs':
        fringe, closed_set = Search.setup_fringe_and_closedset(initial_state)
        valid, path, metrics = Search.dfs_southwest(maze, initial_state, goal_state)

    return valid, path, metrics  




def valid_maze(maze, algo='Astar'):
    """True if there is a valid path from northwest to South-west corner 
       Algo options are 'dfs' and 'Astar'
       Return: boolean_for_validity, path, metrics 
    """

    initial_state = Position([0, 0])
    goal_state = Position([maze.dim - 1, maze.dim - 1])

    if algo == 'Astar':
        fringe, closed_set = Search.Astar_setup_fringe_and_closedset(initial_state)
        valid, path, metrics = Search.A_star_man(maze, initial_state, goal_state)

    elif algo == 'dfs':
        fringe, closed_set = Search.setup_fringe_and_closedset(initial_state)
        valid, path, metrics = Search.dfs_southeast(maze, initial_state, goal_state)

    return valid, path, metrics  






def generate_valid_fire_mazes(dim, q_list=np.linspace(0, 1, 11), p=0.3, 
                              number_of_valid_mazes_per_config=5):

    maze_dict = {}

    for q in q_list: 

        # Get 5 valid mazes of each configuration 
        maze_list_of_five = []

        while(len(maze_list_of_five) < number_of_valid_mazes_per_config):
            maze = Maze(dim, p, q)
            valid_path_exists, _, _ = valid_maze(maze)
            valid_fire_path_exists, _, _ = valid_fire_maze(maze)
            if valid_path_exists and valid_fire_path_exists: 
                maze_list_of_five.append(maze.copy())

        maze_dict[q] = list.copy(maze_list_of_five)

    return maze_dict
