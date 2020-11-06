# -*- coding: utf-8 -*-


from maze import Maze 
from position import Position 
from collections import deque
from queue import PriorityQueue
import math
import collections

#Create the metrics class, which will enable us to compare performance of search algorithms
class Metrics():
    path_length = 0
    total_nodes_expanded = 0
    max_fringe_size = 0
    
    def __str__(self):
        return """path length:\t\t{}\ntotal nodes expanded:\t{}\nmax fringe size:\t{}"""\
                            .format(self.path_length, 
                                    self.total_nodes_expanded, 
                                    self.max_fringe_size)
    def __repr__(self):
        return str(self)


    def copy(self): 
        m = Metrics()
        m.path_length = self.path_length    
        m.total_nodes_expanded = self.total_nodes_expanded  
        m.max_fringe_size  = self.max_fringe_size
        return m 




def quick_setup(dim, p=0.2, q=0):
    """Returns a maze, initial_state, goal_state
    """

    maze = Maze(dim, p, q)

    initial_state = Position([0, 0])
    goal_state = Position([dim-1, dim-1])

    return maze, initial_state, goal_state 



def bid_BFS_setup_fringe_and_closedset(initial_state, goal_state):
    
    
    start_fringe=deque()
    
    # First item in fringe is initial_state and empty path
    start_fringe.append((initial_state, [initial_state]))
    
    goal_fringe=deque()
    # First item in fringe is goal_state and empty path
    goal_fringe.append((goal_state,[goal_state]))
    
    # Set of states already visited.  Set objects do not hold duplicates
    #   The set needs to hold hashable items (immutable).
    #   Thus, tuples representing coordinates will be put into the set.
    #      e.g.    closed_set.add(tuple(position)) 
    closed_set = set()
    
    return start_fringe, goal_fringe, closed_set


    
def Astar_setup_fringe_and_closedset(initial_state):
    fringe=PriorityQueue()
    """Returns a fringe with initial_state and an emtpy closed_set
    """

    # Set of states yet to visit along with path leading up to the state
    #   Implemented using Priority queue.  
    # First item is the weight, second item is a tiebreakter, and third is the
    # path taken
    # First item in fringe is initial_state and empty path
    # fringe.put((0,0,[initial_state]))
    fringe.put((0, initial_state, [initial_state]))


    # Set of states already visited.  Set objects do not hold duplicates
    #   The set needs to hold hashable items (immutable).
    #   Thus, tuples representing coordinates will be put into the set.
    #      e.g.    closed_set.add(tuple(position)) 
    closed_set = set()

    return fringe, closed_set



def setup_fringe_and_closedset(initial_state):
    """Returns a fringe with initial_state and an emtpy closed_set
    """

    # Set of states yet to visit along with path leading up to the state
    #   Implemented using deque.  
    #      deque can be a stack if you use .pop()
    #      deque can be a queue if you use .popleft() 
    #      you can also .append() and .appendleft()...will be useful later. 
    fringe = deque()
    # First item in fringe is initial_state and empty path
    fringe.append((initial_state, [initial_state]))

    # Set of states already visited.  Set objects do not hold duplicates
    #   The set needs to hold hashable items (immutable).
    #   Thus, tuples representing coordinates will be put into the set.
    #      e.g.    closed_set.add(tuple(position)) 
    closed_set = set()

    return fringe, closed_set



def dfs(maze, initial_state, goal_state):
    """Runs a trivial DFS.
        Returns True, path, and metrics on success  
        Returns False, None, and metrics on failure   
    """ 

    # Initialize fringe and closed_set.
    #   Fringe is a collection.deque and closed_set is a set()
    fringe, closed_set = setup_fringe_and_closedset(initial_state)

    # Object that gathers metrics about the search algorithm on this maze
    metrics = Metrics()

    while(len(fringe) > 0):

        # Updates max fringe size for metrics 
        if len(fringe) > metrics.max_fringe_size:
            metrics.max_fringe_size = len(fringe)

       

        #get the node and path taken to get there from top of stack
        s, path = fringe.pop()
        
        #if the node is the goal node, return the path
        if s == goal_state: 
            # print("Found path")
            metrics.path_length = len(path)
            return True, path, metrics
        
        #if the node has not already been seen
        if tuple(s) not in closed_set:
            #increase the total number of nodes expanded
            metrics.total_nodes_expanded += 1
            #add this node to the seen nodes 
            closed_set.add(tuple(s))
        
            # list of all cardinal moves 
            i = s[0]
            j = s[1]

            all_cardinal_moves =[[i+1, j], 
                              [i, j-1], 
                              [i, j+1],
                              [i-1, j]]

            # add valid moves to fringe 
            #    - valid space on grid 
            #    - not in closed_set
            
            
            for move in all_cardinal_moves: #for all cardinal moves
                if tuple(move) not in closed_set: #if the move has not been seen
                    if maze.is_valid_cell(move) and maze.is_empty_cell(move): #if the move is valid (ie inside the maze) and not blocked

                    # Passing the valid state and 
                    #   a copy of the path into the fringe
                        p = list.copy(path) 
                        p.append(move)
                        #append the node and the path to get there to the top of the stack
                        fringe.append((move, p))
                    
                    
    # print('No path')
    return False, None, metrics  



def dfs_southeast(maze, initial_state, goal_state):
    """Runs a trivial DFS.  Prioritizes south and east direcitons 
        Returns True, path, and metrics on success  
        Returns False, None, and metrics on failure   
    """ 

    # Initialize fringe and closed_set.
    #   Fringe is a collection.deque and closed_set is a set()
    fringe, closed_set = setup_fringe_and_closedset(initial_state)

    # Object that gathers metrics about the search algorithm on this maze
    metrics = Metrics()

    while(len(fringe) > 0):

        # Updates max fringe size for metrics 
        if len(fringe) > metrics.max_fringe_size:
            metrics.max_fringe_size = len(fringe)

      

        s, path = fringe.pop()
     

        if s == goal_state: 
            # print("Found path")
            metrics.path_length = len(path)
            return True, path, metrics

        if tuple(s) not in closed_set:
            metrics.total_nodes_expanded += 1
            closed_set.add(tuple(s))
            # list of all cardinal moves 
            all_cardinal_moves = [s.move_north(maze), 
                                  s.move_west(maze), 
                                  s.move_east(maze), 
                                  s.move_south(maze)]  
            # add valid moves to fringe 
            #    - valid space on grid 
            #    - not in closed_set
            for move in all_cardinal_moves:
                if move is not None and \
                   maze.is_empty_cell(move) and \
                   (tuple(move) not in closed_set):

                    # Passing the valid state and 
                    #   a copy of the path into the fringe
                    p = list.copy(path) 
                    p.append(move)
                    fringe.append((move, p))


    # print('No path')
    return False, None, metrics  



def dfs_southwest(maze, initial_state, goal_state):
    """Runs a trivial DFS.  Prioritizes south and west directions.
        Used for Util.valid_fire_maze().   
        Returns True, path, and metrics on success  
        Returns False, None, and metrics on failure   
    """ 

    # Initialize fringe and closed_set.
    #   Fringe is a collection.deque and closed_set is a set()
    fringe, closed_set = setup_fringe_and_closedset(initial_state)

    # Object that gathers metrics about the search algorithm on this maze
    metrics = Metrics()

    while(len(fringe) > 0):

        # Updates max fringe size for metrics 
        if len(fringe) > metrics.max_fringe_size:
            metrics.max_fringe_size = len(fringe)

        

        s, path = fringe.pop()
      

        if s == goal_state: 
            # print("Found path")
            metrics.path_length = len(path)
            return True, path, metrics

        if tuple(s) not in closed_set:
            metrics.total_nodes_expanded += 1
            closed_set.add(tuple(s))
            
            # list of all cardinal moves 
            all_cardinal_moves = [s.move_north(maze), 
                                  s.move_east(maze), 
                                  s.move_west(maze), 
                                  s.move_south(maze)]  
            # add valid moves to fringe 
            #    - valid space on grid 
            #    - not in closed_set
            for move in all_cardinal_moves:
                if move is not None and \
                   maze.is_empty_cell(move) and \
                   (tuple(move) not in closed_set):

                    # Passing the valid state and 
                    #   a copy of the path into the fringe
                    p = list.copy(path) 
                    p.append(move)
                    fringe.append((move, p))


    # print('No path')
    return False, None, metrics  



    
def bfs(maze, initial_state, goal_state):
    """Runs a trivial BFS.
        Returns True, path, and metrics on success  
        Returns False, None, and metrics on failure   
    """

    # Initialize fringe and closed_set.
    #   Fringe is a collection.deque and closed_set is a set()
    fringe, closed_set = setup_fringe_and_closedset(initial_state)

    # Object that gathers metrics about the search algorithm on this maze
    metrics = Metrics()


    while(len(fringe) > 0):

        # Updates max fringe size for metrics 
        if len(fringe) > metrics.max_fringe_size:
            metrics.max_fringe_size = len(fringe)

       

        #get the node and the path taken to get there from the beginnig of the queue
        s, path = fringe.pop()
        
        if s == goal_state: 
            # print("Found path")
            metrics.path_length = len(path)
            return True, path, metrics


        
        if tuple(s) not in closed_set:
            metrics.total_nodes_expanded += 1
            closed_set.add(tuple(s))
    

    
#            else: 
                # list of all cardinal moves 
            i = s[0]
            j = s[1]

            all_cardinal_moves =[[i-1, j], 
                              [i, j-1], 
                              [i, j+1],
                              [i+1, j]]


            # add valid moves to fringe 
            #    - valid space on grid 
            #    - not in closed_set
            for move in all_cardinal_moves: 
                if tuple(move) not in closed_set: 
                    if maze.is_valid_cell(move) and maze.is_empty_cell(move):
                        p = list.copy(path) 
                        p.append(move)
                        #append the node and the path taken to get there to the end of queue
                        fringe.appendleft((move, p))
                        

    # print('No path')
    return False, None, metrics


def manhattan_distance(start_state, goal_state):
    return abs(start_state[0]-goal_state[0])+abs(start_state[1]-goal_state[1])
    

def euclidean_distance(start_state, goal_state):
    return math.sqrt((start_state[0]-goal_state[0])**2+(start_state[1]-goal_state[1])**2)



def A_star_man(maze, initial_state, goal_state):
    """A star search.
        heuristic_function computes the approximate distance from a given cell 
            to the goal cell.  The function must take the form:
            func(position_1, goal_position)
    """
    
    # Setup fringe and closed_set for A_star.  
    #    fringe is a queue.PriorityQueue and closed_set is a set() 
    fringe, closed_set = Astar_setup_fringe_and_closedset(initial_state)
        
    # Object that gathers metrics about the search algorithm on this maze
    metrics = Metrics()


    # dictionary for running cost of Positions
    cost_so_far = {}
    cost_so_far[tuple(initial_state)] = 0
    

    while not fringe.empty():
        
        # Updates max fringe size for metrics 
        if fringe.qsize() > metrics.max_fringe_size:
            metrics.max_fringe_size = fringe.qsize()

      
        

        # Get next item from the priority queue fringe and unpack it 
        _, current_state, state_path = fringe.get()
       
        
        if current_state == goal_state:
            metrics.path_length = len(state_path)
            return True, state_path, metrics 
        
        if tuple(current_state) not in closed_set:
            metrics.total_nodes_expanded += 1
            closed_set.add(tuple(current_state))
            
            # cardinal_moves=current_state.list_of_moves(maze)
            i = current_state[0]
            j = current_state[1]

            cardinal_moves = [[i, j+1], [i, j-1], 
                             [i+1, j], [i-1, j]]

            
            for move in cardinal_moves: 
                if tuple(move) not in closed_set: 
                    if maze.is_valid_cell(move) and maze.is_empty_cell(move):
                        p = list.copy(state_path)
                        p.append(move)
                        #calculate the number of steps taken to reach the new node
                        gNode = len(p)
                        
                        # if the new node has never been seen with a lower cost
                        if tuple(move) not in cost_so_far or gNode < cost_so_far[tuple(move)]:
                            cost_so_far[tuple(move)] = gNode 
                            #calculate the heuristic
                            hNode = manhattan_distance(move, goal_state)
                            fNode = gNode + hNode 
                            #add the node and the path taken to get there and fNode to the priority queue
                            fringe.put((fNode, move, p))

    return False, None, metrics 



def A_star_euc(maze, initial_state, goal_state, question=None):
    """A star search.
        heuristic_function computes the approximate distance from a given cell 
            to the goal cell.  The function must take the form:
            func(position_1, goal_position)
    """
    
    # Setup fringe and closed_set for A_star.  
    #    fringe is a queue.PriorityQueue and closed_set is a set() 
    fringe, closed_set = Astar_setup_fringe_and_closedset(initial_state)
        
    # Object that gathers metrics about the search algorithm on this maze
    metrics = Metrics()


    # dictionary for running cost of Positions
    cost_so_far = {}
    cost_so_far[tuple(initial_state)] = 0
    

    while not fringe.empty():
        
        # Updates max fringe size for metrics 
        if fringe.qsize() > metrics.max_fringe_size:
            metrics.max_fringe_size = fringe.qsize()

       


        _, current_state, state_path = fringe.get()
       
        
        if current_state == goal_state:
            metrics.path_length = len(state_path)
            if question =='2h':
                return True, state_path, metrics, closed_set
            else:
                return True, state_path, metrics
        
        if tuple(current_state) not in closed_set:
            metrics.total_nodes_expanded += 1
            closed_set.add(tuple(current_state))
        
            
            # cardinal_moves=current_state.list_of_moves(maze)
            i = current_state[0]
            j = current_state[1]

            cardinal_moves = [[i, j+1], [i, j-1], 
                             [i+1, j], [i-1, j]]

            
            for move in cardinal_moves: 
                if tuple(move) not in closed_set: 
                    if maze.is_valid_cell(move) and maze.is_empty_cell(move):
                        p = list.copy(state_path)
                        p.append(move)
                        gNode = len(p)
                        
                        if tuple(move) not in cost_so_far or gNode < cost_so_far[tuple(move)]:
                            cost_so_far[tuple(move)] = gNode 
                            hNode = euclidean_distance(move, goal_state)
                            fNode = gNode + hNode 

                            fringe.put((fNode, move, p))
    
    if question== '2h':
        return False, None, metrics, closed_set
    else:
        return False, None, metrics




def already_reached(cell, queue):
    
    #for all paths already taken from the input state
    for recent_path in list(queue):
        #check if the cell at the end of the path is the same as the one 
        #reached from the opposite state
        if recent_path[0]== cell:
            return True,recent_path[1]
    
    
    return False, None
            

def bid_bfs(maze, initial_state, goal_state, question=None):
    
    #create a goal fringe. This could be done with a rewriting of 
    #quick_setup for bidirectional bfs as well 
    goal_fringe=deque()
    goal_fringe.append((goal_state, [goal_state]))
    

    # Object that gathers metrics about the search algorithm on this maze
    metrics = Metrics()

    # Intialize both fringes and closed_set
    #  Both fringes are collection.deque.  Closed set is a set()
    start_fringe, goal_fringe, closed_set = bid_BFS_setup_fringe_and_closedset(initial_state, goal_state)
    
    while (len(start_fringe) > 0) and (len(goal_fringe)>0):

        # Updates max fringe size for metrics 
        if len(start_fringe) + len(goal_fringe) > metrics.max_fringe_size:
            metrics.max_fringe_size = len(start_fringe) + len(goal_fringe)


        if len(start_fringe)>0:
            # Increment total nodes expanded for metrics 
#            metrics.total_nodes_expanded += 1

            #get the cell and the path taken to get there from start state via the starting state fringe queue
            start_s, start_path=start_fringe.pop()
            
                         
            if start_s == goal_state:
                metrics.path_length = len(start_path)
                if question =='2h':
                    return True, start_path, metrics, closed_set
                else:
                    return True, start_path, metrics
            
            if tuple(start_s) not in closed_set:
                metrics.total_nodes_expanded += 1
                closed_set.add(tuple(start_s))
                
                all_cardinal_moves=start_s.list_of_moves(maze)
                for move in all_cardinal_moves:
                    if move is not None: 
                        if maze.is_empty_cell(move) and (tuple(move) not in closed_set):
                            
                            #check if the move has been reached from the goal state already
                            reached, opposite_path=already_reached(move, goal_fringe)
                            
                            #if the move has been reached, return the combined path
                            if reached!=False:
                                p=list.copy(start_path)
                                
                                for i in range(0, len(opposite_path)):
                                    p.append(opposite_path.pop())
                                
                                metrics.path_length = len(p)
                                if question=='2h':
                                    return True, p, metrics, closed_set
                                else:
                                    return True, p, metrics
                            #if the move hasn't been reached, add the path the fringe queue
                            else:
                                
                                p=list.copy(start_path)
                                p.append(move)
                                start_fringe.appendleft((move,p))
                            
                        
        if len(goal_fringe)>0:

            # Increment total nodes expanded for metrics 
#            metrics.total_nodes_expanded += 1

            #get the cell and the path taken to get there from goal state and 
            #run the same algorithm except with the cell at the end of the path
            #from the goal fringe
            goal_s, goal_path=goal_fringe.pop()
            
             
            if goal_s == initial_state:
                metrics.path_length = len(goal_path) 
                return True, goal_path, metrics 
            
            if tuple(goal_s) not in closed_set:
                metrics.total_nodes_expanded += 1
                closed_set.add(tuple(goal_s))
             
                
                all_cardinal_moves=goal_s.list_of_moves(maze)
                for move in all_cardinal_moves:
                    if move is not None: 
                        if maze.is_empty_cell(move) and (tuple(move) not in closed_set):
                            reached, opposite_path=already_reached(move, start_fringe)
                            if reached!=False:
                                p=list.copy(goal_path)
                                for i in range(0, len(opposite_path)):
                                    p.append(opposite_path.pop())
                                metrics.path_length = len(p)
                                if question=='2h':
                                    return True, p, metrics, closed_set
                                else:
                                    return True, p, metrics               
                            
                            p=list.copy(goal_path)
                            p.append(move)
                            goal_fringe.appendleft((move,p))
                        
             

    if question == '2h':
        return False, None, metrics, closed_set
    else:
        return False, None, metrics
