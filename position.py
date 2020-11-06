# -*- coding: utf-8 -*-
import numpy as np

class Position(np.ndarray):
    
    def __new__(cls, *args):
        array = np.array(args, dtype=np.int, copy=True)
        return np.ndarray.__new__(cls, shape=(2,), buffer=array, dtype=np.int)
    
    # The following functions return a new Position object after enacting  
    # the cardinal move action. 
    # Consider (0,0) to be the most northwest point.
     
    def move_north(self, maze):
        i = self[0]
        j = self[1]
        p =  Position([i-1, j])

        if maze.is_valid_cell(p):
            return p
        else: 
            return None 


    def move_south(self, maze):
        i = self[0]
        j = self[1]
        p =  Position([i+1, j])

        if maze.is_valid_cell(p):
            return p
        else: 
            return None 



    def move_east(self, maze):
        i = self[0]
        j = self[1]
        p = Position([i, j+1])

        if maze.is_valid_cell(p):
            return p
        else: 
            return None 


    def move_west(self, maze):
        i = self[0]
        j = self[1]
        p =  Position([i, j-1])

        if maze.is_valid_cell(p):
            return p
        else: 
            return None 



    def list_of_moves(self, maze):
        """Returns a list of States where agent moves 1 unit cardinally
            Consider (0,0) to be the most northwest point.
            The returned list is in this order:
                North, South, East, West  
        """
        i = self[0]
        j = self[1]
        
        return [self.move_north(maze), self.move_south(maze),
                self.move_east(maze), self.move_west(maze)]
    


    def get_valid_neighbors(self, maze):
        """Return all valid neighbors of this position.  
                A valid neighbor is a position on the maze that is 
                within maze bounds and free.  
        """
        valid_neighbors = []

        cardinal_moves = self.list_of_moves(maze)
        for move in cardinal_moves:
            if move is not None and \
               maze.is_empty_cell(move): 
                valid_neighbors.append(move)
        return valid_neighbors 


    def neighbors_away_from_fire(self, maze):
        """Returns all valid neighbors of this position in reverse order 
            depending on how many burning cells surround those neigbors. 
            [(most burning cells surrounding), ..., (least burning cells)]
        """

        neighbors = []

        cardinal_moves = self.list_of_moves(maze)
        value_list = []

        for move in cardinal_moves: 
            if move is not None: 
                if maze.is_empty_cell(move):
                    burned_surrounding = 0
                    move_neighbors = move.list_of_moves(maze)
                    for neigh in move_neighbors: 
                        if neigh is not None:
                            if maze.is_empty_cell(neigh): 
                                if maze.grid[neigh[0], neigh[1]] == 2: 
                                    burned_surrounding += 1
                    
                    value_list.append((move, burned_surrounding))

        value_list.sort(key=lambda x:x[1], reverse=True)

        return [val[0] for val in value_list]



    
    def __str__(self):
        return "({}, {})".format(str(self[0]), str(self[1]))
    
    def __repr__(self): 
        return "({}, {})".format(str(self[0]), str(self[1]))
    
    
    def __eq__(self, other):
        """Equality comparitor.  
            TODO is this needed? Already inheriting from ndarray. 
        """
        if (self[0] == other[0]) and \
            (self[1] == other[1]):
            return True
        else: 
            return False
    