# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
from matplotlib.path import Path
import matplotlib.patches as patches
import matplotlib.animation as animation
from matplotlib.animation import FFMpegWriter

from position import Position 


class Maze():
    """ A Maze object stores the state of the grid and its metadata.
            - grid: the (dim x dim) grid of cells as 2D numpy array 
                Values of the cells are either: 
                    0: empty 
                    1: occupied 
                    2: on fire 
            - dim: the dimension of the grid
            - p: the probability of a cell being occupied (0 < p < 1)
            - q: the flammability rate (0 <= q <= 1)
            - cells_on_fire:  a list of cell coordinates that are on fire 
    """
    
    
    def __init__(self, dim, p, q=0):
        
        if (dim <= 0) or (type(dim) is not int): 
            raise ValueError('Expeted positive integer value for dim.')
        
        if not (0 < p < 1): 
            raise ValueError('Expected value for p to be between 0 and 1 exclusive.')
        
        self.dim = dim
        self.p = p
        self.q = q
        
        self.grid = np.zeros([dim, dim])  #create empty matrix with input dim
        self.cells_on_fire = []     


        #iterate through all cells in matrix
        for i in range(dim):
            for j in range(dim):

                # Cell is occupied (1) if rand val < p
                if np.random.uniform(0, 1) < self.p: 
                    self.grid[i, j] = 1
                
        # set (0,0), the starting point, to 0 
        self.grid[0,0] = 0
        
        # set (-1,-1), the goal point, to 0 
        self.grid[-1, -1] = 0

        if q != 0: 
            self.grid[0, -1] = 2
        
        return 
        


    def fire_grows(self):
        """Simulates one time step of fire growth.
        """

        
        # Temp to store new fire locations 
        next_grid = np.copy(self.grid)

        # For each cell of the maze 
        for i in range(self.dim):
            for j in range(self.dim):
                
                k = 0                   # number of neighbors on fire
                p = Position([i, j])

                # Occupied cells cannot catch fire.  Skip over them.  
                if not self.is_empty_cell(p):
                    continue 

                # Get all neighbors of [i, j] that are not blocked.
                neighbors = p.list_of_moves(self)


                for neighbor in neighbors:              # For each neighbor
                    if neighbor is not None:            #   that is within grid bounds 
                        if self.grid[neighbor[0], neighbor[1]] == 2:  # Check if its on fire
                            k += 1

                # Chance of burn scales according to k
                burn_chance = 1 - (1-self.q)**k

                # Sample to see if [i,j] is now on fire
                if np.random.uniform(0, 1) < burn_chance:
                    next_grid[i, j] = 2
                    self.cells_on_fire.append([i, j])


        self.grid = np.copy(next_grid)
        return 



    def is_valid_cell(self, position):
        """Returns True if cell at position (i, j) is within grid bounds.
            False otherwise.  The contents of the cell are inconsequential 
        """
        i = position[0]
        j = position[1]
        
        if (i >= self.dim or i < 0) or \
            (j >= self.dim or j < 0):
            return False 
        
        else: 
            return True  
    
    


    def is_empty_cell(self, position):
        """Returns True if cell at position (i, j) is empty and within grid bounds.  
            False otherwise (if occupied, on fire, or out of grid bounds)
        """
        i = position[0]
        j = position[1]
        
        if self.grid[i, j] == 0:
            return True 
        else: 
            return False 
    
    
    
    def __str__(self):
        """Print utility"""
        return str(self.grid)
    
    def __repr__(self):
        """Repr utility"""
        return str(self.grid)
        

    def copy(self, ):
        m = Maze(self.dim, self.p, self.q)
        m.grid = np.copy(self.grid)
        m.cells_on_fire = list.copy(m.cells_on_fire)
        return m 
        

    def draw_grid(self, plot_title='Grid', animated=False):
        """Draws the maze (no path...see .draw_path())
        """

        # Making a copy so that self.grid is not mutated 
        grid_copy = np.copy(self.grid)

        # Defining color map for grid.
        #    - [0] white = empty 
        #    - [1] black = occupied 
        #    - [2] red = fire 
        #    - [3] blueish = start 

        cmap = colors.ListedColormap(['white','black','red','lightsteelblue'])
        grid_copy[0, 0] = 3

        fig, axes = plt.subplots()

        # Shows maze.grid as a 2D image with colormap applied.  
        plt.imshow(grid_copy, interpolation='None',
                        cmap=cmap, aspect='equal', axes=axes, animated=animated)

        fig.suptitle(plot_title)
        return fig


    def draw_path(self, path, plot_title='Solution Path', path_color='blue'):
        """ Draws the maze and solution path 
        """

        if path is None or len(path) == 0: 
            raise ValueError('Input path is invalid.')


        # Making a copy so that self.grid is not mutated 
        grid_copy = np.copy(self.grid)

        # Defining color map for grid.
        #    - [0] white = empty 
        #    - [1] black = occupied 
        #    - [2] red = fire 
        #    - [3] blueish = path 
        cmap = colors.ListedColormap(['white','black','red','lightsteelblue'])

        # Setting the positions in the path to 3 
        for position in path: 
            grid_copy[position[0], position[1]] = 3


        # Sigh.  Need to reverse the order of the coordinates because
        #   matplotlib.patches interprets it backwards (i.e. col, row)
        def reverse_coords_order(position):
            return (position[1], position[0])

        # Builds mpl path object to add to the axis.  
        #  Connects all the positions in the path with a line.
        verts = [reverse_coords_order(position) for position in path]
        codes = [Path.MOVETO,]
        for i in range(len(path)-1):
            codes.append(Path.LINETO)
        drawpath = Path(verts, codes)
        patch = patches.PathPatch(drawpath, facecolor='none', 
                                  lw=2, edgecolor=path_color)


        fig, axes = plt.subplots()

        # Shows maze.grid as a 2D image with colormap applied.  
        plt.imshow(grid_copy, interpolation='none', 
                        cmap=cmap, aspect='equal', axes=axes)

        # Add path to the axis
        axes.add_patch(patch)
        fig.suptitle(plot_title)

        return fig





    def animate_path(self, path, filename='we-didnt-start-the-fire', path_color='blue'):
        """Outputs a .mp4 file at workng directory level showing an agent running
            the input path on a copy of self's maze. 
            Also returns the state of the agent (alive or dead) 
        """

        if path is None or len(path) == 0: 
            raise ValueError('Input path is invalid') 

        alive = True 

        fig = plt.figure()
        ims = []

        maze_ = self.copy()

        for i in range(1, len(path)+1):
            
            # Path at iteration i
            p = path[:i]
            
            maze_.fire_grows()
            # Making a copy so that self.grid is not mutated 
            grid_copy = np.copy(maze_.grid)

            # Defining color map for grid.
            #    - [0] white = empty 
            #    - [1] black = occupied 
            #    - [2] red = fire 
            #    - [3] blueish = start 

            cmap = colors.ListedColormap(['white','black','red','lightsteelblue'])
            
            
            # Setting the positions in the path to 3 
            for position in p: 
                grid_copy[position[0], position[1]] = 3

                
            # Sigh.  Need to reverse the order of the coordinates because
            #   matplotlib.patches interprets it backwards (i.e. col, row)
            def reverse_coords_order(position):
                return (position[1], position[0])

            # Builds mpl path object to add to the axis.  
            #  Connects all the positions in the path with a line.
            verts = [reverse_coords_order(position) for position in path]
            codes = [Path.MOVETO,]
            for j in range(len(path)-1):
                codes.append(Path.LINETO)
            drawpath = Path(verts, codes)
            patch = patches.PathPatch(drawpath, facecolor='none', 
                                      lw=2, edgecolor='blue')

            
            # plots path 
            ax = plt.axes()
            ax.add_patch(patch)    
            # Shows maze.grid as a 2D image with colormap applied.  
            im = ax.imshow(grid_copy, interpolation='None',
                            cmap=cmap, aspect='equal', animated=True)

            ims.append([im])

            
            # stop if fire reaches position 
            curr = p[-1]
            if maze_.grid[curr[0], curr[1]] == 2: 
                alive = False 
                break
            


        ani = animation.ArtistAnimation(fig, ims, interval=200, blit=True,
                                        repeat_delay=1000)
        writer = FFMpegWriter(fps=15, metadata=dict(artist='Me'), bitrate=1500)
        ani.save(filename + ".mp4", writer=writer)

        return alive 