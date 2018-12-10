#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import random
import matplotlib.pyplot as plt

from deap import base
from deap import creator
from deap import tools
from deap import algorithms

WIDTH=5 
HEIGHT=5


def generate_labyrinth(width, height, wall_ratio=0.3):
    """ Randomly generates the labyrinth matrix, the values are:
    0 if the cell is free
    1 if there is a wall
    :param width int: width of the matrix
    :param height int: height of the matrix
    :wall_ratio float: chance for a cell to be a wall
    :return: tuple composed of:
    <matrix>: numpy 2d array
    <start_cell>: tuple of i, j indices for the start cell
    <end_cell>: tuple of i, j indices for the end cell
    """
    grid = np.random.rand(width, height)
    grid[grid >= 1 - wall_ratio] = 1
    grid[grid < 1 - wall_ratio] = 0
    free_cell_top = [i for i in range(0, width) if grid[0][i] != 1]
    start_idx = random.choice(free_cell_top)
    start_cell = (0, start_idx)
    free_cell_bottom = [i for i in range(0, width) if grid[-1][i] != 1]
    end_idx = random.choice(free_cell_bottom)
    end_cell = (height - 1, end_idx)
    return grid, start_cell, end_cell


# tuple format: (y,x)
def display_labyrinth(grid, start_cell, end_cell, solution=None):
    """ Display the labyrinth matrix and possibly the solution with matplotlib.
    Free cell will be in light gray.
    Wall cells will be in dark gray.
    Start and end cells will be in dark blue.
    Path cells (start, end excluded) will be in light blue.
    :param grid np.array: labyrinth matrix
    :param start_cell: tuple of i, j indices for the start cell
    :param end_cell: tuple of i, j indices for the end cell
    :param solution: list of successive tuple i, j indices who forms the path
    """
    grid = np.array(grid, copy=True)
    FREE_CELL = 19
    WALL_CELL = 16
    START = 0
    END = 0
    PATH = 2
    grid[grid == 0] = FREE_CELL
    grid[grid == 1] = WALL_CELL
    grid[start_cell] = START
    grid[end_cell] = END
    if solution:
        solution = solution[1:-1]
        for cell in solution:
            grid[cell] = PATH
    else:
        print("No solution has been found")
    plt.matshow(grid, cmap="tab20c")
    plt.show()

def decodePath(start,relativePath):
    absoluteCoorPath=[start]
    for direction in relativePath:
        if direction == "L":
            absoluteCoorPath.append((absoluteCoorPath[-1][0],absoluteCoorPath[-1][1]-1))
        elif direction=="R":
            absoluteCoorPath.append((absoluteCoorPath[-1][0],absoluteCoorPath[-1][1]+1))
        elif direction=="D":
            absoluteCoorPath.append((absoluteCoorPath[-1][0]+1,absoluteCoorPath[-1][1]))
        elif direction=="U":
            absoluteCoorPath.append((absoluteCoorPath[-1][0]-1,absoluteCoorPath[-1][1]))
    return absoluteCoorPath[1:]

def fitness(individual,grid,start_cell,end_cell):
    absoluteCoorPath=decodePath(start_cell,individual)
    fitness=0
    for gene in absoluteCoorPath:
        fitness+=abs(gene[0]-end_cell[0])+abs(gene[1]-end_cell[1])

    
    fitness/=len(individual)
    fitness+=len(individual)
    
    return (fitness,)

def generate_gene(grid,previous_cell):
    notValid=True
    # TO-DO: do something coherent if there are walls all around previous_cell
    while notValid:
        gene=random.choice(['L','D','R','U'])
        coord=decodePath(previous_cell,[gene])[0]
        if coord[0] >= 0 and coord[1] >= 0 and coord[0] <HEIGHT and coord[1]<WIDTH and grid[coord]!=1:
            notValid=False

    return gene

if __name__=="__main__":

    grid, start_cell, end_cell = generate_labyrinth(WIDTH, HEIGHT)
    # donc pos min = (0,0) et pos max = (9,9)

    toolbox = base.Toolbox()
    toolbox.register("fitness", fitness)
    creator.create("FitnessMin", base.Fitness, weights=(-1.0,)) #the algorithm has to minimize the fitness
    creator.create("Individual", list, fitness=creator.FitnessMin) 
    toolbox.register("mate", tools.cxUniform)
    toolbox.register("mutate", tools.mutShuffleIndexes,indpb=0.1)
    toolbox.register("select", tools.selTournament)
    toolbox.register("init_gene", generate_gene, grid,start_cell)
    toolbox.register("init_individual", tools.initRepeat, creator.Individual, toolbox.init_gene, 1)
    toolbox.register("init_population", tools.initRepeat, list, toolbox.init_individual)
    
    pop = toolbox.init_population(n=10)
   
    # Evaluate the entire population
    fitnesses = map(lambda ind: toolbox.fitness(ind,grid,start_cell,end_cell),pop)
    for ind, fit in zip(pop, fitnesses):
        ind.fitness.values = fit
        print(f"fitness:{ind.fitness.values}  {ind}")



   

    solution=[start_cell]
    solution+=decodePath(start_cell,pop[0])
    solution.append(end_cell)


    
    display_labyrinth(grid, start_cell, end_cell)