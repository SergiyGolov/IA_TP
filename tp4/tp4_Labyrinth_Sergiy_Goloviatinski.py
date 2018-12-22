#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import random
import matplotlib.pyplot as plt

from deap import base
from deap import creator
from deap import tools
from deap import algorithms

from copy import deepcopy

import time

WIDTH = 5
HEIGHT = 5
MAX_TIME = 60


class Timer:
    def __enter__(self):
        self.start = time.clock()
        return self

    def __exit__(self, *args):
        self.end = time.clock()
        self.interval = self.end - self.start


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

    if solution:
        solution = solution[1:-1]
        for cell in solution:
            grid[cell] = PATH
    else:
        print("No solution has been found")

    grid[end_cell] = END
    grid[start_cell] = START
    plt.matshow(grid, cmap="tab20c")
    plt.show()


def decodePath(start, relativePath):
    absoluteCoorPath = [start]
    for direction in relativePath:
        if direction == "L":
            absoluteCoorPath.append(
                (absoluteCoorPath[-1][0], absoluteCoorPath[-1][1]-1))
        elif direction == "R":
            absoluteCoorPath.append(
                (absoluteCoorPath[-1][0], absoluteCoorPath[-1][1]+1))
        elif direction == "D":
            absoluteCoorPath.append(
                (absoluteCoorPath[-1][0]+1, absoluteCoorPath[-1][1]))
        elif direction == "U":
            absoluteCoorPath.append(
                (absoluteCoorPath[-1][0]-1, absoluteCoorPath[-1][1]))
    return absoluteCoorPath


def fitness(individual, start_cell, end_cell):
    absoluteCoorPath = decodePath(start_cell, individual)

    fitness = len(individual)

    # = 0 if end cell is reached
    fitness += (abs(end_cell[0]-absoluteCoorPath[-1][0])+abs(end_cell[1]-absoluteCoorPath[-1][1]))


    return (fitness,)


'''
individual: is None if this is the first gene, else it contains all the genes for an individual generated before
'''


def generate_gene(grid, previous_cell, individual=None):
    notValid = True

    previous_cellY = previous_cell[0]
    previous_cellX = previous_cell[1]

    choiceCount = 0
    while notValid:
        choiceCount += 1
        if choiceCount == 50:  # loops until max choiceCount, to avoid being stuck if the cell is surrounded by walls for example
            if individual is None:
                # display labyrinth for debug purposes to proove that indeed there is no path possible
                display_labyrinth(grid, previous_cell, previous_cell)
                # if the starting cell of the labyrinth is surrounded by walls
                raise Exception('No path possible')
            else:
                return 'IGNORE'  # it means that the generated gene has to be ignored and is not valid

        gene = random.choice(['L', 'D', 'R', 'U'])
        coord = decodePath(previous_cell, [gene])[1]

        if (coord[0] >= 0 and coord[1] >= 0 and coord[0] < HEIGHT and coord[1] < WIDTH and grid[coord] != 1
                and not (coord[0] == previous_cellY and coord[1] == previous_cellX)):
            notValid = False

            # To prevent taking twice same cell and loop in the labyrinth
            if individual is not None and coord in individual:
                notValid = True

    return gene


'''
if finds a invalide gene (wall/outside of the grid), cuts off the genes from the invalid gene to the end of the individual dna
'''


def purge_genome(grid, start, individual):
    absolutePath = decodePath(start, individual)
    i = 0
    for cell in absolutePath:
        if cell[0] < 0 or cell[1] < 0 or cell[0] >= HEIGHT or cell[1] >= WIDTH or grid[cell] == 1: # eliminates path portion after invalid cell (outside grid, wall)
            individual[i:]=[]
            break
        # eliminate path portion that passes through same cell as a previous one
        if i>0 and cell in absolutePath[:i]:
            individual[i-1:]=[]
            break

        i += 1
    return individual


if __name__ == "__main__":

    grid = np.load("grids/grid30.npy")
    start_cell = (0,0)
    h = grid.shape[0]
    w = grid.shape[1]
    end_cell = (h - 1, w - 1)
    WIDTH=w
    HEIGHT=h
    # grid, start_cell, end_cell = generate_labyrinth(WIDTH, HEIGHT)

    toolbox = base.Toolbox()
    toolbox.register("fitness", fitness)
    # the algorithm has to minimize the fitness
    creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
    creator.create("Individual", list, fitness=creator.FitnessMin)
    toolbox.register("mate", tools.cxOnePoint)
    toolbox.register("mutate", tools.mutShuffleIndexes, indpb=0.1)
    toolbox.register("select", tools.selRoulette)
    toolbox.register("init_gene", generate_gene, grid, start_cell)
    toolbox.register("init_individual", tools.initRepeat,
                     creator.Individual, toolbox.init_gene, 1)
    toolbox.register("init_population", tools.initRepeat,
                     list, toolbox.init_individual)

    pop = toolbox.init_population(n=400)

    # Evaluate the entire population
    fitnesses = map(lambda ind: toolbox.fitness(
        ind, start_cell, end_cell), pop)
    for ind, fit in zip(pop, fitnesses):
        ind.fitness.values = fit
        # print(f"fitness:{ind.fitness.values}  {ind}")


    remaining_time = MAX_TIME
    cxProba = 0.7
    mutProba = 0.1
    select_nb = int(len(pop)/10)

    best=[]
    winners=[]

    while remaining_time > 0:
        with Timer() as t:

            emptyInd = False
            for ind in pop:
                if len(ind) > 0:
                    ind.append(generate_gene(
                        grid, decodePath(start_cell, ind)[-1], ind))
                else:
                    ind.append(generate_gene(grid, start_cell, ind))

                if ind[-1] == 'IGNORE':
                    ind.pop()
                    if len(ind) == 0:
                        emptyInd = True
                ind=purge_genome(grid, start_cell, ind)

            offspring = toolbox.select(pop, select_nb)

            if emptyInd:
                offspring = [ind for ind in offspring if len(ind) > 0]

            emptyInd = False
            # Apply crossover on the offspring
            for child1, child2 in zip(offspring[::2], offspring[1::2]):
                if random.random() < cxProba and len(child1) > 1 and len(child2) > 1:
                    toolbox.mate(child1, child2)
                    # verify that the individual is valid after the mate
                    child1=purge_genome(grid, start_cell, child1)
                    # verify that the individual is valid after the mate
                    child2=purge_genome(grid, start_cell, child2)
                    if not emptyInd and len(child1) == 0 or len(child2) == 0:
                        emptyInd = True

            if emptyInd:
                offspring = [ind for ind in offspring if len(ind) > 0]

            emptyInd = False
            for ind in offspring:
                if random.random() < mutProba and len(ind) > 1:
                    ind = toolbox.mutate(ind)[0]
                    # verify that the individual is valid after the mutaiton
                    ind=purge_genome(grid, start_cell, ind)
                # delete individuals with 0 valid DNA
                if not emptyInd and len(ind) == 0:
                    emptyInd = True

            if emptyInd:
                offspring = [ind for ind in offspring if len(ind) > 0]

            # childs = algorithms.varAnd(offspring, toolbox, cxProba, mutProba)

            fitnesses = map(lambda ind: toolbox.fitness(
                ind, start_cell, end_cell), pop)
            for ind, fit in zip(pop, fitnesses):
                ind.fitness.values = fit

            pop = sorted(pop, key=lambda ind: ind.fitness.values[0])

            
            pop[:select_nb] = offspring

            winners=[ind for ind in pop if ind.fitness.values[0]==len(ind)] # it means that the manhattan distance from the last cell to the end is 0

            winners = sorted(winners, key=lambda ind: ind.fitness.values[0])

            if len(winners)>0:
                best.append(deepcopy(winners[0]))


        remaining_time -= t.interval



    # best = [ind for i, ind in enumerate(
    #     best) if ind not in best[:i]]  # only unique paths


    best = sorted(best, key=lambda ind: ind.fitness.values[0])
    if(len(best)>0):
        ind = best[0]
        solution = decodePath(start_cell, ind)
        print(solution)
        print(f"length: {len(solution)}")
        display_labyrinth(grid, start_cell, end_cell, solution)
    else:
        display_labyrinth(grid, start_cell, end_cell)
    
