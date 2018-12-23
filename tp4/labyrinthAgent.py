#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from copy import deepcopy
import time
import numpy as np
import random

from deap import base
from deap import creator
from deap import tools


class Timer:
    def __enter__(self):
        self.start = time.clock()
        return self

    def __exit__(self, *args):
        self.end = time.clock()
        self.interval = self.end - self.start


class NoValidPathException(Exception):
    pass


class LabyrinthAgent(object):
    def __init__(self, grid, start_cell, end_cell, max_time_s):
        self.grid = grid
        self.start_cell = start_cell
        self.end_cell = end_cell
        self.max_time_s = max_time_s
        self.labyWidth = grid.shape[1]
        self.labyHeight = grid.shape[0]
        self.testCellSurrounded(self.start_cell)
        self.testCellSurrounded(self.end_cell)

    def decodePath(self, relativePath, start=None):
        if start == None:
            start = self.start_cell

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

    def testCellSurrounded(self, cell):
        '''
        Raises an exception if a cell is surrounded by walls
        '''

        possible_choices = ['L', 'D', 'R', 'U']

        notValid = True

        previous_cellY = cell[0]
        previous_cellX = cell[1]

        while notValid:
            if len(possible_choices) == 0:
                raise NoValidPathException('No path possible')

            gene = random.choice(possible_choices)
            possible_choices.remove(gene)

            coord = self.decodePath([gene], start=cell)[-1]

            if (coord[0] >= 0 and coord[1] >= 0 and coord[0] < self.labyHeight and coord[1] < self.labyWidth and self.grid[coord] != 1
                    and not (coord[0] == previous_cellY and coord[1] == previous_cellX)):
                notValid = False

    def fitness(self, individual):
        self.purge_genome(individual)
        absoluteCoorPath = self.decodePath(individual)

        fitness = len(individual)

        # = 0 if end cell is reached
        fitness += (abs(self.end_cell[0]-absoluteCoorPath[-1]
                        [0])+abs(self.end_cell[1]-absoluteCoorPath[-1][1]))

        return (fitness,)

    def generate_gene(self, cell=None):
        possible_choices = ['L', 'D', 'R', 'U']

        if cell is not None:
            if cell[0] == 0 or self.grid[cell[0]-1, cell[1]] == 1:
                possible_choices.remove('U')
            if cell[1] == 0 or self.grid[cell[0], cell[1]-1] == 1:
                possible_choices.remove('L')
            if cell[0] == self.labyHeight-1 or self.grid[cell[0]+1, cell[1]] == 1:
                possible_choices.remove('D')
            if cell[1] == self.labyWidth-1 or self.grid[cell[0], cell[1]+1] == 1:
                possible_choices.remove('R')

        gene = random.choice(possible_choices)

        return gene

    def purge_genome(self, individual):
        '''
        if finds a invalide gene (wall/outside of the grid), cuts off the genes from the invalid gene to the end of the individual dna
        '''
        # max allowed size of an individual is the manhattan distance from one corner to the opposite one
        max_size = self.labyWidth+self.labyHeight

        if len(individual) > max_size:
            individual[max_size:] = []

        absolutePath = self.decodePath(individual)

        i = 0
        for cell in absolutePath:
            # ignore path after solution
            if cell == self.end_cell:
                individual[i:] = []

            # eliminate path portion that passes through same cell as a previous one
            if cell in absolutePath[:i]:
                individual[i-1:] = []
                break

            # eliminates path portion after invalid cell (outside grid, wall)
            if cell[0] < 0 or cell[1] < 0 or cell[0] >= self.labyHeight or cell[1] >= self.labyWidth or self.grid[cell] == 1:
                individual[i-1:] = []
                break

            i += 1

    def solve(self):
        toolbox = base.Toolbox()
        toolbox.register("fitness", self.fitness)
        # the algorithm has to minimize the fitness
        creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
        creator.create("Individual", list, fitness=creator.FitnessMin)
        toolbox.register("mate", tools.cxOnePoint)
        toolbox.register("mutate", tools.mutShuffleIndexes, indpb=0.05)
        toolbox.register("select", tools.selRoulette)
        toolbox.register("init_gene", self.generate_gene, self.start_cell)
        toolbox.register("init_individual", tools.initRepeat,
                         creator.Individual, toolbox.init_gene, 1)
        toolbox.register("init_population", tools.initRepeat,
                         list, toolbox.init_individual)

        if self.max_time_s < 1:
            time_factor = 1
        else:
            time_factor = self.max_time_s

        pop = toolbox.init_population(
            n=self.labyHeight*self.labyWidth*time_factor)

        # Evaluate the entire population
        fitnesses = map(lambda ind: toolbox.fitness(
            ind), pop)
        for ind, fit in zip(pop, fitnesses):
            ind.fitness.values = fit

        remaining_time = self.max_time_s
        cxProba = 0.7
        mutProba = 0.1

        select_nb = int(len(pop)/10)

        if select_nb < 4:
            select_nb = 4

        best = []
        winners = []

        lastGenerationTop = []

        lastGenerationTopCount = 1

        episode_count = 0

        while remaining_time > 0:
            with Timer() as t:
                episode_count += 1

                for ind in pop:
                    ind.append(self.generate_gene())

                offspring = toolbox.select(pop, select_nb)

                # Apply crossover on the offspring
                for child1, child2 in zip(offspring[::2], offspring[1::2]):
                    if len(child1) > 1 and len(child2) > 1:
                        if random.random() < cxProba:
                            toolbox.mate(child1, child2)

                for ind in offspring:
                    if len(ind) > 1:
                        if random.random() < mutProba:
                            ind = toolbox.mutate(ind)[0]

                fitnesses = map(lambda ind: toolbox.fitness(ind), pop)
                for ind, fit in zip(pop, fitnesses):
                    ind.fitness.values = fit

                fitnesses = map(lambda ind: toolbox.fitness(ind), offspring)
                for ind, fit in zip(offspring, fitnesses):
                    ind.fitness.values = fit

                pop = sorted(pop, key=lambda ind: -1*ind.fitness.values[0])

                pop[:len(offspring)] = offspring

                if len(best) > 0:
                    pop += deepcopy(best)

                if len(lastGenerationTop) > 0 and episode_count > self.labyHeight:
                    pop += lastGenerationTop

                if episode_count > self.labyHeight-1:
                    lastGenerationTop = deepcopy(pop[-lastGenerationTopCount:])

                # it means that the manhattan distance from the last cell to the end is 0
                winners = [
                    ind for ind in pop if ind.fitness.values[0] == len(ind)]

                if len(winners) > 0:
                    winners = sorted(
                        winners, key=lambda ind: ind.fitness.values[0])
                    best.append(deepcopy(winners[0]))

            remaining_time -= t.interval

        if(len(best) > 0):
            best = sorted(best, key=lambda ind: ind.fitness.values[0])
            solution = self.decodePath(best[0])
            print(f"path: {solution}")
            print(f"length: {len(solution)}")
            print(f"total episodes: {episode_count}")
            print(f"total solutions: {len(best)}")
            return solution
        else:
            pop = sorted(pop, key=lambda ind: ind.fitness.values[0])
            partialSolution = self.decodePath(pop[0])
            print(f"total episodes: {episode_count}")
            print("No solution has been found")
            return partialSolution
