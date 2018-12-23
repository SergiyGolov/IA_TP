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
        self.pathLimit = int((self.labyHeight+self.labyWidth)*1.5)

    def decodePath(self, relativePath, start=None):
        """
        Converts a chromosome ("relative path") to a list of cells ("absolute path")
        """
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

    def manhattanDistanceToEnd(self, cell):
        return (abs(self.end_cell[0]-cell
                    [0])+abs(self.end_cell[1]-cell[1]))

    def testCellSurrounded(self, cell):
        """
        Raises an exception if a cell is surrounded by walls and corners
        """

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

    def fitness(self, individual, purge=False):
        if purge:
            self.purge_genome(individual)
        absoluteCoorPath = self.decodePath(individual)

        fitness = len(individual)

        # = 0 if end cell is reached
        fitness += self.manhattanDistanceToEnd(absoluteCoorPath[-1])*25

        return (fitness,)

    def generate_first_gene(self):
        possible_choices = ['L', 'D', 'R', 'U']

        cell = self.start_cell
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

    def generate_gene(self, individual=None):
        possible_choices = ['L', 'D', 'R', 'U']

        if individual is not None:
            if len(individual) > self.pathLimit:
                return "CUT"
            path = self.decodePath(individual)
            cell = path[-1]

            # removes non valid choices (walls, already visited cell or outside of grid)
            if cell[0] == 0 or self.grid[cell[0]-1, cell[1]] == 1 or (cell[0]-1, cell[1]) in path:
                possible_choices.remove('U')
            if cell[1] == 0 or self.grid[cell[0], cell[1]-1] == 1 or (cell[0], cell[1]-1) in path:
                possible_choices.remove('L')
            if cell[0] == self.labyHeight-1 or self.grid[cell[0]+1, cell[1]] == 1 or (cell[0]+1, cell[1]) in path:
                possible_choices.remove('D')
            if cell[1] == self.labyWidth-1 or self.grid[cell[0], cell[1]+1] == 1 or (cell[0], cell[1]+1) in path:
                possible_choices.remove('R')

        if len(possible_choices) > 0:
            if individual is not None and random.random() < 0.5:
                directionsWithScores = {}
                for direction in possible_choices:
                    directionsWithScores[direction] = self.manhattanDistanceToEnd(
                        self.decodePath(direction, cell)[-1])

                if random.random() < 0.75:
                    worstChoice = max(directionsWithScores,
                                      key=directionsWithScores.get)
                    if len(possible_choices) > 1:
                        possible_choices.remove(worstChoice)
                else:
                    bestChoice = min(directionsWithScores,
                                     key=directionsWithScores.get)
                    return bestChoice

            gene = random.choice(possible_choices)
        else:
            gene = "CUT"

        return gene

    def purge_genome(self, individual):
        """
        if finds a invalide gene (wall/outside of the grid/passing through already passed cell), cuts off the genes from the invalid gene to the end of the individual dna
        """

        if len(individual) > self.pathLimit:
            individual[self.pathLimit-1:] = []

        absolutePath = self.decodePath(individual)

        i = 0
        for cell in absolutePath:
            # # ignore path after solution
            # if cell == self.end_cell:
            #     individual[i+1:] = []

            # eliminates path portion after invalid cell (outside grid, wall)
            if cell[0] < 0 or cell[1] < 0 or cell[0] >= self.labyHeight or cell[1] >= self.labyWidth or self.grid[cell] == 1:
                individual[i-1:] = []
                break

            # eliminate path portion that passes through same cell as a previous one
            if cell in absolutePath[:i]:
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
        toolbox.register("mutate", tools.mutShuffleIndexes, indpb=0.1)
        toolbox.register("select", tools.selRoulette)
        toolbox.register(
            "init_gene", self.generate_first_gene)
        toolbox.register("init_individual", tools.initRepeat,
                         creator.Individual, toolbox.init_gene, 1)
        toolbox.register("init_population", tools.initRepeat,
                         list, toolbox.init_individual)

        popLength = int((self.labyWidth+self.labyHeight)*0.25)

        if popLength > 500:
            popLength = 500

        if popLength < 10:
            popLength = 10

        pop = toolbox.init_population(n=popLength)

        # Evaluate the entire population
        fitnesses = map(lambda ind: toolbox.fitness(
            ind), pop)
        for ind, fit in zip(pop, fitnesses):
            ind.fitness.values = fit

        remaining_time = self.max_time_s

        cxProba = 0.25
        mutProba = 0.75

        select_nb = int(popLength/3)

        if select_nb < 4:
            select_nb = 4
        elif select_nb > 150:
            select_nb = 150

        best = []
        winners = []

        champion = deepcopy(pop[0])

        episode_count = 0

        while remaining_time > 0:
            with Timer() as t:
                episode_count += 1

                for ind in pop:
                    newGene = self.generate_gene(ind)
                    if newGene != "CUT":
                        ind.append(newGene)

                offspring = toolbox.select(pop, select_nb)

                # Apply crossover on the offspring
                for child1, child2 in zip(offspring[::2], offspring[1::2]):
                    if len(child1) > 1 and len(child2) > 1:
                        if random.random() < cxProba:
                            toolbox.mate(child1, child2)

                for ind in offspring:
                    if len(ind) > 1:
                        if random.random() < mutProba:
                            toolbox.mutate(ind)

                fitnesses = map(lambda ind: toolbox.fitness(ind), pop)
                for ind, fit in zip(pop, fitnesses):
                    ind.fitness.values = fit

                # because these individuals are generated randomly by the genetic algorithm, we have to purge their dna
                fitnesses = map(lambda ind: toolbox.fitness(
                    ind, purge=True), offspring)
                for ind, fit in zip(offspring, fitnesses):
                    ind.fitness.values = fit

                pop = sorted(pop, key=lambda ind: -1*ind.fitness.values[0])

                # replace weaknest individuals with offspring
                pop[:len(offspring)] = offspring


                if champion is not None and episode_count > self.labyHeight and episode_count % 5 == 0:
                    pop.append(deepcopy(champion))

                if episode_count > self.labyHeight-1:
                    if champion.fitness.values[0] > pop[-1].fitness.values[0]:
                        champion = deepcopy(pop[-1])

                # for a winner, the manhattan distance from the last cell to the end is 0
                winners = [
                    ind for ind in pop if ind.fitness.values[0] == len(ind)]

                if len(winners) > 0:
                    winners = sorted(
                        winners, key=lambda ind: ind.fitness.values[0])
                    best.append(deepcopy(winners[0]))
                    if len(best) == 1:
                        print(
                            f"Found first solution after {self.max_time_s-remaining_time} s")

            remaining_time -= t.interval

        if len(best) > 0:
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
            print(f"Best fitness: {pop[0].fitness.values[0]}")
            print("No solution has been found")
            return partialSolution
