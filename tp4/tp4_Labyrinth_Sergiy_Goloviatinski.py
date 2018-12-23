#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import random
import matplotlib.pyplot as plt
import sys

from labyrinthAgent import LabyrinthAgent
from labyrinthAgent import NoValidPathException


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


def solve_labyrinth(grid, start_cell, end_cell, max_time_s):
    """ Attempt to solve the labyrinth by returning the best path found
    :param grid np.array: numpy 2d array
    :start_cell tuple: tuple of i, j indices for the start cell
    :end_cell tuple: tuple of i, j indices for the end cell
    :max_time_s float: maximum time for running the algorithm
    :return list: list of successive tuple i, j indices who forms the path
    """
    try:
        agent = LabyrinthAgent(grid, start_cell, end_cell, max_time_s)
        display_labyrinth(grid, start_cell, end_cell, agent.solve())
    except NoValidPathException as e:
        print(e)
        display_labyrinth(grid, start_cell, end_cell)


if __name__ == "__main__":

    if len(sys.argv) > 2:
        grid = np.load(sys.argv[1])
        start_cell = (0, 0)
        h = grid.shape[0]
        w = grid.shape[1]
        end_cell = (h - 1, w - 1)
        WIDTH = w
        HEIGHT = h
        MAX_TIME = int(sys.argv[2])
    else:
        WIDTH = 5
        HEIGHT = 5
        MAX_TIME = 0.25
        grid, start_cell, end_cell = generate_labyrinth(WIDTH, HEIGHT)

    solve_labyrinth(grid, start_cell, end_cell, MAX_TIME)
