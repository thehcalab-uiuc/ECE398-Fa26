import json
import numpy as np
from enum import Enum


HAZARD_REWARD = -20
GOAL_REWARD = 10
OTHER_REWARD = -1


class CellType(Enum):
    """ DO NOT MODIFY """
    NORMAL = 0
    SLIPPERY = 1
    HAZARD = 2
    GOAL = 3


class Action(Enum):
    """ DO NOT MODIFY """
    NONE = 0
    UP = 1
    DOWN = 2
    LEFT = 3
    RIGHT = 4


class Cell:
    def __init__(self, world, row, col, type, actions, defaultFollowActionProbability, slipperyFollowActionProbability):
        """ Grid cell
        
        DO NOT MODIFY
        """
        self.world = world
        self.row = row
        self.col = col
        self.type = type
        self.actions = actions
        self.defaultFollowActionProbability = defaultFollowActionProbability
        self.slipperyFollowActionProbability = slipperyFollowActionProbability

    def getNextState(self, action):
        """ Gets the (row,col) location of next state if action is followed at this state (non-stochastic)

        DO NOT MODIFY

        :param action: action to take
        :returns: cell object corresponding to the next state
        """
        row, col = self.row, self.col
        if action == Action.LEFT:
            newRow, newCol = (row, max(0, col - 1))
        elif action == Action.RIGHT:
            newRow, newCol = (row, min(col + 1, self.world.width - 1))
        elif action == Action.UP:
            newRow, newCol = (max(0, row - 1), col)
        elif action == Action.DOWN:
            newRow, newCol = (min(row + 1, self.world.height - 1), col) 
        elif action == Action.NONE:
            newRow, newCol = row, col
        else:
            raise ValueError("Invalid action")
        return self.world.world[newRow][newCol]
    
    def getTransitionProbabilities(self, action):
        """ Returns the transition probabilities for this cell
        
        DO NOT MODIFY

        :param action: action to consider
        :returns: list of (cell, prob) corresponding to possible next states and their probabilities
        """
        if action not in self.actions:
            raise ValueError("Provided action is not in the viable set of actions")
        followProbability = self.slipperyFollowActionProbability if self.type == CellType.SLIPPERY else self.defaultFollowActionProbability
        if followProbability > 1.0 or followProbability < 0.0:
            raise ValueError("Invalid probability")
        remainingProbability = (1.0 - followProbability) / (len(self.actions) - 1) if len(self.actions) > 1 else 0.0
        return [
            (self.getNextState(act), followProbability if act == action else remainingProbability) for act in self.actions
        ]

    def getReward(self):
        """ Returns the reward for this cell
        
        DO NOT MODIFY
        """
        if self.type == CellType.GOAL:
            return GOAL_REWARD
        elif self.type == CellType.HAZARD:
            return HAZARD_REWARD
        else:
            return OTHER_REWARD


class GridWorld:
    def __init__(self, cellTypes, defaultFollowActionProbability=1.0, slipperyFollowActionProbability=0.5):
        """ Create a grid world
        
        DO NOT MODIFY

        :param cellTypes: 2D array of cell types
        """
        self.defaultFollowActionProbability = defaultFollowActionProbability
        self.slipperyFollowActionProbability = slipperyFollowActionProbability
        world = []
        self.width = len(cellTypes[0])
        self.height = len(cellTypes)
        for rowIndex, row in enumerate(cellTypes):
            worldRow = []
            for colIndex, cellType in enumerate(row):
                possibleActions = []
                if cellType != CellType.GOAL and cellType != CellType.HAZARD: # hazard and goal are terminal
                    if rowIndex > 0:
                        possibleActions.append(Action.UP)
                    if colIndex > 0:
                        possibleActions.append(Action.LEFT)
                    if rowIndex < (self.height - 1):
                        possibleActions.append(Action.DOWN)
                    if colIndex < (self.width - 1):
                        possibleActions.append(Action.RIGHT)
                worldRow.append(Cell(
                    world=self, row=rowIndex, col=colIndex, type=cellType, actions=possibleActions,
                    defaultFollowActionProbability=defaultFollowActionProbability, slipperyFollowActionProbability=slipperyFollowActionProbability
                ))
            world.append(worldRow)
        self.world = world

    def getWorldImage(self):
        """ Create an RGB image corresponding to the world
        
        DO NOT MODIFY
        """
        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        for rowIndex, row in enumerate(self.world):
            for colIndex, cell in enumerate(row):
                if cell.type == CellType.HAZARD:
                    color = np.array([255,0,0], dtype=np.uint8)
                elif cell.type == CellType.SLIPPERY:
                    color = np.array([0,0,255], dtype=np.uint8)
                elif cell.type == CellType.GOAL:
                    color = np.array([0,255,0], dtype=np.uint8)
                else:
                    color = np.array([200,200,200], dtype=np.uint8)
                image[rowIndex, colIndex] = color
        return image

    @staticmethod
    def fromJSON(file):
        """ Create a grid world from a provided JSON file
        
        DO NOT MODIFY
        """
        with open(file, 'r') as f:
            data = json.load(f)
        cellTypes = [[CellType(entry) for entry in row] for row in data['world']]
        return GridWorld(cellTypes, data['defaultFollowActionProbability'], data['slipperyFollowActionProbability'])
