#
# This is the file you will be modifying.
#


import numpy as np
from assignments.valueiter.gridworld import GridWorld, Action, Cell, CellType


class Implementation:
    def __init__(self):
        pass

    # Your implementation!
    # --------------------------------------------------------------------------------------------------------
    def valueIteration(self, gridWorld, discountFactor=0.9, convergenceThreshold=0.0001, maxIterations=1000):
        """ Run value iteration

        :param gridWorld: grid world to perform the value iteration on
        :param discountFactor: discount factor
        :param convergenceThreshold: threshold for convergence
        :param maxIterations: maximum number of iterations
        :returns: an np.array of shape (height, width) containing values of each state
        """
        # TODO: Perform value iteration
        # Hint: use gridWorld.world to retrieve the "world" array (list of lists, where each entry is a Cell object)
        # Hint: use cell.getReward and cell.getTransitionProbabilities functions
        # Note: for this implementation, your delta should equal the maximum change in value for any cell for the iteration.
        # Your answer here
        values = np.zeros((gridWorld.height, gridWorld.width))
        for iteration in range(maxIterations):
            delta = 0.0
            if delta < convergenceThreshold:
                break
        return values

    def policyExtraction(self, gridWorld, values, discountFactor):
        """ Run policy extraction to extract the optimal policy
        
        :param gridWorld: grid world to perform the value iteration on
        :param values: np.array of shape (height, width) containing values of each state
        :param discountFactor: discount factor
        :returns: an np.array of shape (height, width) containing best action at each state
        """
        # TODO: Extract the policy
        # Your answer here
        policy = np.full((gridWorld.height, gridWorld.width), Action.NONE, dtype=object)
        for row in gridWorld.world:
            for cell in row:
                pass
        return policy
    # --------------------------------------------------------------------------------------------------------
    
    def run(self, gridWorld, discountFactor=0.9, convergenceThreshold=0.0001, maxIterations=1000):
        """ Run the value iteration and policy extraction loop

        :param gridWorld: grid world to perform the value iteration on
        :param discountFactor: discount factor
        :param convergenceThreshold: threshold for convergence
        :param maxIterations: maximum number of iterations
        :returns: a dictionary with the following:
            'values': an np.array of shape (height, width) containing values of each state
            'policy': an np.array of shape (height, width) containing best action at each state
        """
        values = self.valueIteration(gridWorld, discountFactor=discountFactor, convergenceThreshold=convergenceThreshold, maxIterations=maxIterations)
        policy = self.policyExtraction(gridWorld, values=values, discountFactor=discountFactor)

        # Return output values and policy
        return {
            'values': values,
            'policy': policy
        }
