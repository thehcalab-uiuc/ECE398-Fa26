import sys
sys.path.append('./')

import random
import argparse
import numpy as np
from assignments.valueiter.simulate import simulateGridWorld
from assignments.valueiter.gridworld import GridWorld
from assignments.valueiter.implementation import Implementation
from assignments.valueiter.utils import plotResults


def run(implementationClass, headless=False, discountFactor=0.9):
    """ Runs the main program """
    
    random.seed(123)
    np.random.seed(123)

    # Create grid world
    world = GridWorld.fromJSON('assignments/valueiter/world.json')
    
    # Create implementation and run
    implementation = implementationClass()
    results = implementation.run(world, discountFactor=discountFactor)
    implementation.results = results
    implementation.gridWorld = world
    
    return implementation


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--discountFactor", type=float, default=0.9)
    args = parser.parse_args()

    if args.discountFactor < 0 or args.discountFactor > 1:
        raise ValueError("Discount factor must be in [0,1]")

    implementation = run(Implementation, discountFactor=args.discountFactor)
    plotResults(implementation.gridWorld, implementation.results)
    simulateGridWorld(implementation.gridWorld, implementation.results, (7, 4, 0))
